# SPDX-FileCopyrightText: 2026 Mappalink
#
# SPDX-License-Identifier: MIT

"""Omron ARCL connector — bridges Omron HD1500 ARCL to InOrbit Cloud."""

import asyncio
import contextlib
import logging
import math
import re
from pathlib import Path
from typing import override

from inorbit_connector.commands import parse_custom_command_args
from inorbit_connector.connector import Connector, CommandResultCode
from inorbit_connector.models import MapConfigTemp
from inorbit_edge.robot import (
    COMMAND_CUSTOM_COMMAND,
    COMMAND_MESSAGE,
    COMMAND_NAV_GOAL,
    LaserConfig,
)
from inorbit_edge_executor.inorbit import InOrbitAPI as MissionInOrbitAPI

from .config.models import ConnectorConfig
from .arcl_client import ArclClient
from .goal_tracker import GoalTracker
from .mission_exec import OmronMissionExecutor
from .plc_client import PlcError, TablePlc, declare_client_ams_net_id

logger = logging.getLogger(__name__)

# Block driving name used for pause/resume
BLOCK_NAME = "InOrbit"

# ARCL status prefixes for dock/undock completion polling
_DOCK_ACTIVE_PREFIXES = ("Docking", "Undocking", "Going to", "Driving to")
# "Stopped" is where ARCL settles after a successful undock (observed
# 2026-09-03). This list is used only by the dock/undock wait, never by
# navigation, so accepting it here cannot mask an interrupted drive.
_DOCK_SUCCESS_PREFIXES = ("Parked", "Idle", "Arrived at", "Stopped")
_DOCK_FAILURE_PREFIXES = ("Failed to get to", "Failed going to")
_DOCK_POLL_INTERVAL = 1.0
_DOCK_TIMEOUT = 120.0

# ARCL Status field transitions for executeMacro completion polling.
# Verified 2026-05-08; see fm-fsm-docs/docs/omron/OMRON_DOCKING.md.
# Identity is carried by the *transition* out of `Executing macro <name>`,
# not by the destination prefix — the failure line ("Error: ...") doesn't
# carry the macro name, which is fine because Phase 1 already proved our
# macro owned the Status field.
_MACRO_ACTIVE_PREFIX = "Executing macro "
_MACRO_SUCCESS_PREFIX = "Completed macro "
_MACRO_FAILURE_PREFIXES = ("Error:",)  # tight — bare "Failed" is too generic
_MACRO_KICKOFF_GRACE = 5.0  # max seconds to wait for Status to reflect kickoff
_MACRO_TIMEOUT = 75.0  # PrecisionDrive map config Timeout=60; +15s margin
_MACRO_POLL_INTERVAL = 1.0

# ARCL status field → InOrbit key-value name.
# Known fields are mapped to conventional InOrbit names where possible.
_ARCL_STATUS_MAP: dict[str, str] = {
    "StateOfCharge": "battery_percent",
    "Temperature": "temperature",
    "DockingState": "docking_state",
    "LocalizationScore": "localization_score",
    "Status": "omron_status",
    "ExtendedStatusForHumans": "omron_status_text",
    "ForcedState": "forced_state",
    "ChargeState": "charge_state",
    "BatteryVoltage": "battery_voltage",
    "Eta": "eta",
    "GoalName": "goal_name",
    "HasArrived": "has_arrived",
    "DistToGoal": "distance_to_goal",
    "DistanceToGoal": "distance_to_goal",
    "ErrorState": "error_state",
}

# InOrbit keys that should be published as floats
_NUMERIC_FIELDS: set[str] = {
    "battery_percent",
    "temperature",
    "localization_score",
    "battery_voltage",
    "eta",
    "distance_to_goal",
}

# ARCL fields handled separately (not published as key-values)
_SKIP_FIELDS: set[str] = {"Location"}

# plc_legs --action → named height in TablePlcConfig.heights
_PLC_ACTION_HEIGHT_KEY = {"retract": "retracted", "extend": "pickup"}
# Telemetry cycles to skip a table PLC after a failed poll (~seconds at 1 Hz)
_PLC_POLL_BACKOFF_CYCLES = 15
# Max seconds a telemetry poll may spend on one PLC before treating it offline
_PLC_POLL_TIMEOUT = 3.0
# Button (custom command) plc_legs reports SUCCESS once the PLC is moving;
# it fails if the PLC has not started the move within this many seconds.
_PLC_START_TIMEOUT = 10.0


def _to_snake_case(name: str) -> str:
    """Convert CamelCase to snake_case for unmapped ARCL fields."""
    return re.sub(r"(?<=[a-z0-9])(?=[A-Z])", "_", name).lower()


def cartesian_to_ranges(
    points: list[tuple[float, float]],
    robot_x_mm: float,
    robot_y_mm: float,
    robot_yaw: float,
    angle_min: float,
    angle_max: float,
    n_points: int,
    range_max: float,
) -> list[float]:
    """Convert ARCL world-frame (x_mm, y_mm) points to polar ranges in robot frame.

    ARCL rangeDeviceGetCurrent returns points in the map/world frame.
    We transform to robot-local frame, then bin into polar ranges.

    Returns a list of n_points range values (meters). Bins with no data are
    set to inf (no obstacle detected at that angle).
    """
    ranges = [math.inf] * n_points
    if not points:
        return ranges

    cos_yaw = math.cos(robot_yaw)
    sin_yaw = math.sin(robot_yaw)
    angle_span = angle_max - angle_min

    for x_mm, y_mm in points:
        # Translate to robot-centered coordinates
        dx = x_mm - robot_x_mm
        dy = y_mm - robot_y_mm
        # Rotate from world frame to robot-local frame (inverse rotation)
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy

        angle = math.atan2(local_y, local_x)
        if angle < angle_min or angle > angle_max:
            continue
        distance_m = math.hypot(local_x, local_y) / 1000.0
        if distance_m > range_max:
            continue
        # Map angle to bin index
        idx = int((angle - angle_min) / angle_span * (n_points - 1))
        idx = max(0, min(n_points - 1, idx))
        # Keep the closest reading per bin
        if distance_m < ranges[idx]:
            ranges[idx] = distance_m
    return ranges


class OmronArclConnector(Connector):
    def __init__(self, robot_id: str, config: ConnectorConfig) -> None:
        super().__init__(robot_id=robot_id, config=config)
        cfg = config.connector_config
        self._arcl = ArclClient(
            host=cfg.arcl_host,
            port=cfg.arcl_port,
            password=cfg.arcl_password,
            connection_timeout=cfg.arcl_timeout,
            reconnect_interval=cfg.arcl_reconnect_interval,
        )
        self._map_id = cfg.map_id
        self._map_file = cfg.map_file
        self._map_resolution = cfg.map_resolution
        self._map_origin_x = cfg.map_origin_x
        self._map_origin_y = cfg.map_origin_y
        self._laser_names = cfg.laser_names
        self._laser_angle_min = cfg.laser_angle_min
        self._laser_angle_max = cfg.laser_angle_max
        self._laser_range_min = cfg.laser_range_min
        self._laser_range_max = cfg.laser_range_max
        self._laser_n_points = cfg.laser_n_points
        self._lasers_registered = False
        self._goal_tracker = GoalTracker()
        self._goal_tracker_enabled = True
        # Last navigation command for pause/resume — ARCL block driving
        # cancels the active goal, so resume must re-send it.
        self._last_nav_goal: str | None = None
        self._last_nav_point: tuple[int, int, int] | None = None
        # Workbench lifting-column PLCs (table transport missions)
        self._plc_tables: dict[str, TablePlc] = {
            table_id: TablePlc(
                ip=table_cfg.ip,
                ams_net_id=table_cfg.ams_net_id,
                deadband_mm=cfg.plc_deadband_mm,
                check_position_valid=cfg.plc_check_position_valid,
                stall_secs=cfg.plc_stall_timeout_secs,
            )
            for table_id, table_cfg in cfg.plc_tables.items()
        }
        self._plc_heights = {tid: t.heights for tid, t in cfg.plc_tables.items()}
        self._plc_move_timeout_secs = cfg.plc_move_timeout_secs
        self._plc_poll_skip: dict[str, int] = {}
        # Button-started moves finishing in the background
        self._plc_move_tasks: set[asyncio.Task] = set()
        if self._plc_tables and cfg.plc_client_ams_net_id:
            try:
                declare_client_ams_net_id(cfg.plc_client_ams_net_id)
            except Exception as e:
                logger.error("Failed to declare client AmsNetId: %s", e)
        self._mission_executor = OmronMissionExecutor(
            robot_id=robot_id,
            inorbit_api=MissionInOrbitAPI(
                base_url=self._get_session().inorbit_rest_api_endpoint,
                api_key=self.config.api_key,
            ),
            arcl_client=self._arcl,
            database_file=cfg.mission_database_file,
            on_cloud_resume=self._resume_last_goal,
            plc_tables=self._plc_tables,
            plc_heights=self._plc_heights,
            plc_move_timeout_secs=self._plc_move_timeout_secs,
        )

    # -- Lifecycle ---------------------------------------------------------

    @override
    async def _connect(self) -> None:
        # Start the ARCL connection manager (retries internally).
        # Don't block waiting — the base class needs _connect() to return
        # so it can initialize the InOrbit session. The execution loop
        # already skips telemetry when ARCL is not connected.
        await self._arcl.connect()
        try:
            await self._arcl.wait_for_connection(timeout=self._arcl.timeout)
            logger.info("Connected to Omron ARCL at %s:%s", self._arcl.host, self._arcl.port)
        except TimeoutError:
            logger.warning(
                "Robot at %s:%s not reachable yet, will keep retrying in background",
                self._arcl.host,
                self._arcl.port,
            )
        await self._mission_executor.initialize()

    @override
    async def _disconnect(self) -> None:
        await self._mission_executor.shutdown()
        await self._arcl.disconnect()
        for task in list(self._plc_move_tasks):
            task.cancel()  # releases g_xExecuteMove, which stops the legs
        if self._plc_move_tasks:
            await asyncio.gather(*self._plc_move_tasks, return_exceptions=True)
        for plc in self._plc_tables.values():
            await plc.close()
        logger.info("Disconnected from Omron ARCL")

    # -- Main loop (~1 Hz) ------------------------------------------------

    @override
    async def _execution_loop(self) -> None:
        # Table PLCs are independent machines — publish their state even
        # when the robot itself is unreachable.
        if self._plc_tables:
            await self._publish_plc_telemetry()

        if not self._arcl.is_connected():
            logger.warning("ARCL not connected, skipping telemetry cycle")
            return

        # Re-enable native goal tracking when edge executor is idle
        if not self._goal_tracker_enabled:
            executor_idle = self._get_session().missions_module.executor.wait_until_idle(0)
            if executor_idle:
                self._goal_tracker_enabled = True

        # Register lasers on first loop (session is available now)
        if self._laser_names and not self._lasers_registered:
            configs = [
                LaserConfig(
                    x=0.0,
                    y=0.0,
                    yaw=0.0,
                    angle=(self._laser_angle_min, self._laser_angle_max),
                    range=(self._laser_range_min, self._laser_range_max),
                    n_points=self._laser_n_points,
                )
                for _ in self._laser_names
            ]
            self._get_session().register_lasers(configs)
            self._lasers_registered = True
            logger.info("Registered %d laser(s): %s", len(configs), self._laser_names)

        try:
            status = await self._arcl.query_status()
        except Exception as e:
            logger.error("ARCL status query failed: %s", e)
            return

        if not status:
            return

        # Parse location: "x y theta" in mm mm degrees
        location_str = status.get("Location", "")
        x_mm, y_mm = 0.0, 0.0
        x_m, y_m, yaw_rad = 0.0, 0.0, 0.0
        if location_str:
            try:
                parts = location_str.split()
                x_mm = float(parts[0])
                y_mm = float(parts[1])
                x_m = x_mm / 1000.0
                y_m = y_mm / 1000.0
                yaw_rad = math.radians(float(parts[2]))
            except (ValueError, IndexError) as e:
                logger.warning("Failed to parse location '%s': %s", location_str, e)

        self.publish_pose(x=x_m, y=y_m, yaw=yaw_rad, frame_id=self._map_id)

        # Build key-values from all ARCL status fields
        kv: dict[str, str | float] = {}
        for arcl_key, value in status.items():
            if arcl_key in _SKIP_FIELDS:
                continue
            inorbit_key = _ARCL_STATUS_MAP.get(arcl_key, _to_snake_case(arcl_key))
            if inorbit_key in _NUMERIC_FIELDS:
                try:
                    kv[inorbit_key] = float(value)
                except (ValueError, TypeError):
                    kv[inorbit_key] = value
            else:
                kv[inorbit_key] = value

        if kv:
            self.publish_key_values(**kv)

        # Update goal tracker and publish mission_tracking if changed
        # Suppressed while edge executor is running to avoid double-reporting
        if self._goal_tracker_enabled:
            tracking_payload = self._goal_tracker.update(status)
            if tracking_payload is not None:
                self._publish_mission_tracking(tracking_payload)
                # Do NOT clear _last_nav_* here — ARCL block driving
                # causes "Stopped" which GoalTracker reports as "Done",
                # but we need the saved goal for resume. Explicit stop
                # and successful goto_goal commands handle their own
                # clearing in _handle_custom_command.

        # Query odometer for velocity and publish via publish_odometry
        try:
            odometer = await self._arcl.query_odometer()
            if odometer:
                velocity_mps = float(odometer.get("Velocity", 0)) / 1000.0
                self.publish_odometry(linear_speed=velocity_mps)
        except Exception as e:
            logger.debug("Odometer query failed: %s", e)

        # Query laser scans and publish to InOrbit
        if self._laser_names:
            await self._publish_lasers(x_mm, y_mm, x_m, y_m, yaw_rad)

    async def _publish_plc_telemetry(self) -> None:
        """Publish per-table PLC state as key-values (plc_<table>_*).

        A failed poll marks the table offline and backs off for
        ``_PLC_POLL_BACKOFF_CYCLES`` cycles so an unreachable PLC cannot
        stall the ~1 Hz robot telemetry loop.
        """
        for table_id, plc in self._plc_tables.items():
            skip = self._plc_poll_skip.get(table_id, 0)
            if skip > 0:
                self._plc_poll_skip[table_id] = skip - 1
                continue
            try:
                state = await asyncio.wait_for(plc.read_state(), timeout=_PLC_POLL_TIMEOUT)
            except (PlcError, TimeoutError) as e:
                logger.debug("PLC %s poll failed: %s", table_id, e)
                self._plc_poll_skip[table_id] = _PLC_POLL_BACKOFF_CYCLES
                self.publish_key_values(**{f"plc_{table_id}_online": False})
                continue
            kv: dict[str, str | float | bool] = {
                f"plc_{table_id}_online": True,
                f"plc_{table_id}_height_mm": float(state.height_mm),
                f"plc_{table_id}_busy": state.busy,
                f"plc_{table_id}_done": state.done,
                f"plc_{table_id}_error": state.error,
                f"plc_{table_id}_error_code": float(state.error_code),
                f"plc_{table_id}_status": state.status_text,
            }
            if state.position_valid is not None:
                kv[f"plc_{table_id}_position_valid"] = state.position_valid
            self.publish_key_values(**kv)

    def _publish_mission_tracking(self, payload: dict) -> None:
        """Publish mission_tracking as an event (matches MiR connector pattern)."""
        self._get_session().publish_key_values(
            key_values={"mission_tracking": payload}, is_event=True
        )

    # -- Lasers ------------------------------------------------------------

    async def _publish_lasers(
        self,
        robot_x_mm: float,
        robot_y_mm: float,
        x_m: float,
        y_m: float,
        yaw_rad: float,
    ) -> None:
        """Query all configured lasers and publish to InOrbit."""
        all_ranges: list[list[float]] = []
        for laser_name in self._laser_names:
            try:
                points = await self._arcl.query_laser_scan(laser_name)
                ranges = cartesian_to_ranges(
                    points,
                    robot_x_mm,
                    robot_y_mm,
                    yaw_rad,
                    self._laser_angle_min,
                    self._laser_angle_max,
                    self._laser_n_points,
                    self._laser_range_max,
                )
                all_ranges.append(ranges)
            except Exception as e:
                logger.debug("Laser '%s' query failed: %s", laser_name, e)
                all_ranges.append([math.inf] * self._laser_n_points)

        if all_ranges:
            self._get_session().publish_lasers(
                x=x_m, y=y_m, yaw=yaw_rad, ranges=all_ranges, frame_id=self._map_id
            )

    # -- Map ---------------------------------------------------------------

    @override
    async def fetch_map(self, frame_id: str) -> MapConfigTemp | None:
        """Load the map image from a local file and return it to InOrbit."""
        if not self._map_file:
            logger.warning("No map_file configured, cannot fetch map")
            return None

        path = Path(self._map_file)
        if not path.is_file():
            logger.error("Map file not found: %s", path)
            return None

        image_bytes = path.read_bytes()
        logger.info("Loaded map image from %s for frame_id=%s", path, frame_id)
        return MapConfigTemp(
            image=image_bytes,
            map_id=frame_id,
            map_label=f"Omron {frame_id}",
            origin_x=self._map_origin_x,
            origin_y=self._map_origin_y,
            resolution=self._map_resolution,
        )

    # -- Command handler ---------------------------------------------------

    @override
    async def _inorbit_command_handler(self, command_name, args, options):
        result_fn = options["result_function"]

        if command_name == COMMAND_NAV_GOAL:
            await self._handle_nav_goal(args[0], result_fn)

        elif command_name == COMMAND_CUSTOM_COMMAND:
            # parse_custom_command_args raises CommandFailure on malformed
            # input, which the SDK converts to a FAILURE result automatically.
            script_name, script_args = parse_custom_command_args(args)

            # Try edge-executor mission commands first
            handled = await self._mission_executor.handle_command(script_name, script_args, options)
            if handled:
                # Suppress native goal tracking while edge executor is active
                self._goal_tracker_enabled = False
                return

            await self._handle_custom_command(script_name, script_args, result_fn)

        elif command_name == COMMAND_MESSAGE:
            await self._handle_message(args[0], result_fn)

        else:
            logger.warning("Unhandled command type: %s", command_name)
            result_fn(CommandResultCode.FAILURE)

    async def _handle_custom_command(self, script_name, script_args: dict, result_fn):
        try:
            if script_name == "goto_goal":
                goal_name = script_args["--goal_name"]
                self._goal_tracker.on_goal_dispatched(goal_name)
                self._last_nav_goal = goal_name
                self._last_nav_point = None
                await self._arcl.goto(goal_name)
                logger.info("Sent goto %s", goal_name)
                result_fn(CommandResultCode.SUCCESS)

            elif script_name == "dock":
                await self._arcl.dock()
                logger.info("Sent dock")
                await self._wait_for_dock_completion("dock", result_fn)

            elif script_name == "undock":
                await self._arcl.undock()
                logger.info("Sent undock")
                await self._wait_for_dock_completion("undock", result_fn)

            elif script_name in ("execute_macro", "executeMacro"):
                macro_name = script_args.get("--macro_name") or script_args.get("macro_name")
                if not macro_name:
                    logger.error("execute_macro missing --macro_name / macro_name argument")
                    result_fn(CommandResultCode.FAILURE)
                    return
                await self._arcl.execute_macro(macro_name)
                logger.info("Sent executeMacro %s", macro_name)
                await self._wait_for_macro_completion(macro_name, result_fn)

            elif script_name == "plc_legs":
                await self._handle_plc_legs(script_args, result_fn)

            elif script_name == "stop":
                abort_payload = self._goal_tracker.on_stop()
                if abort_payload is not None:
                    self._publish_mission_tracking(abort_payload)
                self._last_nav_goal = None
                self._last_nav_point = None
                await self._arcl.stop()
                logger.info("Sent stop")
                result_fn(CommandResultCode.SUCCESS)

            elif script_name in ("pause", "pauseRobot"):
                await self._arcl.set_block_driving(
                    BLOCK_NAME, "Paused by InOrbit", "Robot paused via InOrbit cloud command"
                )
                logger.info("Sent pause (set_block_driving)")
                result_fn(CommandResultCode.SUCCESS)

            elif script_name in ("resume", "resumeRobot"):
                await self._arcl.clear_block_driving(BLOCK_NAME)
                await self._resume_last_goal()
                result_fn(CommandResultCode.SUCCESS)

            else:
                logger.warning("Unknown custom command: %s", script_name)
                result_fn(CommandResultCode.FAILURE)

        except KeyError as e:
            logger.error("Custom command '%s' missing argument: %s", script_name, e)
            result_fn(CommandResultCode.FAILURE)
        except Exception as e:
            logger.error("Custom command '%s' failed: %s", script_name, e)
            result_fn(CommandResultCode.FAILURE)

    def _resolve_plc_move(self, script_args: dict) -> tuple[TablePlc, int, str]:
        """Resolve (client, target height, description) for a plc_legs command.

        Raises ValueError with an operator-readable message on bad input.
        """
        table_id = script_args.get("--table") or script_args.get("table")
        action = script_args.get("--action") or script_args.get("action")
        height_arg = script_args.get("--height_mm") or script_args.get("height_mm")

        if not table_id:
            raise ValueError("plc_legs missing --table argument")
        plc = self._plc_tables.get(table_id)
        if plc is None:
            raise ValueError(
                f"unknown table '{table_id}' — configured tables: "
                f"{sorted(self._plc_tables) or 'none'}"
            )
        if height_arg is not None:
            return plc, int(height_arg), f"height {height_arg} mm"
        height_key = _PLC_ACTION_HEIGHT_KEY.get(action or "")
        if height_key is None:
            raise ValueError(
                f"plc_legs needs --action retract|extend or --height_mm (got action={action!r})"
            )
        heights = self._plc_heights.get(table_id, {})
        if height_key not in heights:
            raise ValueError(
                f"no '{height_key}' height configured for table '{table_id}' "
                f"(configured: {sorted(heights) or 'none'})"
            )
        return plc, heights[height_key], action

    async def _handle_plc_legs(self, script_args: dict, result_fn):
        """Start a workbench leg move; report SUCCESS once the PLC is moving.

        This is the button path. The InOrbit UI stops waiting for a result
        after ~30 s while a full stroke takes ~55 s, so the result means
        "started" (the MiR connector's pattern). The move finishes in the
        background; its outcome goes to the log and to plc_<table>_last_move.
        Missions never come through here: the edge executor runs plc_legs and
        omron-plc-legs steps as PlcLegsNode, which waits until the legs arrived.
        """
        try:
            plc, target_mm, description = self._resolve_plc_move(script_args)
        except ValueError as e:
            logger.error("plc_legs: %s", e)
            result_fn(CommandResultCode.FAILURE, execution_status_details=str(e))
            return
        table_id = script_args.get("--table") or script_args.get("table")
        if plc.is_moving:
            msg = f"workbench '{table_id}' is already moving; wait for it to finish"
            logger.warning("plc_legs: %s", msg)
            result_fn(CommandResultCode.FAILURE, execution_status_details=msg)
            return

        started = asyncio.Event()
        move = asyncio.create_task(
            plc.move_to_height(target_mm, timeout_secs=self._plc_move_timeout_secs, started=started)
        )
        self._plc_move_tasks.add(move)
        move.add_done_callback(self._plc_move_tasks.discard)
        move.add_done_callback(
            lambda task: self._on_plc_move_done(table_id, description, target_mm, task)
        )
        started_wait = asyncio.create_task(started.wait())
        try:
            await asyncio.wait(
                {move, started_wait},
                timeout=_PLC_START_TIMEOUT,
                return_when=asyncio.FIRST_COMPLETED,
            )
        finally:
            started_wait.cancel()

        if move.done():
            error = None if move.cancelled() else move.exception()
            if move.cancelled() or error is not None:
                details = str(error) if error is not None else "move cancelled"
                result_fn(CommandResultCode.FAILURE, execution_status_details=details)
            else:
                result_fn(
                    CommandResultCode.SUCCESS,
                    execution_status_details=f"{description}: at {target_mm} mm",
                )
            return
        if started.is_set():
            logger.info(
                "plc_legs %s started (%d mm), finishing in background", description, target_mm
            )
            result_fn(
                CommandResultCode.SUCCESS,
                execution_status_details=f"{description} started: moving to {target_mm} mm",
            )
            return
        move.cancel()  # releases g_xExecuteMove
        with contextlib.suppress(asyncio.CancelledError, Exception):
            await move
        msg = f"PLC did not start moving to {target_mm} mm within {_PLC_START_TIMEOUT:.0f}s"
        logger.error("plc_legs %s: %s", description, msg)
        result_fn(CommandResultCode.FAILURE, execution_status_details=msg)

    def _on_plc_move_done(self, table_id, description, target_mm, task: asyncio.Task) -> None:
        """Log and publish how a button-started move ended."""
        if task.cancelled():
            outcome = f"{description} cancelled"
            logger.warning("plc_legs %s", outcome)
        elif task.exception() is not None:
            outcome = f"{description} FAILED: {task.exception()}"
            logger.error("plc_legs %s", outcome)
        else:
            outcome = f"{description}: at {target_mm} mm"
            logger.info("plc_legs %s", outcome)
        try:
            self.publish_key_values(**{f"plc_{table_id}_last_move": outcome})
        except Exception as e:
            logger.warning("Could not publish plc_%s_last_move: %s", table_id, e)

    async def _wait_for_dock_completion(self, action: str, result_fn):
        """Poll ARCL status until dock/undock completes, then call result_fn."""
        elapsed = 0.0
        while elapsed < _DOCK_TIMEOUT:
            try:
                status = await self._arcl.query_status()
                omron_status = status.get("Status", "") if status else ""

                if any(omron_status.startswith(p) for p in _DOCK_SUCCESS_PREFIXES):
                    logger.info("%s completed: %s", action, omron_status)
                    result_fn(CommandResultCode.SUCCESS)
                    return

                if any(omron_status.startswith(p) for p in _DOCK_FAILURE_PREFIXES):
                    logger.error("%s failed: %s", action, omron_status)
                    result_fn(CommandResultCode.FAILURE)
                    return

                logger.debug("%s in progress: %s (%.0fs)", action, omron_status, elapsed)

            except Exception as e:
                logger.warning("%s status poll error: %s", action, e)

            await asyncio.sleep(_DOCK_POLL_INTERVAL)
            elapsed += _DOCK_POLL_INTERVAL

        logger.error("%s timed out after %.0fs", action, _DOCK_TIMEOUT)
        result_fn(CommandResultCode.FAILURE)

    async def _wait_for_macro_completion(self, macro_name: str, result_fn):
        """Poll ARCL Status field for macro lifecycle completion.

        Phases:
          1. Kickoff guard — wait up to ``_MACRO_KICKOFF_GRACE`` seconds for
             Status to enter ``Executing macro <name>``. If Status reaches a
             terminal value (Completed/Error) directly, accept it (instant
             macros like SFA toggles can complete before our first poll).
             If neither happens, treat as kickoff failure (likely EStop).
          2. Completion poll — wait until Status leaves the active state and
             categorise the destination.
        """
        expected_active = f"{_MACRO_ACTIVE_PREFIX}{macro_name}"
        elapsed = 0.0
        saw_active = False

        # Phase 1: kickoff guard
        while elapsed < _MACRO_KICKOFF_GRACE:
            try:
                status = await self._arcl.query_status()
                cur = status.get("Status", "") if status else ""
            except Exception as e:
                logger.warning("macro %s status poll error: %s", macro_name, e)
                cur = ""

            if cur.startswith(expected_active):
                saw_active = True
                break

            # Instant-macro shortcut: terminal state reached before we saw active
            if cur.startswith(_MACRO_SUCCESS_PREFIX) and cur.endswith(macro_name):
                logger.info("Macro %s completed (instant): %s", macro_name, cur)
                result_fn(CommandResultCode.SUCCESS)
                return
            if any(cur.startswith(p) for p in _MACRO_FAILURE_PREFIXES):
                logger.error("Macro %s failed before active observed: %s", macro_name, cur)
                result_fn(CommandResultCode.FAILURE)
                return

            await asyncio.sleep(_MACRO_POLL_INTERVAL)
            elapsed += _MACRO_POLL_INTERVAL

        if not saw_active:
            logger.warning("Macro %s never reached active status (EStop or rejected?)", macro_name)
            result_fn(CommandResultCode.FAILURE)
            return

        # Phase 2: wait for active → terminal transition
        while elapsed < _MACRO_TIMEOUT:
            try:
                status = await self._arcl.query_status()
                cur = status.get("Status", "") if status else ""
            except Exception as e:
                logger.warning("macro %s status poll error: %s", macro_name, e)
                cur = expected_active  # assume still running on transient errors

            if cur.startswith(_MACRO_SUCCESS_PREFIX):
                logger.info("Macro %s completed: %s", macro_name, cur)
                result_fn(CommandResultCode.SUCCESS)
                return
            if any(cur.startswith(p) for p in _MACRO_FAILURE_PREFIXES):
                logger.error("Macro %s failed: %s", macro_name, cur)
                result_fn(CommandResultCode.FAILURE)
                return
            # Anything else (still active, brief excursion, unknown state) →
            # keep polling. Unknown-destination must not declare SUCCESS — that
            # would be premature SUCCESS, the dangerous failure mode.
            logger.debug("Macro %s status: %s (%.0fs)", macro_name, cur, elapsed)

            await asyncio.sleep(_MACRO_POLL_INTERVAL)
            elapsed += _MACRO_POLL_INTERVAL

        logger.error("Macro %s timed out after %.0fs", macro_name, _MACRO_TIMEOUT)
        result_fn(CommandResultCode.FAILURE)

    async def _handle_message(self, msg, result_fn):
        """Handle COMMAND_MESSAGE — cloud-mode pause/resume."""
        try:
            if msg == "inorbit_pause":
                await self._arcl.set_block_driving(
                    BLOCK_NAME, "Paused by InOrbit", "Robot paused via InOrbit cloud command"
                )
                logger.info("inorbit_pause: set_block_driving")
                result_fn(CommandResultCode.SUCCESS)
            elif msg == "inorbit_resume":
                await self._arcl.clear_block_driving(BLOCK_NAME)
                await self._resume_last_goal()
                result_fn(CommandResultCode.SUCCESS)
            else:
                logger.debug("Unhandled COMMAND_MESSAGE: %s", msg)
        except Exception as e:
            logger.error("COMMAND_MESSAGE '%s' failed: %s", msg, e)
            result_fn(CommandResultCode.FAILURE)

    async def _resume_last_goal(self):
        """Re-send the last navigation command after clearing a block.

        ARCL block driving (abds) abandons the active goal, so a plain ``go``
        after ``abdc`` has nowhere to go.  We re-issue the original goto or
        gotopoint so the robot continues to its destination.
        """
        if self._last_nav_goal:
            self._goal_tracker.on_goal_dispatched(self._last_nav_goal)
            await self._arcl.goto(self._last_nav_goal)
            logger.info("Resumed: re-sent goto %s", self._last_nav_goal)
        elif self._last_nav_point:
            x, y, t = self._last_nav_point
            self._goal_tracker.on_goal_dispatched(f"({x / 1000:.1f}, {y / 1000:.1f})")
            await self._arcl.gotopoint(x, y, t)
            logger.info("Resumed: re-sent gotopoint %d %d %d", x, y, t)
        else:
            await self._arcl.go()
            logger.info("Resumed: no saved goal, sent go")

    async def _handle_nav_goal(self, pose, result_fn):
        """Handle NAV_GOAL by sending gotopoint with the coordinates."""
        try:
            x = float(pose["x"])
            y = float(pose["y"])
            theta = float(pose.get("theta", 0))

            # Convert from meters/radians back to mm/degrees for ARCL
            x_mm = int(x * 1000)
            y_mm = int(y * 1000)
            theta_deg = int(math.degrees(theta))

            self._goal_tracker.on_goal_dispatched(f"({x:.1f}, {y:.1f})")
            self._last_nav_goal = None
            self._last_nav_point = (x_mm, y_mm, theta_deg)
            await self._arcl.gotopoint(x_mm, y_mm, theta_deg)
            logger.info("Sent gotopoint %d %d %d", x_mm, y_mm, theta_deg)
            result_fn(CommandResultCode.SUCCESS)

        except Exception as e:
            logger.error("NAV_GOAL failed: %s", e)
            result_fn(CommandResultCode.FAILURE)
