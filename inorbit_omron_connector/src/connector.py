# SPDX-FileCopyrightText: 2026 Mappalink
#
# SPDX-License-Identifier: MIT

"""Omron ARCL connector — bridges Omron HD1500 ARCL to InOrbit Cloud."""

import logging
import math
import re
from pathlib import Path

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

logger = logging.getLogger(__name__)

# Block driving name used for pause/resume
BLOCK_NAME = "InOrbit"

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
        self._mission_executor = OmronMissionExecutor(
            robot_id=robot_id,
            inorbit_api=MissionInOrbitAPI(
                base_url=self._get_session().inorbit_rest_api_endpoint,
                api_key=self.config.api_key,
            ),
            arcl_client=self._arcl,
            database_file=cfg.mission_database_file,
        )

    # -- Lifecycle ---------------------------------------------------------

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

    async def _disconnect(self) -> None:
        await self._mission_executor.shutdown()
        await self._arcl.disconnect()
        logger.info("Disconnected from Omron ARCL")

    # -- Main loop (~1 Hz) ------------------------------------------------

    async def _execution_loop(self) -> None:
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

    async def _inorbit_command_handler(self, command_name, args, options):
        result_fn = options["result_function"]

        if command_name == COMMAND_NAV_GOAL:
            await self._handle_nav_goal(args[0], result_fn)

        elif command_name == COMMAND_CUSTOM_COMMAND:
            script_name = args[0]
            args_list = list(args[1]) if len(args) > 1 else []
            script_args = dict(zip(args_list[::2], args_list[1::2]))

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
                await self._arcl.goto(goal_name)
                logger.info("Sent goto %s", goal_name)
                result_fn(CommandResultCode.SUCCESS)

            elif script_name == "dock":
                await self._arcl.dock()
                logger.info("Sent dock")
                result_fn(CommandResultCode.SUCCESS)

            elif script_name == "undock":
                await self._arcl.undock()
                logger.info("Sent undock")
                result_fn(CommandResultCode.SUCCESS)

            elif script_name == "stop":
                abort_payload = self._goal_tracker.on_stop()
                if abort_payload is not None:
                    self._publish_mission_tracking(abort_payload)
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
                await self._arcl.go()
                logger.info("Sent resume (clear_block_driving + go)")
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
                await self._arcl.go()
                logger.info("inorbit_resume: clear_block_driving + go")
                result_fn(CommandResultCode.SUCCESS)
            else:
                logger.debug("Unhandled COMMAND_MESSAGE: %s", msg)
        except Exception as e:
            logger.error("COMMAND_MESSAGE '%s' failed: %s", msg, e)
            result_fn(CommandResultCode.FAILURE)

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
            await self._arcl.gotopoint(x_mm, y_mm, theta_deg)
            logger.info("Sent gotopoint %d %d %d", x_mm, y_mm, theta_deg)
            result_fn(CommandResultCode.SUCCESS)

        except Exception as e:
            logger.error("NAV_GOAL failed: %s", e)
            result_fn(CommandResultCode.FAILURE)
