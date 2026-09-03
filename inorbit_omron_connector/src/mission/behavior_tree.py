# SPDX-FileCopyrightText: 2026 Mappalink
#
# SPDX-License-Identifier: MIT

"""Custom behavior tree nodes for executing Omron ARCL missions locally.

Instead of round-tripping each step through the InOrbit cloud API, these nodes
send ARCL commands directly over TCP and poll query_status() for completion.

Step mapping:
    poseWaypoint  -> gotopoint x_mm y_mm theta_deg -> poll until idle
    runAction     -> map actionId to ARCL command -> poll until idle
"""

from __future__ import annotations

import asyncio
import logging
import math
from enum import StrEnum
from typing import Optional

from inorbit_edge_executor.behavior_tree import (
    BehaviorTree,
    BehaviorTreeBuilderContext,
    BehaviorTreeSequential,
    MissionAbortedNode,
    NodeFromStepBuilder,
    register_accepted_node_types,
)
from inorbit_edge_executor.datatypes import (
    MissionStepPoseWaypoint,
    MissionStepRunAction,
)
from inorbit_edge_executor.inorbit import MissionStatus

from inorbit_omron_connector.src.arcl_client import ArclClient
from inorbit_omron_connector.src.plc_client import PlcError, TablePlc

logger = logging.getLogger(__name__)

# plc_legs action → named height in TablePlcConfig.heights (mirrors connector.py)
_PLC_ACTION_HEIGHT_KEY = {"retract": "retracted", "extend": "pickup"}

# Polling interval for ARCL status checks
_POLL_INTERVAL_SECS = 1.0

# ARCL Status values that indicate the robot is actively navigating
_NAVIGATING_PREFIXES = frozenset(
    {
        "Going to goal",
        "Driving to goal",
        "Going to point",
        "Docking",
        "Undocking",
    }
)

# ARCL Status values that indicate success / idle
_SUCCESS_PREFIXES = frozenset(
    {
        "Arrived at",
        "Parked",
        "Idle",
    }
)

# Dock and undock finish in a state navigation never treats as success.
# After `undock` ARCL settles on "Stopped": the robot has left the charger and
# is standing still. That is a completion here, but for a goto it would mean an
# interrupted drive, so "Stopped" stays out of _SUCCESS_PREFIXES and is only
# accepted on dock/undock steps.
# Observed 2026-09-03: undock moved the robot 0.76 m off the dock, Status became
# "Stopped", and the step timed out after 60 s and reported failure.
_DOCK_SUCCESS_PREFIXES = frozenset(_SUCCESS_PREFIXES | {"Stopped"})

# ARCL Status values that indicate failure
_FAILURE_PREFIXES = frozenset(
    {
        "Failed to get to",
        "Failed going to",
    }
)

# "Stopped" / "Stopping" are NOT treated as failures because ARCL reports
# these during block-driving pauses (mission pause / robot pause).  When a
# real abort is requested the executor cancels the BT directly, so we never
# need to detect "Stopped" from the poll loop.

# ARCL Status field transitions for executeMacro completion polling.
# See fm-fsm-docs/docs/omron/OMRON_DOCKING.md for live capture evidence.
# Identity is carried by the transition out of `Executing macro <name>`, not
# by the destination prefix — the failure line ("Error: ...") doesn't carry
# the macro name, which is fine because the kickoff guard already proved our
# macro owned the Status field.
_MACRO_ACTIVE_PREFIX = "Executing macro "
_MACRO_SUCCESS_PREFIX = "Completed macro "
_MACRO_FAILURE_PREFIXES = frozenset({"Error:"})  # tight — bare "Failed" is too generic
_MACRO_KICKOFF_GRACE_SECS = 5.0


class SharedMemoryKeys(StrEnum):
    ARCL_ERROR_MESSAGE = "arcl_error_message"
    # Stores the last navigation command as a dict so WaitForArclCompletionNode
    # can re-send it after a mission pause/resume cycle.  The BT serialises
    # only the *currently running* node; after resume the goto node is already
    # marked finished so only the wait node re-executes.  Without re-sending
    # the command the robot sits idle and the poll loop never sees arrival.
    ARCL_PENDING_NAV = "arcl_pending_nav"


class ArclBehaviorTreeBuilderContext(BehaviorTreeBuilderContext):
    """Extended context carrying an ArclClient and the workbench PLCs."""

    def __init__(
        self,
        arcl_client: ArclClient,
        plc_tables: dict[str, TablePlc] | None = None,
        plc_heights: dict[str, dict[str, int]] | None = None,
        plc_move_timeout_secs: float = 60.0,
        **kwargs,
    ):
        super().__init__(**kwargs)
        self._arcl_client = arcl_client
        self._plc_tables = plc_tables or {}
        self._plc_heights = plc_heights or {}
        self._plc_move_timeout_secs = plc_move_timeout_secs

    @property
    def arcl_client(self) -> ArclClient:
        return self._arcl_client

    @property
    def plc_tables(self) -> dict[str, TablePlc]:
        return self._plc_tables

    @property
    def plc_heights(self) -> dict[str, dict[str, int]]:
        return self._plc_heights

    @property
    def plc_move_timeout_secs(self) -> float:
        return self._plc_move_timeout_secs


# ---------------------------------------------------------------------------
# Polling node — waits for ARCL robot to finish its current task
# ---------------------------------------------------------------------------


class WaitForArclCompletionNode(BehaviorTree):
    """Polls ARCL query_status() until the robot leaves a navigating state.

    Succeeds when status matches a success prefix.
    Fails on failure prefix or timeout.
    """

    def __init__(
        self,
        context: ArclBehaviorTreeBuilderContext,
        timeout_secs: Optional[float] = None,
        success_prefixes: Optional[frozenset] = None,
        **kwargs,
    ):
        super().__init__(**kwargs)
        self._arcl = context.arcl_client
        self._shared_memory = context.shared_memory
        self._timeout_secs = timeout_secs
        self._success_prefixes = frozenset(success_prefixes or _SUCCESS_PREFIXES)

        self._shared_memory.add(SharedMemoryKeys.ARCL_ERROR_MESSAGE, None)
        self._shared_memory.add(SharedMemoryKeys.ARCL_PENDING_NAV, None)

    async def _resend_nav_if_idle(self) -> bool:
        """Re-send the last navigation command if the robot is not navigating.

        After a mission pause/resume the BT only re-executes this wait node,
        not the preceding goto node.  ARCL block driving (used for pause)
        abandons the active goal, so the robot is idle when we resume.

        Returns True if a command was re-sent.
        """
        pending = self._shared_memory.get(SharedMemoryKeys.ARCL_PENDING_NAV)
        if not pending:
            return False
        status = self._arcl.cached_status
        omron_status = status.get("Status", "") if status else ""
        if any(omron_status.startswith(p) for p in _NAVIGATING_PREFIXES):
            return False  # already navigating, nothing to do
        cmd_type = pending.get("type")
        if cmd_type == "goto":
            goal = pending["goal_name"]
            logger.info("Re-sending goto %s after resume", goal)
            await self._arcl.goto(goal)
        elif cmd_type == "gotopoint":
            x, y, t = pending["x_mm"], pending["y_mm"], pending["theta_deg"]
            logger.info("Re-sending gotopoint %d %d %d after resume", x, y, t)
            await self._arcl.gotopoint(x, y, t)
        else:
            return False
        return True

    async def _execute(self):
        logger.info("Waiting for ARCL task completion")
        await self._resend_nav_if_idle()

        # Wait for the ARCL command (sent by the preceding node) to take
        # effect and for the telemetry cache to reflect the new state.
        # Without this, cached_status still shows "Idle"/"Parked" from
        # before the command, causing an immediate false success.
        await asyncio.sleep(_POLL_INTERVAL_SECS * 3)
        elapsed = _POLL_INTERVAL_SECS * 3

        while True:
            if self._timeout_secs and elapsed >= self._timeout_secs:
                error_msg = f"ARCL task timed out after {self._timeout_secs}s"
                logger.error(error_msg)
                self._shared_memory.set(SharedMemoryKeys.ARCL_ERROR_MESSAGE, error_msg)
                raise RuntimeError(error_msg)

            # Read cached status from the telemetry loop (~1 Hz) instead of
            # sending a competing "status" command that would interleave with
            # the telemetry query on the single ARCL TCP socket.
            status = self._arcl.cached_status
            omron_status = status.get("Status", "") if status else ""
            logger.debug("ARCL cached status: %s", omron_status)

            if any(omron_status.startswith(p) for p in self._success_prefixes):
                logger.info("ARCL task completed: %s", omron_status)
                return

            if any(omron_status.startswith(p) for p in _FAILURE_PREFIXES):
                error_msg = f"ARCL task failed: {omron_status}"
                logger.error(error_msg)
                self._shared_memory.set(SharedMemoryKeys.ARCL_ERROR_MESSAGE, error_msg)
                raise RuntimeError(error_msg)

            await asyncio.sleep(_POLL_INTERVAL_SECS)
            elapsed += _POLL_INTERVAL_SECS

    def dump_object(self):
        obj = super().dump_object()
        obj["timeout_secs"] = self._timeout_secs
        obj["success_prefixes"] = sorted(self._success_prefixes)
        return obj

    @classmethod
    def from_object(cls, context, timeout_secs=None, success_prefixes=None, **kwargs):
        return WaitForArclCompletionNode(
            context,
            timeout_secs=timeout_secs,
            success_prefixes=frozenset(success_prefixes) if success_prefixes else None,
            **kwargs,
        )


# ---------------------------------------------------------------------------
# GotoPoint — navigate to coordinates
# ---------------------------------------------------------------------------


class ArclGotoPointNode(BehaviorTree):
    """Sends gotopoint x_mm y_mm theta_deg to ARCL."""

    def __init__(
        self,
        context: ArclBehaviorTreeBuilderContext,
        x_m: float,
        y_m: float,
        theta_rad: float,
        **kwargs,
    ):
        super().__init__(**kwargs)
        self._arcl = context.arcl_client
        self._shared_memory = context.shared_memory
        self._x_m = x_m
        self._y_m = y_m
        self._theta_rad = theta_rad

        self._shared_memory.add(SharedMemoryKeys.ARCL_ERROR_MESSAGE, None)
        self._shared_memory.add(SharedMemoryKeys.ARCL_PENDING_NAV, None)

    async def _execute(self):
        x_mm = int(self._x_m * 1000)
        y_mm = int(self._y_m * 1000)
        theta_deg = int(math.degrees(self._theta_rad))
        logger.info("Sending gotopoint %d %d %d", x_mm, y_mm, theta_deg)
        self._shared_memory.set(
            SharedMemoryKeys.ARCL_PENDING_NAV,
            {"type": "gotopoint", "x_mm": x_mm, "y_mm": y_mm, "theta_deg": theta_deg},
        )
        try:
            await self._arcl.gotopoint(x_mm, y_mm, theta_deg)
        except Exception as e:
            error_msg = f"gotopoint failed: {e}"
            logger.error(error_msg)
            self._shared_memory.set(SharedMemoryKeys.ARCL_ERROR_MESSAGE, error_msg)
            raise RuntimeError(error_msg) from e

    def dump_object(self):
        obj = super().dump_object()
        obj["x_m"] = self._x_m
        obj["y_m"] = self._y_m
        obj["theta_rad"] = self._theta_rad
        return obj

    @classmethod
    def from_object(cls, context, x_m, y_m, theta_rad, **kwargs):
        return ArclGotoPointNode(context, x_m=x_m, y_m=y_m, theta_rad=theta_rad, **kwargs)


# ---------------------------------------------------------------------------
# GotoGoal — navigate to a named goal
# ---------------------------------------------------------------------------


class ArclGotoGoalNode(BehaviorTree):
    """Sends goto <goal_name> to ARCL."""

    def __init__(
        self,
        context: ArclBehaviorTreeBuilderContext,
        goal_name: str,
        **kwargs,
    ):
        super().__init__(**kwargs)
        self._arcl = context.arcl_client
        self._shared_memory = context.shared_memory
        self._goal_name = goal_name

        self._shared_memory.add(SharedMemoryKeys.ARCL_ERROR_MESSAGE, None)
        self._shared_memory.add(SharedMemoryKeys.ARCL_PENDING_NAV, None)

    async def _execute(self):
        logger.info("Sending goto %s", self._goal_name)
        self._shared_memory.set(
            SharedMemoryKeys.ARCL_PENDING_NAV,
            {"type": "goto", "goal_name": self._goal_name},
        )
        try:
            await self._arcl.goto(self._goal_name)
        except Exception as e:
            error_msg = f"goto {self._goal_name} failed: {e}"
            logger.error(error_msg)
            self._shared_memory.set(SharedMemoryKeys.ARCL_ERROR_MESSAGE, error_msg)
            raise RuntimeError(error_msg) from e

    def dump_object(self):
        obj = super().dump_object()
        obj["goal_name"] = self._goal_name
        return obj

    @classmethod
    def from_object(cls, context, goal_name, **kwargs):
        return ArclGotoGoalNode(context, goal_name=goal_name, **kwargs)


# ---------------------------------------------------------------------------
# Dock / Undock nodes
# ---------------------------------------------------------------------------


class ArclDockNode(BehaviorTree):
    """Sends dock command to ARCL."""

    def __init__(self, context: ArclBehaviorTreeBuilderContext, **kwargs):
        super().__init__(**kwargs)
        self._arcl = context.arcl_client
        self._shared_memory = context.shared_memory
        self._shared_memory.add(SharedMemoryKeys.ARCL_ERROR_MESSAGE, None)

    async def _execute(self):
        logger.info("Sending dock")
        try:
            await self._arcl.dock()
        except Exception as e:
            error_msg = f"dock failed: {e}"
            logger.error(error_msg)
            self._shared_memory.set(SharedMemoryKeys.ARCL_ERROR_MESSAGE, error_msg)
            raise RuntimeError(error_msg) from e

    @classmethod
    def from_object(cls, context, **kwargs):
        return ArclDockNode(context, **kwargs)


class ArclUndockNode(BehaviorTree):
    """Sends undock command to ARCL."""

    def __init__(self, context: ArclBehaviorTreeBuilderContext, **kwargs):
        super().__init__(**kwargs)
        self._arcl = context.arcl_client
        self._shared_memory = context.shared_memory
        self._shared_memory.add(SharedMemoryKeys.ARCL_ERROR_MESSAGE, None)

    async def _execute(self):
        logger.info("Sending undock")
        try:
            await self._arcl.undock()
        except Exception as e:
            error_msg = f"undock failed: {e}"
            logger.error(error_msg)
            self._shared_memory.set(SharedMemoryKeys.ARCL_ERROR_MESSAGE, error_msg)
            raise RuntimeError(error_msg) from e

    @classmethod
    def from_object(cls, context, **kwargs):
        return ArclUndockNode(context, **kwargs)


class ArclExecuteMacroNode(BehaviorTree):
    """Sends executeMacro <name> to ARCL.

    Init failure (CommandError on unknown macro name) surfaces as a
    RuntimeError raised by the dispatcher. Runtime completion is observed by
    a following ``WaitForMacroCompletionNode``.
    """

    def __init__(
        self,
        context: ArclBehaviorTreeBuilderContext,
        macro_name: str,
        **kwargs,
    ):
        super().__init__(**kwargs)
        self._arcl = context.arcl_client
        self._macro_name = macro_name
        self._shared_memory = context.shared_memory
        self._shared_memory.add(SharedMemoryKeys.ARCL_ERROR_MESSAGE, None)

    async def _execute(self):
        logger.info("Sending executeMacro %s", self._macro_name)
        try:
            await self._arcl.execute_macro(self._macro_name)
        except Exception as e:
            error_msg = f"executeMacro {self._macro_name} failed: {e}"
            logger.error(error_msg)
            self._shared_memory.set(SharedMemoryKeys.ARCL_ERROR_MESSAGE, error_msg)
            raise RuntimeError(error_msg) from e

    def dump_object(self):
        return {"macro_name": self._macro_name, **super().dump_object()}

    @classmethod
    def from_object(cls, context, macro_name, **kwargs):
        return ArclExecuteMacroNode(context, macro_name=macro_name, **kwargs)


class WaitForMacroCompletionNode(BehaviorTree):
    """Polls ARCL Status until a previously-launched macro reaches a terminal state.

    Phases:
      1. Kickoff guard — wait up to ``_MACRO_KICKOFF_GRACE_SECS`` for Status to
         enter ``Executing macro <name>``. Instant macros (SFA toggles) may
         reach a terminal value before our first poll; that is accepted as a
         shortcut. If neither happens, the macro was likely rejected (EStop).
      2. Completion poll — wait until Status reaches ``Completed macro <name>``
         (success) or ``Error: ...`` (failure). Unknown destinations are not
         treated as success — they keep polling, falling through to timeout.
    """

    def __init__(
        self,
        context: ArclBehaviorTreeBuilderContext,
        macro_name: str,
        timeout_secs: Optional[float] = None,
        **kwargs,
    ):
        super().__init__(**kwargs)
        self._arcl = context.arcl_client
        self._macro_name = macro_name
        self._timeout_secs = timeout_secs
        self._shared_memory = context.shared_memory
        self._shared_memory.add(SharedMemoryKeys.ARCL_ERROR_MESSAGE, None)

    async def _execute(self):
        expected_active = f"{_MACRO_ACTIVE_PREFIX}{self._macro_name}"
        elapsed = 0.0

        # Phase 1: kickoff guard
        saw_active = False
        while elapsed < _MACRO_KICKOFF_GRACE_SECS:
            cur = self._current_status()
            if cur.startswith(expected_active):
                saw_active = True
                break
            if cur.startswith(_MACRO_SUCCESS_PREFIX) and cur.endswith(self._macro_name):
                logger.info("Macro %s completed (instant): %s", self._macro_name, cur)
                return
            if any(cur.startswith(p) for p in _MACRO_FAILURE_PREFIXES):
                error_msg = f"Macro {self._macro_name} failed before active observed: {cur}"
                logger.error(error_msg)
                self._shared_memory.set(SharedMemoryKeys.ARCL_ERROR_MESSAGE, error_msg)
                raise RuntimeError(error_msg)
            await asyncio.sleep(_POLL_INTERVAL_SECS)
            elapsed += _POLL_INTERVAL_SECS

        if not saw_active:
            error_msg = f"Macro {self._macro_name} never reached active status (EStop or rejected?)"
            logger.error(error_msg)
            self._shared_memory.set(SharedMemoryKeys.ARCL_ERROR_MESSAGE, error_msg)
            raise RuntimeError(error_msg)

        # Phase 2: wait for active → terminal transition
        while True:
            if self._timeout_secs and elapsed >= self._timeout_secs:
                error_msg = f"Macro {self._macro_name} timed out after {self._timeout_secs:.0f}s"
                logger.error(error_msg)
                self._shared_memory.set(SharedMemoryKeys.ARCL_ERROR_MESSAGE, error_msg)
                raise RuntimeError(error_msg)

            cur = self._current_status()
            if cur.startswith(_MACRO_SUCCESS_PREFIX):
                logger.info("Macro %s completed: %s", self._macro_name, cur)
                return
            if any(cur.startswith(p) for p in _MACRO_FAILURE_PREFIXES):
                error_msg = f"Macro {self._macro_name} failed: {cur}"
                logger.error(error_msg)
                self._shared_memory.set(SharedMemoryKeys.ARCL_ERROR_MESSAGE, error_msg)
                raise RuntimeError(error_msg)
            # Active or unknown — keep polling. Unknown must NOT report success
            # (premature SUCCESS is the dangerous failure mode).
            logger.debug("Macro %s status: %s (%.0fs)", self._macro_name, cur, elapsed)

            await asyncio.sleep(_POLL_INTERVAL_SECS)
            elapsed += _POLL_INTERVAL_SECS

    def _current_status(self) -> str:
        status = self._arcl.cached_status
        return status.get("Status", "") if status else ""

    def dump_object(self):
        return {
            "macro_name": self._macro_name,
            "timeout_secs": self._timeout_secs,
            **super().dump_object(),
        }

    @classmethod
    def from_object(cls, context, macro_name, timeout_secs=None, **kwargs):
        return WaitForMacroCompletionNode(
            context, macro_name=macro_name, timeout_secs=timeout_secs, **kwargs
        )


# ---------------------------------------------------------------------------
# PLC legs — workbench lifting columns (single node: command + confirm)
# ---------------------------------------------------------------------------


class PlcLegsNode(BehaviorTree):
    """Moves a workbench's lifting columns to a target height and blocks
    until the PLC confirms.

    One node for command + wait: TablePlc.move_to_height() runs the whole
    edge-triggered handshake and releases g_xExecuteMove on any exit, so
    cancellation (mission abort/pause) aborts the PLC move by design. On
    resume the node re-executes and re-issues the move — idempotent thanks
    to the PLC's deadband.
    """

    def __init__(
        self,
        context: ArclBehaviorTreeBuilderContext,
        table_id: str,
        target_mm: int,
        **kwargs,
    ):
        super().__init__(**kwargs)
        self._plc_tables = context.plc_tables
        self._timeout_secs = context.plc_move_timeout_secs
        self._table_id = table_id
        self._target_mm = target_mm
        self._shared_memory = context.shared_memory
        self._shared_memory.add(SharedMemoryKeys.ARCL_ERROR_MESSAGE, None)

    async def _execute(self):
        plc = self._plc_tables.get(self._table_id)
        if plc is None:
            error_msg = f"no PLC configured for table '{self._table_id}'"
            self._shared_memory.set(SharedMemoryKeys.ARCL_ERROR_MESSAGE, error_msg)
            raise RuntimeError(error_msg)
        logger.info("PLC legs: table %s to %d mm", self._table_id, self._target_mm)
        try:
            await plc.move_to_height(self._target_mm, timeout_secs=self._timeout_secs)
        except PlcError as e:
            error_msg = f"PLC legs failed for table '{self._table_id}': {e}"
            logger.error(error_msg)
            self._shared_memory.set(SharedMemoryKeys.ARCL_ERROR_MESSAGE, error_msg)
            raise RuntimeError(error_msg) from e

    def dump_object(self):
        return {
            "table_id": self._table_id,
            "target_mm": self._target_mm,
            **super().dump_object(),
        }

    @classmethod
    def from_object(cls, context, table_id, target_mm, **kwargs):
        return PlcLegsNode(context, table_id=table_id, target_mm=target_mm, **kwargs)


# ---------------------------------------------------------------------------
# Abort node — stops robot before reporting abort
# ---------------------------------------------------------------------------


class ArclMissionAbortedNode(MissionAbortedNode):
    """Extended abort that sends ARCL stop before reporting to InOrbit."""

    def __init__(
        self,
        context: ArclBehaviorTreeBuilderContext,
        status: MissionStatus = MissionStatus.error,
        **kwargs,
    ):
        super().__init__(context, status, **kwargs)
        self._arcl = context.arcl_client
        self._shared_memory = context.shared_memory

    async def _execute(self):
        error_message = self._shared_memory.get(SharedMemoryKeys.ARCL_ERROR_MESSAGE)
        if error_message:
            logger.error("ARCL mission aborted: %s", error_message)

        try:
            await self._arcl.stop()
            logger.info("Sent ARCL stop on mission abort")
        except Exception as e:
            logger.warning("Failed to send ARCL stop on abort: %s", e)

        await super()._execute()

    @classmethod
    def from_object(cls, context, status, **kwargs):
        return ArclMissionAbortedNode(context, MissionStatus(status), **kwargs)


# ---------------------------------------------------------------------------
# Step builder — maps InOrbit mission steps to ARCL BT nodes
# ---------------------------------------------------------------------------


class ArclNodeFromStepBuilder(NodeFromStepBuilder):
    """Builds ARCL-specific behavior tree nodes from mission steps."""

    def __init__(self, context: ArclBehaviorTreeBuilderContext):
        super().__init__(context)
        self._arcl_context = context

    def visit_pose_waypoint(self, step: MissionStepPoseWaypoint) -> BehaviorTree:
        """Convert a pose waypoint to gotopoint + wait."""
        wp = step.waypoint
        label = step.label or f"Go to ({wp.x:.1f}, {wp.y:.1f})"

        sequence = BehaviorTreeSequential(label=label)
        sequence.add_node(
            ArclGotoPointNode(
                self._arcl_context,
                x_m=wp.x,
                y_m=wp.y,
                theta_rad=wp.theta,
                label=f"gotopoint ({wp.x:.1f}, {wp.y:.1f})",
            )
        )
        sequence.add_node(
            WaitForArclCompletionNode(
                self._arcl_context,
                timeout_secs=step.timeout_secs,
                label=f"Wait for arrival at ({wp.x:.1f}, {wp.y:.1f})",
            )
        )
        return sequence

    def visit_run_action(self, step: MissionStepRunAction) -> BehaviorTree:
        """Map known ARCL actions to local commands."""
        action_id = step.action_id
        arguments = step.arguments or {}

        if action_id == "goto_goal":
            goal_name = arguments.get("goal_name") or arguments.get("--goal_name", "")
            if not goal_name:
                raise RuntimeError("goto_goal action missing 'goal_name' argument")
            sequence = BehaviorTreeSequential(label=step.label or f"Go to {goal_name}")
            sequence.add_node(
                ArclGotoGoalNode(
                    self._arcl_context,
                    goal_name=goal_name,
                    label=f"goto {goal_name}",
                )
            )
            sequence.add_node(
                WaitForArclCompletionNode(
                    self._arcl_context,
                    timeout_secs=step.timeout_secs,
                    label=f"Wait for arrival at {goal_name}",
                )
            )
            return sequence

        if action_id == "dock":
            sequence = BehaviorTreeSequential(label=step.label or "Dock")
            sequence.add_node(ArclDockNode(self._arcl_context, label="dock"))
            sequence.add_node(
                WaitForArclCompletionNode(
                    self._arcl_context,
                    timeout_secs=step.timeout_secs,
                    success_prefixes=_DOCK_SUCCESS_PREFIXES,
                    label="Wait for dock completion",
                )
            )
            return sequence

        if action_id == "undock":
            sequence = BehaviorTreeSequential(label=step.label or "Undock")
            sequence.add_node(ArclUndockNode(self._arcl_context, label="undock"))
            sequence.add_node(
                WaitForArclCompletionNode(
                    self._arcl_context,
                    timeout_secs=step.timeout_secs,
                    success_prefixes=_DOCK_SUCCESS_PREFIXES,
                    label="Wait for undock completion",
                )
            )
            return sequence

        if action_id in ("execute_macro", "executeMacro"):
            macro_name = arguments.get("macro_name") or arguments.get("--macro_name", "")
            if not macro_name:
                raise RuntimeError("execute_macro action missing 'macro_name' argument")
            sequence = BehaviorTreeSequential(label=step.label or f"Macro {macro_name}")
            sequence.add_node(
                ArclExecuteMacroNode(
                    self._arcl_context,
                    macro_name=macro_name,
                    label=f"executeMacro {macro_name}",
                )
            )
            sequence.add_node(
                WaitForMacroCompletionNode(
                    self._arcl_context,
                    macro_name=macro_name,
                    timeout_secs=step.timeout_secs,
                    label=f"Wait for macro {macro_name} completion",
                )
            )
            return sequence

        if action_id == "plc_legs":
            return self._build_plc_legs(step, arguments)

        # Unknown action — fall back to default (cloud round-trip)
        logger.warning("Unknown action '%s' — falling back to cloud execution", action_id)
        return super().visit_run_action(step)

    def _build_plc_legs(self, step: MissionStepRunAction, arguments: dict) -> BehaviorTree:
        """Resolve a plc_legs step to a PlcLegsNode at build time so bad
        input fails the mission before anything moves."""
        table_id = arguments.get("table") or arguments.get("--table", "")
        action = arguments.get("action") or arguments.get("--action", "")
        height_arg = arguments.get("height_mm") or arguments.get("--height_mm")
        if not table_id:
            raise RuntimeError("plc_legs action missing 'table' argument")
        if table_id not in self._arcl_context.plc_tables:
            raise RuntimeError(
                f"unknown table '{table_id}' — configured tables: "
                f"{sorted(self._arcl_context.plc_tables) or 'none'}"
            )
        if height_arg is not None:
            target_mm = int(height_arg)
        else:
            height_key = _PLC_ACTION_HEIGHT_KEY.get(action)
            if height_key is None:
                raise RuntimeError(
                    f"plc_legs needs action retract|extend or height_mm (got action={action!r})"
                )
            heights = self._arcl_context.plc_heights.get(table_id, {})
            if height_key not in heights:
                raise RuntimeError(
                    f"no '{height_key}' height configured for table '{table_id}' "
                    f"(configured: {sorted(heights) or 'none'})"
                )
            target_mm = heights[height_key]
        return PlcLegsNode(
            self._arcl_context,
            table_id=table_id,
            target_mm=target_mm,
            label=step.label or f"PLC legs {action or f'{target_mm} mm'} ({table_id})",
        )


# Register node types for serialization/deserialization (crash recovery)
arcl_node_types = [
    ArclGotoPointNode,
    ArclGotoGoalNode,
    ArclDockNode,
    ArclUndockNode,
    ArclExecuteMacroNode,
    WaitForArclCompletionNode,
    WaitForMacroCompletionNode,
    PlcLegsNode,
    ArclMissionAbortedNode,
]
register_accepted_node_types(arcl_node_types)
