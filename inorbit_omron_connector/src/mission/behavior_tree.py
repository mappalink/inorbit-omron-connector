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

logger = logging.getLogger(__name__)

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

# ARCL Status values that indicate failure
_FAILURE_PREFIXES = frozenset(
    {
        "Failed to get to",
        "Failed going to",
        "Stopped",
        "Stopping",
    }
)


class SharedMemoryKeys(StrEnum):
    ARCL_ERROR_MESSAGE = "arcl_error_message"


class ArclBehaviorTreeBuilderContext(BehaviorTreeBuilderContext):
    """Extended context carrying an ArclClient reference."""

    def __init__(self, arcl_client: ArclClient, **kwargs):
        super().__init__(**kwargs)
        self._arcl_client = arcl_client

    @property
    def arcl_client(self) -> ArclClient:
        return self._arcl_client


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
        **kwargs,
    ):
        super().__init__(**kwargs)
        self._arcl = context.arcl_client
        self._shared_memory = context.shared_memory
        self._timeout_secs = timeout_secs

        self._shared_memory.add(SharedMemoryKeys.ARCL_ERROR_MESSAGE, None)

    async def _execute(self):
        logger.info("Waiting for ARCL task completion")
        elapsed = 0.0

        while True:
            if self._timeout_secs and elapsed >= self._timeout_secs:
                error_msg = f"ARCL task timed out after {self._timeout_secs}s"
                logger.error(error_msg)
                self._shared_memory.set(SharedMemoryKeys.ARCL_ERROR_MESSAGE, error_msg)
                raise RuntimeError(error_msg)

            try:
                status = await self._arcl.query_status()
                omron_status = status.get("Status", "") if status else ""

                if any(omron_status.startswith(p) for p in _SUCCESS_PREFIXES):
                    logger.info("ARCL task completed: %s", omron_status)
                    return

                if any(omron_status.startswith(p) for p in _FAILURE_PREFIXES):
                    error_msg = f"ARCL task failed: {omron_status}"
                    logger.error(error_msg)
                    self._shared_memory.set(SharedMemoryKeys.ARCL_ERROR_MESSAGE, error_msg)
                    raise RuntimeError(error_msg)

                logger.debug("ARCL status: %s, waiting...", omron_status)

            except RuntimeError:
                raise
            except Exception as e:
                logger.warning("ARCL status poll failed: %s", e)

            await asyncio.sleep(_POLL_INTERVAL_SECS)
            elapsed += _POLL_INTERVAL_SECS

    def dump_object(self):
        obj = super().dump_object()
        obj["timeout_secs"] = self._timeout_secs
        return obj

    @classmethod
    def from_object(cls, context, timeout_secs=None, **kwargs):
        return WaitForArclCompletionNode(context, timeout_secs=timeout_secs, **kwargs)


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

    async def _execute(self):
        x_mm = int(self._x_m * 1000)
        y_mm = int(self._y_m * 1000)
        theta_deg = int(math.degrees(self._theta_rad))
        logger.info("Sending gotopoint %d %d %d", x_mm, y_mm, theta_deg)
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

    async def _execute(self):
        logger.info("Sending goto %s", self._goal_name)
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
                theta_rad=wp.yaw,
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
                    label="Wait for undock completion",
                )
            )
            return sequence

        # Unknown action — fall back to default (cloud round-trip)
        logger.warning("Unknown action '%s' — falling back to cloud execution", action_id)
        return super().visit_run_action(step)


# Register node types for serialization/deserialization (crash recovery)
arcl_node_types = [
    ArclGotoPointNode,
    ArclGotoGoalNode,
    ArclDockNode,
    ArclUndockNode,
    WaitForArclCompletionNode,
    ArclMissionAbortedNode,
]
register_accepted_node_types(arcl_node_types)
