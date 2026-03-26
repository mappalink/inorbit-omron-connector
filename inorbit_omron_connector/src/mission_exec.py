# SPDX-FileCopyrightText: 2026 Mappalink
#
# SPDX-License-Identifier: MIT

"""Edge-executor mission support for Omron ARCL connector.

Extends inorbit-edge-executor with ARCL-specific pause/resume/abort that
forward to the robot via ARCL TCP commands.
"""

import json
import logging
from collections.abc import Awaitable, Callable
from enum import Enum

from inorbit_connector.connector import CommandResultCode
from inorbit_edge_executor.datatypes import MissionRuntimeOptions
from inorbit_edge_executor.db import get_db
from inorbit_edge_executor.mission import Mission
from inorbit_edge_executor.worker_pool import WorkerPool

from .arcl_client import ArclClient
from .mission.behavior_tree import ArclBehaviorTreeBuilderContext
from .mission.tree_builder import ArclTreeBuilder

logger = logging.getLogger(__name__)

# Block driving name used for pause/resume
BLOCK_NAME = "InOrbit"


class MissionScriptName(Enum):
    """Mission-related custom commands sent by InOrbit edge executor."""

    EXECUTE_MISSION_ACTION = "executeMissionAction"
    CANCEL_MISSION_ACTION = "cancelMissionAction"
    UPDATE_MISSION_ACTION = "updateMissionAction"


class ArclWorkerPool(WorkerPool):
    """WorkerPool that executes steps locally via ARCL and forwards
    pause/resume/abort to the robot."""

    def __init__(
        self,
        arcl_client: ArclClient,
        *args,
        on_cloud_resume: Callable[[], Awaitable[None]] | None = None,
        **kwargs,
    ):
        self._arcl = arcl_client
        self._on_cloud_resume = on_cloud_resume
        super().__init__(behavior_tree_builder=ArclTreeBuilder(), *args, **kwargs)

    def create_builder_context(self) -> ArclBehaviorTreeBuilderContext:
        return ArclBehaviorTreeBuilderContext(arcl_client=self._arcl)

    async def pause_mission(self, mission_id):
        # Always send block driving to stop the robot, even if the mission
        # isn't tracked by the edge executor (e.g. cloud-dispatched missions).
        try:
            await self._arcl.set_block_driving(
                BLOCK_NAME, "Paused by InOrbit", "Mission paused via edge executor"
            )
            logger.info("ARCL pause (set_block_driving) on mission pause")
        except Exception as e:
            logger.warning("Failed to pause ARCL robot: %s", e)
        try:
            await super().pause_mission(mission_id)
        except Exception:
            logger.debug(
                "No edge worker for mission %s (cloud mission?), skipping BT pause", mission_id
            )

    async def resume_mission(self, mission_id):
        # Clear block driving BEFORE resuming the BT so the robot is ready
        # to accept navigation commands when the wait node re-sends them.
        try:
            await self._arcl.clear_block_driving(BLOCK_NAME)
            logger.info("ARCL clear_block_driving on mission resume")
        except Exception as e:
            logger.warning("Failed to clear block driving: %s", e)
        try:
            await super().resume_mission(mission_id)
        except Exception:
            # Cloud mission — BT doesn't manage the goal, so re-send it.
            logger.debug(
                "No edge worker for mission %s (cloud mission?), skipping BT resume", mission_id
            )
            if self._on_cloud_resume:
                await self._on_cloud_resume()
                logger.info("Re-sent last nav goal after cloud resume")

    async def abort_mission(self, mission_id):
        try:
            await self._arcl.stop()
            logger.info("ARCL stop on mission abort")
        except Exception as e:
            logger.warning("Failed to stop ARCL robot on abort: %s", e)
        try:
            super().abort_mission(mission_id)
        except Exception:
            logger.debug(
                "No edge worker for mission %s (cloud mission?), skipping BT abort", mission_id
            )


class OmronMissionExecutor:
    """Handles edge-executor mission commands for a single Omron robot."""

    def __init__(
        self,
        robot_id: str,
        inorbit_api,
        arcl_client: ArclClient,
        database_file: str | None = None,
        on_cloud_resume: Callable[[], Awaitable[None]] | None = None,
    ):
        self._robot_id = robot_id
        self._inorbit_api = inorbit_api
        self._arcl_client = arcl_client
        self._on_cloud_resume = on_cloud_resume
        if database_file:
            if database_file == "dummy":
                self._database_file = "dummy"
            else:
                self._database_file = f"sqlite:{database_file}"
        else:
            self._database_file = f"sqlite:missions_{robot_id}.db"
        self._worker_pool: ArclWorkerPool | None = None
        self._initialized = False

    async def initialize(self):
        if self._initialized:
            return
        db = await get_db(self._database_file)
        self._worker_pool = ArclWorkerPool(
            arcl_client=self._arcl_client,
            api=self._inorbit_api,
            db=db,
            on_cloud_resume=self._on_cloud_resume,
        )
        await self._worker_pool.start()
        self._initialized = True
        logger.info("Omron Mission Executor initialized")

    async def shutdown(self):
        if self._worker_pool:
            await self._worker_pool.shutdown()
            logger.info("Omron Mission Executor shut down")

    async def handle_command(self, script_name: str, script_args: dict, options: dict) -> bool:
        """Route mission commands. Returns True if handled, False otherwise."""
        if not self._initialized:
            logger.warning("Mission executor not initialized, cannot handle commands")
            return False

        if script_name == MissionScriptName.EXECUTE_MISSION_ACTION.value:
            await self._handle_execute_mission_action(script_args, options)
            return True
        elif script_name == MissionScriptName.CANCEL_MISSION_ACTION.value:
            await self._handle_cancel_mission(script_args, options)
            return True
        elif script_name == MissionScriptName.UPDATE_MISSION_ACTION.value:
            await self._handle_update_mission_action(script_args, options)
            return True
        return False

    async def _handle_execute_mission_action(self, script_args: dict, options: dict) -> None:
        try:
            mission_id = script_args.get("missionId")
            mission_definition = json.loads(script_args.get("missionDefinition", "{}"))
            mission_args = json.loads(script_args.get("missionArgs", "{}"))
            mission_options_dict = json.loads(script_args.get("options", "{}"))

            mission = Mission(
                id=mission_id,
                robot_id=self._robot_id,
                definition=mission_definition,
                arguments=mission_args,
            )
            mission_runtime_options = MissionRuntimeOptions(**mission_options_dict)
            await self._worker_pool.submit_work(mission, mission_runtime_options)
            options["result_function"](CommandResultCode.SUCCESS)
        except json.JSONDecodeError as e:
            logger.error("Invalid JSON in mission definition: %s", e)
            options["result_function"](
                CommandResultCode.FAILURE,
                execution_status_details=f"Invalid JSON: {e}",
            )
        except Exception as e:
            logger.error("Failed to execute mission: %s", e)
            options["result_function"](
                CommandResultCode.FAILURE,
                execution_status_details=str(e),
            )

    async def _handle_cancel_mission(self, script_args: dict, options: dict) -> None:
        mission_id = script_args.get("missionId")
        logger.info("Handling cancelMission for mission %s", mission_id)
        try:
            result = await self._worker_pool.abort_mission(mission_id)
            if result is False:
                options["result_function"](CommandResultCode.FAILURE, "Mission not found")
            else:
                options["result_function"](CommandResultCode.SUCCESS)
        except Exception as e:
            logger.error("Failed to cancel mission %s: %s", mission_id, e)
            options["result_function"](
                CommandResultCode.FAILURE,
                execution_status_details=str(e),
            )

    async def _handle_update_mission_action(self, script_args: dict, options: dict) -> None:
        mission_id = script_args.get("missionId")
        action = script_args.get("action")
        logger.info("Handling updateMissionAction %s for mission %s", action, mission_id)
        try:
            if action == "pause":
                await self._worker_pool.pause_mission(mission_id)
            elif action == "resume":
                await self._worker_pool.resume_mission(mission_id)
            else:
                raise ValueError(f"Unknown action: {action}")
            options["result_function"](CommandResultCode.SUCCESS)
        except Exception as e:
            logger.error("Failed to update mission %s (action=%s): %s", mission_id, action, e)
            options["result_function"](
                CommandResultCode.FAILURE,
                execution_status_details=str(e),
            )
