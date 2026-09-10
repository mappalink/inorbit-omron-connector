# SPDX-FileCopyrightText: 2026 Mappalink
#
# SPDX-License-Identifier: MIT

"""Tests for the connector command handler — routing, pause/resume, nav goal."""

from __future__ import annotations

import asyncio
from unittest.mock import ANY, AsyncMock, MagicMock, patch

import pytest

from inorbit_connector.connector import CommandResultCode
from inorbit_edge.robot import COMMAND_CUSTOM_COMMAND, COMMAND_MESSAGE, COMMAND_NAV_GOAL
from inorbit_omron_connector.src.goal_tracker import GoalTracker


@pytest.fixture
def connector():
    """Create an OmronArclConnector with a mocked ArclClient (no real TCP)."""
    with patch("inorbit_omron_connector.src.connector.ArclClient") as MockArclClient:
        mock_arcl = MockArclClient.return_value
        mock_arcl.set_block_driving = AsyncMock()
        mock_arcl.clear_block_driving = AsyncMock()
        mock_arcl.go = AsyncMock()
        mock_arcl.goto = AsyncMock()
        mock_arcl.gotopoint = AsyncMock()
        mock_arcl.dock = AsyncMock()
        mock_arcl.undock = AsyncMock()
        mock_arcl.execute_macro = AsyncMock()
        mock_arcl.stop = AsyncMock()

        from inorbit_omron_connector.src.connector import OmronArclConnector

        # Bypass Connector.__init__ (needs InOrbit session, MQTT, etc.)
        instance = object.__new__(OmronArclConnector)
        instance._arcl = mock_arcl
        instance._map_id = "test-map"
        instance._goal_tracker = GoalTracker()
        instance._logger = MagicMock()

        # Mock mission executor — handle_command returns False (not a mission cmd)
        mock_mission_executor = AsyncMock()
        mock_mission_executor.handle_command = AsyncMock(return_value=False)
        instance._mission_executor = mock_mission_executor
        instance._goal_tracker_enabled = True
        instance._last_nav_goal = None
        instance._last_nav_point = None

        # Workbench PLC integration — one mocked table
        mock_plc = AsyncMock()
        mock_plc.is_moving = False
        instance._plc_tables = {"wb1": mock_plc}
        instance._plc_heights = {"wb1": {"retracted": 800, "pickup": 1131}}
        instance._plc_move_timeout_secs = 60.0
        instance._plc_poll_skip = {}
        instance._plc_move_tasks = set()
        instance.publish_key_values = MagicMock()

        yield instance


@pytest.fixture
def result_fn():
    return MagicMock()


@pytest.fixture
def options(result_fn):
    return {"result_function": result_fn}


# -- COMMAND_MESSAGE (cloud-mode pause/resume) --------------------------------


class TestHandleMessage:
    @pytest.mark.asyncio
    async def test_inorbit_pause(self, connector, result_fn):
        await connector._handle_message("inorbit_pause", result_fn)

        connector._arcl.set_block_driving.assert_awaited_once()
        call_args = connector._arcl.set_block_driving.call_args
        assert call_args[0][0] == "InOrbit"
        result_fn.assert_called_once_with(CommandResultCode.SUCCESS)

    @pytest.mark.asyncio
    async def test_inorbit_resume(self, connector, result_fn):
        await connector._handle_message("inorbit_resume", result_fn)

        connector._arcl.clear_block_driving.assert_awaited_once_with("InOrbit")
        connector._arcl.go.assert_awaited_once()
        result_fn.assert_called_once_with(CommandResultCode.SUCCESS)

    @pytest.mark.asyncio
    async def test_unhandled_message_no_result(self, connector, result_fn):
        """Unrecognized messages (e.g. inorbit_run_mission) are passed through
        to MissionsModule — result_fn should NOT be called."""
        await connector._handle_message("inorbit_run_mission abc123 {}", result_fn)

        result_fn.assert_not_called()

    @pytest.mark.asyncio
    async def test_pause_arcl_error_returns_failure(self, connector, result_fn):
        connector._arcl.set_block_driving.side_effect = Exception("TCP error")

        await connector._handle_message("inorbit_pause", result_fn)

        result_fn.assert_called_once_with(CommandResultCode.FAILURE)


# -- Routing via _inorbit_command_handler -------------------------------------


class TestCommandRouting:
    @pytest.mark.asyncio
    async def test_routes_command_message(self, connector, options, result_fn):
        await connector._inorbit_command_handler(COMMAND_MESSAGE, ["inorbit_pause"], options)

        connector._arcl.set_block_driving.assert_awaited_once()
        result_fn.assert_called_once_with(CommandResultCode.SUCCESS)

    @pytest.mark.asyncio
    async def test_routes_nav_goal(self, connector, options, result_fn):
        pose = {"x": 5.0, "y": 3.0, "theta": 1.57}
        await connector._inorbit_command_handler(COMMAND_NAV_GOAL, [pose], options)

        connector._arcl.gotopoint.assert_awaited_once()
        result_fn.assert_called_once_with(CommandResultCode.SUCCESS)

    @pytest.mark.asyncio
    async def test_routes_dock(self, connector, options, result_fn):
        connector._arcl.query_status = AsyncMock(return_value={"Status": "Parked"})
        await connector._inorbit_command_handler(COMMAND_CUSTOM_COMMAND, ["dock", []], options)

        connector._arcl.dock.assert_awaited_once()
        result_fn.assert_called_once_with(CommandResultCode.SUCCESS)

    @pytest.mark.asyncio
    async def test_routes_undock(self, connector, options, result_fn):
        connector._arcl.query_status = AsyncMock(return_value={"Status": "Idle"})
        await connector._inorbit_command_handler(COMMAND_CUSTOM_COMMAND, ["undock", []], options)

        connector._arcl.undock.assert_awaited_once()
        result_fn.assert_called_once_with(CommandResultCode.SUCCESS)

    @pytest.mark.asyncio
    async def test_routes_pauseRobot(self, connector, options, result_fn):
        await connector._inorbit_command_handler(
            COMMAND_CUSTOM_COMMAND, ["pauseRobot", []], options
        )

        connector._arcl.set_block_driving.assert_awaited_once()
        result_fn.assert_called_once_with(CommandResultCode.SUCCESS)

    @pytest.mark.asyncio
    async def test_routes_resumeRobot(self, connector, options, result_fn):
        await connector._inorbit_command_handler(
            COMMAND_CUSTOM_COMMAND, ["resumeRobot", []], options
        )

        connector._arcl.clear_block_driving.assert_awaited_once()
        connector._arcl.go.assert_awaited_once()
        result_fn.assert_called_once_with(CommandResultCode.SUCCESS)

    @pytest.mark.asyncio
    async def test_unknown_command_returns_failure(self, connector, options, result_fn):
        await connector._inorbit_command_handler("unknownCommand", ["something"], options)

        result_fn.assert_called_once_with(CommandResultCode.FAILURE)


# -- executeMacro custom command ---------------------------------------------


class TestExecuteMacro:
    @pytest.mark.asyncio
    async def test_missing_macro_name_returns_failure(self, connector, options, result_fn):
        await connector._inorbit_command_handler(
            COMMAND_CUSTOM_COMMAND, ["execute_macro", []], options
        )

        connector._arcl.execute_macro.assert_not_awaited()
        result_fn.assert_called_once_with(CommandResultCode.FAILURE)

    @pytest.mark.asyncio
    async def test_instant_success_via_completed_macro_status(self, connector, options, result_fn):
        """Instant macros (SFA toggles) may reach `Completed macro <name>`
        before the first poll observes the active state. The kickoff guard's
        instant-success shortcut must accept this and report SUCCESS."""
        macro = "SFA_WS1_On"
        connector._arcl.query_status = AsyncMock(
            return_value={"Status": f"Completed macro {macro}"}
        )

        await connector._inorbit_command_handler(
            COMMAND_CUSTOM_COMMAND,
            ["execute_macro", ["--macro_name", macro]],
            options,
        )

        connector._arcl.execute_macro.assert_awaited_once_with(macro)
        result_fn.assert_called_once_with(CommandResultCode.SUCCESS)

    @pytest.mark.asyncio
    async def test_active_then_completed_returns_success(self, connector, options, result_fn):
        macro = "PrecisionDriveTafel_LB"
        connector._arcl.query_status = AsyncMock(
            side_effect=[
                {"Status": f"Executing macro {macro}"},
                {"Status": f"Completed macro {macro}"},
            ]
        )

        await connector._inorbit_command_handler(
            COMMAND_CUSTOM_COMMAND,
            ["execute_macro", ["--macro_name", macro]],
            options,
        )

        connector._arcl.execute_macro.assert_awaited_once_with(macro)
        result_fn.assert_called_once_with(CommandResultCode.SUCCESS)

    @pytest.mark.asyncio
    async def test_active_then_error_returns_failure(self, connector, options, result_fn):
        """Runtime failure (e.g. PrecisionDrive can't find target) shows up as
        `Error: <reason>` in the Status field. The line itself doesn't carry
        the macro name — identity is carried by the prior active-state observation."""
        macro = "PrecisionDriveTafel_LB"
        connector._arcl.query_status = AsyncMock(
            side_effect=[
                {"Status": f"Executing macro {macro}"},
                {"Status": "Error: Failed to drive to Target"},
            ]
        )

        await connector._inorbit_command_handler(
            COMMAND_CUSTOM_COMMAND,
            ["execute_macro", ["--macro_name", macro]],
            options,
        )

        connector._arcl.execute_macro.assert_awaited_once_with(macro)
        result_fn.assert_called_once_with(CommandResultCode.FAILURE)

    @pytest.mark.asyncio
    async def test_edge_arg_form_macro_name_without_prefix(self, connector, options, result_fn):
        """Edge MissionDefinition steps pass arguments without the `--` prefix.
        The handler accepts both `--macro_name` (cloud) and `macro_name` (edge)."""
        macro = "SFA_WS1_Off"
        connector._arcl.query_status = AsyncMock(
            return_value={"Status": f"Completed macro {macro}"}
        )

        await connector._inorbit_command_handler(
            COMMAND_CUSTOM_COMMAND,
            ["execute_macro", ["macro_name", macro]],
            options,
        )

        connector._arcl.execute_macro.assert_awaited_once_with(macro)
        result_fn.assert_called_once_with(CommandResultCode.SUCCESS)


# -- plc_legs (workbench lifting columns) -------------------------------------


class TestPlcLegs:
    """Button path: SUCCESS once the PLC is moving; the move finishes in background."""

    @staticmethod
    async def _drain(connector):
        if connector._plc_move_tasks:
            await asyncio.gather(*connector._plc_move_tasks, return_exceptions=True)
        await asyncio.sleep(0)  # let done-callbacks run

    @staticmethod
    async def _run(connector, options, args):
        await connector._inorbit_command_handler(
            COMMAND_CUSTOM_COMMAND, ["plc_legs", args], options
        )

    @pytest.mark.asyncio
    async def test_retract_resolves_configured_height(self, connector, options, result_fn):
        await self._run(connector, options, ["--action", "retract", "--table", "wb1"])
        connector._plc_tables["wb1"].move_to_height.assert_awaited_once_with(
            800, timeout_secs=60.0, started=ANY
        )
        assert result_fn.call_args[0][0] == CommandResultCode.SUCCESS

    @pytest.mark.asyncio
    async def test_extend_resolves_pickup_height(self, connector, options, result_fn):
        await self._run(connector, options, ["--action", "extend", "--table", "wb1"])
        connector._plc_tables["wb1"].move_to_height.assert_awaited_once_with(
            1131, timeout_secs=60.0, started=ANY
        )
        assert result_fn.call_args[0][0] == CommandResultCode.SUCCESS

    @pytest.mark.asyncio
    async def test_explicit_height_overrides_action(self, connector, options, result_fn):
        await self._run(connector, options, ["--table", "wb1", "--height_mm", "950"])
        connector._plc_tables["wb1"].move_to_height.assert_awaited_once_with(
            950, timeout_secs=60.0, started=ANY
        )

    @pytest.mark.asyncio
    async def test_edge_arg_form_without_prefix(self, connector, options, result_fn):
        """Edge MissionDefinition steps pass arguments without the `--` prefix."""
        await self._run(connector, options, ["action", "retract", "table", "wb1"])
        connector._plc_tables["wb1"].move_to_height.assert_awaited_once_with(
            800, timeout_secs=60.0, started=ANY
        )

    @pytest.mark.asyncio
    async def test_unknown_table_fails_with_details(self, connector, options, result_fn):
        await self._run(connector, options, ["--action", "retract", "--table", "nope"])
        connector._plc_tables["wb1"].move_to_height.assert_not_awaited()
        assert result_fn.call_args[0][0] == CommandResultCode.FAILURE
        assert "unknown table" in result_fn.call_args[1]["execution_status_details"]

    @pytest.mark.asyncio
    async def test_missing_action_and_height_fails(self, connector, options, result_fn):
        await self._run(connector, options, ["--table", "wb1"])
        assert result_fn.call_args[0][0] == CommandResultCode.FAILURE

    @pytest.mark.asyncio
    async def test_plc_error_before_start_surfaces_details(self, connector, options, result_fn):
        from inorbit_omron_connector.src.plc_client import PlcError

        connector._plc_tables["wb1"].move_to_height.side_effect = PlcError(
            "PLC has a latched error (code 7): 'E-stop open'"
        )
        await self._run(connector, options, ["--action", "extend", "--table", "wb1"])
        assert result_fn.call_args[0][0] == CommandResultCode.FAILURE
        assert "E-stop open" in result_fn.call_args[1]["execution_status_details"]

    @pytest.mark.asyncio
    async def test_reports_success_once_moving_and_finishes_in_background(
        self, connector, options, result_fn
    ):
        gate = asyncio.Event()

        async def move(target_mm, timeout_secs, started):
            started.set()
            await gate.wait()

        connector._plc_tables["wb1"].move_to_height.side_effect = move
        await self._run(connector, options, ["--action", "extend", "--table", "wb1"])

        result_fn.assert_called_once()
        assert result_fn.call_args[0][0] == CommandResultCode.SUCCESS
        assert "started" in result_fn.call_args[1]["execution_status_details"]
        assert len(connector._plc_move_tasks) == 1  # still moving

        gate.set()
        await self._drain(connector)
        connector.publish_key_values.assert_called_with(plc_wb1_last_move="extend: at 1131 mm")
        result_fn.assert_called_once()  # no second result after the move ends

    @pytest.mark.asyncio
    async def test_failure_after_start_is_published(self, connector, options, result_fn):
        from inorbit_omron_connector.src.plc_client import PlcError

        async def move(target_mm, timeout_secs, started):
            started.set()
            await asyncio.sleep(0.01)
            raise PlcError("move to 1131 mm stalled: no progress for 20s")

        connector._plc_tables["wb1"].move_to_height.side_effect = move
        await self._run(connector, options, ["--action", "extend", "--table", "wb1"])
        await self._drain(connector)

        result_fn.assert_called_once()
        assert result_fn.call_args[0][0] == CommandResultCode.SUCCESS
        published = connector.publish_key_values.call_args[1]["plc_wb1_last_move"]
        assert published.startswith("extend FAILED") and "stalled" in published

    @pytest.mark.asyncio
    async def test_not_started_in_time_fails_and_cancels_move(
        self, connector, options, result_fn, monkeypatch
    ):
        from inorbit_omron_connector.src import connector as connector_module

        monkeypatch.setattr(connector_module, "_PLC_START_TIMEOUT", 0.05)
        seen = {}

        async def move(target_mm, timeout_secs, started):
            try:
                await asyncio.Event().wait()
            except asyncio.CancelledError:
                seen["cancelled"] = True
                raise

        connector._plc_tables["wb1"].move_to_height.side_effect = move
        await self._run(connector, options, ["--action", "retract", "--table", "wb1"])

        assert result_fn.call_args[0][0] == CommandResultCode.FAILURE
        assert "did not start" in result_fn.call_args[1]["execution_status_details"]
        assert seen.get("cancelled") is True

    @pytest.mark.asyncio
    async def test_refuses_while_already_moving(self, connector, options, result_fn):
        connector._plc_tables["wb1"].is_moving = True
        await self._run(connector, options, ["--action", "retract", "--table", "wb1"])
        connector._plc_tables["wb1"].move_to_height.assert_not_awaited()
        assert result_fn.call_args[0][0] == CommandResultCode.FAILURE
        assert "already moving" in result_fn.call_args[1]["execution_status_details"]


class TestPlcCheck:
    @pytest.mark.asyncio
    async def test_check_passes_and_never_moves(self, connector, options, result_fn):
        from inorbit_omron_connector.src.plc_client import PlcState

        plc = connector._plc_tables["wb1"]
        plc.check_at_height.return_value = PlcState(1131, False, False, False, 0, "Idle")
        await connector._inorbit_command_handler(
            COMMAND_CUSTOM_COMMAND,
            ["plc_check", ["--table", "wb1", "--state", "extended", "--wait_secs", "5"]],
            options,
        )
        plc.check_at_height.assert_awaited_once_with(1131, wait_secs=5.0)
        plc.move_to_height.assert_not_awaited()
        assert result_fn.call_args[0][0] == CommandResultCode.SUCCESS
        assert "1131 mm" in result_fn.call_args[1]["execution_status_details"]

    @pytest.mark.asyncio
    async def test_check_failure_reports_reason(self, connector, options, result_fn):
        from inorbit_omron_connector.src.plc_client import PlcError

        plc = connector._plc_tables["wb1"]
        plc.check_at_height.side_effect = PlcError("height is 799 mm, expected 1131 ± 10 mm")
        await connector._inorbit_command_handler(
            COMMAND_CUSTOM_COMMAND,
            ["plc_check", ["--table", "wb1", "--state", "extended"]],
            options,
        )
        assert result_fn.call_args[0][0] == CommandResultCode.FAILURE
        assert "799" in result_fn.call_args[1]["execution_status_details"]

    @pytest.mark.asyncio
    async def test_negative_wait_rejected(self, connector, options, result_fn):
        plc = connector._plc_tables["wb1"]
        await connector._inorbit_command_handler(
            COMMAND_CUSTOM_COMMAND,
            ["plc_check", ["--table", "wb1", "--state", "extended", "--wait_secs", "-1"]],
            options,
        )
        plc.check_at_height.assert_not_awaited()
        assert result_fn.call_args[0][0] == CommandResultCode.FAILURE
