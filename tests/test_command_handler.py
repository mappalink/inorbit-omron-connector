# SPDX-FileCopyrightText: 2026 Mappalink
#
# SPDX-License-Identifier: MIT

"""Tests for the connector command handler — routing, pause/resume, nav goal."""

from __future__ import annotations

from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from inorbit_connector.connector import CommandResultCode
from inorbit_edge.robot import COMMAND_CUSTOM_COMMAND, COMMAND_MESSAGE, COMMAND_NAV_GOAL
from inorbit_omron_connector.src.goal_tracker import GoalTracker


@pytest.fixture
def connector():
    """Create an OmronArclConnector with a mocked ArclClient (no real TCP)."""
    with patch(
        "inorbit_omron_connector.src.connector.ArclClient"
    ) as MockArclClient:
        mock_arcl = MockArclClient.return_value
        mock_arcl.set_block_driving = AsyncMock()
        mock_arcl.clear_block_driving = AsyncMock()
        mock_arcl.go = AsyncMock()
        mock_arcl.goto = AsyncMock()
        mock_arcl.gotopoint = AsyncMock()
        mock_arcl.dock = AsyncMock()
        mock_arcl.undock = AsyncMock()
        mock_arcl.stop = AsyncMock()

        from inorbit_omron_connector.src.connector import OmronArclConnector

        # Bypass Connector.__init__ (needs InOrbit session, MQTT, etc.)
        instance = object.__new__(OmronArclConnector)
        instance._arcl = mock_arcl
        instance._map_id = "test-map"
        instance._goal_tracker = GoalTracker()
        instance._logger = MagicMock()

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
        await connector._inorbit_command_handler(
            COMMAND_MESSAGE, ["inorbit_pause"], options
        )

        connector._arcl.set_block_driving.assert_awaited_once()
        result_fn.assert_called_once_with(CommandResultCode.SUCCESS)

    @pytest.mark.asyncio
    async def test_routes_nav_goal(self, connector, options, result_fn):
        pose = {"x": 5.0, "y": 3.0, "theta": 1.57}
        await connector._inorbit_command_handler(
            COMMAND_NAV_GOAL, [pose], options
        )

        connector._arcl.gotopoint.assert_awaited_once()
        result_fn.assert_called_once_with(CommandResultCode.SUCCESS)

    @pytest.mark.asyncio
    async def test_routes_custom_command(self, connector, options, result_fn):
        await connector._inorbit_command_handler(
            COMMAND_CUSTOM_COMMAND, ["dock", []], options
        )

        connector._arcl.dock.assert_awaited_once()
        result_fn.assert_called_once_with(CommandResultCode.SUCCESS)

    @pytest.mark.asyncio
    async def test_unknown_command_returns_failure(self, connector, options, result_fn):
        await connector._inorbit_command_handler(
            "unknownCommand", ["something"], options
        )

        result_fn.assert_called_once_with(CommandResultCode.FAILURE)
