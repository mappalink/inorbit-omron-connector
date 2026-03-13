# SPDX-FileCopyrightText: 2026 Mappalink
#
# SPDX-License-Identifier: MIT

"""Tests for the edge-executor mission support (mission_exec.py)."""

from __future__ import annotations

import json
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from inorbit_connector.connector import CommandResultCode
from inorbit_omron_connector.src.mission_exec import (
    OmronMissionExecutor,
    ArclWorkerPool,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
def mock_arcl_client():
    client = AsyncMock()
    client.set_block_driving = AsyncMock()
    client.clear_block_driving = AsyncMock()
    client.go = AsyncMock()
    client.stop = AsyncMock()
    return client


@pytest.fixture
def result_collector():
    """Collects (code, kwargs) tuples from result_function calls."""
    results = []

    def _fn(code, **kwargs):
        results.append((code, kwargs))

    return results, _fn


@pytest.fixture
def make_executor(mock_arcl_client):
    """Create an OmronMissionExecutor with mocked internals (skip real DB/pool)."""

    def _factory():
        executor = OmronMissionExecutor(
            robot_id="omron-1",
            inorbit_api=MagicMock(),
            arcl_client=mock_arcl_client,
            database_file="dummy",
        )
        # Fake an initialised worker pool so handle_command doesn't bail out
        executor._initialized = True
        executor._worker_pool = AsyncMock()
        return executor

    return _factory


# ---------------------------------------------------------------------------
# OmronMissionExecutor.handle_command — routing
# ---------------------------------------------------------------------------


class TestHandleCommandRouting:
    @pytest.mark.asyncio
    async def test_routes_execute_mission_action(self, make_executor, result_collector):
        executor = make_executor()
        results, result_fn = result_collector
        options = {"result_function": result_fn}

        args = {
            "missionId": "m-1",
            "missionDefinition": "{}",
            "missionArgs": "{}",
            "options": "{}",
        }
        handled = await executor.handle_command("executeMissionAction", args, options)
        assert handled is True

    @pytest.mark.asyncio
    async def test_routes_cancel_mission_action(self, make_executor, result_collector):
        executor = make_executor()
        results, result_fn = result_collector
        options = {"result_function": result_fn}

        handled = await executor.handle_command(
            "cancelMissionAction", {"missionId": "m-1"}, options
        )
        assert handled is True

    @pytest.mark.asyncio
    async def test_routes_update_mission_action(self, make_executor, result_collector):
        executor = make_executor()
        results, result_fn = result_collector
        options = {"result_function": result_fn}

        handled = await executor.handle_command(
            "updateMissionAction", {"missionId": "m-1", "action": "pause"}, options
        )
        assert handled is True

    @pytest.mark.asyncio
    async def test_ignores_non_executor_commands(self, make_executor, result_collector):
        executor = make_executor()
        _, result_fn = result_collector
        options = {"result_function": result_fn}

        assert await executor.handle_command("goto_goal", {}, options) is False
        assert await executor.handle_command("unknown_cmd", {}, options) is False

    @pytest.mark.asyncio
    async def test_returns_false_when_not_initialized(self, mock_arcl_client, result_collector):
        executor = OmronMissionExecutor(
            robot_id="omron-1",
            inorbit_api=MagicMock(),
            arcl_client=mock_arcl_client,
            database_file="dummy",
        )
        _, result_fn = result_collector
        options = {"result_function": result_fn}

        assert (
            await executor.handle_command("executeMissionAction", {"missionId": "m-1"}, options)
            is False
        )


# ---------------------------------------------------------------------------
# OmronMissionExecutor — action handlers
# ---------------------------------------------------------------------------


class TestExecuteMissionAction:
    @pytest.mark.asyncio
    async def test_parses_json_and_submits(self, make_executor, result_collector):
        executor = make_executor()
        results, result_fn = result_collector
        options = {"result_function": result_fn}

        definition = {"steps": [{"runAction": {"actionId": "goto_goal", "arguments": {}}}]}
        args = {
            "missionId": "m-42",
            "missionDefinition": json.dumps(definition),
            "missionArgs": "{}",
            "options": "{}",
        }
        await executor.handle_command("executeMissionAction", args, options)

        executor._worker_pool.submit_work.assert_awaited_once()
        mission = executor._worker_pool.submit_work.call_args[0][0]
        assert mission.id == "m-42"
        assert results[-1][0] == CommandResultCode.SUCCESS

    @pytest.mark.asyncio
    async def test_reports_failure_on_bad_json(self, make_executor, result_collector):
        executor = make_executor()
        results, result_fn = result_collector
        options = {"result_function": result_fn}

        args = {
            "missionId": "m-bad",
            "missionDefinition": "{not-json",
        }
        await executor.handle_command("executeMissionAction", args, options)
        assert results[-1][0] == CommandResultCode.FAILURE


class TestCancelMission:
    @pytest.mark.asyncio
    async def test_calls_abort(self, make_executor, result_collector):
        executor = make_executor()
        results, result_fn = result_collector
        options = {"result_function": result_fn}

        await executor.handle_command("cancelMissionAction", {"missionId": "m-1"}, options)
        executor._worker_pool.abort_mission.assert_awaited_once_with("m-1")
        assert results[-1][0] == CommandResultCode.SUCCESS


class TestUpdateMissionAction:
    @pytest.mark.asyncio
    async def test_pause(self, make_executor, result_collector):
        executor = make_executor()
        results, result_fn = result_collector
        options = {"result_function": result_fn}

        await executor.handle_command(
            "updateMissionAction", {"missionId": "m-1", "action": "pause"}, options
        )
        executor._worker_pool.pause_mission.assert_awaited_once_with("m-1")
        assert results[-1][0] == CommandResultCode.SUCCESS

    @pytest.mark.asyncio
    async def test_resume(self, make_executor, result_collector):
        executor = make_executor()
        results, result_fn = result_collector
        options = {"result_function": result_fn}

        await executor.handle_command(
            "updateMissionAction", {"missionId": "m-1", "action": "resume"}, options
        )
        executor._worker_pool.resume_mission.assert_awaited_once_with("m-1")
        assert results[-1][0] == CommandResultCode.SUCCESS

    @pytest.mark.asyncio
    async def test_unknown_action_fails(self, make_executor, result_collector):
        executor = make_executor()
        results, result_fn = result_collector
        options = {"result_function": result_fn}

        await executor.handle_command(
            "updateMissionAction", {"missionId": "m-1", "action": "bogus"}, options
        )
        assert results[-1][0] == CommandResultCode.FAILURE


# ---------------------------------------------------------------------------
# ArclWorkerPool — ARCL command forwarding
# ---------------------------------------------------------------------------


class TestArclWorkerPool:
    @pytest.mark.asyncio
    async def test_pause_calls_arcl_block_driving(self, mock_arcl_client):
        pool = ArclWorkerPool(
            arcl_client=mock_arcl_client,
            api=MagicMock(),
            db=MagicMock(),
        )
        with patch("inorbit_edge_executor.worker_pool.WorkerPool.pause_mission", AsyncMock()):
            await pool.pause_mission("m-1")

        mock_arcl_client.set_block_driving.assert_awaited_once()

    @pytest.mark.asyncio
    async def test_resume_calls_arcl_clear_block_and_go(self, mock_arcl_client):
        pool = ArclWorkerPool(
            arcl_client=mock_arcl_client,
            api=MagicMock(),
            db=MagicMock(),
        )
        with patch("inorbit_edge_executor.worker_pool.WorkerPool.resume_mission", AsyncMock()):
            await pool.resume_mission("m-1")

        mock_arcl_client.clear_block_driving.assert_awaited_once()
        mock_arcl_client.go.assert_awaited_once()

    @pytest.mark.asyncio
    async def test_abort_calls_arcl_stop(self, mock_arcl_client):
        pool = ArclWorkerPool(
            arcl_client=mock_arcl_client,
            api=MagicMock(),
            db=MagicMock(),
        )
        with patch("inorbit_edge_executor.worker_pool.WorkerPool.abort_mission", MagicMock()):
            await pool.abort_mission("m-1")

        mock_arcl_client.stop.assert_awaited_once()
