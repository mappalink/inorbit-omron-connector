# SPDX-FileCopyrightText: 2026 Mappalink
#
# SPDX-License-Identifier: MIT

"""Tests for the PlcLegsNode edge BT node and its step builder branch."""

from __future__ import annotations

from unittest.mock import AsyncMock, MagicMock

import pytest

from inorbit_edge_executor.datatypes import (
    MissionRuntimeOptions,
    MissionRuntimeSharedMemory,
    MissionStepRunAction,
)

from inorbit_omron_connector.src.mission.behavior_tree import (
    ArclBehaviorTreeBuilderContext,
    ArclNodeFromStepBuilder,
    PlcLegsNode,
    SharedMemoryKeys,
)
from inorbit_omron_connector.src.plc_client import PlcError


def make_context(**kwargs) -> ArclBehaviorTreeBuilderContext:
    defaults = dict(
        arcl_client=MagicMock(),
        plc_tables={"wb1": AsyncMock()},
        plc_heights={"wb1": {"retracted": 800, "pickup": 1131}},
        plc_move_timeout_secs=45.0,
        shared_memory=MissionRuntimeSharedMemory(),
        mission=MagicMock(arguments={}),
        options=MissionRuntimeOptions(),
    )
    defaults.update(kwargs)
    return ArclBehaviorTreeBuilderContext(**defaults)


def run_action_step(arguments: dict, label: str = "step") -> MissionStepRunAction:
    return MissionStepRunAction(
        label=label, runAction={"actionId": "plc_legs", "arguments": arguments}
    )


class TestBuildPlcLegs:
    def test_retract_resolves_configured_height(self):
        context = make_context()
        builder = ArclNodeFromStepBuilder(context)
        node = builder.visit_run_action(run_action_step({"table": "wb1", "action": "retract"}))
        assert isinstance(node, PlcLegsNode)
        assert node._table_id == "wb1"
        assert node._target_mm == 800

    def test_extend_resolves_pickup_height(self):
        builder = ArclNodeFromStepBuilder(make_context())
        node = builder.visit_run_action(run_action_step({"table": "wb1", "action": "extend"}))
        assert node._target_mm == 1131

    def test_explicit_height_overrides_action(self):
        builder = ArclNodeFromStepBuilder(make_context())
        node = builder.visit_run_action(run_action_step({"table": "wb1", "height_mm": 950}))
        assert node._target_mm == 950

    def test_cloud_prefixed_arguments_accepted(self):
        builder = ArclNodeFromStepBuilder(make_context())
        node = builder.visit_run_action(run_action_step({"--table": "wb1", "--action": "retract"}))
        assert node._target_mm == 800

    def test_unknown_table_fails_at_build_time(self):
        builder = ArclNodeFromStepBuilder(make_context())
        with pytest.raises(RuntimeError, match="unknown table 'nope'"):
            builder.visit_run_action(run_action_step({"table": "nope", "action": "retract"}))

    def test_missing_height_config_fails_at_build_time(self):
        context = make_context(plc_heights={"wb1": {}})
        builder = ArclNodeFromStepBuilder(context)
        with pytest.raises(RuntimeError, match="no 'retracted' height configured"):
            builder.visit_run_action(run_action_step({"table": "wb1", "action": "retract"}))

    def test_missing_action_and_height_fails_at_build_time(self):
        builder = ArclNodeFromStepBuilder(make_context())
        with pytest.raises(RuntimeError, match="retract|extend"):
            builder.visit_run_action(run_action_step({"table": "wb1"}))


class TestPlcLegsNodeExecute:
    @pytest.mark.asyncio
    async def test_moves_and_completes(self):
        context = make_context()
        node = PlcLegsNode(context, table_id="wb1", target_mm=1131)
        await node._execute()
        context.plc_tables["wb1"].move_to_height.assert_awaited_once_with(1131, timeout_secs=45.0)

    @pytest.mark.asyncio
    async def test_plc_error_sets_shared_memory_and_raises(self):
        context = make_context()
        context.plc_tables["wb1"].move_to_height.side_effect = PlcError("E-stop open")
        node = PlcLegsNode(context, table_id="wb1", target_mm=1131)
        context.shared_memory.freeze()
        with pytest.raises(RuntimeError, match="E-stop open"):
            await node._execute()
        assert "E-stop open" in context.shared_memory.get(SharedMemoryKeys.ARCL_ERROR_MESSAGE)

    @pytest.mark.asyncio
    async def test_unconfigured_table_raises(self):
        context = make_context(plc_tables={})
        node = PlcLegsNode(context, table_id="wb1", target_mm=800)
        context.shared_memory.freeze()
        with pytest.raises(RuntimeError, match="no PLC configured"):
            await node._execute()

    def test_serialization_round_trip(self):
        context = make_context()
        node = PlcLegsNode(context, table_id="wb1", target_mm=800, label="PLC legs retract (wb1)")
        dumped = node.dump_object()
        assert dumped["table_id"] == "wb1"
        assert dumped["target_mm"] == 800
        restored = PlcLegsNode.from_object(
            context, table_id=dumped["table_id"], target_mm=dumped["target_mm"]
        )
        assert restored._table_id == "wb1"
        assert restored._target_mm == 800


def test_button_action_id_runs_locally_and_waits():
    """omron-plc-legs in a mission must block like plc_legs, not take the
    button's report-on-start cloud path."""
    builder = ArclNodeFromStepBuilder(make_context())
    step = MissionStepRunAction(
        label="legs",
        runAction={
            "actionId": "omron-plc-legs",
            "arguments": {"--table": "wb1", "--action": "retract"},
        },
    )
    node = builder.visit_run_action(step)
    assert isinstance(node, PlcLegsNode)
    assert node._target_mm == 800
