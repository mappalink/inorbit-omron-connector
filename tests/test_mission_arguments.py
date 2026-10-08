# SPDX-FileCopyrightText: 2026 Mappalink
#
# SPDX-License-Identifier: MIT

"""Dispatch-time arguments ({_arguments: key}) resolved before native compile."""

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
    resolve_mission_arguments,
)


def make_context(arguments: dict) -> ArclBehaviorTreeBuilderContext:
    return ArclBehaviorTreeBuilderContext(
        arcl_client=MagicMock(),
        plc_tables={"wb1": AsyncMock()},
        plc_heights={"wb1": {"retracted": 800, "pickup": 1131}},
        shared_memory=MissionRuntimeSharedMemory(),
        mission=MagicMock(arguments=arguments),
        options=MissionRuntimeOptions(),
    )


def step(action_id: str, arguments: dict) -> MissionStepRunAction:
    return MissionStepRunAction(runAction={"actionId": action_id, "arguments": arguments})


def test_goto_goal_takes_the_goal_from_the_mission_arguments():
    builder = ArclNodeFromStepBuilder(make_context({"goal_name": "warehouse3"}))
    tree = step("goto_goal", {"goal_name": {"_arguments": "goal_name"}}).accept(builder)
    assert "warehouse3" in tree.label


def test_macro_takes_the_name_from_the_mission_arguments():
    builder = ArclNodeFromStepBuilder(make_context({"macro": "PickCell5"}))
    tree = step("execute_macro", {"macro_name": {"_arguments": "macro"}}).accept(builder)
    assert "PickCell5" in tree.label


def test_a_missing_mission_argument_names_the_key():
    builder = ArclNodeFromStepBuilder(make_context({}))
    with pytest.raises(RuntimeError, match="goal_name"):
        step("goto_goal", {"goal_name": {"_arguments": "goal_name"}}).accept(builder)


def test_literal_arguments_pass_through_unchanged():
    mission = MagicMock(arguments={"goal_name": "elsewhere"})
    assert resolve_mission_arguments({"goal_name": "Goal1", "n": 2}, mission) == {
        "goal_name": "Goal1",
        "n": 2,
    }


def test_other_operators_are_refused_at_build_time():
    with pytest.raises(RuntimeError, match="_data"):
        resolve_mission_arguments({"goal_name": {"_data": "x"}}, MagicMock(arguments={}))
