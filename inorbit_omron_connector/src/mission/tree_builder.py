# SPDX-FileCopyrightText: 2026 Mappalink
#
# SPDX-License-Identifier: MIT

"""Tree builder for Omron ARCL missions with local execution."""

from __future__ import annotations

from inorbit_edge_executor.behavior_tree import (
    BehaviorTree,
    BehaviorTreeErrorHandler,
    BehaviorTreeSequential,
    DefaultTreeBuilder,
    MissionCompletedNode,
    MissionInProgressNode,
    MissionPausedNode,
)
from inorbit_edge_executor.inorbit import MissionStatus

from .behavior_tree import (
    ArclBehaviorTreeBuilderContext,
    ArclMissionAbortedNode,
    ArclNodeFromStepBuilder,
)


class ArclTreeBuilder(DefaultTreeBuilder):
    """Tree builder that uses ARCL-specific step nodes for local execution."""

    def __init__(self, **kwargs):
        super().__init__(step_builder_factory=ArclNodeFromStepBuilder, **kwargs)

    def build_tree_for_mission(self, context: ArclBehaviorTreeBuilderContext) -> BehaviorTree:
        mission = context.mission
        tree = BehaviorTreeSequential(label=f"mission {mission.id}")

        tree.add_node(MissionInProgressNode(context, label="mission start"))

        step_builder = ArclNodeFromStepBuilder(context)

        for ix, step in enumerate(mission.definition.steps):
            try:
                node = step.accept(step_builder)
            except Exception as e:
                raise RuntimeError(f"Error building step #{ix} [{step}]: {e}") from e
            if node:
                tree.add_node(node)

        tree.add_node(MissionCompletedNode(context, label="mission completed"))

        # Error handling
        on_error = BehaviorTreeSequential(label="error handlers")
        on_error.add_node(
            ArclMissionAbortedNode(context, status=MissionStatus.error, label="mission aborted")
        )

        on_cancel = BehaviorTreeSequential(label="cancel handlers")
        on_cancel.add_node(
            ArclMissionAbortedNode(context, status=MissionStatus.ok, label="mission cancelled")
        )

        on_pause = BehaviorTreeSequential(label="pause handlers")
        on_pause.add_node(MissionPausedNode(context, label="mission paused"))

        tree = BehaviorTreeErrorHandler(
            context,
            tree,
            on_error,
            on_cancel,
            on_pause,
            context.error_context,
            label=f"mission {mission.id}",
        )

        return tree
