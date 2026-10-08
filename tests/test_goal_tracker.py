# SPDX-FileCopyrightText: 2026 Mappalink
#
# SPDX-License-Identifier: MIT

"""Tests for inorbit_omron_connector.src.goal_tracker.

Status strings are the ones the HD1500 sent on 2026-09-16 (fm-fsm-docs,
scripts/omron/captures/): the goal or macro name is part of the Status line,
and the status reply has no GoalName / HasArrived / DistToGoal fields.
"""

from __future__ import annotations

import pytest

from inorbit_omron_connector.src import goal_tracker
from inorbit_omron_connector.src.goal_tracker import GoalTracker


def status(text: str) -> dict:
    return {"ExtendedStatusForHumans": text, "Status": text}


@pytest.fixture
def clock(monkeypatch):
    """A settable wall clock for the tracker."""

    class Clock:
        now = 1_789_550_000.0

    monkeypatch.setattr(goal_tracker.time, "time", lambda: Clock.now)
    return Clock


class TestForeignGoals:
    """Goals this connector did not send (MobilePlanner, another ARCL client)."""

    def test_goal_is_tracked_from_the_status_alone(self):
        tracker = GoalTracker()

        payload = tracker.update(status("Going to Pickup_WS2_2"))

        assert payload["inProgress"] is True
        assert payload["state"] == "Executing"
        assert payload["label"] == "Go to Pickup_WS2_2"
        assert payload["missionId"].startswith("omron-goal-")

    def test_arrival_reports_done(self):
        tracker = GoalTracker()
        started = tracker.update(status("Going to Pickup_WS2_2"))

        payload = tracker.update(status("Arrived at Pickup_WS2_2"))

        assert payload["missionId"] == started["missionId"]
        assert payload["inProgress"] is False
        assert payload["state"] == "Done"
        assert payload["status"] == "OK"
        assert not tracker.is_active

    def test_timestamps_are_whole_milliseconds(self, clock):
        """InOrbit only lists the mission when startTs / endTs are integers.

        On 2026-10-08 a Go to warehouse3 reached the robot's mission_tracking
        attribute with startTs 1791449794784.9387 and never appeared in the
        mission list; the MiR connector's integer timestamps do.
        """
        clock.now = 1_789_550_000.4387
        tracker = GoalTracker()
        started = tracker.update(status("Going to warehouse3"))
        clock.now += 36.335

        done = tracker.update(status("Arrived at warehouse3"))

        assert type(started["startTs"]) is int
        assert started["startTs"] == 1_789_550_000_438
        assert type(done["endTs"]) is int
        assert done["endTs"] == 1_789_550_036_773

    def test_failed_goal_reports_aborted(self):
        tracker = GoalTracker()
        tracker.update(status("Going to Pickup_WS2_2"))

        payload = tracker.update(status("Error: Failed going to goal"))

        assert payload["inProgress"] is False
        assert payload["state"] == "Aborted"
        assert payload["status"] == "error"

    def test_goto_point_is_tracked(self):
        tracker = GoalTracker()

        payload = tracker.update(status("Going to point 25972 9078 90"))

        assert payload["label"] == "Go to point 25972 9078 90"


class TestDispatchedGoals:
    """Goals sent through this connector (on_goal_dispatched)."""

    def test_goal_is_not_closed_by_the_status_from_before_the_command(self, clock):
        """The robot still reports the previous state for a moment after a goto."""
        tracker = GoalTracker()
        tracker.on_goal_dispatched("Dropoff_WS1_1")

        clock.now += 1
        assert tracker.update(status("Stopped")) is None
        assert tracker.is_active

        clock.now += 1
        payload = tracker.update(status("Going to Dropoff_WS1_1"))
        assert payload["state"] == "Executing"
        assert payload["label"] == "Go to Dropoff_WS1_1"

    def test_goal_that_never_starts_is_aborted_after_the_grace(self, clock):
        """An EStopped robot ignores the goto and never starts driving."""
        tracker = GoalTracker()
        tracker.on_goal_dispatched("Dropoff_WS1_1")

        clock.now += goal_tracker._START_GRACE_SECS + 1
        payload = tracker.update(status("EStop pressed"))

        assert payload["state"] == "Aborted"
        assert payload["status"] == "error"
        assert not tracker.is_active

    def test_goal_the_robot_already_stands_on_completes_at_once(self, clock):
        tracker = GoalTracker()
        tracker.on_goal_dispatched("WS1")

        clock.now += 1
        payload = tracker.update(status("Arrived at WS1"))

        assert payload["state"] == "Done"
        assert payload["status"] == "OK"

    def test_goal_paused_by_the_connector_is_reported_done(self, clock):
        """Decided 2026-09-30: a pause (block driving) shows as `Stopped`, and the
        resume re-sends the goal as a new mission. Reporting that `Stopped` as a
        failure would turn every paused goal into a failed mission."""
        tracker = GoalTracker()
        tracker.on_goal_dispatched("WS1")
        tracker.update(status("Going to WS1"))
        tracker.on_pause()

        payload = tracker.update(status("Stopped"))

        assert payload["state"] == "Done"
        assert payload["status"] == "OK"

    def test_goal_stopped_short_by_someone_else_is_aborted(self, clock):
        """2026-10-08: an E-stop during Go to warehouse1 ended in `Stopped`
        39 cm short of the goal and InOrbit listed the mission as Done. A stop
        this connector did not send leaves the goal unreached."""
        tracker = GoalTracker()
        tracker.on_goal_dispatched("warehouse1")
        tracker.update(status("Going to warehouse1"))
        tracker.update(status("EStop pressed"))

        payload = tracker.update(status("Stopped"))

        assert payload["state"] == "Aborted"
        assert payload["status"] == "error"
        assert not tracker.is_active

    def test_estop_keeps_the_goal_executing_until_the_robot_stops(self, clock):
        tracker = GoalTracker()
        tracker.on_goal_dispatched("warehouse1")
        tracker.update(status("Going to warehouse1"))

        payload = tracker.update(status("EStop pressed"))

        assert payload is None or payload["state"] == "Executing"
        assert tracker.is_active

    def test_pause_leaves_the_status_at_going_to(self, clock):
        """2026-10-08: during a 40 s pause (block driving) every status sample
        still read `Going to warehouse1`; the pause closes nothing by itself."""
        tracker = GoalTracker()
        tracker.on_goal_dispatched("warehouse1")
        tracker.update(status("Going to warehouse1"))
        tracker.on_pause()

        payload = tracker.update(status("Going to warehouse1"))

        assert payload is None or payload["state"] == "Executing"
        assert tracker.is_active

    def test_resume_closes_the_paused_goal_done_and_starts_a_new_one(self, clock):
        tracker = GoalTracker()
        first = tracker.on_goal_dispatched("warehouse1")
        started = tracker.update(status("Going to warehouse1"))
        tracker.on_pause()
        tracker.update(status("Going to warehouse1"))

        clock.now += 41.0
        closing = tracker.on_goal_dispatched("warehouse1")

        assert first is None
        assert closing["missionId"] == started["missionId"]
        assert closing["state"] == "Done"
        assert closing["status"] == "OK"
        assert tracker.is_active
        assert tracker.update(status("Going to warehouse1"))["missionId"] != started["missionId"]

    def test_new_goal_pre_empting_an_active_one_aborts_it(self, clock):
        tracker = GoalTracker()
        tracker.on_goal_dispatched("warehouse1")
        started = tracker.update(status("Going to warehouse1"))

        closing = tracker.on_goal_dispatched("warehouse3")

        assert closing["missionId"] == started["missionId"]
        assert closing["state"] == "Aborted"
        assert tracker.update(status("Going to warehouse3"))["label"] == "Go to warehouse3"

    def test_pause_flag_does_not_outlive_the_goal(self, clock):
        tracker = GoalTracker()
        tracker.on_goal_dispatched("WS1")
        tracker.update(status("Going to WS1"))
        tracker.on_pause()
        tracker.update(status("Stopped"))

        tracker.on_goal_dispatched("WS1")
        tracker.update(status("Going to WS1"))
        payload = tracker.update(status("Stopped"))

        assert payload["state"] == "Aborted"

    def test_stopped_goal_is_not_picked_up_again_from_a_stale_status(self, clock):
        tracker = GoalTracker()
        tracker.on_goal_dispatched("WS1")
        tracker.update(status("Going to WS1"))

        assert tracker.on_stop()["state"] == "Aborted"
        # The status read just before the stop took effect
        assert tracker.update(status("Going to WS1")) is None
        assert not tracker.is_active
        assert tracker.update(status("Stopped")) is None

        payload = tracker.update(status("Going to WS2"))
        assert payload["label"] == "Go to WS2"


class TestMacros:
    def test_macro_is_tracked_until_it_completes(self):
        tracker = GoalTracker()

        started = tracker.update(status("Executing macro PrecisionDriveTafel_LO"))
        assert started["inProgress"] is True
        assert started["label"] == "Macro PrecisionDriveTafel_LO"
        assert started["missionId"].startswith("omron-macro-")

        done = tracker.update(status("Completed macro PrecisionDriveTafel_LO"))
        assert done["missionId"] == started["missionId"]
        assert done["state"] == "Done"
        assert done["status"] == "OK"

    def test_failed_macro_reports_aborted(self):
        tracker = GoalTracker()
        tracker.update(status("Executing macro PrecisionDriveTafel_LO"))

        payload = tracker.update(status("Error: Failed to drive to Target"))

        assert payload["state"] == "Aborted"
        assert payload["status"] == "error"

    def test_interrupted_macro_reports_aborted(self):
        """A macro that ends in anything but `Completed macro` did not complete."""
        tracker = GoalTracker()
        tracker.update(status("Executing macro PrecisionDriveTafel_LO"))

        payload = tracker.update(status("Stopped"))

        assert payload["state"] == "Aborted"
        assert payload["status"] == "error"

    def test_latched_completed_status_starts_nothing(self):
        tracker = GoalTracker()
        tracker.update(status("Executing macro SFA_WS2_2_Off"))
        tracker.update(status("Completed macro SFA_WS2_2_Off"))

        assert tracker.update(status("Completed macro SFA_WS2_2_Off")) is None
        assert not tracker.is_active


class TestIncompleteStatus:
    def test_reply_without_a_status_line_changes_nothing(self):
        tracker = GoalTracker()
        tracker.update(status("Executing macro PrecisionDriveTafel_LO"))

        assert tracker.update({"StateOfCharge": "57.0"}) is None
        assert tracker.is_active
