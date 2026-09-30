# SPDX-FileCopyrightText: 2026 Mappalink
#
# SPDX-License-Identifier: MIT

"""Native goal tracking for Omron ARCL — publishes mission_tracking key-values.

Monitors the ARCL Status line to detect when the robot is navigating to a goal
or running a macro and reports progress to InOrbit so that both show up in the
Missions view, whoever started them (this connector, MobilePlanner, another
ARCL client). The goal or macro name is part of the Status line itself
("Going to Pickup_WS2_2", "Executing macro PrecisionDriveTafel_LO").
"""

import logging
import time

logger = logging.getLogger(__name__)

# ARCL Status prefixes while the robot is working; the rest of the line is the name
_GOAL_ACTIVE_PREFIXES = ("Going to ", "Driving to ")
_MACRO_ACTIVE_PREFIX = "Executing macro "
_MACRO_DONE_PREFIX = "Completed macro "

# ARCL Status values that indicate an idle/arrived state
_IDLE_STATUSES = frozenset(
    {
        "Arrived at",
        "Stopping",
        "Stopped",
        "Parked",
        "Idle",
    }
)

# "Failed going to ...", "Error: Failed going to goal", "Error: Failed to drive to Target"
_FAILED_PREFIXES = ("Failed", "Error")

# After a goto the robot reports its previous state for a moment. A goal is
# only closed once it was seen driving; one that never starts is aborted after
# this many seconds (an EStopped robot ignores the command).
_START_GRACE_SECS = 10.0


def _name_after(status: str, prefixes: tuple[str, ...]) -> str | None:
    for prefix in prefixes:
        if status.startswith(prefix):
            return status[len(prefix) :].strip() or None
    return None


class GoalTracker:
    """Track ARCL goals and macros and build mission_tracking payloads.

    Usage::

        tracker = GoalTracker()

        # When a goto command is dispatched:
        tracker.on_goal_dispatched("WS1")

        # Each telemetry cycle, pass the ARCL status dict:
        payload = tracker.update(status)
        if payload is not None:
            session.publish_key_values(
                key_values={"mission_tracking": payload}, is_event=True
            )
    """

    def __init__(self) -> None:
        self._mission_id: str | None = None
        self._kind: str = "goal"  # "goal" or "macro"
        self._goal_label: str | None = None
        self._start_ts: float = 0.0
        self._seen_active: bool = False
        self._initial_distance: float | None = None
        self._last_reported: dict | None = None
        # After a stop, the next status may still show the stopped goal
        self._ignore_stale_active: bool = False

    @property
    def is_active(self) -> bool:
        return self._mission_id is not None

    def on_goal_dispatched(self, goal_label: str) -> None:
        """Call when a goto or gotopoint command is sent to ARCL."""
        self._start("goal", goal_label)

    def _start(self, kind: str, label: str) -> None:
        self._mission_id = f"omron-{kind}-{int(time.time())}"
        self._kind = kind
        self._goal_label = label
        self._start_ts = time.time()
        self._seen_active = False
        self._initial_distance = None
        self._last_reported = None
        self._ignore_stale_active = False
        logger.info("%s tracking started: %s (id=%s)", kind.capitalize(), label, self._mission_id)

    def on_stop(self) -> dict | None:
        """Call when a stop command is sent during navigation. Returns abort payload."""
        if not self.is_active:
            return None
        payload = self._build_payload(
            in_progress=False,
            state="Aborted",
            status="error",
            completed_percent=0.0,
        )
        self._reset()
        self._ignore_stale_active = True
        return payload

    def update(self, arcl_status: dict) -> dict | None:
        """Process an ARCL status poll and return a mission_tracking payload if needed.

        Returns None if there is nothing new to report.
        """
        omron_status = arcl_status.get("Status", "")
        if not omron_status:
            return None  # cut-short reply: no information

        goal_name = _name_after(omron_status, _GOAL_ACTIVE_PREFIXES)
        macro_name = _name_after(omron_status, (_MACRO_ACTIVE_PREFIX,))

        if self._ignore_stale_active:
            if goal_name or macro_name:
                return None
            self._ignore_stale_active = False

        # Auto-detect work this connector did not start (or did not announce)
        if not self.is_active:
            if goal_name:
                self._start("goal", goal_name)
            elif macro_name:
                self._start("macro", macro_name)

        if not self.is_active:
            return None

        if self._kind == "macro":
            return self._update_macro(omron_status)
        return self._update_goal(arcl_status, omron_status, navigating=goal_name is not None)

    def _update_macro(self, omron_status: str) -> dict | None:
        if omron_status.startswith(_MACRO_ACTIVE_PREFIX):
            return self._report_progress(0.0, 0.0)
        # The macro owned the Status line; whatever replaced it is its outcome,
        # and only "Completed macro" is a success.
        return self._finish(failed=not omron_status.startswith(_MACRO_DONE_PREFIX))

    def _update_goal(self, arcl_status: dict, omron_status: str, navigating: bool) -> dict | None:
        has_arrived = arcl_status.get("HasArrived", "0") == "1"
        dist_str = arcl_status.get("DistToGoal") or arcl_status.get("DistanceToGoal") or "0"

        try:
            distance_mm = float(dist_str)
        except (ValueError, TypeError):
            distance_mm = 0.0

        if navigating:
            self._seen_active = True

        # Capture initial distance for progress estimation
        if self._initial_distance is None and distance_mm > 0:
            self._initial_distance = distance_mm

        arrived_here = omron_status == f"Arrived at {self._goal_label}"
        started = self._seen_active or has_arrived or arrived_here
        if not navigating and not started:
            if time.time() - self._start_ts < _START_GRACE_SECS:
                return None  # still the status from before the command
            return self._finish(failed=True, distance_mm=distance_mm)

        # Check for completion
        failed = omron_status.startswith(_FAILED_PREFIXES)
        idle = any(omron_status.startswith(s) for s in _IDLE_STATUSES)
        if not navigating and (has_arrived or failed or idle):
            return self._finish(failed=failed, distance_mm=distance_mm)

        # Still navigating — report progress
        completed = 0.0
        if self._initial_distance and self._initial_distance > 0:
            completed = max(0.0, min(1.0, 1.0 - distance_mm / self._initial_distance))
        return self._report_progress(completed, distance_mm)

    def _finish(self, *, failed: bool, distance_mm: float = 0.0) -> dict:
        payload = self._build_payload(
            in_progress=False,
            state="Aborted" if failed else "Done",
            status="error" if failed else "OK",
            completed_percent=0.0 if failed else 1.0,
            end_ts=time.time(),
            distance_mm=distance_mm,
        )
        self._reset()
        return payload

    def _report_progress(self, completed: float, distance_mm: float) -> dict | None:
        payload = self._build_payload(
            in_progress=True,
            state="Executing",
            status="OK",
            completed_percent=completed,
            distance_mm=distance_mm,
        )

        # Deduplicate: only report if state or progress changed meaningfully
        if self._last_reported is not None:
            if (
                self._last_reported.get("state") == payload.get("state")
                and abs(
                    self._last_reported.get("completedPercent", 0)
                    - payload.get("completedPercent", 0)
                )
                < 0.05
            ):
                return None

        self._last_reported = payload
        return payload

    def _build_payload(
        self,
        *,
        in_progress: bool,
        state: str,
        status: str,
        completed_percent: float,
        end_ts: float | None = None,
        distance_mm: float = 0.0,
    ) -> dict:
        if self._kind == "macro":
            label = f"Macro {self._goal_label}"
            data = {"Macro": self._goal_label}
        else:
            label = f"Go to {self._goal_label}"
            data = {
                "Goal": self._goal_label,
                "Distance Remaining (m)": round(distance_mm / 1000.0, 2),
            }
        payload = {
            "missionId": self._mission_id,
            "inProgress": in_progress,
            "state": state,
            "status": status,
            "label": label,
            "startTs": self._start_ts * 1000,
            "completedPercent": completed_percent,
            "data": data,
        }
        if end_ts is not None:
            payload["endTs"] = end_ts * 1000
        return payload

    def _reset(self) -> None:
        logger.info(
            "%s tracking ended: %s (id=%s)",
            self._kind.capitalize(),
            self._goal_label,
            self._mission_id,
        )
        self._mission_id = None
        self._goal_label = None
        self._start_ts = 0.0
        self._seen_active = False
        self._initial_distance = None
        self._last_reported = None
