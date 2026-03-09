# SPDX-FileCopyrightText: 2026 Mappalink
#
# SPDX-License-Identifier: MIT

"""Native goal tracking for Omron ARCL — publishes mission_tracking key-values.

Monitors ARCL status fields (GoalName, HasArrived, DistToGoal, Status) to detect
when the robot is navigating to a goal and reports progress to InOrbit so that
goals show up in the Missions view.
"""

import logging
import time

logger = logging.getLogger(__name__)

# ARCL Status values that indicate the robot is actively navigating
_NAVIGATING_STATUSES = frozenset({
    "Going to goal",
    "Driving to goal",
    "Going to point",
})

# ARCL Status values that indicate an idle/arrived state
_IDLE_STATUSES = frozenset({
    "Arrived at",
    "Stopping",
    "Stopped",
    "Parked",
    "Idle",
    "Failed to get to",
    "Failed going to",
})


class GoalTracker:
    """Track ARCL goto/gotopoint goals and build mission_tracking payloads.

    Usage::

        tracker = GoalTracker()

        # When a goto command is dispatched:
        tracker.on_goal_dispatched("WS1")

        # Each telemetry cycle, pass the ARCL status dict:
        payload = tracker.update(status)
        if payload is not None:
            session.publish_key_values(mission_tracking=payload, is_event=True)
    """

    def __init__(self) -> None:
        self._mission_id: str | None = None
        self._goal_label: str | None = None
        self._start_ts: float = 0.0
        self._initial_distance: float | None = None
        self._last_reported: dict | None = None

    @property
    def is_active(self) -> bool:
        return self._mission_id is not None

    def on_goal_dispatched(self, goal_label: str) -> None:
        """Call when a goto or gotopoint command is sent to ARCL."""
        self._mission_id = f"omron-goal-{int(time.time())}"
        self._goal_label = goal_label
        self._start_ts = time.time()
        self._initial_distance = None
        self._last_reported = None
        logger.info("Goal tracking started: %s (id=%s)", goal_label, self._mission_id)

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
        return payload

    def update(self, arcl_status: dict) -> dict | None:
        """Process an ARCL status poll and return a mission_tracking payload if needed.

        Returns None if there is nothing new to report.
        """
        omron_status = arcl_status.get("Status", "")
        has_arrived = arcl_status.get("HasArrived", "0")
        goal_name = arcl_status.get("GoalName", "")
        dist_str = arcl_status.get("DistToGoal") or arcl_status.get("DistanceToGoal") or "0"

        try:
            distance_mm = float(dist_str)
        except (ValueError, TypeError):
            distance_mm = 0.0

        # Auto-detect navigation start if we missed the command dispatch
        if not self.is_active:
            if any(omron_status.startswith(s) for s in _NAVIGATING_STATUSES) and goal_name:
                self.on_goal_dispatched(goal_name)

        if not self.is_active:
            return None

        # Capture initial distance for progress estimation
        if self._initial_distance is None and distance_mm > 0:
            self._initial_distance = distance_mm

        # Check for completion
        if has_arrived == "1" or any(
            omron_status.startswith(s) for s in _IDLE_STATUSES
        ):
            failed = any(omron_status.startswith(s) for s in ("Failed",))
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

        # Still navigating — report progress
        completed = 0.0
        if self._initial_distance and self._initial_distance > 0:
            completed = max(0.0, min(1.0, 1.0 - distance_mm / self._initial_distance))

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
        payload = {
            "missionId": self._mission_id,
            "inProgress": in_progress,
            "state": state,
            "status": status,
            "label": f"Go to {self._goal_label}",
            "startTs": self._start_ts * 1000,
            "completedPercent": completed_percent,
            "data": {
                "Goal": self._goal_label,
                "Distance Remaining (m)": round(distance_mm / 1000.0, 2),
            },
        }
        if end_ts is not None:
            payload["endTs"] = end_ts * 1000
        return payload

    def _reset(self) -> None:
        logger.info("Goal tracking ended: %s (id=%s)", self._goal_label, self._mission_id)
        self._mission_id = None
        self._goal_label = None
        self._start_ts = 0.0
        self._initial_distance = None
        self._last_reported = None
