# SPDX-FileCopyrightText: 2026 Mappalink
#
# SPDX-License-Identifier: MIT

"""Tests for inorbit_omron_connector.src.connector."""

from __future__ import annotations

import math

import pytest


class TestTelemetryConversion:
    """Test the coordinate conversion logic used in _execution_loop."""

    def test_mm_to_meters(self):
        """ARCL reports mm, connector publishes meters."""
        x_mm, y_mm = 36467, 13659
        x_m = x_mm / 1000.0
        y_m = y_mm / 1000.0
        assert x_m == pytest.approx(36.467)
        assert y_m == pytest.approx(13.659)

    def test_degrees_to_radians(self):
        """ARCL reports degrees, connector publishes radians."""
        theta_deg = 93
        theta_rad = math.radians(theta_deg)
        assert theta_rad == pytest.approx(1.6232, abs=1e-3)

    def test_zero_location(self):
        """Zero location converts correctly."""
        assert 0 / 1000.0 == 0.0
        assert math.radians(0) == 0.0

    def test_negative_coordinates(self):
        """Negative coordinates (robot behind origin) convert correctly."""
        x_mm, y_mm = -5000, -3000
        assert x_mm / 1000.0 == pytest.approx(-5.0)
        assert y_mm / 1000.0 == pytest.approx(-3.0)

    def test_360_degree_wrap(self):
        """360 degrees wraps to ~2*pi."""
        assert math.radians(360) == pytest.approx(2 * math.pi, abs=1e-6)


class TestNavGoalConversion:
    """Test the reverse conversion for NAV_GOAL (meters/rad -> mm/deg)."""

    def test_meters_to_mm(self):
        x_m, y_m = 36.467, 13.659
        assert int(x_m * 1000) == 36467
        assert int(y_m * 1000) == 13659

    def test_radians_to_degrees(self):
        theta_rad = math.radians(90)
        assert int(math.degrees(theta_rad)) == 90

    def test_fractional_truncation(self):
        """int() truncates toward zero, which is acceptable for mm precision."""
        x_m = 1.9999
        assert int(x_m * 1000) == 1999


class TestLocationParsing:
    """Test parsing the Location string from ARCL status."""

    def test_parse_location_string(self):
        location_str = "36467 13659 93"
        parts = location_str.split()
        x_m = float(parts[0]) / 1000.0
        y_m = float(parts[1]) / 1000.0
        yaw_rad = math.radians(float(parts[2]))

        assert x_m == pytest.approx(36.467)
        assert y_m == pytest.approx(13.659)
        assert yaw_rad == pytest.approx(math.radians(93))

    def test_parse_negative_location(self):
        location_str = "-5000 -3000 -45"
        parts = location_str.split()
        x_m = float(parts[0]) / 1000.0
        y_m = float(parts[1]) / 1000.0
        yaw_rad = math.radians(float(parts[2]))

        assert x_m == pytest.approx(-5.0)
        assert y_m == pytest.approx(-3.0)
        assert yaw_rad == pytest.approx(math.radians(-45))

    def test_empty_location_defaults(self):
        """When location is empty, defaults to 0,0,0."""
        location_str = ""
        x_m, y_m, yaw_rad = 0.0, 0.0, 0.0
        if location_str:
            parts = location_str.split()
            x_m = float(parts[0]) / 1000.0
            y_m = float(parts[1]) / 1000.0
            yaw_rad = math.radians(float(parts[2]))
        assert x_m == 0.0
        assert y_m == 0.0
        assert yaw_rad == 0.0


class TestKeyValueExtraction:
    """Test extracting key-values from ARCL status dict."""

    def test_battery_percent(self):
        status = {"StateOfCharge": "86.0"}
        assert float(status["StateOfCharge"]) == pytest.approx(86.0)

    def test_temperature(self):
        status = {"Temperature": "40"}
        assert float(status["Temperature"]) == pytest.approx(40.0)

    def test_docking_state(self):
        status = {"DockingState": "Undocked"}
        assert status["DockingState"] == "Undocked"

    def test_localization_score(self):
        status = {"LocalizationScore": "0.371681"}
        assert float(status["LocalizationScore"]) == pytest.approx(0.371681)


class TestStatusFieldMapping:
    """Test the ARCL → InOrbit key mapping used in _execution_loop."""

    def _build_kv(self, status: dict) -> dict:
        """Replicate the mapping logic from _execution_loop."""
        from inorbit_omron_connector.src.connector import (
            _ARCL_STATUS_MAP,
            _NUMERIC_FIELDS,
            _SKIP_FIELDS,
            _to_snake_case,
        )

        kv: dict = {}
        for arcl_key, value in status.items():
            if arcl_key in _SKIP_FIELDS:
                continue
            inorbit_key = _ARCL_STATUS_MAP.get(arcl_key, _to_snake_case(arcl_key))
            if inorbit_key in _NUMERIC_FIELDS:
                try:
                    kv[inorbit_key] = float(value)
                except (ValueError, TypeError):
                    kv[inorbit_key] = value
            else:
                kv[inorbit_key] = value
        return kv

    def test_known_fields_mapped(self):
        """Known ARCL fields map to standard InOrbit key names."""
        status = {
            "StateOfCharge": "86.0",
            "Temperature": "40",
            "DockingState": "Undocked",
            "LocalizationScore": "0.371681",
            "Status": "Stopped",
            "ExtendedStatusForHumans": "Stopped by user",
        }
        kv = self._build_kv(status)
        assert kv["battery_percent"] == pytest.approx(86.0)
        assert kv["temperature"] == pytest.approx(40.0)
        assert kv["docking_state"] == "Undocked"
        assert kv["localization_score"] == pytest.approx(0.371681)
        assert kv["omron_status"] == "Stopped"
        assert kv["omron_status_text"] == "Stopped by user"

    def test_numeric_fields_are_float(self):
        """Numeric fields are converted to float."""
        status = {"BatteryVoltage": "25.3", "Eta": "120"}
        kv = self._build_kv(status)
        assert kv["battery_voltage"] == pytest.approx(25.3)
        assert kv["eta"] == pytest.approx(120.0)

    def test_location_skipped(self):
        """Location is handled separately for pose, not as a key-value."""
        status = {"Location": "36467 13659 93", "Status": "Moving"}
        kv = self._build_kv(status)
        assert "location" not in kv
        assert "Location" not in kv
        assert kv["omron_status"] == "Moving"

    def test_unknown_fields_pass_through_as_snake_case(self):
        """ARCL fields not in the mapping are converted to snake_case."""
        status = {"SomeNewField": "hello", "AnotherValue": "42"}
        kv = self._build_kv(status)
        assert kv["some_new_field"] == "hello"
        assert kv["another_value"] == "42"

    def test_full_status_response(self):
        """A realistic full ARCL status response produces all expected keys."""
        status = {
            "ExtendedStatusForHumans": "Going to Goal1",
            "Status": "Moving",
            "DockingState": "Undocked",
            "ForcedState": "None",
            "ChargeState": "Not charging",
            "StateOfCharge": "72.5",
            "Location": "6189 89 179",
            "Temperature": "38",
            "LocalizationScore": "0.85",
            "BatteryVoltage": "25.1",
            "GoalName": "Goal1",
            "DistToGoal": "3500",
            "HasArrived": "false",
            "Eta": "15",
        }
        kv = self._build_kv(status)
        # All fields present except Location
        assert len(kv) == len(status) - 1
        assert kv["battery_percent"] == pytest.approx(72.5)
        assert kv["omron_status"] == "Moving"
        assert kv["omron_status_text"] == "Going to Goal1"
        assert kv["forced_state"] == "None"
        assert kv["charge_state"] == "Not charging"
        assert kv["goal_name"] == "Goal1"
        assert kv["distance_to_goal"] == pytest.approx(3500.0)
        assert kv["has_arrived"] == "false"
        assert kv["eta"] == pytest.approx(15.0)

    def test_non_numeric_value_in_numeric_field_kept_as_string(self):
        """If a numeric field has a non-parseable value, keep as string."""
        status = {"StateOfCharge": "N/A"}
        kv = self._build_kv(status)
        assert kv["battery_percent"] == "N/A"


class TestSnakeCaseConversion:
    """Test the CamelCase → snake_case helper."""

    def test_simple_camel(self):
        from inorbit_omron_connector.src.connector import _to_snake_case

        assert _to_snake_case("StateOfCharge") == "state_of_charge"
        assert _to_snake_case("DockingState") == "docking_state"
        assert _to_snake_case("ExtendedStatusForHumans") == "extended_status_for_humans"

    def test_already_lowercase(self):
        from inorbit_omron_connector.src.connector import _to_snake_case

        assert _to_snake_case("status") == "status"

    def test_acronym_like(self):
        from inorbit_omron_connector.src.connector import _to_snake_case

        assert _to_snake_case("DistToGoal") == "dist_to_goal"


class TestOdometerConversion:
    """Test velocity conversion from ARCL odometer (mm/s → m/s)."""

    def test_velocity_mm_to_mps(self):
        velocity_mm = 1500
        velocity_mps = velocity_mm / 1000.0
        assert velocity_mps == pytest.approx(1.5)

    def test_zero_velocity(self):
        assert float("0") / 1000.0 == 0.0

    def test_negative_velocity(self):
        """Reverse movement."""
        velocity_mps = float("-500") / 1000.0
        assert velocity_mps == pytest.approx(-0.5)


class TestCartesianToRanges:
    """Test conversion from ARCL Cartesian points to InOrbit polar ranges."""

    def _angle_to_bin(self, angle, angle_min, angle_max, n_points):
        """Helper to compute the expected bin index for an angle."""
        return int((angle - angle_min) / (angle_max - angle_min) * (n_points - 1))

    def test_basic_conversion(self):
        from inorbit_omron_connector.src.connector import cartesian_to_ranges

        # Robot at origin facing 0°, point at (1000, 0) mm → 0° angle, 1.0m
        points = [(1000.0, 0.0)]
        ranges = cartesian_to_ranges(
            points,
            0.0,
            0.0,
            0.0,
            angle_min=-math.pi,
            angle_max=math.pi,
            n_points=360,
            range_max=30.0,
        )
        mid = self._angle_to_bin(0, -math.pi, math.pi, 360)
        assert ranges[mid] == pytest.approx(1.0, abs=0.01)
        assert ranges[0] == math.inf
        assert ranges[mid + 50] == math.inf

    def test_empty_points(self):
        from inorbit_omron_connector.src.connector import cartesian_to_ranges

        ranges = cartesian_to_ranges(
            [],
            0.0,
            0.0,
            0.0,
            angle_min=-math.pi,
            angle_max=math.pi,
            n_points=100,
            range_max=30.0,
        )
        assert all(r == math.inf for r in ranges)
        assert len(ranges) == 100

    def test_out_of_range_filtered(self):
        from inorbit_omron_connector.src.connector import cartesian_to_ranges

        # Point at 50m, beyond range_max of 30m
        points = [(50000.0, 0.0)]
        ranges = cartesian_to_ranges(
            points,
            0.0,
            0.0,
            0.0,
            angle_min=-math.pi,
            angle_max=math.pi,
            n_points=360,
            range_max=30.0,
        )
        assert all(r == math.inf for r in ranges)

    def test_closest_point_kept(self):
        from inorbit_omron_connector.src.connector import cartesian_to_ranges

        # Two points at the same angle, different distances
        points = [(2000.0, 0.0), (1000.0, 0.0)]
        ranges = cartesian_to_ranges(
            points,
            0.0,
            0.0,
            0.0,
            angle_min=-math.pi,
            angle_max=math.pi,
            n_points=360,
            range_max=30.0,
        )
        mid = self._angle_to_bin(0, -math.pi, math.pi, 360)
        assert ranges[mid] == pytest.approx(1.0, abs=0.01)

    def test_multiple_angles(self):
        from inorbit_omron_connector.src.connector import cartesian_to_ranges

        # Robot at origin facing 0°, points at 0° and 90°
        points = [(1000.0, 0.0), (0.0, 2000.0)]
        ranges = cartesian_to_ranges(
            points,
            0.0,
            0.0,
            0.0,
            angle_min=-math.pi,
            angle_max=math.pi,
            n_points=360,
            range_max=30.0,
        )
        mid = self._angle_to_bin(0, -math.pi, math.pi, 360)
        quarter = self._angle_to_bin(math.pi / 2, -math.pi, math.pi, 360)
        assert ranges[mid] == pytest.approx(1.0, abs=0.01)
        assert ranges[quarter] == pytest.approx(2.0, abs=0.02)

    def test_world_frame_transform(self):
        from inorbit_omron_connector.src.connector import cartesian_to_ranges

        # Robot at (5000, 3000) mm facing 90° (pi/2).
        # World-frame point at (5000, 4000) → 1m directly ahead.
        points = [(5000.0, 4000.0)]
        ranges = cartesian_to_ranges(
            points,
            5000.0,
            3000.0,
            math.pi / 2,
            angle_min=-math.pi,
            angle_max=math.pi,
            n_points=360,
            range_max=30.0,
        )
        # Should be at 0° in robot-local frame (straight ahead), 1.0m
        mid = self._angle_to_bin(0, -math.pi, math.pi, 360)
        assert ranges[mid] == pytest.approx(1.0, abs=0.01)
