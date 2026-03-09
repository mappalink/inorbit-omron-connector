# SPDX-FileCopyrightText: 2026 Mappalink
#
# SPDX-License-Identifier: MIT

"""Tests for `inorbit_omron_connector.src.config.models`."""

from __future__ import annotations

import copy

import pytest

from inorbit_omron_connector.src.config.models import (
    OmronConfig,
    OmronConnectorConfig,
    OmronRobotConfig,
    CONNECTOR_TYPE,
)


REQUIRED_FLEET_CONFIG = {
    "fleet_host": "fleet.example.com",
    "fleet_port": 8080,
    "fleet_username": "test-user",
    "fleet_password": "test-pass",
}


@pytest.fixture()
def base_config_data() -> dict:
    """Return a minimal, valid OmronConnectorConfig payload."""

    return {
        "connector_type": "omron",
        "connector_config": {
            "fleet_host": "fleet.example.com",
            "fleet_port": 8080,
            "fleet_username": "dummy-user",
            "fleet_password": "dummy-pass",
        },
        "fleet": [
            {"robot_id": "robot-alpha", "fleet_robot_id": 101, "cameras": []},
            {"robot_id": "robot-beta", "fleet_robot_id": 102, "cameras": []},
        ],
    }


def test_connector_type_must_match(base_config_data: dict) -> None:
    config = OmronConnectorConfig(**base_config_data)
    assert config.connector_type == CONNECTOR_TYPE


def test_invalid_connector_type_raises(base_config_data: dict) -> None:
    data = copy.deepcopy(base_config_data)
    data["connector_type"] = f"not-{CONNECTOR_TYPE}"

    with pytest.raises(ValueError, match=f"Expected connector type '{CONNECTOR_TYPE}'"):
        OmronConnectorConfig(**data)


def test_unique_fleet_robot_ids_are_required(base_config_data: dict) -> None:
    data = copy.deepcopy(base_config_data)
    data["fleet"][1]["fleet_robot_id"] = data["fleet"][0]["fleet_robot_id"]

    with pytest.raises(ValueError, match="fleet_robot_id values must be unique"):
        OmronConnectorConfig(**data)


def test_valid_config_instantiates_models(base_config_data: dict) -> None:
    config = OmronConnectorConfig(**base_config_data)

    assert isinstance(config.connector_config, OmronConfig)
    assert all(isinstance(robot, OmronRobotConfig) for robot in config.fleet)


def test_omron_config_reads_from_environment_variables(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Test that OmronConfig reads missing fields from environment variables."""
    monkeypatch.setenv("INORBIT_OMRON_FLEET_HOST", "env-fleet.example.com")
    monkeypatch.setenv("INORBIT_OMRON_FLEET_PORT", "8443")
    monkeypatch.setenv("INORBIT_OMRON_FLEET_USERNAME", "env-user")
    monkeypatch.setenv("INORBIT_OMRON_FLEET_PASSWORD", "env-pass")

    # All required fields come from environment variables
    config = OmronConfig(
        fleet_host="env-fleet.example.com",
        fleet_username="env-user",
        fleet_password="env-pass",
    )

    assert config.fleet_host == "env-fleet.example.com"
    assert config.fleet_port == 8443
    assert config.fleet_username == "env-user"
    assert config.fleet_password == "env-pass"


def test_omron_config_prioritizes_yaml_over_environment(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Test that YAML values take precedence over environment variables."""
    monkeypatch.setenv("INORBIT_OMRON_FLEET_HOST", "env-fleet.example.com")
    monkeypatch.setenv("INORBIT_OMRON_FLEET_PORT", "8443")
    monkeypatch.setenv("INORBIT_OMRON_FLEET_USERNAME", "env-user")
    monkeypatch.setenv("INORBIT_OMRON_FLEET_PASSWORD", "env-pass")

    config = OmronConfig(
        fleet_host="yaml-fleet.example.com",
        fleet_port=8080,
        fleet_username="yaml-user",
        fleet_password="yaml-pass",
    )

    assert config.fleet_host == "yaml-fleet.example.com"
    assert config.fleet_port == 8080
    # YAML values take precedence
    assert config.fleet_username == "yaml-user"
    assert config.fleet_password == "yaml-pass"


def test_omron_config_uses_env_for_missing_fields(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Test that environment variables fill in missing YAML fields."""
    monkeypatch.setenv("INORBIT_OMRON_FLEET_USERNAME", "env-user")
    monkeypatch.setenv("INORBIT_OMRON_FLEET_PASSWORD", "env-pass")

    config = OmronConfig(
        fleet_host="yaml-fleet.example.com",
        fleet_port=8080,
        fleet_username="env-user",
        fleet_password="env-pass",
    )

    assert config.fleet_host == "yaml-fleet.example.com"
    assert config.fleet_port == 8080
    assert config.fleet_username == "env-user"
    assert config.fleet_password == "env-pass"


def test_omron_config_default_port() -> None:
    """Test default value for fleet_port."""
    config = OmronConfig(
        fleet_host="fleet.example.com",
        fleet_username="test-user",
        fleet_password="test-pass",
    )

    assert config.fleet_port == 80
