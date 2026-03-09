# SPDX-FileCopyrightText: 2026 Mappalink
#
# SPDX-License-Identifier: MIT

"""Tests for `inorbit_omron_connector.src.config.models`."""

from __future__ import annotations

import pytest

from inorbit_omron_connector.src.config.models import (
    ConnectorConfig,
    OmronArclConnectorConfig,
)


MINIMAL_OMRON_CONFIG = {
    "arcl_host": "10.200.0.2",
    "arcl_password": "test-password",
}


@pytest.fixture()
def base_config_data() -> dict:
    """Return a minimal, valid ConnectorConfig payload."""
    return {
        "connector_type": "OmronARCL",
        "connector_config": {
            "arcl_host": "10.200.0.2",
            "arcl_password": "test-password",
        },
    }


def test_valid_config_instantiates(base_config_data: dict) -> None:
    config = ConnectorConfig(**base_config_data)
    assert isinstance(config.connector_config, OmronArclConnectorConfig)


def test_arcl_defaults(base_config_data: dict) -> None:
    config = ConnectorConfig(**base_config_data)
    cfg = config.connector_config
    assert cfg.arcl_port == 7171
    assert cfg.arcl_timeout == 10
    assert cfg.arcl_reconnect_interval == 5
    assert cfg.poll_frequency == 1.0
    assert cfg.map_id == "map"
    assert cfg.map_file is None
    assert cfg.laser_names == []
    assert cfg.laser_n_points == 720


def test_missing_arcl_host_raises() -> None:
    with pytest.raises(Exception):
        ConnectorConfig(connector_config={"arcl_password": "pw"})


def test_missing_arcl_password_raises() -> None:
    with pytest.raises(Exception):
        ConnectorConfig(connector_config={"arcl_host": "10.0.0.1"})


def test_omron_config_reads_from_env(monkeypatch: pytest.MonkeyPatch) -> None:
    """Test that OmronArclConnectorConfig reads from INORBIT_OMRON_ env vars."""
    monkeypatch.setenv("INORBIT_OMRON_ARCL_HOST", "env-host")
    monkeypatch.setenv("INORBIT_OMRON_ARCL_PASSWORD", "env-pw")
    monkeypatch.setenv("INORBIT_OMRON_ARCL_PORT", "7272")

    config = OmronArclConnectorConfig(arcl_host="env-host", arcl_password="env-pw")
    assert config.arcl_host == "env-host"
    assert config.arcl_port == 7272


def test_laser_config_override() -> None:
    config = OmronArclConnectorConfig(
        arcl_host="10.0.0.1",
        arcl_password="pw",
        laser_names=["Laser_1", "Laser_2"],
        laser_n_points=360,
        laser_range_max=20.0,
    )
    assert config.laser_names == ["Laser_1", "Laser_2"]
    assert config.laser_n_points == 360
    assert config.laser_range_max == 20.0
