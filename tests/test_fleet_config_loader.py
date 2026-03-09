# SPDX-FileCopyrightText: 2026 Mappalink
#
# SPDX-License-Identifier: MIT

"""Tests for inorbit_omron_connector.src.config.fleet_config_loader."""

from __future__ import annotations

import pytest

from inorbit_omron_connector.src.config.fleet_config_loader import get_robot_config


SAMPLE_YAML = """\
common:
  location_tz: Europe/Amsterdam
  connector_type: OmronARCL
  arcl_port: 7171
  arcl_password: secret

omron-1:
  robot_name: "Omron HD1500"
  arcl_host: "10.200.0.2"
  map_id: "map"

omron-2:
  robot_name: "Omron HD1500 #2"
  arcl_host: "10.200.0.3"
"""


def test_loads_robot(tmp_path):
    p = tmp_path / "fleet.yaml"
    p.write_text(SAMPLE_YAML)
    cfg = get_robot_config(str(p), "omron-1")

    assert cfg["robot_name"] == "Omron HD1500"
    assert cfg["connector_config"]["arcl_host"] == "10.200.0.2"
    assert cfg["connector_config"]["arcl_port"] == 7171
    assert cfg["connector_config"]["arcl_password"] == "secret"
    assert cfg["connector_config"]["map_id"] == "map"


def test_common_merge(tmp_path):
    p = tmp_path / "fleet.yaml"
    p.write_text(SAMPLE_YAML)
    cfg = get_robot_config(str(p), "omron-2")

    # Common fields merged in
    assert cfg["connector_config"]["arcl_password"] == "secret"
    assert cfg["connector_config"]["arcl_port"] == 7171
    # Per-robot field
    assert cfg["connector_config"]["arcl_host"] == "10.200.0.3"


def test_robot_override(tmp_path):
    yaml_content = """\
common:
  arcl_port: 7171
  arcl_password: common-pass

robot-a:
  arcl_host: "1.2.3.4"
  arcl_password: override-pass
"""
    p = tmp_path / "fleet.yaml"
    p.write_text(yaml_content)
    cfg = get_robot_config(str(p), "robot-a")
    assert cfg["connector_config"]["arcl_password"] == "override-pass"


def test_missing_robot(tmp_path):
    p = tmp_path / "fleet.yaml"
    p.write_text(SAMPLE_YAML)
    with pytest.raises(IndexError, match="not-here"):
        get_robot_config(str(p), "not-here")


def test_env_var_expansion(tmp_path, monkeypatch):
    monkeypatch.setenv("MY_PASSWORD", "expanded-pass")
    yaml_content = """\
common:
  arcl_password: "${MY_PASSWORD}"
robot-x:
  arcl_host: "1.2.3.4"
"""
    p = tmp_path / "fleet.yaml"
    p.write_text(yaml_content)
    cfg = get_robot_config(str(p), "robot-x")
    assert cfg["connector_config"]["arcl_password"] == "expanded-pass"
