# SPDX-FileCopyrightText: 2026 Mappalink
#
# SPDX-License-Identifier: MIT

"""Fleet YAML loader — merges common + per-robot, nests Omron ARCL fields."""

import os
import logging
from copy import deepcopy
from typing import Any

import yaml

logger = logging.getLogger(__name__)

# Fields that belong under connector_config (Omron ARCL-specific)
OMRON_FIELDS = [
    "arcl_host",
    "arcl_port",
    "arcl_password",
    "arcl_timeout",
    "arcl_reconnect_interval",
    "poll_frequency",
    "map_id",
    "map_file",
    "map_resolution",
    "map_origin_x",
    "map_origin_y",
    "laser_names",
    "laser_angle_min",
    "laser_angle_max",
    "laser_range_min",
    "laser_range_max",
    "laser_n_points",
]


def get_robot_config(config_filename: str, robot_id: str) -> dict[str, Any]:
    """Load config for a single robot, merging common + per-robot sections."""
    with open(config_filename, "r", encoding="utf-8") as f:
        full_config = yaml.safe_load(f) or {}

    full_config = _expand_env_vars(full_config)

    if robot_id not in full_config:
        available = [k for k in full_config if k not in ("common",)]
        raise IndexError(f"Robot '{robot_id}' not found. Available: {available}")

    # Merge common + per-robot
    robot_config = deepcopy(full_config.get("common", {}))
    robot_config.update(full_config[robot_id])

    # Nest Omron-specific fields under connector_config
    if "connector_config" not in robot_config:
        robot_config["connector_config"] = {}
    for field in OMRON_FIELDS:
        if field in robot_config:
            robot_config["connector_config"][field] = robot_config.pop(field)

    return robot_config


def _expand_env_vars(obj: Any) -> Any:
    if isinstance(obj, dict):
        return {k: _expand_env_vars(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [_expand_env_vars(i) for i in obj]
    elif isinstance(obj, str):
        return os.path.expandvars(obj)
    return obj
