# SPDX-FileCopyrightText: 2026 Mappalink
#
# SPDX-License-Identifier: MIT

"""Configuration models for the Omron ARCL connector."""

from typing import Optional

from pydantic import BaseModel
from pydantic_settings import BaseSettings, SettingsConfigDict
from inorbit_connector.models import InorbitConnectorConfig


class TablePlcConfig(BaseModel):
    """One workbench lifting-column PLC (Beckhoff TwinCAT over ADS).

    See fm-fsm-docs/docs/omron/OMRON_PLC_INTERFACE_SPEC.md.
    """

    ip: str
    ams_net_id: str  # PLC AmsNetId (usually <ip>.1.1, confirm on the machine)
    heights: dict[str, int] = {}  # named heights in mm, e.g. retracted: 800, pickup: 1131


class OmronArclConnectorConfig(BaseSettings):
    """Omron ARCL-specific configuration."""

    model_config = SettingsConfigDict(
        env_prefix="INORBIT_OMRON_",
        env_ignore_empty=True,
        extra="allow",
    )

    arcl_host: str  # Robot IP (e.g. "10.200.0.2")
    arcl_port: int = 7171
    arcl_password: str  # ARCL password
    arcl_timeout: int = 10  # Connection timeout (seconds)
    arcl_reconnect_interval: int = 5  # Reconnect delay (seconds)
    poll_frequency: float = 1.0  # Hz
    map_id: str = "map"  # InOrbit map frame ID
    map_file: Optional[str] = None  # Path to map PNG on disk
    map_resolution: float = 0.05  # Meters per pixel
    map_origin_x: float = 0.0
    map_origin_y: float = 0.0
    laser_names: list[str] = []  # e.g. ["Laser_1", "Laser_2", "Laser_3"]
    laser_angle_min: float = -3.14159  # -pi (full 360°)
    laser_angle_max: float = 3.14159  # +pi
    laser_range_min: float = 0.05  # meters
    laser_range_max: float = 30.0  # meters
    laser_n_points: int = 720  # 360° at 0.5° resolution
    mission_database_file: Optional[str] = None  # SQLite path for edge-executor persistence
    # Workbench PLC integration (table transport missions)
    plc_tables: dict[str, TablePlcConfig] = {}  # keyed by table id
    # AmsNetId this connector declares; must equal the client AmsNetId in the
    # static ADS route on each PLC (ours: 10.102.0.18.1.1)
    plc_client_ams_net_id: Optional[str] = None
    plc_deadband_mm: int = 10  # match the PLC's deadband (value pending from FM)
    plc_move_timeout_secs: float = 60.0
    plc_check_position_valid: bool = False  # enable once g_xPositionValid exists on the PLC


class ConnectorConfig(InorbitConnectorConfig):
    """Full config: InOrbit base + Omron ARCL specifics."""

    connector_config: OmronArclConnectorConfig
