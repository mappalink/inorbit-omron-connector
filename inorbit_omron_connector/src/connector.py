# SPDX-FileCopyrightText: 2026 Mappalink
#
# SPDX-License-Identifier: MIT

"""Omron multi-robot connector for InOrbit."""

# Standard
from typing import override

# InOrbit
from inorbit_connector.connector import (
    CommandResultCode,
    FleetConnector,
)
from inorbit_connector.models import MapConfigTemp

# Local
from inorbit_omron_connector import __version__ as connector_version
from inorbit_omron_connector.src.config.models import OmronConnectorConfig


class OmronConnector(FleetConnector):
    """Connector between Omron and InOrbit.

    Inherits from FleetConnector and implements Omron-specific logic.
    """

    def __init__(self, config: OmronConnectorConfig) -> None:
        """Initialize the connector.

        Args:
            config: Omron connector configuration
        """
        super().__init__(
            config,
            register_user_scripts=True,
            create_user_scripts_dir=True,
            publish_connector_system_stats=True,
        )

        self._logger.info("Initialized Omron Connector")

    @override
    async def _connect(self) -> None:
        """Connect to Omron API and start polling."""
        self._logger.info("Connected to Omron API")

    @override
    async def _disconnect(self) -> None:
        """Disconnect from Omron API and stop polling."""
        self._logger.info("Disconnected from Omron API")

    @override
    async def _execution_loop(self) -> None:
        """Main execution loop - publish cached robot data to InOrbit."""
        for robot_id in self.robot_ids:
            self.publish_robot_key_values(robot_id, {
                "connector_version": connector_version,
            })
        self._logger.debug("Executing main execution loop")

    @override
    async def _inorbit_robot_command_handler(
        self, robot_id: str, command_name: str, args: list, options: dict
    ) -> None:
        """Handle InOrbit commands for a specific robot.

        Validation and parsing of command arguments is handled by the CommandModel classes.
        If the arguments are invalid, a CommandFailure will be raised and the error will be
        handled accordingly and logged as an error.

        Args:
            robot_id: Robot ID that received the command
            command_name: Name of the command
            args: Command arguments
            options: Command options including result_function
        """
        self._logger.debug(
            f"Received command '{command_name}' for robot '{robot_id}'\n"
            f"  Args: {args}\n"
            f"  Options: {options}"
        )

        # Call the result function to indicate success
        options["result_function"](CommandResultCode.SUCCESS)

    @override
    async def fetch_robot_map(
        self, robot_id: str, frame_id: str
    ) -> MapConfigTemp | None:
        """Fetch a map from the Omron API.

        This method is called automatically by the base class when a pose is published
        with a frame_id that doesn't have a pre-configured map.

        Args:
            robot_id: Robot ID requesting the map
            frame_id: Frame ID of the map to fetch

        Returns:
            MapConfigTemp with map data, or None if fetch fails
        """
        self._logger.info(f"Fetching map '{frame_id}' for robot '{robot_id}'")

        try:
            # Fetch the map

            return MapConfigTemp(
                image=bytes(),
                map_id=frame_id,
                map_label="",
                origin_x=0.0,
                origin_y=0.0,
                resolution=0.0,
            )

        except Exception as ex:
            self._logger.error(
                f"Failed to fetch map '{frame_id}' from Omron API: {ex}"
            )
            return None
