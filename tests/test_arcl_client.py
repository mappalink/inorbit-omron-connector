# SPDX-FileCopyrightText: 2026 Mappalink
#
# SPDX-License-Identifier: MIT

"""Tests for inorbit_omron_connector.src.arcl_client."""

from __future__ import annotations

import asyncio

import pytest
import pytest_asyncio

from inorbit_omron_connector.src.arcl_client import ArclClient, CommandType


class MockArclServer:
    """Minimal mock ARCL server for testing."""

    def __init__(self, host="127.0.0.1", port=0, password="testpass"):
        self.host = host
        self.port = port
        self.password = password
        self.server = None
        self.received_commands: list[str] = []
        self._clients: list[asyncio.StreamWriter] = []
        # Map of command prefix -> response lines
        self.responses: dict[str, list[str]] = {}

    async def start(self):
        self.server = await asyncio.start_server(self._handle_client, self.host, self.port)
        # Get the actual port (if port=0 was used)
        self.port = self.server.sockets[0].getsockname()[1]
        return self

    async def stop(self):
        if self.server:
            self.server.close()
            await self.server.wait_closed()
        for writer in self._clients:
            try:
                writer.close()
            except Exception:
                pass
        self._clients.clear()

    async def _handle_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        self._clients.append(writer)
        try:
            # Handshake
            writer.write(b"Welcome to Omron ARCL\r\nEnter password:")
            await writer.drain()

            password_line = await reader.readline()
            if not password_line:
                return
            if password_line.decode().strip() != self.password:
                writer.close()
                return

            writer.write(b"End of commands\r\n")
            await writer.drain()

            # Command loop
            while True:
                data = await reader.readline()
                if not data:
                    break
                cmd = data.decode().strip()
                self.received_commands.append(cmd)

                # Send configured responses
                cmd_word = cmd.split()[0].lower() if cmd else ""
                if cmd_word in self.responses:
                    for line in self.responses[cmd_word]:
                        writer.write(f"{line}\r\n".encode())
                    await writer.drain()

        except Exception:
            pass
        finally:
            if writer in self._clients:
                self._clients.remove(writer)
            try:
                writer.close()
            except Exception:
                pass


@pytest_asyncio.fixture
async def mock_server():
    server = MockArclServer()
    await server.start()
    yield server
    await server.stop()


@pytest_asyncio.fixture
async def connected_client(mock_server):
    client = ArclClient(
        host=mock_server.host,
        port=mock_server.port,
        password=mock_server.password,
        connection_timeout=5,
        reconnect_interval=1,
    )
    await client.connect()
    await client.wait_for_connection(timeout=5)
    yield client
    await client.disconnect()


class TestConnection:
    @pytest.mark.asyncio
    async def test_connects_and_authenticates(self, mock_server):
        client = ArclClient(
            host=mock_server.host,
            port=mock_server.port,
            password=mock_server.password,
        )
        await client.connect()
        await client.wait_for_connection(timeout=5)
        assert client.is_connected()
        await client.disconnect()
        assert not client.is_connected()

    @pytest.mark.asyncio
    async def test_wrong_password_does_not_connect(self):
        server = MockArclServer(password="correct")
        await server.start()
        try:
            client = ArclClient(
                host=server.host,
                port=server.port,
                password="wrong",
                connection_timeout=2,
                reconnect_interval=0.5,
            )
            await client.connect()
            # Should not become connected — wait briefly and check
            with pytest.raises(asyncio.TimeoutError):
                await client.wait_for_connection(timeout=2)
        finally:
            await client.disconnect()
            await server.stop()


class TestCommandQueue:
    @pytest.mark.asyncio
    async def test_sends_command(self, connected_client, mock_server):
        await connected_client._enqueue_command(CommandType.GENERIC, "test_cmd\r\n")
        # Give the write loop time to process
        await asyncio.sleep(0.2)
        assert "test_cmd" in mock_server.received_commands

    @pytest.mark.asyncio
    async def test_set_block_removes_go_and_clear(self, connected_client):
        """SET_BLOCK should remove pending CLEAR_BLOCK and GO commands."""
        async with connected_client._queue_lock:
            connected_client._queue.append((CommandType.GO, "go\r\n"))
            connected_client._queue.append((CommandType.CLEAR_BLOCK, "abdc test\r\n"))

        await connected_client._enqueue_command(CommandType.SET_BLOCK, 'abds "t" "t" "t"\r\n')

        async with connected_client._queue_lock:
            types = [item[0] for item in connected_client._queue]
        assert CommandType.GO not in types
        assert CommandType.CLEAR_BLOCK not in types
        assert CommandType.SET_BLOCK in types

    @pytest.mark.asyncio
    async def test_high_level_dock_undock(self, connected_client, mock_server):
        await connected_client.dock()
        await connected_client.undock()
        await asyncio.sleep(0.3)
        assert "dock" in mock_server.received_commands
        assert "undock" in mock_server.received_commands

    @pytest.mark.asyncio
    async def test_goto_goal(self, connected_client, mock_server):
        await connected_client.goto("charger1")
        await asyncio.sleep(0.2)
        assert "goto charger1" in mock_server.received_commands

    @pytest.mark.asyncio
    async def test_gotopoint(self, connected_client, mock_server):
        await connected_client.gotopoint(1000, 2000, 90)
        await asyncio.sleep(0.2)
        assert "gotopoint 1000 2000 90" in mock_server.received_commands

    @pytest.mark.asyncio
    async def test_stop(self, connected_client, mock_server):
        await connected_client.stop()
        await asyncio.sleep(0.2)
        assert "stop" in mock_server.received_commands


class TestQueryStatus:
    @pytest.mark.asyncio
    async def test_parses_status_response(self, mock_server):
        mock_server.responses["status"] = [
            "ExtendedStatusForHumans: Stopped",
            "Status: Stopped",
            "DockingState: Undocked",
            "StateOfCharge: 86.0",
            "Location: 36467 13659 93",
            "Temperature: 40",
            "LocalizationScore: 0.371681",
        ]

        client = ArclClient(
            host=mock_server.host,
            port=mock_server.port,
            password=mock_server.password,
        )
        await client.connect()
        await client.wait_for_connection(timeout=5)

        try:
            status = await client.query_status()
            assert status["DockingState"] == "Undocked"
            assert status["StateOfCharge"] == "86.0"
            assert status["Location"] == "36467 13659 93"
            assert status["Temperature"] == "40"
            assert status["LocalizationScore"] == "0.371681"
        finally:
            await client.disconnect()


class TestQueryGoals:
    @pytest.mark.asyncio
    async def test_parses_goals_response(self, mock_server):
        mock_server.responses["getgoals"] = [
            'Goal: "charger1"',
            'Goal: "pickup_zone"',
            'Goal: "dropoff_zone"',
            "End of goals",
        ]

        client = ArclClient(
            host=mock_server.host,
            port=mock_server.port,
            password=mock_server.password,
        )
        await client.connect()
        await client.wait_for_connection(timeout=5)

        try:
            goals = await client.query_goals()
            assert goals == ["charger1", "pickup_zone", "dropoff_zone"]
        finally:
            await client.disconnect()


class TestStatusParsing:
    """Unit tests for status parsing logic (no server needed)."""

    def test_parse_status_lines(self):
        """Test the parsing logic of query_status without a real connection."""
        lines = [
            "ExtendedStatusForHumans: Stopped",
            "Status: Stopped",
            "DockingState: Undocked",
            "StateOfCharge: 86.0",
            "Location: 36467 13659 93",
            "Temperature: 40",
        ]
        # Replicate the parsing logic from query_status
        result = {}
        for line in lines:
            colon_idx = line.find(":")
            if colon_idx == -1:
                continue
            key = line[:colon_idx].strip()
            value = line[colon_idx + 1 :].strip()
            result[key] = value

        assert result["DockingState"] == "Undocked"
        assert result["StateOfCharge"] == "86.0"
        assert result["Location"] == "36467 13659 93"
        assert result["Temperature"] == "40"
        assert result["Status"] == "Stopped"

    def test_parse_odometer_lines(self):
        lines = [
            "Odometer: Velocity: 0",
            "Odometer: LeftVelocity: 150",
            "Odometer: RightVelocity: 148",
        ]
        result = {}
        for line in lines:
            if not line.startswith("Odometer:"):
                continue
            rest = line[len("Odometer:") :].strip()
            colon_idx = rest.find(":")
            if colon_idx == -1:
                continue
            key = rest[:colon_idx].strip()
            value = rest[colon_idx + 1 :].strip()
            result[key] = value

        assert result["Velocity"] == "0"
        assert result["LeftVelocity"] == "150"
        assert result["RightVelocity"] == "148"

    def test_parse_goal_lines(self):
        lines = [
            'Goal: "charger1"',
            'Goal: "pickup_zone"',
        ]
        goals = []
        for line in lines:
            if line.startswith("Goal:"):
                name = line[len("Goal:") :].strip().strip('"')
                goals.append(name)
        assert goals == ["charger1", "pickup_zone"]

    def test_parse_laser_scan_line(self):
        """Test parsing rangeDeviceGetCurrent response into (x_mm, y_mm) points."""
        line = "RangeDeviceGetCurrent: Laser_1 1000 2000 -500 300 3000 4000"
        points = []
        if line.startswith("RangeDeviceGetCurrent"):
            parts = line.split()
            values = parts[2:]
            for i in range(0, len(values) - 1, 2):
                x = float(values[i])
                y = float(values[i + 1])
                points.append((x, y))
        assert points == [(1000.0, 2000.0), (-500.0, 300.0), (3000.0, 4000.0)]

    def test_parse_laser_scan_empty(self):
        """Empty scan returns no points."""
        line = "RangeDeviceGetCurrent: Laser_1"
        parts = line.split()
        values = parts[2:]
        assert len(values) == 0


class TestQueryLaserScan:
    @pytest.mark.asyncio
    async def test_queries_laser_scan(self, mock_server):
        mock_server.responses["rangedevicegetcurrent"] = [
            "RangeDeviceGetCurrent: Laser_1 1000 2000 3000 4000",
        ]

        client = ArclClient(
            host=mock_server.host,
            port=mock_server.port,
            password=mock_server.password,
        )
        await client.connect()
        await client.wait_for_connection(timeout=5)

        try:
            points = await client.query_laser_scan("Laser_1")
            assert points == [(1000.0, 2000.0), (3000.0, 4000.0)]
            assert "rangeDeviceGetCurrent Laser_1" in mock_server.received_commands
        finally:
            await client.disconnect()
