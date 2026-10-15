# SPDX-FileCopyrightText: 2026 Mappalink
#
# SPDX-License-Identifier: MIT

"""Tests for robot liveness: an Omron whose ARCL connection is down, or that
has not answered a status query for ``liveness_timeout`` seconds, is reported
offline to InOrbit instead of staying online with its last known values.

The edge SDK publishes a retained "online" status when the session connects
and InOrbit only asks ``_is_robot_online`` (get_state) once it already believes
the robot is offline, so the connector must push the change itself.
"""

from __future__ import annotations

import asyncio
import time
from unittest.mock import AsyncMock, MagicMock

import pytest

from inorbit_omron_connector.src.config.models import OmronArclConnectorConfig
from inorbit_omron_connector.src.connector import OmronArclConnector

STATUS = {"Location": "1000 2000 90", "Status": "Stopped", "StateOfCharge": "80"}


@pytest.fixture()
def clock(monkeypatch):
    now = {"t": 1000.0}
    monkeypatch.setattr(time, "monotonic", lambda: now["t"])
    return now


def _connector(*, connected: bool = True, status: dict | Exception = STATUS) -> OmronArclConnector:
    """A connector stub, bypassing __init__, with a scripted ARCL client."""
    conn = object.__new__(OmronArclConnector)
    conn._plc_tables = {}
    conn._arcl = MagicMock()
    conn._mission_executor = MagicMock(is_busy=AsyncMock(return_value=False))
    conn._laser_names = []
    conn._lasers_registered = True
    conn._goal_tracker_enabled = False
    conn._goal_tracker = MagicMock()
    conn._map_id = "map"
    conn._liveness_timeout = 60.0
    conn._last_status_at = None
    conn._robot_online = None  # unknown until ARCL answers
    conn._reported_online = None
    conn._offline_reasserted_at = None
    conn.publish_pose = MagicMock()
    conn.publish_key_values = MagicMock()
    conn.publish_odometry = MagicMock()
    conn._session = MagicMock()
    conn._get_session = MagicMock(return_value=conn._session)
    _arcl(conn, connected=connected, status=status)
    return conn


def _arcl(conn, *, connected: bool = True, status: dict | Exception = STATUS) -> None:
    conn._arcl.is_connected = MagicMock(return_value=connected)
    if isinstance(status, Exception):
        conn._arcl.query_status = AsyncMock(side_effect=status)
    else:
        conn._arcl.query_status = AsyncMock(return_value=status)
    conn._arcl.query_odometer = AsyncMock(return_value={})


def _tick(conn) -> None:
    asyncio.run(conn._execution_loop())


def _status_calls(conn) -> list[bool]:
    return [c.kwargs["online"] for c in conn._session._send_robot_status.call_args_list]


def test_answering_robot_is_online_and_needs_no_status_push(clock) -> None:
    conn = _connector()
    _tick(conn)
    _tick(conn)
    assert conn._is_robot_online() is True
    assert _status_calls(conn) == []
    conn.publish_key_values.assert_called()


def test_never_connected_robot_is_offline_after_timeout(clock) -> None:
    conn = _connector(connected=False)
    _tick(conn)
    assert _status_calls(conn) == []  # unknown yet, give it the timeout
    clock["t"] += 61.0
    _tick(conn)
    assert conn._is_robot_online() is False
    assert _status_calls(conn) == [False]


def test_short_connection_loss_keeps_robot_online(clock) -> None:
    conn = _connector()
    _tick(conn)
    _arcl(conn, connected=False)
    clock["t"] += 30.0
    _tick(conn)
    assert conn._is_robot_online() is True
    assert _status_calls(conn) == []


def test_long_connection_loss_is_reported_offline(clock) -> None:
    conn = _connector()
    _tick(conn)
    _arcl(conn, connected=False)
    clock["t"] += 61.0
    _tick(conn)
    assert conn._is_robot_online() is False
    assert _status_calls(conn) == [False]


def test_silent_socket_is_reported_offline(clock) -> None:
    # Connected but the status query returns nothing (half-open socket).
    conn = _connector()
    _tick(conn)
    _arcl(conn, status={})
    clock["t"] += 61.0
    _tick(conn)
    assert conn._is_robot_online() is False
    assert _status_calls(conn) == [False]


def test_failing_status_query_is_reported_offline(clock) -> None:
    conn = _connector()
    _tick(conn)
    _arcl(conn, status=ConnectionError("ARCL connection lost"))
    clock["t"] += 61.0
    _tick(conn)
    assert conn._is_robot_online() is False
    assert _status_calls(conn) == [False]


def test_robot_coming_back_is_reported_online(clock) -> None:
    conn = _connector(connected=False)
    _tick(conn)
    clock["t"] += 61.0
    _tick(conn)
    _arcl(conn, connected=True)
    _tick(conn)
    assert conn._is_robot_online() is True
    assert _status_calls(conn) == [False, True]


def test_offline_is_reasserted_periodically(clock) -> None:
    conn = _connector(connected=False)
    _tick(conn)
    clock["t"] += 61.0
    _tick(conn)
    clock["t"] += 10.0
    _tick(conn)
    assert _status_calls(conn) == [False]
    clock["t"] += 25.0
    _tick(conn)
    assert _status_calls(conn) == [False, False]


def _cfg(**kw) -> OmronArclConnectorConfig:
    return OmronArclConnectorConfig(arcl_host="10.0.0.2", arcl_password="x", **kw)


def test_liveness_timeout_defaults_to_a_minute() -> None:
    assert _cfg().liveness_timeout == 60.0


def test_liveness_timeout_must_be_positive() -> None:
    with pytest.raises(ValueError):
        _cfg(liveness_timeout=0)
