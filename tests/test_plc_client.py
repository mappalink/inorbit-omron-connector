# SPDX-FileCopyrightText: 2026 Mappalink
#
# SPDX-License-Identifier: MIT

"""Tests for the workbench PLC ADS client (GVL_HMI handshake)."""

from __future__ import annotations

import asyncio

import pyads
import pytest

from inorbit_omron_connector.src import plc_client
from inorbit_omron_connector.src.plc_client import PlcError, PlcState, TablePlc


class FakePlcConnection:
    """Stateful stand-in for pyads.Connection implementing GVL_HMI semantics.

    Enforces the spec's handshake rules: moves start on the rising edge of
    g_xExecuteMove, a new move is refused while the bit is still latched
    after done, and dropping the bit mid-move aborts the move.
    """

    STEP_MM = 100  # height progress per read of g_xDone

    def __init__(self, ams_net_id, port, ip):
        self.ams_net_id = ams_net_id
        self.ip = ip
        self.is_open = False
        self.vars = {
            "GVL_HMI.g_nTargetHeight": 0,
            "GVL_HMI.g_xExecuteMove": False,
            "GVL_HMI.g_xStop": False,
            "GVL_HMI.g_nCurrentHeight": 800,
            "GVL_HMI.g_xBusy": False,
            "GVL_HMI.g_xDone": False,
            "GVL_HMI.g_xError": False,
            "GVL_HMI.g_dwErrorCode": 0,
            "GVL_HMI.g_sStatus": "healthy",
            "GVL_HMI.g_xPositionValid": True,
        }
        self._moving = False
        self._released_since_done = True
        self.stop_pulses = 0
        self.error_on_read: pyads.ADSError | None = None
        self.fail_mid_move = False
        self.done_height_offset = 0  # simulate deadband mismatch
        self.sets_done = True  # False: behave like wb1, which never raises g_xDone
        self.stuck = False  # True: busy but the height stops changing

    def open(self):
        self.is_open = True

    def close(self):
        self.is_open = False

    def read_by_name(self, name):
        if self.error_on_read is not None:
            raise self.error_on_read
        if name == "GVL_HMI.g_xDone":
            self._advance()
        return self.vars[name]

    def write_by_name(self, name, value):
        prev = self.vars.get(name)
        self.vars[name] = value
        if name == "GVL_HMI.g_xExecuteMove":
            if value and not prev:
                # Rising edge — only accepted once released after last done
                if self._released_since_done:
                    self._moving = True
                    self._released_since_done = False
                    self.vars["GVL_HMI.g_xBusy"] = True
                    self.vars["GVL_HMI.g_xDone"] = False
            elif not value:
                if self._moving:
                    self._moving = False  # drop mid-move aborts
                    self.vars["GVL_HMI.g_xBusy"] = False
                self._released_since_done = True
        elif name == "GVL_HMI.g_xStop" and value:
            self.stop_pulses += 1
            self._moving = False
            self.vars["GVL_HMI.g_xBusy"] = False

    def _advance(self):
        if not self._moving:
            return
        if self.stuck:
            return
        if self.fail_mid_move:
            self.vars["GVL_HMI.g_xError"] = True
            self.vars["GVL_HMI.g_dwErrorCode"] = 0x4650
            self.vars["GVL_HMI.g_sStatus"] = "drive not enabled"
            self._moving = False
            return
        target = self.vars["GVL_HMI.g_nTargetHeight"]
        cur = self.vars["GVL_HMI.g_nCurrentHeight"]
        if abs(target - cur) <= self.STEP_MM:
            self.vars["GVL_HMI.g_nCurrentHeight"] = target + self.done_height_offset
            self.vars["GVL_HMI.g_xDone"] = self.sets_done
            self.vars["GVL_HMI.g_xBusy"] = False
            self._moving = False
        else:
            self.vars["GVL_HMI.g_nCurrentHeight"] = cur + (
                self.STEP_MM if target > cur else -self.STEP_MM
            )


@pytest.fixture()
def fake_plc(monkeypatch):
    holder: dict[str, FakePlcConnection] = {}

    def factory(ams_net_id, port, ip):
        conn = FakePlcConnection(ams_net_id, port, ip)
        holder["conn"] = conn
        return conn

    monkeypatch.setattr(plc_client.pyads, "Connection", factory)
    yield holder


def make_table(**kwargs) -> TablePlc:
    return TablePlc(ip="10.102.1.50", ams_net_id="10.102.1.50.1.1", **kwargs)


@pytest.mark.asyncio
@pytest.mark.usefixtures("_fast_asyncio_sleep")
async def test_move_to_height_completes_and_releases_execute(fake_plc):
    table = make_table()
    await table.move_to_height(1131, timeout_secs=30)
    conn = fake_plc["conn"]
    assert conn.vars["GVL_HMI.g_nCurrentHeight"] == 1131
    assert conn.vars["GVL_HMI.g_xExecuteMove"] is False  # released after done


@pytest.mark.asyncio
@pytest.mark.usefixtures("_fast_asyncio_sleep")
async def test_move_within_deadband_completes_immediately(fake_plc):
    # Fake PLC's current height starts at 800 — target equals it
    table = make_table(deadband_mm=10)
    await table.move_to_height(800, timeout_secs=30)


@pytest.mark.asyncio
@pytest.mark.usefixtures("_fast_asyncio_sleep")
async def test_done_with_height_mismatch_raises(fake_plc):
    table = make_table(deadband_mm=10)
    # First read creates the connection lazily; configure the offset after
    await table.read_state()
    fake_plc["conn"].done_height_offset = 25
    with pytest.raises(PlcError, match="expected 1131"):
        await table.move_to_height(1131, timeout_secs=30)
    assert fake_plc["conn"].vars["GVL_HMI.g_xExecuteMove"] is False


@pytest.mark.asyncio
@pytest.mark.usefixtures("_fast_asyncio_sleep")
async def test_latched_error_refuses_move_without_touching_execute(fake_plc):
    table = make_table()
    await table.read_state()
    conn = fake_plc["conn"]
    conn.vars["GVL_HMI.g_xError"] = True
    conn.vars["GVL_HMI.g_dwErrorCode"] = 7
    conn.vars["GVL_HMI.g_sStatus"] = "E-stop open"
    with pytest.raises(PlcError, match="clear it at the machine"):
        await table.move_to_height(1131, timeout_secs=30)
    assert conn.vars["GVL_HMI.g_xExecuteMove"] is False
    # g_xClearError must never be written
    assert "GVL_HMI.g_xClearError" not in conn.vars


@pytest.mark.asyncio
@pytest.mark.usefixtures("_fast_asyncio_sleep")
async def test_error_mid_move_raises_with_code_and_status(fake_plc):
    table = make_table()
    await table.read_state()
    fake_plc["conn"].fail_mid_move = True
    with pytest.raises(PlcError, match="drive not enabled"):
        await table.move_to_height(1131, timeout_secs=30)
    assert fake_plc["conn"].vars["GVL_HMI.g_xExecuteMove"] is False


@pytest.mark.asyncio
async def test_timeout_pulses_stop_and_releases_execute(fake_plc):
    table = make_table()
    await table.read_state()
    conn = fake_plc["conn"]
    conn.STEP_MM = 0  # never progresses
    with pytest.raises(PlcError, match="timed out"):
        await table.move_to_height(1131, timeout_secs=0.05)
    assert conn.stop_pulses >= 1
    assert conn.vars["GVL_HMI.g_xExecuteMove"] is False


@pytest.mark.asyncio
async def test_cancellation_releases_execute(fake_plc):
    table = make_table()
    await table.read_state()
    conn = fake_plc["conn"]
    conn.STEP_MM = 0  # move never completes

    task = asyncio.create_task(table.move_to_height(1131, timeout_secs=60))
    while not conn.vars["GVL_HMI.g_xExecuteMove"]:
        await asyncio.sleep(0.01)
    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await task
    # Release happens shielded; give it a moment
    for _ in range(50):
        if conn.vars["GVL_HMI.g_xExecuteMove"] is False:
            break
        await asyncio.sleep(0.01)
    assert conn.vars["GVL_HMI.g_xExecuteMove"] is False
    assert conn._moving is False  # drop aborted the move


@pytest.mark.asyncio
@pytest.mark.usefixtures("_fast_asyncio_sleep")
async def test_position_valid_check(fake_plc):
    table = make_table(check_position_valid=True)
    await table.read_state()
    fake_plc["conn"].vars["GVL_HMI.g_xPositionValid"] = False
    with pytest.raises(PlcError, match="not referenced"):
        await table.move_to_height(1131, timeout_secs=30)


@pytest.mark.asyncio
async def test_read_state_snapshot(fake_plc):
    table = make_table(check_position_valid=True)
    state = await table.read_state()
    assert state == PlcState(
        height_mm=800,
        busy=False,
        done=False,
        error=False,
        error_code=0,
        status_text="healthy",
        position_valid=True,
    )


@pytest.mark.asyncio
async def test_read_state_without_position_valid_flag(fake_plc):
    table = make_table(check_position_valid=False)
    state = await table.read_state()
    assert state.position_valid is None


@pytest.mark.asyncio
async def test_ads_error_wrapped_and_connection_closed(fake_plc):
    table = make_table()
    await table.read_state()
    conn = fake_plc["conn"]
    conn.error_on_read = pyads.ADSError(text="target machine not found")
    with pytest.raises(PlcError, match="ADS read failed"):
        await table.read_state()
    assert conn.is_open is False  # closed so next op reopens


@pytest.mark.asyncio
@pytest.mark.usefixtures("_fast_asyncio_sleep")
async def test_completes_without_done_once_idle_within_deadband(fake_plc, monkeypatch):
    # wb1 (2026-09-10): busy drops at the target but g_xDone never goes TRUE
    monkeypatch.setattr(plc_client, "_SETTLE_SECS", 0.05)
    table = make_table(deadband_mm=10)
    task = asyncio.ensure_future(table.move_to_height(1131, timeout_secs=30))
    await asyncio.sleep(0)
    fake_plc["conn"].sets_done = False
    await task
    conn = fake_plc["conn"]
    assert conn.vars["GVL_HMI.g_nCurrentHeight"] == 1131
    assert conn.vars["GVL_HMI.g_xDone"] is False
    assert conn.vars["GVL_HMI.g_xExecuteMove"] is False
    assert conn.stop_pulses == 0


@pytest.mark.asyncio
@pytest.mark.usefixtures("_fast_asyncio_sleep")
async def test_no_done_and_outside_deadband_times_out(fake_plc, monkeypatch):
    # Idle but short of the target: the fallback must not report success
    monkeypatch.setattr(plc_client, "_SETTLE_SECS", 0.01)
    table = make_table(deadband_mm=10)
    task = asyncio.ensure_future(table.move_to_height(1131, timeout_secs=0.3))
    await asyncio.sleep(0)
    fake_plc["conn"].sets_done = False
    fake_plc["conn"].done_height_offset = -50
    with pytest.raises(PlcError, match="timed out"):
        await task
    assert fake_plc["conn"].stop_pulses == 1
    assert fake_plc["conn"].vars["GVL_HMI.g_xExecuteMove"] is False


@pytest.mark.asyncio
@pytest.mark.usefixtures("_fast_asyncio_sleep")
async def test_stall_without_progress_stops_and_fails(fake_plc):
    table = make_table(stall_secs=0.1)
    task = asyncio.ensure_future(table.move_to_height(1131, timeout_secs=30))
    await asyncio.sleep(0)
    fake_plc["conn"].stuck = True
    with pytest.raises(PlcError, match="stalled"):
        await task
    assert fake_plc["conn"].stop_pulses == 1
    assert fake_plc["conn"].vars["GVL_HMI.g_xExecuteMove"] is False
    assert table.is_moving is False


@pytest.mark.asyncio
async def test_started_event_set_once_busy(fake_plc):
    table = make_table()
    started = asyncio.Event()
    await table.move_to_height(1131, timeout_secs=30, started=started)
    assert started.is_set()


@pytest.mark.asyncio
async def test_read_state_during_move_returns_move_snapshot(fake_plc):
    table = make_table(stall_secs=60)
    started = asyncio.Event()
    task = asyncio.ensure_future(table.move_to_height(1131, timeout_secs=30, started=started))
    await asyncio.wait_for(started.wait(), 2)
    fake_plc["conn"].stuck = True
    assert table.is_moving
    # Returns immediately from the move loop's snapshot, no lock wait
    state = await asyncio.wait_for(table.read_state(), 0.5)
    assert state.busy is True
    fake_plc["conn"].stuck = False
    await task
    assert table.is_moving is False
