# SPDX-FileCopyrightText: 2026 Mappalink
#
# SPDX-License-Identifier: MIT

"""ADS client for the Beckhoff lifting-column PLC on FM workbenches.

Talks to the TwinCAT PLC over ADS (pyads) using the flat ``GVL_HMI``
variable interface. The PLC owns the state machine, retries and safety
interlocks; this client only sets intent and waits on status.

Contract: fm-fsm-docs/docs/omron/OMRON_PLC_INTERFACE_SPEC.md (2026-08-18).
Handshake rules implemented here:
  - g_xExecuteMove is edge-triggered and must be released after g_xDone;
    the PLC refuses the next move while it is latched.
  - Dropping g_xExecuteMove mid-move aborts the move. This is our abort
    path: move_to_height() releases the bit on cancellation/timeout.
  - g_xClearError is never written. E-stop/error recovery is a deliberate
    human action at the machine; we abort and surface the condition.
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
import threading
from dataclasses import dataclass

import pyads

logger = logging.getLogger(__name__)

PLC_RUNTIME_PORT = 851  # TwinCAT 3 PLC

_PREFIX = "GVL_HMI."
_VAR_TARGET_HEIGHT = _PREFIX + "g_nTargetHeight"
_VAR_EXECUTE_MOVE = _PREFIX + "g_xExecuteMove"
_VAR_STOP = _PREFIX + "g_xStop"
_VAR_CURRENT_HEIGHT = _PREFIX + "g_nCurrentHeight"
_VAR_BUSY = _PREFIX + "g_xBusy"
_VAR_DONE = _PREFIX + "g_xDone"
_VAR_ERROR = _PREFIX + "g_xError"
_VAR_ERROR_CODE = _PREFIX + "g_dwErrorCode"
_VAR_STATUS_TEXT = _PREFIX + "g_sStatus"
_VAR_POSITION_VALID = _PREFIX + "g_xPositionValid"  # pending on PLC side

_POLL_INTERVAL_SECS = 0.1
_STOP_PULSE_SECS = 0.2

# Declaring the client AmsNetId is process-global in pyads; do it once.
_local_address_lock = threading.Lock()
_local_address_set: str | None = None


def declare_client_ams_net_id(client_ams_net_id: str) -> None:
    """Declare the AmsNetId this process presents to PLCs.

    Must equal the AmsNetId in the static route configured on the PLC —
    pyads on Linux does not derive it from the host IP by itself.
    """
    global _local_address_set
    with _local_address_lock:
        if _local_address_set == client_ams_net_id:
            return
        if _local_address_set is not None:
            raise PlcError(
                f"client AmsNetId already declared as {_local_address_set}, "
                f"cannot redeclare as {client_ams_net_id}"
            )
        pyads.open_port()
        pyads.set_local_address(client_ams_net_id)
        _local_address_set = client_ams_net_id
        logger.info("Declared client AmsNetId %s", client_ams_net_id)


class PlcError(Exception):
    """PLC command failed (latched error, timeout, or connection problem)."""


@dataclass
class PlcState:
    """Snapshot of the GVL_HMI status variables."""

    height_mm: int
    busy: bool
    done: bool
    error: bool
    error_code: int
    status_text: str
    position_valid: bool | None = None  # None until the PLC exposes the flag


class TablePlc:
    """One workbench lifting-column PLC, addressed over ADS.

    pyads is synchronous; every ADS call runs in a worker thread via
    asyncio.to_thread so the connector's event loop is never blocked.
    """

    def __init__(
        self,
        ip: str,
        ams_net_id: str,
        deadband_mm: int = 10,
        check_position_valid: bool = False,
    ) -> None:
        self.ip = ip
        self.ams_net_id = ams_net_id
        self.deadband_mm = deadband_mm
        self.check_position_valid = check_position_valid
        self._plc = pyads.Connection(ams_net_id, PLC_RUNTIME_PORT, ip)
        # ADS has no concurrent-command semantics we want to rely on; one
        # in-flight operation per table at a time.
        self._op_lock = asyncio.Lock()

    # -- Sync helpers (run in worker threads) ------------------------------

    def _ensure_open(self) -> None:
        if not self._plc.is_open:
            self._plc.open()

    def _close(self) -> None:
        with contextlib.suppress(Exception):
            self._plc.close()

    def _read(self, name: str):
        self._ensure_open()
        return self._plc.read_by_name(name)

    def _write(self, name: str, value) -> None:
        self._ensure_open()
        self._plc.write_by_name(name, value)

    def _read_state_sync(self) -> PlcState:
        self._ensure_open()
        state = PlcState(
            height_mm=int(self._plc.read_by_name(_VAR_CURRENT_HEIGHT)),
            busy=bool(self._plc.read_by_name(_VAR_BUSY)),
            done=bool(self._plc.read_by_name(_VAR_DONE)),
            error=bool(self._plc.read_by_name(_VAR_ERROR)),
            error_code=int(self._plc.read_by_name(_VAR_ERROR_CODE)),
            status_text=str(self._plc.read_by_name(_VAR_STATUS_TEXT)),
        )
        if self.check_position_valid:
            state.position_valid = bool(self._plc.read_by_name(_VAR_POSITION_VALID))
        return state

    # -- Async API ---------------------------------------------------------

    async def read_state(self) -> PlcState:
        """Read the full status snapshot. Raises PlcError on ADS failure."""
        async with self._op_lock:
            try:
                return await asyncio.to_thread(self._read_state_sync)
            except pyads.ADSError as e:
                await asyncio.to_thread(self._close)
                raise PlcError(f"ADS read failed for {self.ip}: {e}") from e

    async def move_to_height(self, target_mm: int, timeout_secs: float = 60.0) -> None:
        """One full edge-triggered move handshake.

        Raises PlcError on a latched PLC error, invalid position, height
        mismatch after done, timeout, or ADS failure. On any exit —
        including task cancellation (mission abort/pause) — g_xExecuteMove
        is released, which aborts an in-progress move by design.
        """
        async with self._op_lock:
            try:
                await self._move_to_height_locked(target_mm, timeout_secs)
            except pyads.ADSError as e:
                await asyncio.to_thread(self._close)
                raise PlcError(f"ADS communication failed for {self.ip}: {e}") from e

    async def _move_to_height_locked(self, target_mm: int, timeout_secs: float) -> None:
        state = await asyncio.to_thread(self._read_state_sync)
        if state.error:
            raise PlcError(
                f"PLC has a latched error (code {state.error_code}): "
                f"{state.status_text!r} — clear it at the machine"
            )
        if self.check_position_valid and state.position_valid is False:
            raise PlcError("axis not referenced (g_xPositionValid FALSE) — run startup procedure")

        logger.info("PLC %s: move to %d mm (current %d mm)", self.ip, target_mm, state.height_mm)
        await asyncio.to_thread(self._write, _VAR_TARGET_HEIGHT, target_mm)
        await asyncio.to_thread(self._write, _VAR_EXECUTE_MOVE, True)
        try:
            loop = asyncio.get_running_loop()
            deadline = loop.time() + timeout_secs
            while True:
                state = await asyncio.to_thread(self._read_state_sync)
                if state.error:
                    raise PlcError(
                        f"move to {target_mm} mm failed (code {state.error_code}): "
                        f"{state.status_text!r}"
                    )
                if state.done:
                    # Variables are read one by one, so the height read may be
                    # staler than the done flag — take a fresh snapshot to verify.
                    state = await asyncio.to_thread(self._read_state_sync)
                    if abs(state.height_mm - target_mm) > self.deadband_mm:
                        raise PlcError(
                            f"PLC reports done but height is {state.height_mm} mm, "
                            f"expected {target_mm} ± {self.deadband_mm} mm"
                        )
                    logger.info("PLC %s: at %d mm", self.ip, state.height_mm)
                    return
                if loop.time() > deadline:
                    with contextlib.suppress(Exception):
                        await self._pulse_stop()
                    raise PlcError(
                        f"move to {target_mm} mm timed out after {timeout_secs:.0f}s "
                        f"(height {state.height_mm} mm, status {state.status_text!r})"
                    )
                await asyncio.sleep(_POLL_INTERVAL_SECS)
        finally:
            # Always release the execute bit: after done the PLC refuses the
            # next move while it is latched, and mid-move release is the
            # intended abort. Shielded so mission-abort cancellation cannot
            # skip it.
            release = asyncio.ensure_future(
                asyncio.to_thread(self._write, _VAR_EXECUTE_MOVE, False)
            )
            with contextlib.suppress(asyncio.CancelledError, Exception):
                await asyncio.shield(release)

    async def stop(self) -> None:
        """Pulse g_xStop for an immediate controlled stop."""
        async with self._op_lock:
            try:
                await asyncio.to_thread(self._write, _VAR_EXECUTE_MOVE, False)
                await self._pulse_stop()
            except pyads.ADSError as e:
                await asyncio.to_thread(self._close)
                raise PlcError(f"ADS stop failed for {self.ip}: {e}") from e

    async def _pulse_stop(self) -> None:
        await asyncio.to_thread(self._write, _VAR_STOP, True)
        await asyncio.sleep(_STOP_PULSE_SECS)
        await asyncio.to_thread(self._write, _VAR_STOP, False)

    async def close(self) -> None:
        await asyncio.to_thread(self._close)
