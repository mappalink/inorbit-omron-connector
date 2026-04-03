# SPDX-FileCopyrightText: 2026 Mappalink
#
# SPDX-License-Identifier: MIT

"""Async ARCL TCP client for Omron AMRs.

Adapted from the FLOWCore connector's write-only ARCL client, extended with
response parsing for query commands (status, odometer, getGoals).
"""

import asyncio
import logging
from collections import deque
from enum import Enum, auto

logger = logging.getLogger(__name__)


class CommandType(Enum):
    GENERIC = auto()
    GO = auto()
    DOCK = auto()
    UNDOCK = auto()
    SET_BLOCK = auto()
    CLEAR_BLOCK = auto()


class ArclClient:
    def __init__(
        self,
        host: str,
        port: int,
        password: str,
        connection_timeout: int = 10,
        reconnect_interval: int = 5,
    ):
        self.host = host
        self.port = port
        self.password = password
        self.timeout = connection_timeout
        self.reconnect_interval = reconnect_interval

        self._shutdown_event = asyncio.Event()
        self._connected_event = asyncio.Event()

        # Command queue
        self._queue: deque = deque()
        self._queue_lock = asyncio.Lock()
        self._new_item_event = asyncio.Event()

        # Request-response tracking: maps a command string to a Future
        self._pending_requests: dict[str, asyncio.Future] = {}
        self._pending_lock = asyncio.Lock()

        # Cached status from the most recent successful query_status() call.
        # Used by the BT wait node to avoid sending competing "status" commands.
        self._cached_status: dict = {}

        # Internal tasks
        self._manager_task: asyncio.Task | None = None
        self._writer: asyncio.StreamWriter | None = None
        self._reader: asyncio.StreamReader | None = None

    async def connect(self):
        """Starts the connection manager loop."""
        if self._manager_task and not self._manager_task.done():
            logger.warning("Connection manager already running.")
            return

        self._shutdown_event.clear()
        self._manager_task = asyncio.create_task(self._connection_manager())
        logger.info("ARCL Client task started.")

    async def disconnect(self):
        """Stops the loop and closes connections."""
        logger.info("Disconnecting...")
        self._shutdown_event.set()
        self._new_item_event.set()

        if self._manager_task:
            try:
                await self._manager_task
            except asyncio.CancelledError:
                pass
            self._manager_task = None

        await self._close_socket()

        # Cancel any pending request futures
        async with self._pending_lock:
            for fut in self._pending_requests.values():
                if not fut.done():
                    fut.cancel()
            self._pending_requests.clear()

        logger.info("ARCL Client disconnected.")

    async def _close_socket(self):
        """Safely closes the socket."""
        self._connected_event.clear()
        if self._writer:
            try:
                self._writer.close()
                await asyncio.wait_for(self._writer.wait_closed(), timeout=2.0)
            except Exception:
                pass
            self._writer = None
            self._reader = None

    async def _connection_manager(self):
        """Main loop: connect, authenticate, spawn reader+writer, reconnect on failure."""
        while not self._shutdown_event.is_set():
            try:
                logger.info("Connecting to ARCL at %s:%s...", self.host, self.port)
                self._reader, self._writer = await asyncio.wait_for(
                    asyncio.open_connection(self.host, self.port),
                    timeout=self.timeout,
                )
                logger.info("Socket connected to %s:%s", self.host, self.port)

                # Read until password prompt
                logger.info("Waiting for ARCL password prompt...")
                await asyncio.wait_for(
                    self._read_until_prompt(b"Enter password:"),
                    timeout=max(10, self.timeout),
                )

                self._writer.write(f"{self.password}\n".encode())
                await self._writer.drain()

                # Wait for login confirmation ("End of commands")
                async def _verify_login():
                    while True:
                        line = await self._reader.readline()
                        if not line:
                            raise PermissionError(
                                "ARCL login failed: connection closed (invalid password)"
                            )
                        msg = line.decode("utf-8", errors="ignore").strip()
                        logger.debug("Login RX: %s", msg)
                        if "End of commands" in msg:
                            return

                await asyncio.wait_for(_verify_login(), timeout=self.timeout)

                logger.info("ARCL login successful.")
                self._connected_event.set()

                # Run reader and writer concurrently
                reader_task = asyncio.create_task(self._read_loop())
                writer_task = asyncio.create_task(self._write_loop())

                done, pending = await asyncio.wait(
                    [reader_task, writer_task, asyncio.create_task(self._shutdown_event.wait())],
                    return_when=asyncio.FIRST_COMPLETED,
                )

                for task in pending:
                    task.cancel()
                    try:
                        await task
                    except asyncio.CancelledError:
                        pass

                for task in done:
                    if not task.cancelled() and task.exception():
                        logger.error("Task failed: %s", task.exception())

            except PermissionError as e:
                logger.error("Permission error: %s", e)
            except (OSError, asyncio.TimeoutError) as e:
                logger.error("Connection error: %s: %s", type(e).__name__, e)
            except Exception as e:
                logger.exception(
                    "Unexpected error in connection manager: %s: %s", type(e).__name__, e
                )
            finally:
                await self._close_socket()
                # Cancel pending request futures on disconnect
                async with self._pending_lock:
                    for fut in self._pending_requests.values():
                        if not fut.done():
                            fut.set_exception(ConnectionError("ARCL connection lost"))
                    self._pending_requests.clear()

            if not self._shutdown_event.is_set():
                logger.info("Reconnecting in %s seconds...", self.reconnect_interval)
                await asyncio.sleep(self.reconnect_interval)

    async def _read_until_prompt(self, prompt: bytes):
        """Reads stream until specific bytes are found."""
        if not self._reader:
            return

        buffer = b""
        while prompt not in buffer:
            chunk = await self._reader.read(1024)
            if not chunk:
                raise ConnectionResetError("Remote closed connection during handshake")
            buffer += chunk

    async def _read_loop(self):
        """Reads lines from robot, dispatches to pending request futures."""
        while not self._shutdown_event.is_set():
            if not self._reader:
                break

            line = await self._reader.readline()
            if not line:
                raise ConnectionResetError("Robot closed connection (EOF)")

            msg = line.decode("utf-8", errors="ignore").strip()
            if not msg:
                continue

            logger.debug("RX: %s", msg)

            # Dispatch response lines to pending request futures
            await self._dispatch_response(msg)

    async def _dispatch_response(self, msg: str):
        """Route an ARCL response line to the appropriate pending request."""
        async with self._pending_lock:
            for cmd_key, fut in list(self._pending_requests.items()):
                if fut.done():
                    continue

                # Match response lines to the command that generated them
                if cmd_key == "status" and ":" in msg:
                    # Accumulate all Key: Value lines from the status response
                    if not hasattr(fut, "_lines"):
                        fut._lines = []
                    fut._lines.append(msg)

                elif cmd_key == "odometer" and msg.startswith("Odometer:"):
                    if not hasattr(fut, "_lines"):
                        fut._lines = []
                    fut._lines.append(msg)

                elif cmd_key == "getgoals":
                    if msg.startswith("Goal:"):
                        if not hasattr(fut, "_lines"):
                            fut._lines = []
                        fut._lines.append(msg)
                    elif msg == "End of goals":
                        lines = getattr(fut, "_lines", [])
                        fut.set_result(lines)

                elif cmd_key == "rangedevicegetcurrent" and msg.startswith("RangeDeviceGetCurrent"):
                    # Single-line response: complete immediately
                    fut.set_result([msg])
                    del self._pending_requests[cmd_key]
                    return

                elif msg.startswith("CommandError"):
                    fut.set_exception(RuntimeError(msg))
                    del self._pending_requests[cmd_key]
                    return

        # Finalize odometer when a non-matching line arrives after accumulation.
        # Status uses timeout-based completion (ARCL sends no terminator line).
        async with self._pending_lock:
            if "odometer" in self._pending_requests:
                fut = self._pending_requests["odometer"]
                if not fut.done():
                    if not msg.startswith("Odometer:") and hasattr(fut, "_lines") and fut._lines:
                        fut.set_result(list(fut._lines))
                        del self._pending_requests["odometer"]

    async def _write_loop(self):
        """Consumes the command queue and sends to robot."""
        while not self._shutdown_event.is_set():
            await self._new_item_event.wait()

            if self._shutdown_event.is_set():
                break

            cmd_str = None

            async with self._queue_lock:
                if self._queue:
                    _, cmd_str = self._queue.popleft()
                if not self._queue:
                    self._new_item_event.clear()

            if cmd_str and self._writer:
                try:
                    logger.info("TX: %s", cmd_str.strip())
                    self._writer.write(cmd_str.encode())
                    await self._writer.drain()
                except OSError as e:
                    logger.error("Failed to write: %s", e)
                    raise

    # --- Queue Logic ---

    async def _enqueue_command(self, cmd_type: CommandType, command_str: str):
        """Adds command to queue. SET_BLOCK removes pending CLEAR_BLOCK/GO (safety-first)."""
        async with self._queue_lock:
            if cmd_type == CommandType.SET_BLOCK:
                original_len = len(self._queue)
                self._queue = deque(
                    item
                    for item in self._queue
                    if item[0] not in (CommandType.CLEAR_BLOCK, CommandType.GO)
                )
                removed_count = original_len - len(self._queue)
                if removed_count > 0:
                    logger.warning(
                        "SET_BLOCK priority: removed %d pending GO/CLEAR commands.", removed_count
                    )

            self._queue.append((cmd_type, command_str))
            self._new_item_event.set()

    # --- Request-Response ---

    async def send_command(self, cmd: str, timeout: float = 5.0) -> list[str]:
        """Send a command and await parsed response lines.

        Returns a list of response lines. For fire-and-forget commands,
        use _enqueue_command directly instead.
        """
        loop = asyncio.get_event_loop()
        fut = loop.create_future()

        # Use the command word as the key for matching responses
        cmd_key = cmd.strip().split()[0].lower() if cmd.strip() else cmd.strip()

        async with self._pending_lock:
            self._pending_requests[cmd_key] = fut

        # Send the command
        await self._enqueue_command(CommandType.GENERIC, f"{cmd}\n")

        try:
            result = await asyncio.wait_for(fut, timeout=timeout)
            return result
        except asyncio.TimeoutError:
            async with self._pending_lock:
                self._pending_requests.pop(cmd_key, None)
            # Return whatever lines accumulated before timeout
            lines = getattr(fut, "_lines", [])
            if lines:
                return lines
            raise
        finally:
            async with self._pending_lock:
                self._pending_requests.pop(cmd_key, None)

    async def query_status(self) -> dict:
        """Send 'status' and parse the response into a dict.

        Returns dict with keys like: DockingState, StateOfCharge, Location,
        Temperature, LocalizationScore, etc.

        The Location value is kept as a raw string "x y theta" (in mm and degrees).
        ARCL returns lines in "Key: Value" format (e.g. "Location: 6189 89 179").
        """
        lines = await self.send_command("status", timeout=1.0)
        result = {}
        for line in lines:
            colon_idx = line.find(":")
            if colon_idx == -1:
                continue
            key = line[:colon_idx].strip()
            value = line[colon_idx + 1 :].strip()
            result[key] = value
        if result:
            self._cached_status = result
        return result

    @property
    def cached_status(self) -> dict:
        """Return the most recent successfully parsed status dict.

        The telemetry loop calls query_status() at ~1 Hz, keeping this
        fresh.  BT wait nodes should read this instead of calling
        query_status() directly to avoid concurrent "status" commands
        whose responses get interleaved on the single ARCL TCP socket.
        """
        return self._cached_status

    async def query_odometer(self) -> dict:
        """Send 'odometer' and parse the response into a dict.

        Returns dict with keys like: Velocity, LeftVelocity, RightVelocity.
        """
        lines = await self.send_command("odometer")
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
        return result

    async def query_goals(self) -> list[str]:
        """Send 'getGoals' and return a list of goal names."""
        lines = await self.send_command("getGoals", timeout=10.0)
        goals = []
        for line in lines:
            # Format: Goal: "goal_name"
            if line.startswith("Goal:"):
                name = line[len("Goal:") :].strip().strip('"')
                goals.append(name)
        return goals

    async def query_laser_scan(self, laser_name: str) -> list[tuple[float, float]]:
        """Send 'rangeDeviceGetCurrent <laser_name>' and return (x_mm, y_mm) points.

        ARCL returns a single line:
          RangeDeviceGetCurrent: <laser_name> x1 y1 x2 y2 ...
        Coordinates are in mm, in the map/world frame (not robot-local).
        """
        lines = await self.send_command(f"rangeDeviceGetCurrent {laser_name}", timeout=2.0)
        points: list[tuple[float, float]] = []
        for line in lines:
            if not line.startswith("RangeDeviceGetCurrent"):
                continue
            parts = line.split()
            # Skip "RangeDeviceGetCurrent:" and laser name
            values = parts[2:]
            for i in range(0, len(values) - 1, 2):
                try:
                    x = float(values[i])
                    y = float(values[i + 1])
                    points.append((x, y))
                except (ValueError, IndexError):
                    continue
        return points

    # --- High-Level Fire-and-Forget Commands ---

    async def goto(self, goal_name: str):
        """Navigate to a named goal."""
        await self._enqueue_command(CommandType.GENERIC, f"goto {goal_name}\n")

    async def gotopoint(self, x: int, y: int, theta: int):
        """Navigate to a specific point (mm, mm, degrees)."""
        await self._enqueue_command(CommandType.GENERIC, f"gotopoint {x} {y} {theta}\n")

    async def stop(self):
        """Stop the robot."""
        await self._enqueue_command(CommandType.GENERIC, "stop\n")

    async def go(self):
        """Resume movement."""
        await self._enqueue_command(CommandType.GO, "go\n")

    async def dock(self):
        """Dock the robot."""
        await self._enqueue_command(CommandType.DOCK, "dock\n")

    async def undock(self):
        """Undock the robot."""
        await self._enqueue_command(CommandType.UNDOCK, "undock\n")

    async def set_block_driving(self, name: str, short_desc: str, long_desc: str):
        """Pause the robot (Application Block Driving Set)."""
        cmd = f'abds "{name}" "{short_desc}" "{long_desc}"\n'
        await self._enqueue_command(CommandType.SET_BLOCK, cmd)

    async def clear_block_driving(self, name: str):
        """Clear a driving block."""
        await self._enqueue_command(CommandType.CLEAR_BLOCK, f"abdc {name}\n")

    async def shutdown_robot(self):
        """Shut down the AMR OS."""
        await self._enqueue_command(CommandType.GENERIC, "shutdown\n")

    def is_connected(self) -> bool:
        return self._connected_event.is_set()

    async def wait_for_connection(self, timeout: float | None = None):
        """Wait until the client is connected."""
        await asyncio.wait_for(self._connected_event.wait(), timeout=timeout)
