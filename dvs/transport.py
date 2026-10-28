"""Half-duplex RS485 transport for the DVS bus.

Two things make this fiddlier than a normal serial link:

1. Response lengths are unknown. Nothing in the frame says how long the reply
   is, and it varies per register, so we read until the bus has been quiet for
   `idle_gap` seconds rather than reading a fixed count.

2. Cheap USB-RS485 adapters echo. Most of the common ones (and anything using a
   simple auto-direction transceiver) put the transmitted bytes straight back
   into the receive buffer. We detect that by checking whether the first bytes
   read back match what we just sent, and drop them.

Every byte in both directions is timestamped into a log file, because when
you're reverse engineering the useful artefact is the log, not the screen.
"""

import threading
import time
from pathlib import Path
from typing import Iterator

import serial

from .config import Config
from .protocol import DEADMAN_REGISTER, STOP_REGISTER, Frame, Response, hexdump


class BusError(RuntimeError):
    pass


class DvsBus:
    def __init__(self, config: Config):
        self.config = config
        self.serial: serial.Serial | None = None
        # The keepalive thread and the foreground menu share one port.
        self.lock = threading.RLock()
        self._log: Path | None = Path(config.log_path) if config.log_path else None

    # -- lifecycle ---------------------------------------------------------

    @property
    def is_open(self) -> bool:
        return self.serial is not None and self.serial.is_open

    def open(self) -> None:
        cfg = self.config
        if not cfg.port:
            raise BusError("no serial port configured")
        self.close()
        self.serial = serial.Serial(
            port=cfg.port,
            baudrate=cfg.baud,
            bytesize=cfg.byte_size,
            parity=cfg.parity,
            stopbits=cfg.stop_bits,
            timeout=0.01,
            write_timeout=2.0,
        )
        self.log(f"# opened {cfg.port} {cfg.baud} {cfg.byte_size}{cfg.parity}{cfg.stop_bits}")

    def close(self) -> None:
        if self.serial is not None:
            try:
                self.serial.close()
            finally:
                self.serial = None

    def __enter__(self) -> "DvsBus":
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # -- logging -----------------------------------------------------------

    def log(self, line: str) -> None:
        if self._log is None:
            return
        # Milliseconds, not seconds: the controller's poll cycle is ~257 ms and
        # the fan answers ~76 ms after a request, so a seconds-only stamp can't
        # show the ordering that matters. Same format as the bus capture, but
        # spelled out here rather than imported - monitor sits *above* transport.
        now = time.time()
        stamp = f"{time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(now))}.{int(now % 1 * 1000):03d}"
        with self._log.open("a") as fh:
            fh.write(f"{stamp} {line}\n")

    # -- I/O ---------------------------------------------------------------

    def _read_until_idle(self, timeout: float, idle_gap: float) -> bytes:
        assert self.serial is not None
        buf = bytearray()
        deadline = time.monotonic() + timeout
        last_byte_at: float | None = None
        while True:
            waiting = self.serial.in_waiting
            if waiting:
                buf += self.serial.read(waiting)
                last_byte_at = time.monotonic()
            else:
                now = time.monotonic()
                if last_byte_at is not None and now - last_byte_at >= idle_gap:
                    break  # response complete
                if now >= deadline:
                    break  # nothing came, or it never stopped
                time.sleep(0.002)
        return bytes(buf)

    def transaction(self, frame: Frame, timeout: float | None = None) -> Response:
        """Send a frame and collect whatever comes back."""
        if not self.is_open:
            raise BusError("serial port is not open")
        assert self.serial is not None
        cfg = self.config
        raw_tx = frame.encode(cfg.crc)
        notes: list[str] = []

        with self.lock:
            self.serial.reset_input_buffer()
            self.serial.write(raw_tx)
            self.serial.flush()
            self.log(f"TX {hexdump(raw_tx)}")
            rx = self._read_until_idle(
                cfg.response_timeout if timeout is None else timeout, cfg.idle_gap
            )
            time.sleep(cfg.turnaround)

        if rx:
            self.log(f"RX {hexdump(rx)}")
        else:
            self.log("RX (timeout)")

        if cfg.strip_echo and rx.startswith(raw_tx):
            rx = rx[len(raw_tx) :]
            notes.append("stripped adapter echo of the transmitted frame")

        return Response(raw=rx, crc=cfg.crc, request=frame, notes=notes)

    def send_raw(self, data: bytes, timeout: float | None = None) -> bytes:
        """Send bytes exactly as given - no address, register or checksum added."""
        if not self.is_open:
            raise BusError("serial port is not open")
        assert self.serial is not None
        cfg = self.config
        with self.lock:
            self.serial.reset_input_buffer()
            self.serial.write(data)
            self.serial.flush()
            self.log(f"TX(raw) {hexdump(data)}")
            rx = self._read_until_idle(
                cfg.response_timeout if timeout is None else timeout, cfg.idle_gap
            )
            time.sleep(cfg.turnaround)
        self.log(f"RX {hexdump(rx)}" if rx else "RX (timeout)")
        if cfg.strip_echo and rx.startswith(data):
            rx = rx[len(data) :]
        return rx

    # -- passive listening -------------------------------------------------

    def listen(
        self,
        gap: float | None = None,
        stop: threading.Event | None = None,
        poll: float = 0.002,
    ) -> Iterator[tuple[float, bytes]]:
        """Yield `(wall clock time, bytes)` for every burst heard on the bus.

        Purely passive - nothing is transmitted - so the real controller and the
        fan carry on as though we weren't wired in. That's the point: it's the
        only way to see what a frame the controller sends actually looks like.

        A burst ends after `gap` seconds of silence, the same rule
        `_read_until_idle` uses. A burst is *not* the same thing as a frame: if
        the fan answers faster than `gap` the request and its reply arrive as one
        burst, and a burst may equally be a fragment. Splitting bursts into
        frames needs the checksum, so it belongs to the decoder, not here.

        The port lock is taken per poll rather than held for the run, so a
        foreground command still works while this is going - but anything we
        transmit lands in the capture too, so stop the deadman first if you want
        a clean recording of someone else's traffic.
        """
        if not self.is_open:
            raise BusError("serial port is not open")
        assert self.serial is not None
        gap = self.config.idle_gap if gap is None else gap
        buf = bytearray()
        started = 0.0
        last_byte_at = 0.0
        while stop is None or not stop.is_set():
            with self.lock:
                waiting = self.serial.in_waiting
                chunk = self.serial.read(waiting) if waiting else b""
            if chunk:
                if not buf:
                    started = time.time()
                buf += chunk
                last_byte_at = time.monotonic()
                continue
            if buf and time.monotonic() - last_byte_at >= gap:
                data = bytes(buf)
                buf.clear()
                self.log(f"RX(monitor) {hexdump(data)}")
                yield started, data
            else:
                time.sleep(poll)
        if buf:  # don't drop a burst that was still arriving when we were stopped
            data = bytes(buf)
            self.log(f"RX(monitor) {hexdump(data)}")
            yield started, data

    # -- convenience -------------------------------------------------------

    def read_register(self, register: int, address: int | None = None) -> Response:
        return self.transaction(
            Frame(self.config.address if address is None else address, register)
        )

    def write_register(
        self, register: int, payload: bytes = b"", address: int | None = None
    ) -> Response:
        return self.transaction(
            Frame(self.config.address if address is None else address, register, payload)
        )


class Deadman:
    """Background thread that kicks register 0x03 so the fan keeps spinning.

    The fan spins down a couple of seconds after the last kick, so anything
    that needs the fan running - a calibration sweep, a setpoint hunt - has to
    have this going.

    Confirmed on hardware (verify.py T5): with 0x15 writes stopped and nothing
    but 0x03 kicks going out, the fan held its speed for the whole window, while
    total silence had it coasting to a standstill. So 0x03 really is a
    keepalive, as the write-up says.

    Writing 0x15 refreshes the same timeout, which is what the real wall
    controller relies on - it never sends 0x03 at all. Either will do; don't
    read the controller's choice as evidence that 0x03 does nothing.
    """

    REGISTER = DEADMAN_REGISTER

    def __init__(self, bus: DvsBus):
        self.bus = bus
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self.kicks = 0
        self.failures = 0

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        if self.running:
            return
        self._stop.clear()
        self.kicks = 0
        self.failures = 0
        self._thread = threading.Thread(target=self._run, name="deadman", daemon=True)
        self._thread.start()

    def stop(self, send_stop: bool = False) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._thread = None
        if send_stop:
            try:
                self.bus.write_register(STOP_REGISTER)
            except Exception:
                pass

    def kick(self, timeout: float | None = None) -> Response:
        return self.bus.transaction(
            Frame(self.bus.config.address, self.REGISTER, self.bus.config.deadman_bytes),
            timeout=self.bus.config.deadman_response_timeout if timeout is None else timeout,
        )

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self.kick()
                self.kicks += 1
            except Exception as exc:  # keep the thread alive through transient errors
                self.failures += 1
                self.bus.log(f"# deadman error: {exc}")
            self._stop.wait(self.bus.config.deadman_interval)


class SpeedHold:
    """Holds a speed demand by rewriting it on a timer, the way the controller does.

    The wall controller never sends a 0x03 kick at all. It holds the fan up
    purely by writing the demand to 0x15 about four times a second, and T5
    confirmed on hardware that either register refreshes the fan's timeout.

    For anything that has to hold *one known speed* for minutes at a time - the
    calibration sweep - the controller's way is the better of the two. A demand
    written once and then merely kept alive by 0x03 survives only as long as the
    fan remembers it; re-asserting it means a dropped frame costs one interval
    rather than the rest of the step, and the reply to every write carries the
    actual speed back for free.

    Three near-identical things now exist, and they stay separate deliberately:

      * `Deadman` kicks 0x03 and carries no demand. Which of the two keeps the
        fan alive is a measured fact, and `verify.py` T5 can only keep measuring
        it by driving each one independently.
      * `verify.SpeedDriver` does what this class does, but is wired to that
        module's JSONL `Recorder` and its T5 instrumentation (pause/resume,
        lowest-seen tracking). `transport` cannot depend on `verify`, and the
        sweep has no use for a transcript.
    """

    def __init__(
        self,
        bus: DvsBus,
        register: int,
        demand: int = 0,
        interval: float | None = None,
    ):
        self.bus = bus
        self.register = register
        self.interval = bus.config.speed_hold_interval if interval is None else interval
        self._demand = demand
        self._actual: int | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.writes = 0
        self.failures = 0

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    @property
    def demand(self) -> int:
        with self._lock:
            return self._demand

    @property
    def actual(self) -> int | None:
        """Speed from the most recent reply, or None if none has answered yet.

        Only meaningful when the demand register is the one that reports the
        speed - which is the whole point of 0x15, but need not hold for whatever
        else someone points the sweep at.
        """
        with self._lock:
            return self._actual

    def set_demand(self, value: int) -> None:
        """Change the held demand and put it on the wire immediately.

        Writing straight away rather than waiting for the next tick keeps a
        settle time honest: otherwise every step would silently begin with up to
        one interval of the previous demand still in force.
        """
        with self._lock:
            self._demand = value
        self.write_once()

    def write_once(self) -> Response:
        response = self.bus.transaction(
            Frame(self.bus.config.address, self.register, bytes([self.demand])),
            timeout=self.bus.config.speed_hold_response_timeout,
        )
        self.writes += 1
        payload = response.payload
        if len(payload) >= 2:
            with self._lock:
                self._actual = int.from_bytes(payload[:2], "little")
        return response

    def start(self) -> None:
        if self.running:
            return
        self._stop.clear()
        self.writes = 0
        self.failures = 0
        self._thread = threading.Thread(target=self._run, name="speed-hold", daemon=True)
        self._thread.start()

    def stop(self, send_stop: bool = False) -> None:
        """Stop holding the demand, optionally parking the fan on the way out.

        Same shape as `Deadman.stop`, and for the same reason: letting the
        writes lapse only spins the fan down after the timeout, so anything that
        wants it stopped now has to say so explicitly.
        """
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._thread = None
        if send_stop:
            try:
                self.bus.write_register(STOP_REGISTER)
            except Exception:
                pass

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self.write_once()
            except Exception as exc:  # keep the hold going through transient errors
                self.failures += 1
                self.bus.log(f"# speed hold error: {exc}")
            self._stop.wait(self.interval)
