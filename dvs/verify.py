"""Putting the register 0x15 speed-control theory on the bus.

A passive capture of the real wall controller changing the speed boost dial
suggests a specific story for how speed is controlled, and parts of that story
disagree with what this codebase currently assumes. This module states each
claim, drives the fan to test it, and records what actually happened.

The claims, and what would falsify each:

  T1  A write to 0x15 carrying one payload byte is answered with two payload
      bytes, little-endian - so a write returns a reading.
                                    Falsified by any other reply length.
  T2  The demand is 7-bit: 0x00..0x7F, with 0x7F the top of the range.
                                    Falsified if bit 7 is not masked.
  T3  The two-byte answer is the *actual* speed in the same units as the demand,
      so it rises and falls with the demand and settles near it.
                                    Falsified if it doesn't track the demand.
  T4  The answer is a rate, not a tacho period: with the impeller stopped it
      reads 00 00 rather than saturating.
                                    Falsified if a stopped fan reads high.
  T5  Writes to 0x15 are what keep the fan turning, and 0x03 - the register
      `transport.Deadman` kicks, which the real controller never sends - does
      nothing.
                                    Falsified if 0x03 alone holds the fan up.
  T6  There is a minimum runnable demand (the capture suggests 24) below which
      the fan will not start.
                                    Falsified by a smooth ramp from 1 upward.

What the hardware said, over three runs on 2026-08-05 (transcript in
dvs_verify.jsonl; the numbers are in protocol.py next to the register):

    T1 supported     T2 CONTRADICTED - bit 7 is not masked, the demand is wider
    T3 supported     T5 CONTRADICTED - 0x03 is a keepalive after all
    T4 supported     T6 supported, but the floor is 12, not the 24 in the capture

Both contradictions were the same mistake in the capture analysis: reading a
*wall controller policy* as a *fan property*. The dial only ever sent 0x00-0x7F,
and the controller only ever kept the fan alive with 0x15 writes, but neither
was a limit of the hardware. Only driving the fan directly could separate them,
which is what this module is for.

Two of the checks needed rebuilding after their first hardware run, both because
they measured something the fan wouldn't tell them: T2 compared two demands a
saturated impeller cannot distinguish, and T5 tried to read a register that
stops answering while the bus is silent. Neither failure was visible against the
fake fans. If a check comes back with a tidy answer, be suspicious of the method
before believing the result.

Nothing here decides anything on its own. Each check writes a verdict plus the
evidence behind it, and every frame in both directions goes into a transcript
with a millisecond timestamp, so a run can be argued about afterwards instead of
being taken on trust. A check that cannot tell says `inconclusive` rather than
guessing, and `check_rate_not_period` in particular is written so that it can
come back `contradicted` - a test tool that can only ever agree with the theory
is worth nothing.

The transcript is deliberately dumb - hex, numbers and labels, no prose - for
the same reason the bus capture is: it stays valid when we change our mind about
what the bytes mean. Interpretation lives in the findings and in `timeline`.

Only three registers are ever written here: 0x15 (the demand), 0x05 (stop) and
0x03 (kicked in one arm of T5). 0x04 is never touched.
"""

import json
import threading
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Callable, Iterable, Sequence

from .monitor import format_stamp
from .protocol import DEADMAN_REGISTER, STOP_REGISTER, Frame, Response, hexdump
from .transport import Deadman, DvsBus

Reporter = Callable[[str], None]

SPEED_REGISTER = 0x15

# The real controller wrote the demand every ~257 ms. Matching that matters:
# under T5 this interval is also the keepalive period, so a slower one would
# fail T5 for the wrong reason.
DRIVE_INTERVAL = 0.25

# Top of the demand range under T2, and the value used to probe past it.
MAX_DEMAND = 0x7F

# A missed reply is not evidence of anything, so reads that a verdict rests on
# are retried. Mirrors discovery.SPEED_READ_ATTEMPTS.
SPEED_READ_ATTEMPTS = 4
SPEED_READ_RETRY_DELAY = 0.2

VERDICTS = ("supported", "contradicted", "inconclusive")


# -- what a run records ----------------------------------------------------


@dataclass
class Exchange:
    """One request and whatever came back, as it went onto the wire."""

    at: float  # seconds since the run started
    wall: str  # wall clock, millisecond resolution
    check: str  # which claim was being tested
    phase: str  # what the check was doing at the time
    tx: str  # hex, exactly as transmitted
    rx: str  # hex, exactly as received (after echo stripping)
    crc_ok: bool
    register: int | None = None
    demand: int | None = None  # payload written, for a 1-byte write to 0x15
    actual: int | None = None  # u16le of the reply payload, when 2 bytes came back


@dataclass
class Finding:
    """The verdict on one claim, and the numbers it rests on."""

    claim: str
    title: str
    verdict: str
    detail: str
    evidence: list[str] = field(default_factory=list)
    # Set when the claim is a precondition for the rest of the run - no point
    # spending another eight minutes if the fan isn't answering at all.
    fatal: bool = False


@dataclass
class Plan:
    """Timings and ladders for a run.

    Real hardware needs the long defaults: a ducted centrifugal fan takes tens
    of seconds to reach a steady speed, and the actual-speed register only
    updated every ~4 s in the capture, so a short settle reads the ramp rather
    than the steady state. Tests shrink all of these.
    """

    # Where the demand is written. Defaults to the speed register itself, which
    # is what the wall controller does. Point it elsewhere - `config.
    # setpoint_register`, say - to ask whether that register controls the speed
    # too; the actual is always read back from SPEED_REGISTER either way, since
    # that is the one known to report it.
    demand_register: int = SPEED_REGISTER

    settle: float = 25.0  # hold a demand this long before believing the answer
    spindown: float = 20.0 # how long a parked impeller gets to actually stop
    quiet: float = 6.0  # T5 window; the write-up's spin-down is ~2 s
    # T5 watches this long after resuming the driver. Has to outlast a full
    # coast to zero: on the test unit that took ~9 s from an actual of 55.
    recovery_watch: float = 20.0
    threshold_settle: float = 12.0
    drive_interval: float = DRIVE_INTERVAL
    # Mid-range demand used wherever a check just needs the fan turning. 0x38 is
    # what the controller sat at with the boost dial at 0.
    nominal_demand: int = 0x38
    tracking_ladder: Sequence[int] = (0x18, 0x28, 0x38, 0x50, 0x68, 0x7F)
    # T2 probes a pair differing only in bit 7 - `low` and `low | 0x80` - plus
    # `top` to prove there was headroom to tell them apart. Comparing 0x7F
    # against 0xFF looks like the obvious test and is worthless: this fan
    # saturates at an actual of 80, well under either demand, so both read the
    # same whether the top bit is masked or not.
    ceiling_probe_low: int = 0x40
    ceiling_probe_top: int = 0x7F
    threshold_ladder: Sequence[int] = (0x04, 0x08, 0x0C, 0x10, 0x14, 0x18, 0x1C, 0x20, 0x24, 0x28)
    # An actual at or below this counts as stopped.
    stopped_tolerance: int = 2
    # Two consecutive ladder steps whose answers differ by no more than this,
    # at rising demand, count as the impeller having run out of headroom.
    saturation_tolerance: int = 2
    # T3: how far the actual may sit from the demand and still count as "same
    # units". Generous, because the top of the range may be physically
    # unreachable - which is reported rather than scored.
    tracking_tolerance: int = 8

    @property
    def estimated_seconds(self) -> float:
        """Rough wall-clock cost of a full run, for the CLI's warning."""
        return (
            len(self.tracking_ladder) * self.settle
            + 2 * self.settle  # T2
            + 3 * self.quiet + 2 * self.settle  # T5, including recovery
            + self.spindown + self.settle  # T4
            + len(self.threshold_ladder) * self.threshold_settle
        )


# -- recording -------------------------------------------------------------


class Recorder:
    """Sends frames, and writes every one of them to a transcript as it goes.

    Appends and flushes per exchange rather than buffering to the end: these runs
    take minutes and the usual way out of a bad one is Ctrl-C, which still has to
    leave a reviewable record.

    All bus I/O goes through `DvsBus.transaction`, so the ordinary bus log gets
    every byte too. This transcript is the structured view for later analysis,
    never a replacement for that log.
    """

    def __init__(
        self,
        bus: DvsBus,
        path: Path | str | None = None,
        report: Reporter = print,
        demand_register: int = SPEED_REGISTER,
    ):
        self.bus = bus
        self.report = report
        self.demand_register = demand_register
        self.path = Path(path) if path else None
        self.exchanges: list[Exchange] = []
        self.findings: list[Finding] = []
        self.check = ""
        self.phase = ""
        self._started = time.monotonic()
        self._lock = threading.Lock()  # the driver thread writes here too
        self._fh = None

    def __enter__(self) -> "Recorder":
        if self.path is not None:
            self._fh = self.path.open("a")
            self._emit(
                {
                    "kind": "run",
                    "wall": format_stamp(time.time()),
                    "crc": str(self.bus.config.crc),
                    "address": self.bus.config.address,
                    "speed_register": SPEED_REGISTER,
                    "demand_register": self.demand_register,
                }
            )
        return self

    def __exit__(self, *exc) -> None:
        if self._fh is not None:
            self._fh.close()
            self._fh = None

    def _emit(self, record: dict) -> None:
        if self._fh is None:
            return
        self._fh.write(json.dumps(record) + "\n")
        self._fh.flush()

    # -- sending -----------------------------------------------------------

    def send(self, frame: Frame, timeout: float | None = None, phase: str = "") -> Response:
        """Run one transaction and record both directions of it."""
        raw_tx = frame.encode(self.bus.config.crc)
        response = self.bus.transaction(frame, timeout=timeout)
        payload = response.payload
        exchange = Exchange(
            at=round(time.monotonic() - self._started, 3),
            wall=format_stamp(time.time()),
            check=self.check,
            phase=phase or self.phase,
            tx=hexdump(raw_tx),
            rx=hexdump(response.raw),
            crc_ok=bool(response) and response.crc_ok,
            register=frame.register,
            demand=frame.payload[0] if len(frame.payload) == 1 else None,
            actual=int.from_bytes(payload[:2], "little") if len(payload) >= 2 else None,
        )
        with self._lock:
            self.exchanges.append(exchange)
            self._emit({"kind": "exchange", **asdict(exchange)})
        return response

    def write_demand(self, value: int, phase: str = ""):
        return self.send(
            Frame(self.bus.config.address, self.demand_register, bytes([value])),
            phase=phase,
        )

    def read_speed(self, phase: str = ""):
        """Empty-payload read of 0x15.

        The captured controller never did this - it only ever wrote - so whether
        a bare read works at all is new information either way.
        """
        return self.send(Frame(self.bus.config.address, SPEED_REGISTER), phase=phase)

    def kick_deadman(self, phase: str = ""):
        return self.send(
            Frame(self.bus.config.address, DEADMAN_REGISTER, self.bus.config.deadman_bytes),
            timeout=self.bus.config.deadman_response_timeout,
            phase=phase,
        )

    def stop_fan(self, phase: str = ""):
        return self.send(Frame(self.bus.config.address, STOP_REGISTER), phase=phase)

    # -- findings ----------------------------------------------------------

    def add_finding(self, finding: Finding) -> Finding:
        with self._lock:
            self.findings.append(finding)
            self._emit({"kind": "finding", **asdict(finding)})
        return finding


class SpeedDriver:
    """Writes the demand to 0x15 on a timer, the way the wall controller does.

    Under T5 this is also the keepalive, so every check that needs the fan
    turning needs one of these running.

    Deliberately a separate class from `transport.Deadman`, which kicks 0x03:
    which of the two actually keeps the fan alive is exactly what T5 is for, and
    merging them here would bake in the answer before it has been measured.
    """

    def __init__(self, recorder: Recorder, demand: int = 0, interval: float = DRIVE_INTERVAL):
        self.recorder = recorder
        self.interval = interval
        self._demand = demand
        self._actual: int | None = None
        # Lowest answer since the last reset. Tracked here, off every reply,
        # rather than sampled by the caller: T5's spin-down can pass through
        # zero between two polls and a missed dip inverts its verdict.
        self._lowest: int | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._resumed = threading.Event()
        self._resumed.set()
        self._thread: threading.Thread | None = None
        self.writes = 0
        self.answers = 0

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    @property
    def demand(self) -> int:
        with self._lock:
            return self._demand

    @property
    def actual(self) -> int | None:
        """Latest answer, or None if the fan has never answered a write."""
        with self._lock:
            return self._actual

    @property
    def lowest(self) -> int | None:
        """Lowest answer since `reset_extremes`, or None if none since."""
        with self._lock:
            return self._lowest

    def reset_extremes(self) -> None:
        """Start a fresh minimum, so the next watch isn't coloured by the last."""
        with self._lock:
            self._lowest = None

    def start(self) -> None:
        if self.running:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="speed-driver", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._resumed.set()  # a paused thread has to wake up to notice
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._thread = None

    def pause(self) -> None:
        """Stop transmitting, without stopping the thread."""
        self._resumed.clear()

    def resume(self) -> None:
        self._resumed.set()

    def set_demand(self, value: int, phase: str = "") -> None:
        """Change the held demand and write it immediately.

        Writing straight away rather than waiting for the next tick keeps the
        settle timing honest: otherwise every step silently includes up to one
        interval of the previous demand.
        """
        with self._lock:
            self._demand = value
        self._write_once(phase=phase or f"set demand {value}")

    def _write_once(self, phase: str = "") -> None:
        # Fall back to whatever the check has set on the recorder, so the driver's
        # own writes carry that context. T5's recovery watch is measured *by*
        # these writes, and labelling them all "hold" made the one window that
        # matters indistinguishable from every other in the transcript.
        label = phase or self.recorder.phase or "hold"
        response = self.recorder.write_demand(self.demand, phase=label)
        self.writes += 1
        if self.recorder.demand_register != SPEED_REGISTER:
            # The reply to a write to some other register is not the speed - it
            # may not even be a reply. The actual always comes from the register
            # known to report it, at the cost of a second frame per tick.
            response = self.recorder.read_speed(phase=f"{label}: read actual")
        payload = response.payload
        if len(payload) >= 2:
            self.answers += 1
            value = int.from_bytes(payload[:2], "little")
            with self._lock:
                self._actual = value
                self._lowest = value if self._lowest is None else min(self._lowest, value)

    def _run(self) -> None:
        while not self._stop.is_set():
            if self._resumed.is_set():
                try:
                    self._write_once()
                except Exception as exc:  # keep the keepalive going through blips
                    self.recorder.bus.log(f"# speed driver error: {exc}")
            self._stop.wait(self.interval)


# -- helpers ---------------------------------------------------------------


def _hold(
    driver: SpeedDriver, seconds: float, report: Reporter, label: str = "  "
) -> list[tuple[float, int]]:
    """Hold the current demand for `seconds`, reporting the actual as it moves.

    Returns `(elapsed, actual)` for each *change* in the reading, which is what
    tells a check whether the value had settled by the end of the window or was
    still on its way there.
    """
    seen: list[tuple[float, int]] = []
    started = time.monotonic()
    deadline = started + seconds
    while time.monotonic() < deadline:
        value = driver.actual
        if value is not None and (not seen or seen[-1][1] != value):
            elapsed = round(time.monotonic() - started, 2)
            seen.append((elapsed, value))
            report(f"{label}  +{elapsed:5.1f}s  actual {value}")
        time.sleep(0.1)
    return seen


def _read_actual(
    rec: Recorder, phase: str, report: Reporter, attempts: int = SPEED_READ_ATTEMPTS
) -> int | None:
    """Read the actual speed, retrying before believing a missed reply.

    One dropped answer is common - this fan misses roughly one read in twelve on
    a quiet bus, and is markedly worse on the first frame after a silent spell,
    which is exactly when T4 and T5 need to read. Taking that at face value once
    cost a whole run: T5 reported `inconclusive` because a single post-window
    read timed out, not because the experiment said anything. Same reasoning as
    `discovery._speed_word`.

    Retries are quick and the attempt count is recorded, because on a spinning-
    down fan every extra second biases the reading toward "stopped".
    """
    for attempt in range(1, attempts + 1):
        response = rec.read_speed(phase=f"{phase} (attempt {attempt})")
        payload = response.payload
        # Check the frame before believing the number. Without this a corrupt
        # or misaddressed reply is accepted as a speed, and the erratic values
        # that come back from a barely-answering fan are exactly the case where
        # a verdict most needs to be sure the bytes are real.
        if response.crc_ok and len(response.raw) >= 2 and response.raw[1] != SPEED_REGISTER:
            report(f"    ignoring a reply for register 0x{response.raw[1]:02X}")
        elif response.crc_ok and len(payload) >= 2:
            if attempt > 1:
                report(f"    (answered on attempt {attempt})")
            return int.from_bytes(payload[:2], "little")
        elif response:
            report(f"    ignoring a reply that failed its checksum: {hexdump(response.raw)}")
        if attempt < attempts:
            time.sleep(SPEED_READ_RETRY_DELAY)
    report(f"    no reply after {attempts} attempts")
    return None


def _still_moving(seen: list[tuple[float, int]], window: float) -> bool:
    """True if the reading was still changing in the last third of the hold."""
    if len(seen) < 2:
        return False
    return seen[-1][0] > window * (2 / 3)


def _park(
    recorder: Recorder,
    driver: SpeedDriver,
    plan: Plan,
    report: Reporter,
    phase: str = "park",
) -> None:
    """Bring the fan to a genuine standstill.

    A zero demand alone is not enough while anything is still writing 0x15 or
    kicking 0x03, so: zero the demand, stop driving it, send the stop command,
    and only then wait out the spin-down.
    """
    driver.set_demand(0, phase=f"{phase}: zero demand")
    driver.pause()
    if recorder.demand_register != SPEED_REGISTER:
        # 0x15 may still be holding an old demand from an earlier run; leave it
        # set and the fan can come back up on its own the moment it is kicked.
        recorder.send(
            Frame(recorder.bus.config.address, SPEED_REGISTER, b"\x00"),
            phase=f"{phase}: zero 0x{SPEED_REGISTER:02X} too",
        )
    recorder.stop_fan(phase=f"{phase}: stop command")
    report(f"    zeroed 0x{SPEED_REGISTER:02X} and sent stop (0x{STOP_REGISTER:02X}),"
           f" waiting {plan.spindown:g}s for the impeller to stop")
    time.sleep(plan.spindown)


def _spin_up(
    driver: SpeedDriver, plan: Plan, report: Reporter, demand: int | None = None
) -> list[tuple[float, int]]:
    """Get the fan turning at a mid-range demand and wait for it to settle."""
    target = plan.nominal_demand if demand is None else demand
    driver.resume()
    driver.set_demand(target, phase=f"spin up to {target}")
    report(f"    demand {target} (0x{target:02X}), settling {plan.settle:g}s")
    return _hold(driver, plan.settle, report)


# -- the checks ------------------------------------------------------------


def check_reply_shape(rec: Recorder, driver: SpeedDriver, plan: Plan, report: Reporter) -> Finding:
    """T1: a 1-byte write to 0x15 comes back with a 2-byte little-endian actual."""
    rec.check = "T1"
    demand = plan.nominal_demand
    report(f"    writing 0x{demand:02X} to 0x{rec.demand_register:02X}")
    write = rec.write_demand(demand, phase="1-byte write")
    report(f"    rx {hexdump(write.raw)}")
    # Read back the register we just wrote, not SPEED_REGISTER. They are the
    # same thing by default, but when the demand goes somewhere else this was
    # comparing a write to one register against a read of another and calling
    # the difference interesting.
    report(f"    and an empty-payload read of 0x{rec.demand_register:02X}")
    read = rec.send(
        Frame(rec.bus.config.address, rec.demand_register), phase="empty-payload read"
    )
    report(f"    rx {hexdump(read.raw)}")

    evidence = [
        f"write rx={hexdump(write.raw)} payload={hexdump(write.payload)}"
        f" crc_ok={bool(write) and write.crc_ok}",
        f"read  rx={hexdump(read.raw)} payload={hexdump(read.payload)}"
        f" crc_ok={bool(read) and read.crc_ok}",
    ]

    if not write:
        return Finding(
            "T1",
            CHECKS["T1"][0],
            "inconclusive",
            "The fan did not answer a write to 0x15 at all, so nothing else in "
            "this run would mean anything. Wrong CRC variant, wrong address or "
            "A/B swapped are the usual causes - a mismatch on a reply almost "
            "always means the CRC variant, not bad data.",
            evidence,
            fatal=True,
        )
    if not write.crc_ok:
        return Finding(
            "T1",
            CHECKS["T1"][0],
            "inconclusive",
            f"The fan answered {len(write.raw)} bytes but the checksum did not "
            "verify, which nearly always means the configured CRC variant is "
            "wrong rather than that the data is corrupt. Run menu 'c' first.",
            evidence,
            fatal=True,
        )

    length = len(write.payload)
    # Compared by length, not by value: the two frames are ~15 ms apart and the
    # speed moves between them, so equal bytes would be luck and unequal bytes
    # would prove nothing. What's worth knowing is whether a bare read is
    # answered at all, and in the same shape - the captured controller only ever
    # wrote, so reads are untested territory.
    detail_read = (
        "an empty-payload read got no answer at all, so reads of this register "
        "may simply not be supported"
        if not read
        else f"an empty-payload read answered with {len(read.payload)} payload "
        f"byte(s) too, so reads work as well as writes"
        if len(read.payload) == length
        else f"an empty-payload read answered with {len(read.payload)} payload "
        f"byte(s) instead of {length}, which is worth a closer look"
    )
    if length == 2:
        return Finding(
            "T1",
            CHECKS["T1"][0],
            "supported",
            f"A 1-byte write is answered with 2 payload bytes "
            f"({hexdump(write.payload)} = {int.from_bytes(write.payload, 'little')} "
            f"little-endian); {detail_read}.",
            evidence,
        )
    return Finding(
        "T1",
        CHECKS["T1"][0],
        "contradicted",
        f"A 1-byte write is answered with {length} payload byte(s), not 2 "
        f"({hexdump(write.payload)}); {detail_read}. The rest of this run reads "
        "the first two bytes little-endian regardless, so treat its numbers "
        "with that in mind.",
        evidence,
    )


def check_demand_tracking(
    rec: Recorder, driver: SpeedDriver, plan: Plan, report: Reporter
) -> Finding:
    """T3: the answer rises and falls with the demand, in the same units."""
    rec.check = "T3"
    driver.resume()
    steps: list[tuple[int, int | None, bool]] = []  # demand, settled actual, still moving
    for demand in plan.tracking_ladder:
        driver.set_demand(demand, phase=f"ladder demand {demand}")
        report(f"    demand {demand:3d} (0x{demand:02X}), settling {plan.settle:g}s")
        seen = _hold(driver, plan.settle, report)
        settled = driver.actual
        moving = _still_moving(seen, plan.settle)
        steps.append((demand, settled, moving))
        report(
            f"    demand {demand:3d} -> actual {settled}"
            + ("  (still moving at the end of the hold)" if moving else "")
        )

    evidence = [f"demand={d} actual={a} still_moving={m}" for d, a, m in steps]
    pairs = [(d, a) for d, a, _ in steps if a is not None]
    if len(pairs) < 2:
        return Finding(
            "T3", CHECKS["T3"][0], "inconclusive",
            "Not enough of the ladder produced a reading to see a trend.",
            evidence,
        )

    rises = sum(1 for (d1, a1), (d2, a2) in zip(pairs, pairs[1:]) if a2 > a1)
    falls = sum(1 for (d1, a1), (d2, a2) in zip(pairs, pairs[1:]) if a2 < a1)

    # Find where the impeller ran out of headroom: the longest run at the top of
    # the ladder where the demand kept rising but the answer didn't. Residuals
    # only mean something below that point - above it the gap measures what the
    # fan physically can't do, not what the units are. Left in, a saturated top
    # step dominates the worst-gap figure and buries the real result: this fan
    # matches the demand exactly up to 56 and then tops out at 80, which reads
    # as "worst gap 47" if the plateau is scored with everything else.
    plateau_from = len(pairs)
    for i in range(len(pairs) - 1, 0, -1):
        if abs(pairs[i][1] - pairs[i - 1][1]) > plan.saturation_tolerance:
            break
        plateau_from = i
    responsive, saturated = pairs[:plateau_from], pairs[plateau_from:]

    scored = responsive or pairs
    worst = max(abs(a - d) for d, a in scored)
    close = [1 for d, a in scored if abs(a - d) <= plan.tracking_tolerance]
    ceiling = (
        f" The impeller saturates: from demand {saturated[0][0]} upward the "
        f"answer held at {saturated[0][1]}, so that is a mechanical ceiling, "
        "not a protocol one, and the gap above it is excluded here."
        if saturated
        else ""
    )
    units = (
        f"Below that, the reading sits within {plan.tracking_tolerance} of the "
        f"demand at {len(close)}/{len(scored)} steps (worst gap {worst}), so "
        "demand and answer are the same units."
        if len(close) >= len(scored) - 1
        else f"Below that, the reading tracks the demand but does not equal it "
        f"- worst gap {worst} over {len(scored)} unsaturated step(s). If those "
        "steps were still moving when the hold ended, the settle time is too "
        "short rather than the units being different."
    ) + ceiling

    if rises and not falls:
        return Finding(
            "T3", CHECKS["T3"][0], "supported",
            f"The answer rose at every one of {rises} step(s) up the ladder and "
            f"never fell. {units}",
            evidence,
        )
    if falls and not rises:
        return Finding(
            "T3", CHECKS["T3"][0], "contradicted",
            f"The answer *fell* at every one of {falls} step(s) as the demand "
            "rose. That is the signature of a period or an inverted scale, not "
            "an actual-speed reading in the same units.",
            evidence,
        )
    if not rises and not falls:
        return Finding(
            "T3", CHECKS["T3"][0], "contradicted",
            "The answer never moved across the whole ladder, so it is not "
            "reporting anything that depends on the demand.",
            evidence,
        )
    return Finding(
        "T3", CHECKS["T3"][0], "inconclusive",
        f"The answer rose {rises} time(s) and fell {falls} time(s) up the "
        "ladder. Either the settle time is too short to be reading steady "
        "state, or something other than the demand is moving it.",
        evidence,
    )


def check_demand_ceiling(
    rec: Recorder, driver: SpeedDriver, plan: Plan, report: Reporter
) -> Finding:
    """T2: is the demand 7-bit? Compare a pair differing only in bit 7.

    The naive test - 0x7F against 0xFF - cannot work on a fan whose impeller
    saturates below the top of the demand range, which this one does at an
    actual of 80. Both demands then read the same no matter what the top bit
    does, and the check reports `supported` for entirely the wrong reason.

    So instead: probe `low` and `low | 0x80`, which differ *only* in bit 7, and
    also `top` to establish there was headroom between `low` and the maximum. If
    masking the top bit sends the fan back down to `low`, the demand is 7-bit.
    If it stays high, it isn't. If `low` and `top` already read the same, the fan
    is saturated at or below `low` and the run simply cannot tell - which is
    `inconclusive`, not `supported`.
    """
    rec.check = "T2"
    driver.resume()
    low = plan.ceiling_probe_low
    top = plan.ceiling_probe_top
    high_bit = low | 0x80
    results: dict[int, int | None] = {}
    for demand in (low, top, high_bit):
        driver.set_demand(demand, phase=f"ceiling demand {demand}")
        report(f"    demand {demand:3d} (0x{demand:02X}), settling {plan.settle:g}s")
        _hold(driver, plan.settle, report)
        results[demand] = driver.actual
        report(f"    demand 0x{demand:02X} -> actual {results[demand]}")

    evidence = [f"demand=0x{d:02X} actual={a}" for d, a in results.items()]
    at_low, at_top, at_high_bit = results[low], results[top], results[high_bit]
    if at_low is None or at_top is None or at_high_bit is None:
        return Finding(
            "T2", CHECKS["T2"][0], "inconclusive",
            "One of the three demands produced no reading.", evidence,
        )

    headroom = at_top - at_low
    if headroom <= plan.tracking_tolerance:
        return Finding(
            "T2", CHECKS["T2"][0], "inconclusive",
            f"0x{low:02X} and 0x{top:02X} both read about the same "
            f"({at_low} vs {at_top}), so the fan is already saturated at "
            f"0x{low:02X} and there is no headroom to tell a masked top bit "
            "from a maxed-out impeller. Re-run with a lower "
            "`ceiling_probe_low`, below where this unit tops out.",
            evidence,
        )

    if abs(at_high_bit - at_low) <= plan.tracking_tolerance:
        return Finding(
            "T2", CHECKS["T2"][0], "supported",
            f"0x{high_bit:02X} ran at {at_high_bit}, the same as 0x{low:02X} "
            f"({at_low}) and well below 0x{top:02X} ({at_top}). Setting the top "
            "bit put the fan back where the low 7 bits point, so the demand is "
            "7-bit and bit 7 is masked.",
            evidence,
        )
    return Finding(
        "T2", CHECKS["T2"][0], "contradicted",
        f"0x{high_bit:02X} ran at {at_high_bit} rather than dropping back to "
        f"0x{low:02X}'s {at_low}, so bit 7 is not masked and the demand is "
        "wider than 7 bits (or clamped at the top rather than truncated). The "
        "0x00-0x7F range in the capture was just the range the dial used.",
        evidence,
    )


def check_keepalive_register(
    rec: Recorder, driver: SpeedDriver, plan: Plan, report: Reporter
) -> Finding:
    """T5: which register keeps the fan turning - 0x15 writes, or 0x03 kicks?

    Spins the fan up, then goes quiet for `plan.quiet` seconds in two different
    ways and checks whether it is still turning at the end of each:

      arm A  total silence, nothing transmitted at all
      arm B  0x03 kicks only, no 0x15 writes

    Reading the speed *during* the window does not work, and two hardware runs
    were wasted finding that out. This fan stops answering altogether while the
    bus is silent - four retries got nothing - and on the one occasion it did
    answer, the register still held its pre-window value, because it only
    refreshes every few seconds and the fan is not being polled.

    So the measurement is the recovery instead: resume the driver and watch. If
    the window stopped the impeller it has to coast all the way to zero before
    it can climb again, and that dip is unmistakable. If the window did not stop
    it, the reading never dips. The driver's own writes are the observation
    channel, which sidesteps both problems at once - they are the frames the fan
    reliably answers.
    """
    rec.check = "T5"
    _spin_up(driver, plan, report)
    running = driver.actual
    if running is None or running <= plan.stopped_tolerance:
        return Finding(
            "T5", CHECKS["T5"][0], "inconclusive",
            f"The fan never got turning (actual {running}), so there is nothing "
            "to keep alive. Check the demand actually spins this unit.",
            [f"running actual={running}"],
        )
    report(f"    fan turning at actual {running}")

    def quiet_window(label: str, kick: bool) -> int | None:
        report(f"    arm {label}: {'kicking 0x03 only' if kick else 'silence'}"
               f" for {plan.quiet:g}s")
        driver.pause()
        deadline = time.monotonic() + plan.quiet
        while time.monotonic() < deadline:
            if kick:
                rec.kick_deadman(phase=f"arm {label}: 0x03 kick")
                time.sleep(rec.bus.config.deadman_interval)
            else:
                time.sleep(0.1)
        rec.phase = f"arm {label}: recovery"
        report(f"    arm {label}: resuming, watching {plan.recovery_watch:g}s"
               " for a coast to zero")
        driver.reset_extremes()  # before resuming, so the first answer counts
        driver.resume()
        _hold(driver, plan.recovery_watch, report, label=f"    arm {label}")
        low = driver.lowest
        rec.phase = ""  # or every later hold inherits this arm's label
        report(f"    arm {label}: lowest actual after the window was {low}")
        return low

    low_after_silence = quiet_window("A", kick=False)
    _spin_up(driver, plan, report)  # get it fully back up before the second arm
    recovered = driver.actual
    low_after_kicks = quiet_window("B", kick=True)
    driver.resume()

    evidence = [
        f"running actual={running}",
        f"lowest after silence={low_after_silence}",
        f"recovered before arm B actual={recovered}",
        f"lowest after 0x03 kicks={low_after_kicks}",
    ]
    after_silence, after_kicks = low_after_silence, low_after_kicks

    def stopped(value: int | None) -> bool:
        return value is not None and value <= max(plan.stopped_tolerance, running // 2)

    if after_silence is None or after_kicks is None:
        return Finding(
            "T5", CHECKS["T5"][0], "inconclusive",
            "One of the windows ended without a readable answer.", evidence,
        )
    if recovered is None or recovered <= plan.stopped_tolerance:
        return Finding(
            "T5", CHECKS["T5"][0], "inconclusive",
            "The fan did not come back up after arm A, so arm B started from a "
            "standstill and its result says nothing about 0x03.",
            evidence,
        )

    if stopped(after_silence) and stopped(after_kicks):
        return Finding(
            "T5", CHECKS["T5"][0], "supported",
            f"The fan wound down under silence ({running} -> {after_silence}) "
            f"and wound down just the same while 0x03 was being kicked "
            f"({recovered} -> {after_kicks}). Writes to 0x15 are what hold it "
            "up; 0x03 does not. transport.Deadman is kicking the wrong "
            "register.",
            evidence,
        )
    if stopped(after_silence) and not stopped(after_kicks):
        return Finding(
            "T5", CHECKS["T5"][0], "contradicted",
            f"The fan wound down under silence ({running} -> {after_silence}) "
            f"but kept turning on 0x03 kicks alone ({recovered} -> "
            f"{after_kicks}). 0x03 is a keepalive after all, as the write-up "
            "says - even though the captured controller never sent it.",
            evidence,
        )
    if not stopped(after_silence):
        return Finding(
            "T5", CHECKS["T5"][0], "contradicted",
            f"After {plan.quiet:g}s of complete silence the reading never "
            f"dipped below {after_silence}, against {running} before the "
            "window - the impeller never coasted down, so nothing has to be "
            "sent to keep it going and the whole spin-down premise needs "
            "revisiting. Either this unit free-runs or the window is shorter "
            "than its timeout; try a longer one.",
            evidence,
        )
    return Finding(
        "T5", CHECKS["T5"][0], "inconclusive",
        f"Silence bottomed out at {after_silence} and 0x03 kicks at "
        f"{after_kicks}, from {running} and {recovered}. Neither arm is a clean "
        "stop, so this does not separate the two registers.",
        evidence,
    )


def check_rate_not_period(
    rec: Recorder, driver: SpeedDriver, plan: Plan, report: Reporter
) -> Finding:
    """T4: with the impeller stopped the register reads ~0, not a saturated count.

    The discriminator is the stopped reading compared against the *running* one,
    rather than against a magic constant: a rate register reads near zero when
    stopped, a tacho period reads far higher than it ever does while turning.
    """
    rec.check = "T4"
    _spin_up(driver, plan, report)
    running = driver.actual
    if running is None or running <= plan.stopped_tolerance:
        return Finding(
            "T4", CHECKS["T4"][0], "inconclusive",
            f"The fan never got turning (actual {running}), so there is no "
            "running reading to compare a stopped one against.",
            [f"running actual={running}"],
        )
    report(f"    fan turning at actual {running}")

    _park(rec, driver, plan, report, phase="T4 park")
    stopped = _read_actual(rec, "stopped read", report)
    report(f"    stopped read -> actual {stopped}")

    evidence = [f"running actual={running}", f"stopped actual={stopped}"]
    if stopped is None:
        return Finding(
            "T4", CHECKS["T4"][0], "inconclusive",
            "The stopped fan did not answer with a readable value.", evidence,
        )
    if stopped <= plan.stopped_tolerance:
        return Finding(
            "T4", CHECKS["T4"][0], "supported",
            f"Stopped, the register reads {stopped} against {running} while "
            "turning. It is a RATE - it goes to zero at zero speed rather than "
            "saturating, so the period model is dead and decode_fan_speed's "
            "1/u16le column can go.",
            evidence,
        )
    if stopped > running * 2:
        return Finding(
            "T4", CHECKS["T4"][0], "contradicted",
            f"Stopped, the register reads {stopped} - far *higher* than the "
            f"{running} it read while turning. That is a tacho PERIOD or timer "
            "count, so speed goes as 1/raw and the rate model is wrong.",
            evidence,
        )
    return Finding(
        "T4", CHECKS["T4"][0], "inconclusive",
        f"Stopped it reads {stopped} against {running} running - neither near "
        "zero nor saturated. Either the impeller had not actually stopped "
        f"after {plan.spindown:g}s (try longer) or the register is neither a "
        "plain rate nor a plain period.",
        evidence,
    )


def check_minimum_runnable(
    rec: Recorder, driver: SpeedDriver, plan: Plan, report: Reporter
) -> Finding:
    """T6: the lowest demand that actually gets the fan turning.

    Walked upward from a standstill, because that is the direction the capture
    showed and because stiction makes the threshold hysteretic: the demand a
    already-turning fan keeps running at is lower than the one needed to start
    it. The number here is the *starting* threshold under a rising ramp, and is
    not the same as the demand at which a turning fan gives up.
    """
    rec.check = "T6"
    _park(rec, driver, plan, report, phase="T6 park")
    driver.resume()

    walked: list[tuple[int, int | None]] = []
    first_turning: int | None = None
    for demand in plan.threshold_ladder:
        driver.set_demand(demand, phase=f"threshold demand {demand}")
        report(f"    demand {demand:3d} (0x{demand:02X}), settling {plan.threshold_settle:g}s")
        _hold(driver, plan.threshold_settle, report)
        value = driver.actual
        walked.append((demand, value))
        report(f"    demand {demand:3d} -> actual {value}")
        if value is not None and value > plan.stopped_tolerance:
            first_turning = demand
            break

    evidence = [f"demand={d} actual={a}" for d, a in walked]
    lowest = plan.threshold_ladder[0]
    if first_turning is None:
        return Finding(
            "T6", CHECKS["T6"][0], "inconclusive",
            f"Nothing up to {walked[-1][0]} got the fan turning, so the "
            "threshold is above the ladder. Extend it upward and re-run.",
            evidence,
        )
    if first_turning == lowest:
        return Finding(
            "T6", CHECKS["T6"][0], "inconclusive",
            f"The fan was already turning at the lowest demand tried "
            f"({lowest}), so this run cannot say where the floor is - or "
            "whether there is one. Extend the ladder downward.",
            evidence,
        )
    return Finding(
        "T6", CHECKS["T6"][0], "supported",
        f"The fan stayed stopped up to demand {walked[-2][0]} and started at "
        f"{first_turning} (0x{first_turning:02X}), so there is a floor. The "
        "capture suggested 24 (0x18). Rising-ramp starting threshold only - the "
        "demand a turning fan coasts to a stop at will be lower.",
        evidence,
    )


# Ordered by how a run should go, not by claim number: T1 is the cheap premise,
# the ladders come next while the fan is already turning, and the two checks
# that need a genuine standstill are last so only one spin-down is paid for.
CHECKS: dict[str, tuple[str, Callable[[Recorder, SpeedDriver, Plan, Reporter], Finding]]] = {
    "T1": ("a write to 0x15 is answered with a 2-byte actual", check_reply_shape),
    "T3": ("the answer tracks the demand, in the same units", check_demand_tracking),
    "T2": ("the demand is 7-bit, 0x00..0x7F", check_demand_ceiling),
    "T5": ("0x15 writes are the keepalive, not 0x03 kicks", check_keepalive_register),
    "T4": ("the answer is a rate, not a tacho period", check_rate_not_period),
    "T6": ("there is a minimum runnable demand", check_minimum_runnable),
}


def run_checks(
    bus: DvsBus,
    which: Iterable[str] | None = None,
    plan: Plan | None = None,
    path: Path | str | None = None,
    report: Reporter = print,
    background_keepalive: bool = False,
) -> tuple[list[Finding], list[Exchange]]:
    """Run the selected checks and return their findings plus every exchange.

    `background_keepalive` runs `transport.Deadman` alongside, kicking 0x03.
    That is needed to test a `plan.demand_register` other than 0x15: if writes
    to it don't happen to be a keepalive the fan never turns, every check
    reports "fan never got turning", and a register that does control the speed
    looks inert. Holding the fan up independently separates the two questions.

    It is refused with T5, which exists to find out what keeps the fan alive -
    a keepalive running in the background is precisely what it must not have.

    The fan is parked and the driver stopped in a `finally`, so a Ctrl-C part
    way through still leaves the impeller stopping and the transcript closed.
    """
    plan = plan or Plan()
    selected = list(CHECKS) if which is None else [c for c in CHECKS if c in set(which)]
    if not selected:
        raise ValueError("no known checks selected")
    if background_keepalive and "T5" in selected:
        raise ValueError(
            "T5 measures what keeps the fan alive, so it cannot run with a "
            "background keepalive kicking 0x03. Run it in a separate pass."
        )

    with Recorder(bus, path, report, demand_register=plan.demand_register) as rec:
        driver = SpeedDriver(rec, demand=0, interval=plan.drive_interval)
        driver.start()
        report(f"driving 0x{plan.demand_register:02X} every {plan.drive_interval:g}s"
               + (f", reading the actual from 0x{SPEED_REGISTER:02X}"
                  if plan.demand_register != SPEED_REGISTER else ""))
        keepalive = Deadman(bus) if background_keepalive else None
        if keepalive is not None:
            keepalive.start()
            report(f"background keepalive kicking 0x{DEADMAN_REGISTER:02X} every "
                   f"{bus.config.deadman_interval:g}s (T5 excluded)")
        try:
            for name in selected:
                title, check = CHECKS[name]
                report(f"\n{name}  {title}")
                finding = rec.add_finding(check(rec, driver, plan, report))
                report(f"  -> {finding.verdict.upper()}: {finding.detail}")
                if finding.fatal:
                    report("  That claim is the premise for the rest - stopping here.")
                    break
        finally:
            rec.check = "shutdown"
            try:
                if keepalive is not None:
                    # Before the park, or its kicks hold the fan up against it.
                    keepalive.stop()
                _park(rec, driver, plan, report, phase="shutdown")
            finally:
                driver.stop()
            report("driver stopped, fan told to stop")
        return list(rec.findings), list(rec.exchanges)


# -- reviewing a transcript afterwards -------------------------------------


@dataclass
class Transcript:
    runs: list[dict] = field(default_factory=list)
    exchanges: list[Exchange] = field(default_factory=list)
    findings: list[Finding] = field(default_factory=list)


def load_transcript(path: Path | str) -> Transcript:
    """Read back a transcript written by `Recorder`.

    Unknown `kind`s and unknown keys are skipped rather than raising, so an
    older transcript still loads after a field is added - the same reason
    Config.load filters.
    """
    transcript = Transcript()
    exchange_fields = set(Exchange.__dataclass_fields__)
    finding_fields = set(Finding.__dataclass_fields__)
    with Path(path).open() as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            record = json.loads(line)
            kind = record.pop("kind", None)
            if kind == "run":
                transcript.runs.append(record)
            elif kind == "exchange":
                transcript.exchanges.append(
                    Exchange(**{k: v for k, v in record.items() if k in exchange_fields})
                )
            elif kind == "finding":
                transcript.findings.append(
                    Finding(**{k: v for k, v in record.items() if k in finding_fields})
                )
    return transcript


def timeline(transcript: Transcript, bar_scale: int = 2) -> list[str]:
    """Every exchange as one line: what went out, what came back, and the speed.

    The bar is there so a spin-up or a coast-down is visible by eye at the sort
    of length these runs produce - the same trick that made the captured dial
    changes obvious.
    """
    lines = []
    for exchange in transcript.exchanges:
        demand = "   -" if exchange.demand is None else f"{exchange.demand:4d}"
        actual = "   -" if exchange.actual is None else f"{exchange.actual:4d}"
        bar = "#" * (exchange.actual // bar_scale) if exchange.actual else ""
        flag = "" if exchange.crc_ok or not exchange.rx.strip("() ") else "  CRC?"
        lines.append(
            f"{exchange.wall}  +{exchange.at:8.3f}s  {exchange.check:<9}"
            f"{exchange.phase:<28} tx {exchange.tx:<20} rx {exchange.rx:<26}"
            f" demand {demand}  actual {actual} {bar}{flag}"
        )
    return lines


def summarise(findings: Iterable[Finding]) -> list[str]:
    """The verdicts, one per line, with the evidence indented under each."""
    lines = []
    for finding in findings:
        lines.append(f"{finding.claim}  {finding.verdict.upper():<13} {finding.title}")
        lines.append(f"    {finding.detail}")
        for item in finding.evidence:
            lines.append(f"      {item}")
    return lines
