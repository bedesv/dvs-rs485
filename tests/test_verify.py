"""Theory-check tests against fake fans on a pty pair.

The point of these is that the checks can come back *contradicted*. A test tool
that only ever agrees with the theory it was built from proves nothing, so most
of what's below is a fan deliberately built to break one claim, and an assertion
that the matching check notices.

Timings are shrunk hard (settle 0.6 s against 25 s on real hardware) and the
fake fan's dynamics are scaled to match, so the whole file runs in seconds.
"""

import os
import select
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import pytest  # noqa: E402

from dvs import verify  # noqa: E402
from dvs.config import Config  # noqa: E402
from dvs.crc import CrcSpec  # noqa: E402
from dvs.transport import DvsBus  # noqa: E402

# Same rule as the other pty tests: the fake fan speaks whatever CRC Config
# defaults to, so pinning a new variant doesn't silently mute it.
CRC = Config().crc

SPEED = verify.SPEED_REGISTER
DEADMAN = 0x03
STOP = 0x05


class TheoryFan:
    """A fan that behaves the way the 0x15 theory says, with knobs to break it.

    Each knob defeats exactly one claim, which is how the checks get tested for
    their ability to disagree:

      mode="period"        breaks T4 (and T3, since the reading then inverts)
      keepalive=(0x15,0x03) breaks T5 - 0x03 holds the fan up too
      keepalive=()         breaks T5 the other way - nothing is needed at all
      seven_bit=False      breaks T2 - 0xFF really does run faster
      min_runnable=0       breaks T6 - there is no floor
      answer=False         breaks T1 - the fan says nothing
    """

    PERIOD_CONST = 60000  # picked so a period reading is nowhere near a rate one
    TEMPERATURE = b"\x1f\x01\x00"

    def __init__(
        self,
        address=0x02,
        crc: CrcSpec | None = None,
        *,
        mode="rate",
        keepalive=(SPEED,),
        demand_register=SPEED,
        min_runnable=24,
        ceiling=0x7F,
        seven_bit=True,
        alive_for=0.2,
        slew=4,
        tick=0.005,
        answer=True,
    ):
        self.master, slave = os.openpty()
        self.device = os.ttyname(slave)
        os.close(slave)
        self.address = address
        self.crc = CRC if crc is None else crc
        self.mode = mode
        self.keepalive = set(keepalive)
        self.demand_register = demand_register
        self.min_runnable = min_runnable
        self.ceiling = ceiling
        self.seven_bit = seven_bit
        self.alive_for = alive_for
        self.slew = slew
        self.tick = tick
        self.answer = answer

        self.received: list[bytes] = []
        self._demand = 0
        self._actual = 0
        self._alive_until = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._threads = [
            threading.Thread(target=self._serve, daemon=True),
            threading.Thread(target=self._physics, daemon=True),
        ]
        for thread in self._threads:
            thread.start()

    # -- the wire ----------------------------------------------------------

    def _split(self, buf: bytearray):
        """Pull complete frames off the front of `buf`; leave any partial one.

        Nothing says how long a frame is, so the checksum is the boundary -
        the same walk the bus monitor does, bounded because the longest frame
        here is a 1-byte write.
        """
        while len(buf) >= 4:
            for end in range(4, min(len(buf), 8) + 1):
                if self.crc.verify(bytes(buf[:end])):
                    frame = bytes(buf[:end])
                    del buf[:end]
                    yield frame
                    break
            else:
                return

    def _serve(self):
        buf = bytearray()
        while not self._stop.is_set():
            # select rather than a bare blocking read, so the loop comes back
            # around to check `_stop` even when the bus is quiet. A blocking
            # read can only be interrupted by the other end closing, which makes
            # teardown depend on the bus being closed before the fan - and hang
            # for good when a test closes them the other way round.
            if not select.select([self.master], [], [], 0.05)[0]:
                continue
            try:
                chunk = os.read(self.master, 64)
            except OSError:
                return
            if not chunk:
                continue
            buf += chunk
            for frame in self._split(buf):
                self.received.append(frame)
                if frame[0] != self.address:
                    continue
                reply = self._handle(frame[1], frame[2:-2])
                if reply is not None and self.answer:
                    body = bytes([self.address, frame[1]]) + reply
                    try:
                        os.write(self.master, self.crc.append(body))
                    except OSError:
                        return

    def _handle(self, register: int, payload: bytes) -> bytes | None:
        now = time.monotonic()
        with self._lock:
            if register == self.demand_register and register != SPEED:
                # A second register that also sets the speed. It answers with a
                # bare ack, not a speed - a driver that mistook this for the
                # actual would read 1 no matter how fast the fan was going.
                if payload:
                    self._demand = payload[0]
                    if register in self.keepalive:
                        self._alive_until = now + self.alive_for
                return b"\x01"
            if register == SPEED:
                if payload:  # a write, which is the keepalive under the theory
                    self._demand = payload[0]
                    if SPEED in self.keepalive:
                        self._alive_until = now + self.alive_for
                return self._reported().to_bytes(2, "little")
            if register == DEADMAN:
                if DEADMAN in self.keepalive:
                    self._alive_until = now + self.alive_for
                return None  # UNKNOWN whether a real fan acks a kick; assume not
            if register == STOP:
                self._alive_until = 0.0
                return None
            if register == 0x10:
                return self.TEMPERATURE
        return None

    def _reported(self) -> int:
        if self.mode == "period":
            # A tacho period: large when slow, saturated when stopped.
            return 0xFFFF if self._actual == 0 else max(1, self.PERIOD_CONST // self._actual)
        return self._actual

    # -- the impeller ------------------------------------------------------

    def _physics(self):
        while not self._stop.is_set():
            with self._lock:
                alive = not self.keepalive or time.monotonic() < self._alive_until
                demand = self._demand & 0x7F if self.seven_bit else self._demand
                target = 0 if (not alive or demand < self.min_runnable) else min(demand, self.ceiling)
                if self._actual < target:
                    self._actual = min(target, self._actual + self.slew)
                elif self._actual > target:
                    self._actual = max(target, self._actual - self.slew)
            self._stop.wait(self.tick)

    @property
    def actual(self) -> int:
        with self._lock:
            return self._actual

    def close(self):
        """Shut down cleanly whichever order the fan and the bus are closed in.

        `_serve` polls with a timeout, so setting `_stop` is enough to end it
        and the master is only closed once no thread is reading it - closing an
        fd another thread is blocked on hangs on macOS.
        """
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=2.0)
        try:
            os.close(self.master)
        except OSError:
            pass


def make_bus(device, **overrides):
    config = Config(
        port=device,
        address=0x02,
        log_path="",
        # The fake fan answers in microseconds. Kept short mainly for the tests
        # where it deliberately says nothing at all - those pay this timeout on
        # every single frame, driver writes included.
        response_timeout=0.2,
        turnaround=0.0,
        deadman_interval=0.05,
        **overrides,
    )
    return DvsBus(config)


def fast_plan(**overrides) -> verify.Plan:
    """Real-hardware timings shrunk ~40x, with ladders trimmed to match."""
    defaults = dict(
        settle=0.6,
        spindown=0.6,
        quiet=0.4,
        recovery_watch=0.6,
        threshold_settle=0.4,
        drive_interval=0.05,
        tracking_ladder=(0x18, 0x38, 0x58, 0x7F),
        threshold_ladder=(0x04, 0x0C, 0x14, 0x1C, 0x24),
    )
    defaults.update(overrides)
    return verify.Plan(**defaults)


def silent(*_args, **_kwargs):
    """Checks report through a callback; tests don't want the commentary."""


def run(fan, which=None, plan=None, path=None):
    with make_bus(fan.device) as bus:
        return verify.run_checks(
            bus, which=which, plan=plan or fast_plan(), path=path, report=silent
        )


def verdicts(findings):
    return {f.claim: f.verdict for f in findings}


@pytest.fixture
def fan():
    f = TheoryFan()
    yield f
    f.close()


@pytest.fixture
def fan_factory():
    """Build a fan with custom knobs and close it however the test ends."""
    built = []

    def make(**kwargs):
        f = TheoryFan(**kwargs)
        built.append(f)
        return f

    yield make
    for f in built:
        f.close()


# -- the theory holding up -------------------------------------------------


def test_fan_built_to_the_theory_supports_every_claim(fan):
    findings, _ = run(fan)
    assert verdicts(findings) == {c: "supported" for c in verify.CHECKS}


def test_write_is_answered_with_two_bytes(fan):
    findings, exchanges = run(fan, which=["T1"])
    assert findings[0].verdict == "supported"
    write = next(e for e in exchanges if e.demand is not None)
    assert write.actual is not None and write.crc_ok


# -- the theory being wrong ------------------------------------------------


def test_period_register_contradicts_the_rate_claim():
    f = TheoryFan(mode="period")
    try:
        findings, _ = run(f, which=["T3", "T4"])
    finally:
        f.close()
    seen = verdicts(findings)
    # A period reading falls as the fan speeds up, so both the tracking claim
    # and the rate claim have to notice, not just the one aimed at it.
    assert seen["T4"] == "contradicted"
    assert seen["T3"] == "contradicted"
    stopped = next(f for f in findings if f.claim == "T4")
    assert "PERIOD" in stopped.detail


def test_deadman_register_also_holding_the_fan_contradicts_the_keepalive_claim():
    f = TheoryFan(keepalive=(SPEED, DEADMAN))
    try:
        findings, _ = run(f, which=["T5"])
    finally:
        f.close()
    finding = findings[0]
    assert finding.verdict == "contradicted"
    assert "0x03 is a keepalive after all" in finding.detail


def test_free_running_fan_contradicts_the_keepalive_claim_the_other_way():
    f = TheoryFan(keepalive=())
    try:
        findings, _ = run(f, which=["T5"])
    finally:
        f.close()
    finding = findings[0]
    assert finding.verdict == "contradicted"
    assert "never dipped below" in finding.detail


def test_wider_than_seven_bit_demand_contradicts_the_ceiling_claim():
    f = TheoryFan(seven_bit=False, ceiling=0xFF)
    try:
        findings, _ = run(f, which=["T2"])
    finally:
        f.close()
    assert findings[0].verdict == "contradicted"
    assert "wider than 7 bits" in findings[0].detail


def test_tracking_excludes_the_saturation_plateau_from_the_residuals():
    """The real fan matches the demand exactly, then tops out at an actual of 80.

    Scored with the plateau included that reads as "worst gap 47" and buries the
    result. The plateau is a mechanical ceiling, so it gets reported separately
    and kept out of the same-units arithmetic.
    """
    f = TheoryFan(ceiling=80)
    plan = fast_plan(tracking_ladder=(0x18, 0x38, 0x50, 0x68, 0x7F))
    try:
        findings, _ = run(f, which=["T3"], plan=plan)
    finally:
        f.close()
    finding = findings[0]
    assert finding.verdict == "supported"
    assert "the same units" in finding.detail
    assert "saturates" in finding.detail and "held at 80" in finding.detail
    assert "worst gap 0" in finding.detail


def test_saturated_impeller_leaves_the_ceiling_claim_inconclusive():
    """The real fan tops out at an actual of 80, well under the demand range.

    Comparing 0x7F against 0xFF then reads the same either way, which the first
    version of this check scored as `supported`. With no headroom above the
    probe point the honest answer is that the run cannot tell.
    """
    f = TheoryFan(ceiling=50)  # saturates below ceiling_probe_low (0x40)
    try:
        findings, _ = run(f, which=["T2"])
    finally:
        f.close()
    assert findings[0].verdict == "inconclusive"
    assert "saturated" in findings[0].detail


def test_ceiling_probe_stays_inside_the_range_the_fan_can_reach():
    """Regression: the probe must not be two demands the fan can't tell apart."""
    plan = verify.Plan()
    assert plan.ceiling_probe_low < plan.ceiling_probe_top <= verify.MAX_DEMAND
    # low and low|0x80 must differ in exactly one bit, or the comparison is not
    # about the top bit at all.
    assert (plan.ceiling_probe_low ^ (plan.ceiling_probe_low | 0x80)) == 0x80


def test_fan_with_no_floor_leaves_the_threshold_claim_inconclusive():
    f = TheoryFan(min_runnable=0)
    try:
        findings, _ = run(f, which=["T6"])
    finally:
        f.close()
    assert findings[0].verdict == "inconclusive"
    assert "Extend the ladder downward" in findings[0].detail


def test_silent_fan_is_fatal_and_stops_the_run():
    f = TheoryFan(answer=False)
    try:
        findings, _ = run(f)
    finally:
        f.close()
    assert len(findings) == 1, "a fan that says nothing should abort the run"
    assert findings[0].claim == "T1"
    assert findings[0].fatal
    assert findings[0].verdict == "inconclusive"


def test_wrong_crc_is_reported_as_a_crc_problem_not_bad_data():
    """A mismatch on a reply almost always means the wrong variant, and says so."""
    f = TheoryFan(crc=CrcSpec("CCITT-FALSE", "little"))
    try:
        with make_bus(f.device, crc_algorithm="XMODEM", crc_byte_order="big") as bus:
            findings, _ = verify.run_checks(
                bus, which=["T1"], plan=fast_plan(), report=silent
            )
    finally:
        f.close()
    assert findings[0].fatal
    assert "CRC" in findings[0].detail


# -- the transcript --------------------------------------------------------


def test_transcript_records_both_directions_and_reloads(fan, tmp_path):
    path = tmp_path / "run.jsonl"
    findings, exchanges = run(fan, which=["T1", "T4"], path=path)

    reloaded = verify.load_transcript(path)
    assert len(reloaded.exchanges) == len(exchanges)
    assert [f.claim for f in reloaded.findings] == [f.claim for f in findings]
    assert reloaded.runs and reloaded.runs[0]["crc"] == str(Config().crc)

    for exchange in reloaded.exchanges:
        assert exchange.tx, "every exchange records what was sent"
        assert exchange.rx, "and what came back, even if that was nothing"
        assert exchange.check and exchange.phase
    # Timestamps have to be fine enough to order frames inside one poll cycle.
    assert any("." in e.wall for e in reloaded.exchanges)
    assert reloaded.exchanges == sorted(reloaded.exchanges, key=lambda e: e.at)


def test_transcript_survives_an_interrupted_run(fan, tmp_path):
    """Ctrl-C part way through still has to leave a reviewable record."""
    path = tmp_path / "run.jsonl"
    plan = fast_plan()
    with make_bus(fan.device) as bus:
        with verify.Recorder(bus, path, report=silent) as rec:
            driver = verify.SpeedDriver(rec, interval=plan.drive_interval)
            driver.start()
            try:
                rec.check = "T1"
                verify.check_reply_shape(rec, driver, plan, silent)
                raise KeyboardInterrupt
            except KeyboardInterrupt:
                pass
            finally:
                driver.stop()
    assert verify.load_transcript(path).exchanges


def test_recovery_window_is_identifiable_in_the_transcript(fan, tmp_path):
    """T5's verdict rests on the recovery watch, so it has to be findable later.

    The driver's writes *are* that measurement. Labelling them all "hold" left
    the one window that matters indistinguishable from every other in a
    2000-line transcript.
    """
    path = tmp_path / "run.jsonl"
    run(fan, which=["T5"], path=path)
    transcript = verify.load_transcript(path)
    phases = {e.phase for e in transcript.exchanges if e.check == "T5"}
    assert "arm A: recovery" in phases
    assert "arm B: recovery" in phases
    recovery = [e for e in transcript.exchanges if e.phase == "arm A: recovery"]
    assert len(recovery) > 1, "the whole watch should be labelled, not one frame"
    assert all(e.demand is not None for e in recovery), "driver writes carry the demand"
    # And the label must not leak into everything that follows it.
    assert "hold" in phases


def test_transcript_load_skips_unknown_kinds_and_fields(tmp_path):
    path = tmp_path / "old.jsonl"
    path.write_text(
        '{"kind": "run", "wall": "2026-08-05T09:00:00.000", "crc": "x", "address": 2}\n'
        '{"kind": "exchange", "at": 0.1, "wall": "w", "check": "T1", "phase": "p",'
        ' "tx": "02 15 38", "rx": "02 15 38 00", "crc_ok": true, "future_field": 9}\n'
        '{"kind": "something-new", "whatever": 1}\n'
        "\n"
    )
    transcript = verify.load_transcript(path)
    assert len(transcript.exchanges) == 1
    assert transcript.exchanges[0].check == "T1"
    assert not transcript.findings


def test_timeline_and_summary_render(fan, tmp_path):
    path = tmp_path / "run.jsonl"
    findings, _ = run(fan, which=["T1"], path=path)
    transcript = verify.load_transcript(path)
    lines = verify.timeline(transcript)
    assert len(lines) == len(transcript.exchanges)
    assert all("tx" in line and "rx" in line for line in lines)
    summary = verify.summarise(findings)
    assert any("SUPPORTED" in line for line in summary)


# -- the driver ------------------------------------------------------------


def test_driver_catches_a_dip_the_caller_would_sample_straight_past(fan):
    """T5's verdict hangs on spotting a coast to zero.

    The fake fan restarts within ~70 ms of the writes resuming, well inside the
    100 ms the watcher polls at, so sampling `driver.actual` can miss the dip
    entirely and invert the verdict. Tracking the minimum off every reply can't.
    """
    plan = fast_plan()
    with make_bus(fan.device) as bus:
        with verify.Recorder(bus, None, report=silent) as rec:
            driver = verify.SpeedDriver(rec, interval=plan.drive_interval)
            driver.start()
            try:
                driver.set_demand(0x38)
                time.sleep(0.4)
                assert driver.actual and driver.actual > 0

                driver.pause()
                time.sleep(fan.alive_for + 0.2)  # fan coasts to a stop
                driver.reset_extremes()
                driver.resume()
                time.sleep(0.5)  # long enough to be back up to speed
            finally:
                driver.stop()
    assert driver.actual > 0, "should have recovered by the end"
    assert driver.lowest == 0, "the coast to zero has to be caught, not sampled past"


def test_reset_extremes_forgets_the_previous_window(fan):
    plan = fast_plan()
    with make_bus(fan.device) as bus:
        with verify.Recorder(bus, None, report=silent) as rec:
            driver = verify.SpeedDriver(rec, interval=plan.drive_interval)
            driver.start()
            try:
                driver.set_demand(0x38)
                time.sleep(0.5)
                assert driver.lowest == 0, "started from a standstill"
                driver.reset_extremes()
                assert driver.lowest is None
                time.sleep(0.3)
                assert driver.lowest and driver.lowest > 0, "no stale zero"
            finally:
                driver.stop()


def test_driver_keeps_the_fan_turning_and_pausing_lets_it_stop(fan):
    plan = fast_plan()
    with make_bus(fan.device) as bus:
        with verify.Recorder(bus, None, report=silent) as rec:
            driver = verify.SpeedDriver(rec, interval=plan.drive_interval)
            driver.start()
            try:
                driver.set_demand(0x38)
                time.sleep(0.4)
                assert driver.actual and driver.actual > 0
                assert fan.actual > 0

                driver.pause()
                time.sleep(fan.alive_for + 0.3)
                assert fan.actual == 0, "0x15 writes are what hold this fan up"

                driver.resume()
                time.sleep(0.4)
                assert fan.actual > 0
            finally:
                driver.stop()
    assert driver.writes > 0 and driver.answers > 0


def test_run_parks_the_fan_even_when_a_check_raises(fan):
    plan = fast_plan()

    def explode(rec, driver, plan, report):
        raise RuntimeError("boom")

    original = verify.CHECKS["T1"]
    verify.CHECKS["T1"] = (original[0], explode)
    try:
        with make_bus(fan.device) as bus:
            with pytest.raises(RuntimeError):
                verify.run_checks(bus, which=["T1"], plan=plan, report=silent)
    finally:
        verify.CHECKS["T1"] = original
    assert fan.actual == 0, "the impeller must be stopping even after a crash"


# -- a demand register other than 0x15 -------------------------------------

OTHER = 0x11  # what config.setpoint_register claims, unverified


def test_a_second_register_that_controls_speed_is_detected(fan_factory):
    """0x11 sets the speed, and answers writes with an ack rather than a speed.

    The actual has to come from 0x15 regardless. Reading the demand register's
    own reply would give a constant 1 and score the ladder as flat.
    """
    f = fan_factory(demand_register=OTHER, keepalive=(OTHER,))
    plan = fast_plan(demand_register=OTHER)
    findings, exchanges = run(f, which=["T3"], plan=plan)
    assert findings[0].verdict == "supported"
    assert "the same units" in findings[0].detail
    # Both registers appear: the demand goes to one, the actual is read from the
    # other, so the transcript must show both per tick.
    regs = {e.register for e in exchanges}
    assert regs >= {OTHER, SPEED}


def test_an_inert_demand_register_does_not_look_like_a_working_one(fan_factory):
    """Writing somewhere that does nothing must not read as a supported claim."""
    f = fan_factory(demand_register=OTHER, keepalive=(SPEED,))
    plan = fast_plan(demand_register=OTHER)
    # 0x03 holds the fan up, so "nothing happened" can't be blamed on a fan that
    # was never turning - except here nothing holds it up, so it stays stopped.
    findings, _ = run(f, which=["T3"], plan=plan)
    assert findings[0].verdict in ("contradicted", "inconclusive")
    assert findings[0].verdict != "supported"


def test_a_speed_read_that_fails_its_checksum_is_not_believed(fan_factory):
    """A corrupt reply must not be accepted as a speed.

    The fan answers erratically after a silent spell, which is precisely when
    T4 and T5 read it, so a mis-signed frame reaching a verdict as a number is
    the failure that would be hardest to spot afterwards.
    """
    f = fan_factory(crc=CrcSpec("CCITT-FALSE", "little"))
    with make_bus(f.device, crc_algorithm="XMODEM", crc_byte_order="big") as bus:
        with verify.Recorder(bus, None, report=silent) as rec:
            # The fan replies, but signed with a variant the bus won't verify.
            assert verify._read_actual(rec, "probe", silent, attempts=2) is None


def test_background_keepalive_is_refused_with_t5(fan):
    with make_bus(fan.device) as bus:
        with pytest.raises(ValueError, match="cannot run with a background keepalive"):
            verify.run_checks(
                bus, which=["T5"], plan=fast_plan(), report=silent,
                background_keepalive=True,
            )


def test_background_keepalive_holds_the_fan_up_for_an_inert_register(fan_factory):
    """The point of the option: separate "does nothing" from "isn't a keepalive".

    This fan's speed is set by 0x11 but only 0x03 keeps it alive. Without the
    background keepalive it never turns and 0x11 looks inert.
    """
    f = fan_factory(demand_register=OTHER, keepalive=(DEADMAN,))
    plan = fast_plan(demand_register=OTHER)
    with make_bus(f.device) as bus:
        try:
            findings, _ = verify.run_checks(
                bus, which=["T3"], plan=plan, report=silent,
                background_keepalive=True,
            )
        finally:
            f.close()
    assert findings[0].verdict == "supported", (
        "0x11 does control the speed; only the keepalive was missing"
    )


def test_unknown_check_name_is_rejected(fan):
    with make_bus(fan.device) as bus:
        with pytest.raises(ValueError):
            verify.run_checks(bus, which=["T99"], plan=fast_plan(), report=silent)
