import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from dvs import calibrate, config, crc, discovery, monitor, protocol, transport  # noqa: E402


def test_crc_check_values():
    """Every algorithm reproduces its catalogue check value for b'123456789'."""
    for algorithm in crc.ALGORITHMS.values():
        assert algorithm(b"123456789") == algorithm.check, algorithm.name


def test_trailer_byte_order():
    spec_le = crc.CrcSpec("CCITT-FALSE", "little")
    spec_be = crc.CrcSpec("CCITT-FALSE", "big")
    body = b"\x01\x15"
    value = spec_le.compute(body)
    assert spec_le.trailer(body) == value.to_bytes(2, "little")
    assert spec_be.trailer(body) == value.to_bytes(2, "big")
    assert spec_le.trailer(body) == spec_be.trailer(body)[::-1]


def test_append_and_verify_roundtrip():
    for name in crc.ALGORITHMS:
        for order in crc.BYTE_ORDERS:
            spec = crc.CrcSpec(name, order)
            frame = spec.append(b"\x01\x15\xde\xad")
            assert spec.verify(frame)
            assert not spec.verify(frame[:-1] + bytes([frame[-1] ^ 0xFF]))


def test_identify_narrows_candidates():
    """A frame built with a known spec is always among the identified survivors."""
    truth = crc.CrcSpec("KERMIT", "big")
    frames = [
        truth.append(b"\x01\x15"),
        truth.append(b"\x01\x10\x00\x01\x02\x03"),
        truth.append(b"\x01\x03\x7f"),
    ]
    survivors = crc.identify(frames)
    assert truth in survivors
    assert len(survivors) == 1


def test_identify_rejects_bad_frames():
    assert crc.identify([b"\x01\x15\x00\x00", b"\x01\x10\xff\xff"]) == []


def test_frame_encode():
    spec = crc.CrcSpec("CCITT-FALSE", "little")
    frame = protocol.Frame(0x01, 0x15)
    encoded = frame.encode(spec)
    assert encoded[:2] == b"\x01\x15"
    assert len(encoded) == 4
    assert spec.verify(encoded)
    assert frame.is_read

    write = protocol.Frame(0x01, 0x03, b"\x7f")
    assert len(write.encode(spec)) == 5
    assert not write.is_read


def test_frame_rejects_out_of_range():
    spec = crc.CrcSpec()
    for bad in (protocol.Frame(0x100, 0x15), protocol.Frame(0x01, -1)):
        try:
            bad.encode(spec)
        except ValueError:
            continue
        raise AssertionError("expected ValueError")


def test_response_slicing():
    spec = crc.CrcSpec()
    raw = spec.append(b"\x01\x15\x12\x34")
    response = protocol.Response(raw=raw, crc=spec)
    assert response.crc_ok
    assert response.body == b"\x01\x15\x12\x34"
    assert response.payload == b"\x12\x34"
    assert not protocol.Response(raw=b"", crc=spec)


def test_unsafe_registers_excluded_from_probing():
    assert 0x04 in protocol.UNSAFE_TO_PROBE  # set address
    assert 0x05 in protocol.UNSAFE_TO_PROBE  # stop


class _ReplayBus:
    """Hands out canned replies to read_register, counting the reads."""

    def __init__(self, replies):
        self.replies = list(replies)
        self.reads = 0

    def read_register(self, register, address=None):
        raw = self.replies[self.reads]
        self.reads += 1
        return protocol.Response(raw=raw, crc=crc.CrcSpec())


def _speed_reply(value):
    return crc.CrcSpec().append(bytes([0x01, 0x15]) + value.to_bytes(2, "little"))


def test_speed_read_retries_before_giving_up_on_none():
    """One missed reply must not be recorded as an unreadable speed."""
    attempts = discovery.SPEED_READ_ATTEMPTS
    bus = _ReplayBus([b""] * (attempts - 1) + [_speed_reply(54)])
    assert discovery._speed_word(bus, 0x15, retry_delay=0) == 54
    assert bus.reads == attempts


def test_speed_read_gives_up_after_the_configured_attempts():
    attempts = discovery.SPEED_READ_ATTEMPTS
    assert attempts == 4
    bus = _ReplayBus([b""] * attempts)
    assert discovery._speed_word(bus, 0x15, retry_delay=0) is None
    assert bus.reads == attempts


def test_speed_read_stops_early_when_the_fan_answers():
    bus = _ReplayBus([_speed_reply(1200)] + [b""] * 3)
    assert discovery._speed_word(bus, 0x15, retry_delay=0) == 1200
    assert bus.reads == 1


def test_speed_read_only_reports_the_give_up():
    """A recovery on a later attempt is not worth a line of output."""
    lines = []
    bus = _ReplayBus([b"", _speed_reply(54)])
    discovery._speed_word(bus, 0x15, retry_delay=0, report=lines.append)
    assert lines == []
    bus = _ReplayBus([b""] * discovery.SPEED_READ_ATTEMPTS)
    discovery._speed_word(bus, 0x15, retry_delay=0, report=lines.append)
    assert len(lines) == 1 and "no reply" in lines[0]


def test_unreadable_candidate_is_not_a_measured_zero():
    """A None end means 'no data', which must be distinguishable from no movement."""
    missing = discovery.SetpointCandidate(0x11, None, 54)
    still = discovery.SetpointCandidate(0x12, 54, 54)
    assert missing.unreadable and missing.delta == 0
    assert not still.unreadable and still.delta == 0


def test_moved_covers_any_speed_change():
    assert discovery.SetpointCandidate(0x11, 0, 1200).moved
    assert discovery.SetpointCandidate(0x11, 300, 1200).moved
    assert discovery.SetpointCandidate(0x11, 1200, 300).moved
    assert not discovery.SetpointCandidate(0x11, 0, 0).moved
    assert not discovery.SetpointCandidate(0x11, 300, 300).moved
    # An unreadable end is not evidence of movement either way.
    assert not discovery.SetpointCandidate(0x11, 0, None).moved
    assert not discovery.SetpointCandidate(0x11, None, 1200).moved


class _FakeFanBus:
    """One register really controls speed; 0x15 reports the resulting speed.

    `start_threshold` is the setpoint below which the fan won't turn at all, so
    a high one makes the register read 0 at the hunt's low end and a 0 makes it
    already spinning there - both have to get parked.

    A deadman kick puts the fan in its running state and the stop command takes
    it out again; the setpoint only has an effect while it's running. That is
    what makes a zero write on its own insufficient - the keepalive is still
    kicking, so anything but a real stop leaves the fan turning.

    Enough of DvsBus for hunt_setpoint and its Deadman: every call is recorded
    so a test can assert on the write order, and a lock keeps the keepalive
    thread from interleaving with the foreground reads.
    """

    def __init__(self, setpoint_register, start_threshold=0x40):
        self.config = config.Config()
        self.setpoint_register = setpoint_register
        self.start_threshold = start_threshold
        # Starts running: hunt_setpoint has a deadman going from the first write.
        self.running = True
        self.setpoint = 0
        self.writes = []
        self.lock = threading.RLock()

    @property
    def speed(self):
        if not self.running or self.setpoint < self.start_threshold:
            return 0
        return self.setpoint * 10

    def log(self, message):
        pass

    def transaction(self, frame, timeout=None):
        """Deadman kicks and SpeedHold writes both land here rather than on write_register.

        A payload frame is routed through `write_register` so the two paths stay
        indistinguishable to the fan, which is what makes `self.writes` a
        complete record of what went out however it was sent.
        """
        with self.lock:
            if frame.register == transport.Deadman.REGISTER:
                self.running = True
                return protocol.Response(raw=b"", crc=self.config.crc)
            if frame.payload:
                self.write_register(frame.register, frame.payload)
            if frame.register == 0x15:
                return protocol.Response(raw=_speed_reply(self.speed), crc=crc.CrcSpec())
            return protocol.Response(raw=b"", crc=self.config.crc)

    def read_register(self, register, address=None):
        with self.lock:
            if register == 0x15:
                return protocol.Response(raw=_speed_reply(self.speed), crc=crc.CrcSpec())
            return protocol.Response(raw=b"", crc=crc.CrcSpec())

    def write_register(self, register, payload=b"", address=None):
        with self.lock:
            self.writes.append((register, payload))
            if register == protocol.STOP_REGISTER:
                self.running = False
            elif register == self.setpoint_register and payload:
                self.setpoint = payload[0]
            return protocol.Response(raw=b"", crc=crc.CrcSpec())


def test_park_fan_stops_the_fan_and_brings_the_keepalive_back():
    """A zero setpoint can't park the fan on its own - the stop command has to go out."""
    bus = _FakeFanBus(setpoint_register=0x11, start_threshold=0)
    deadman = transport.Deadman(bus)
    deadman.start()
    try:
        bus.write_register(0x11, b"\xc0")
        assert bus.speed > 0  # the fan really is spinning before we park it
        discovery._park_fan(bus, deadman, 0x11, 0, lambda _: None)
        stop = (protocol.STOP_REGISTER, b"")
        assert stop in bus.writes
        # Zero the setpoint first, so a resumed keepalive can't spin it back up.
        assert bus.writes.index((0x11, b"\x00")) < bus.writes.index(stop)
        assert bus.speed == 0
        assert deadman.running  # ...and the keepalive is back for the next candidate
    finally:
        deadman.stop()


def test_hunt_parks_a_moving_register_before_the_next_candidate():
    """A register that spun the fan up must be parked before the next candidate."""
    bus = _FakeFanBus(setpoint_register=0x11)
    results = discovery.hunt_setpoint(
        bus, bus.config.address, candidates=[0x10, 0x11, 0x12], settle=0, report=lambda _: None
    )
    interesting = (0x10, 0x11, 0x12, protocol.STOP_REGISTER)
    writes = [(reg, payload) for reg, payload in bus.writes if reg in interesting]
    # Zero write, then the stop command, both after 0x11's high write and before
    # 0x12 is touched.
    zeroed = writes.index((0x11, b"\x00"))
    assert writes[zeroed - 1] == (0x11, bytes([0xC0]))
    assert writes[zeroed + 1] == (protocol.STOP_REGISTER, b"")
    assert all(reg != 0x12 for reg, _ in writes[: zeroed + 1])
    # ...and it really stopped the fan, so 0x12 starts from rest.
    by_register = {c.register: c for c in results}
    assert by_register[0x11].moved
    assert by_register[0x12].low_speed == 0 and by_register[0x12].delta == 0


def test_hunt_parks_a_register_that_was_already_spinning_at_the_low_end():
    """Movement counts even when the low end never read 0."""
    bus = _FakeFanBus(setpoint_register=0x11, start_threshold=0)
    results = discovery.hunt_setpoint(
        bus, bus.config.address, candidates=[0x11, 0x12], settle=0, report=lambda _: None
    )
    by_register = {c.register: c for c in results}
    assert by_register[0x11].low_speed > 0 and by_register[0x11].moved
    assert (0x11, b"\x00") in bus.writes
    assert (protocol.STOP_REGISTER, b"") in bus.writes
    assert by_register[0x12].low_speed == 0


def test_hunt_leaves_a_still_register_alone():
    """No zeroing write for a register that never moved the fan."""
    bus = _FakeFanBus(setpoint_register=0x11)
    discovery.hunt_setpoint(
        bus, bus.config.address, candidates=[0x10], settle=0, report=lambda _: None
    )
    assert (0x10, b"\x00") not in bus.writes


def _run_short_sweep(bus, setpoints=(0x40, 0x60), settle=0.2):
    bus.config.speed_hold_interval = 0.05
    return calibrate.run_sweep(
        bus,
        setpoint_register=bus.setpoint_register,
        setpoints=list(setpoints),
        settle=settle,
        report=lambda _: None,
    )


def test_sweep_holds_the_demand_the_way_the_controller_does():
    """Each step re-asserts its demand for the whole settle, rather than writing it once.

    The wall controller holds the fan up by rewriting 0x15 about four times a
    second, and the sweep now does the same. Writing the demand once and leaving
    a deadman to keep the fan alive is what let a real sweep coast to a halt
    mid-step: one lost frame there costs the rest of the step.
    """
    bus = _FakeFanBus(setpoint_register=0x15, start_threshold=0)
    _run_short_sweep(bus, setpoints=(0x40, 0x60))

    for demand in (b"\x40", b"\x60"):
        # ~4 per step at these timings; the point is "many", not the exact count.
        assert sum(1 for reg, payload in bus.writes if (reg, payload) == (0x15, demand)) > 2
    # ...and none of it leans on the other keepalive.
    assert not any(reg == transport.Deadman.REGISTER for reg, _ in bus.writes)


def test_sweep_reads_each_setpoint_only_after_it_has_been_asserted():
    """The demand must be on the wire before the settle, not after it."""
    bus = _FakeFanBus(setpoint_register=0x15, start_threshold=0)
    sweep = _run_short_sweep(bus, setpoints=(0x40, 0x60))

    assert [p.setpoint for p in sweep.points] == [0x40, 0x60]
    assert [p.raw_u16le for p in sweep.points] == [0x40 * 10, 0x60 * 10]


def test_sweep_parks_the_fan_when_it_finishes():
    """Letting the writes lapse only spins down after the timeout - say stop explicitly."""
    bus = _FakeFanBus(setpoint_register=0x15, start_threshold=0)
    _run_short_sweep(bus, setpoints=(0x40,))

    assert bus.writes[-1] == (protocol.STOP_REGISTER, b"")
    assert bus.speed == 0


def test_sweep_parks_the_fan_even_when_a_step_blows_up():
    """A sweep abandoned part way through must not leave the impeller turning."""
    bus = _FakeFanBus(setpoint_register=0x15, start_threshold=0)

    def explode(setpoint, raw_text):
        raise KeyboardInterrupt

    bus.config.speed_hold_interval = 0.05
    try:
        calibrate.run_sweep(
            bus,
            setpoint_register=0x15,
            setpoints=[0x40],
            settle=0.05,
            prompt_rpm=explode,
            report=lambda _: None,
        )
    except KeyboardInterrupt:
        pass
    assert bus.writes[-1] == (protocol.STOP_REGISTER, b"")
    assert bus.speed == 0


def test_speed_hold_set_demand_writes_immediately():
    """Waiting for the next tick would spend part of every settle on the old demand."""
    bus = _FakeFanBus(setpoint_register=0x15, start_threshold=0)
    hold = transport.SpeedHold(bus, register=0x15, interval=60.0)

    hold.set_demand(0x33)  # never started, so only the immediate write can appear
    assert bus.writes == [(0x15, b"\x33")]
    assert hold.demand == 0x33
    assert hold.actual == 0x33 * 10  # the reply to a write carries the actual speed


def test_speed_hold_survives_a_failing_write():
    """A blip on the bus must not silently end the hold and drop the fan."""
    bus = _FakeFanBus(setpoint_register=0x15, start_threshold=0)
    failures = []

    original = bus.transaction

    def flaky(frame, timeout=None):
        if len(failures) < 1:
            failures.append(frame)
            raise transport.BusError("nothing came back")
        return original(frame, timeout)

    bus.transaction = flaky
    hold = transport.SpeedHold(bus, register=0x15, demand=0x20, interval=0.02)
    hold.start()
    try:
        deadline = time.monotonic() + 2.0
        while hold.writes < 3 and time.monotonic() < deadline:
            time.sleep(0.01)
    finally:
        hold.stop()
    assert hold.failures == 1
    assert hold.writes >= 3  # kept going after the failure


def _sweep_from(pairs):
    sweep = calibrate.Sweep()
    for raw, rpm in pairs:
        sweep.points.append(
            calibrate.SweepPoint(
                setpoint=0,
                raw_bytes="",
                raw_u8=None,
                raw_u16le=raw,
                raw_u16be=raw,
                measured_rpm=rpm,
            )
        )
    return sweep


def test_fit_picks_rate_model():
    """Synthetic data where raw is exactly RPM must fit the rate model."""
    rpms = [200, 400, 600, 900, 1200, 1500]
    sweep = _sweep_from([(rpm, rpm) for rpm in rpms])
    fits = calibrate.fit_models(sweep)
    assert fits[0].model == "rate"
    assert abs(fits[0].slope - 1.0) < 1e-6
    assert fits[0].r_squared > 0.999


def test_fit_picks_period_model():
    """Synthetic data where raw is a tacho period must fit the period model."""
    k = 1_000_000
    rpms = [200, 400, 600, 900, 1200, 1500]
    sweep = _sweep_from([(round(k / rpm), rpm) for rpm in rpms])
    fits = calibrate.fit_models(sweep)
    assert fits[0].model == "period"
    assert abs(fits[0].slope - k) / k < 0.01
    assert fits[0].r_squared > 0.999


def test_fit_needs_enough_points():
    sweep = _sweep_from([(100, 100), (200, 200)])
    try:
        calibrate.fit_models(sweep)
    except ValueError:
        return
    raise AssertionError("expected ValueError for too few points")


def test_sweep_csv_roundtrip(tmp_path):
    sweep = _sweep_from([(100, 200.0), (300, 600.0)])
    path = tmp_path / "sweep.csv"
    sweep.save_csv(path)
    loaded = calibrate.Sweep.load_csv(path)
    assert [p.raw_u16le for p in loaded.points] == [100, 300]
    assert [p.measured_rpm for p in loaded.points] == [200.0, 600.0]
    assert loaded.points[0].raw_u8 is None


def test_interpret_reports_direct_rpm():
    rpms = [200, 400, 600, 900, 1200, 1500]
    fits = calibrate.fit_models(_sweep_from([(rpm, rpm) for rpm in rpms]))
    text = " ".join(calibrate.interpret(fits))
    assert "RATE" in text
    assert "raw is RPM directly" in text


# The four (panel reading, raw count) pairs the 1/16 degC scale was derived
# from - read off the wall controller with the bus watched at the same time.
PANEL_READINGS = [(30, 478), (21, 325), (19, 303), (18, 292)]


def test_decode_temperature_matches_the_panel():
    for shown, raw in PANEL_READINGS:
        payload = raw.to_bytes(2, "little") + b"\x00"
        text = protocol.decode_temperature(payload)
        assert f"raw={raw}" in text
        assert f"{raw / 16:.2f} degC" in text
        # Panel rounds to whole degrees and sits a little above raw/16.
        assert abs(raw / 16 - shown) < 0.75


def test_scale_makes_the_0x12_constants_round():
    """The six bytes 0x12 always answers with are +40, -35, +80 degC on this scale.

    Independent of the panel readings, and the reason no offset is applied: these
    come out round without one. If someone rescales the temperature decode, this
    should stop passing.
    """
    payload = bytes.fromhex("8002d0fd0005")
    degrees = [
        int.from_bytes(payload[i : i + 2], "little", signed=True)
        / protocol.TEMPERATURE_COUNTS_PER_DEGREE
        for i in (0, 2, 4)
    ]
    assert degrees == [40.0, -35.0, 80.0]


def test_decode_temperature_flags_unexpected_trailing_bytes():
    assert "trailing" not in protocol.decode_temperature(b"\x24\x01\x00")
    assert "trailing bytes 07" in protocol.decode_temperature(b"\x24\x01\x07")


def test_decoders_do_not_crash_on_short_payloads():
    for payload in (b"", b"\x01", b"\x01\x02", b"\x01\x02\x00"):
        protocol.decode_temperature(payload)
        protocol.decode_fan_speed(payload)


# -- bus monitor -----------------------------------------------------------

MONITOR_CRC = crc.CrcSpec()


def _frame(*body: int, payload: bytes = b"") -> bytes:
    return MONITOR_CRC.append(bytes(body) + payload)


def test_capture_line_roundtrip():
    data = b"\x02\x15\xe8\x03\xab\xcd"
    stamp, parsed = monitor.parse_line(monitor.format_line(1_700_000_000.5, data))
    assert parsed == data
    assert stamp.endswith(".500")


def test_capture_lines_ignore_comments_and_junk():
    assert monitor.parse_line("# capture started") is None
    assert monitor.parse_line("") is None
    assert monitor.parse_line("2026-08-05T09:00:00.000 not hex") is None


def test_split_frames_uses_the_checksum_as_the_boundary():
    """A request and its reply in one burst come out as two frames."""
    request = _frame(0x02, 0x10)
    reply = _frame(0x02, 0x10, payload=b"\xd2\x00\x00")
    frames, rest = monitor.split_frames(request + reply, MONITOR_CRC)
    assert frames == [request, reply]
    assert rest == b""


def test_split_frames_keeps_what_it_cannot_verify():
    """Bytes signed with another CRC variant are handed back, not dropped."""
    other = crc.CrcSpec("MODBUS", "little")
    junk = other.append(b"\x02\x15\x01")
    frames, rest = monitor.split_frames(junk, MONITOR_CRC)
    assert frames == []
    assert rest == junk


def _comments(decoder, *frames):
    return [m.comment for f in frames for m in decoder.feed("t", f)]


def test_decoder_names_the_commands():
    decoder = monitor.Decoder(MONITOR_CRC, setpoint_register=0x11)
    kick, stop, setpoint, address = _comments(
        decoder,
        _frame(0x02, 0x03),
        _frame(0x02, 0x05),
        _frame(0x02, 0x11, payload=b"\xc0"),
        _frame(0x02, 0x04, payload=b"\x07"),
    )
    assert "deadman kick" in kick and kick.startswith("0x02")
    assert "stop fan" in stop
    assert "set fan speed setpoint to 192" in setpoint
    assert "SET BUS ADDRESS to 0x07" in address


def test_decoder_pairs_a_reply_with_the_read_before_it():
    """Nothing in a payload frame says whether it's a write or an answer."""
    decoder = monitor.Decoder(MONITOR_CRC)
    read, reply = _comments(
        decoder, _frame(0x02, 0x10), _frame(0x02, 0x10, payload=b"\xd2\x00\x00")
    )
    assert "read temperature" in read
    assert "temperature is" in reply and "raw=210" in reply


def test_decoder_reads_an_unpaired_payload_frame_as_a_write():
    decoder = monitor.Decoder(MONITOR_CRC)
    (write,) = _comments(decoder, _frame(0x02, 0x10, payload=b"\xd2\x00\x00"))
    assert write.startswith("0x02  write temperature")
    # ...and a reply only pairs with a read of the *same* register.
    decoder = monitor.Decoder(MONITOR_CRC)
    _, second = _comments(
        decoder, _frame(0x02, 0x15), _frame(0x02, 0x10, payload=b"\xd2\x00\x00")
    )
    assert "write temperature" in second


def test_decoder_describes_unknown_registers_without_guessing():
    decoder = monitor.Decoder(MONITOR_CRC)
    read, write = _comments(
        decoder, _frame(0x02, 0x2A), _frame(0x02, 0x2B, payload=b"\x01\x02")
    )
    assert "read register 0x2A" in read
    assert "write register 0x2B (0x2B) = 01 02" in write


def test_decoder_flags_unframed_bytes_as_a_crc_problem():
    decoder = monitor.Decoder(crc.CrcSpec("MODBUS", "little"))
    (message,) = decoder.feed("t", _frame(0x02, 0x03))
    assert not message.framed
    assert "CRC variant" in message.comment


def test_decode_file_roundtrip(tmp_path):
    path = tmp_path / "capture.log"
    path.write_text(
        "# capture started\n"
        + monitor.format_line(1_700_000_000.0, _frame(0x02, 0x03))
        + "\n"
        + monitor.format_line(
            1_700_000_001.0,
            _frame(0x02, 0x15) + _frame(0x02, 0x15, payload=b"\xe8\x03"),
        )
        + "\n"
    )
    messages = monitor.decode_file(path, MONITOR_CRC, report=lambda _: None)
    assert [m.framed for m in messages] == [True, True, True]
    assert "deadman kick" in messages[0].comment
    assert "read fan_speed" in messages[1].comment
    assert "actual=1000 (u16le)" in messages[2].comment
    assert messages[1].stamp == messages[2].stamp  # one burst, two frames
