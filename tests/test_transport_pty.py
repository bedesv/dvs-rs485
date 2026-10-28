"""Integration tests against a fake fan on a pty pair.

Exercises the bits that only show up on a real link: reading a reply of unknown
length, the idle-gap terminator, echo suppression, and the deadman thread.
"""

import os
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import pytest  # noqa: E402

from dvs.config import Config  # noqa: E402
from dvs.crc import CrcSpec  # noqa: E402
from dvs.protocol import Frame  # noqa: E402
from dvs.transport import Deadman, DvsBus  # noqa: E402

# The fake fan signs and verifies frames with whatever CRC the bus defaults to,
# so these tests follow Config as hardware probing pins down variants rather
# than going silent against a spec hardcoded here.
CRC = Config().crc


class FakeFan:
    """Answers reads on a pty master, the way the air handler answers the bus.

    Pass `crc` to make the fan speak a variant other than the configured
    default; tests that deliberately mis-sign a frame instead leave the fan on
    the default and override the *bus*.
    """

    def __init__(self, address=0x01, echo=False, crc: CrcSpec | None = None):
        self.master, slave = os.openpty()
        self.device = os.ttyname(slave)
        os.close(slave)
        self.address = address
        self.echo = echo
        self.crc = CRC if crc is None else crc
        self.received: list[bytes] = []
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    REPLIES = {
        0x15: b"\xe8\x03",  # 1000, little endian
        0x10: b"\xd2\x00\x00",  # 210, LSB MSB 0
    }

    def _run(self):
        buf = bytearray()
        while not self._stop.is_set():
            try:
                buf += os.read(self.master, 64)
            except OSError:
                return
            while len(buf) >= 4:
                frame, buf = bytes(buf[:4]), buf[4:]
                self.received.append(frame)
                if frame[0] != self.address or not self.crc.verify(frame):
                    continue  # wrong address or bad checksum: stay silent
                payload = self.REPLIES.get(frame[1])
                if payload is None:
                    continue
                out = self.crc.append(bytes([self.address, frame[1]]) + payload)
                if self.echo:
                    out = frame + out
                os.write(self.master, out)

    def close(self):
        self._stop.set()
        os.close(self.master)


@pytest.fixture
def fan():
    f = FakeFan()
    yield f
    f.close()


def make_bus(device, **overrides):
    config = Config(port=device, log_path="", response_timeout=1.0, turnaround=0.0, **overrides)
    return DvsBus(config)


def test_read_register_roundtrip(fan):
    with make_bus(fan.device) as bus:
        response = bus.read_register(0x15)
    assert response.crc_ok
    assert response.payload == b"\xe8\x03"
    assert int.from_bytes(response.payload, "little") == 1000


def test_reply_length_discovered_not_assumed(fan):
    """The 3-byte temperature reply and 2-byte speed reply both arrive intact."""
    with make_bus(fan.device) as bus:
        assert bus.read_register(0x10).payload == b"\xd2\x00\x00"
        assert bus.read_register(0x15).payload == b"\xe8\x03"


def test_wrong_address_gets_no_reply(fan):
    with make_bus(fan.device, address=0x02) as bus:
        assert not bus.read_register(0x15)


def test_bad_checksum_gets_no_reply(fan):
    """A frame with the wrong CRC variant is ignored, as on the real bus."""
    with make_bus(fan.device, crc_algorithm="XMODEM") as bus:
        assert not bus.read_register(0x15)


def test_unknown_register_is_silent(fan):
    with make_bus(fan.device) as bus:
        assert not bus.read_register(0x42)


def test_echo_is_stripped():
    fan = FakeFan(echo=True)
    try:
        with make_bus(fan.device) as bus:
            response = bus.read_register(0x15)
        assert response.payload == b"\xe8\x03", "echo leaked into the payload"
        assert any("echo" in note for note in response.notes)
    finally:
        fan.close()


def test_echo_left_alone_when_disabled():
    fan = FakeFan(echo=True)
    try:
        with make_bus(fan.device, strip_echo=False) as bus:
            response = bus.read_register(0x15)
        assert response.raw.startswith(Frame(0x01, 0x15).encode(CRC))
    finally:
        fan.close()


def test_deadman_kicks_repeatedly(fan):
    with make_bus(fan.device, deadman_interval=0.05) as bus:
        deadman = Deadman(bus)
        deadman.start()
        time.sleep(0.4)
        assert deadman.running
        deadman.stop()
        assert not deadman.running
    assert deadman.kicks >= 3
    assert deadman.failures == 0
    assert any(f[1] == 0x03 for f in fan.received), "no deadman frames reached the fan"


def test_deadman_and_foreground_reads_do_not_interleave(fan):
    """The keepalive thread and menu commands share one port under a lock."""
    with make_bus(fan.device, deadman_interval=0.02) as bus:
        deadman = Deadman(bus)
        deadman.start()
        try:
            for _ in range(10):
                response = bus.read_register(0x15)
                assert response.payload == b"\xe8\x03", f"corrupted: {response.raw.hex()}"
        finally:
            deadman.stop()


def test_send_raw_bypasses_checksum(fan):
    with make_bus(fan.device) as bus:
        good = Frame(0x01, 0x15).encode(CRC)
        assert bus.send_raw(good)
        assert not bus.send_raw(b"\x01\x15\x00\x00")


def test_scan_finds_the_fan(fan):
    from dvs.discovery import scan_addresses

    with make_bus(fan.device) as bus:
        hits = scan_addresses(bus, 0x15, range(0x00, 0x05), report=lambda _: None)
    assert [h.address for h in hits] == [0x01]


def test_listen_frames_bursts_by_the_idle_gap(fan):
    """Two writes an idle gap apart are two bursts; back to back they're one."""
    with make_bus(fan.device, idle_gap=0.05) as bus:
        bursts = []
        stop = threading.Event()
        listener = threading.Thread(
            target=lambda: bursts.extend(bus.listen(stop=stop)), daemon=True
        )
        listener.start()
        os.write(fan.master, b"\x02\x03")
        os.write(fan.master, b"\xd2\xc1")  # same burst: no gap between the writes
        time.sleep(0.3)
        os.write(fan.master, b"\x02\x05\xb2\x07")
        time.sleep(0.3)
        stop.set()
        listener.join(timeout=2.0)
    assert [data for _, data in bursts] == [b"\x02\x03\xd2\xc1", b"\x02\x05\xb2\x07"]


def test_listen_transmits_nothing(fan):
    """The whole point of monitoring: the bus must not notice us."""
    with make_bus(fan.device, idle_gap=0.05) as bus:
        stop = threading.Event()
        listener = threading.Thread(target=lambda: list(bus.listen(stop=stop)), daemon=True)
        listener.start()
        time.sleep(0.3)
        stop.set()
        listener.join(timeout=2.0)
    assert fan.received == []


def test_capture_writes_a_decodable_file(tmp_path, fan):
    from dvs import monitor

    path = tmp_path / "capture.log"
    with make_bus(fan.device, idle_gap=0.05) as bus:
        stop = threading.Event()
        writer = threading.Thread(
            target=lambda: monitor.capture(bus, path, stop=stop), daemon=True
        )
        writer.start()
        os.write(fan.master, CRC.append(b"\x02\x03"))
        time.sleep(0.3)
        stop.set()
        writer.join(timeout=2.0)
    messages = monitor.decode_file(path, CRC, report=lambda _: None)
    assert len(messages) == 1
    assert "deadman kick" in messages[0].comment
    assert path.read_text().startswith("# capture started")


def test_crc_probe_identifies_the_variant(fan):
    from dvs.discovery import identify_crc_by_probe

    with make_bus(fan.device, crc_algorithm="MODBUS") as bus:
        winners = identify_crc_by_probe(bus, 0x01, 0x15, report=lambda _: None)
        # The probe must leave the config as it found it.
        assert bus.config.crc_algorithm == "MODBUS"
    assert CRC in winners
