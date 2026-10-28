"""Discovery routines for the parts of the protocol the write-up left open.

Three things are unknown and have to be found on your own unit:

  * the fan's bus address (never stated in the post)
  * which CRC variant and byte order are actually in use
  * the speed *setpoint* register number (described in prose, absent from the
    published register table)
"""

import time
from dataclasses import dataclass
from typing import Callable, Iterable

from .crc import ALGORITHMS, BYTE_ORDERS, CrcSpec
from .protocol import STOP_REGISTER, UNSAFE_TO_PROBE, Frame, hexdump
from .transport import Deadman, DvsBus

Reporter = Callable[[str], None]


# -- CRC identification ----------------------------------------------------


def identify_crc_by_probe(
    bus: DvsBus, address: int, register: int = 0x15, report: Reporter = print
) -> list[CrcSpec]:
    """Find the CRC variant by asking the fan and seeing which frame it answers.

    A read of a known register is 4 bytes: address, register, and two checksum
    bytes. Only the right checksum gets a reply, so we walk the candidates and
    keep the ones that produce one. This does not depend on having a logic
    analyser capture.
    """
    original = bus.config.crc_algorithm, bus.config.crc_byte_order
    winners: list[CrcSpec] = []
    try:
        for name in ALGORITHMS:
            for order in BYTE_ORDERS:
                bus.config.crc_algorithm, bus.config.crc_byte_order = name, order
                spec = CrcSpec(name, order)
                frame = Frame(address, register)
                response = bus.transaction(frame)
                mark = "REPLY" if response else "  -  "
                report(f"  {mark}  {spec:<26} tx={hexdump(frame.encode(spec))}")
                if response:
                    winners.append(spec)
                time.sleep(0.05)
    finally:
        bus.config.crc_algorithm, bus.config.crc_byte_order = original
    return winners


# -- address scan ----------------------------------------------------------


@dataclass
class AddressHit:
    address: int
    response: bytes


def scan_addresses(
    bus: DvsBus,
    register: int = 0x15,
    addresses: Iterable[int] = range(0x100),
    report: Reporter = print,
) -> list[AddressHit]:
    """Read one register at every address; anything that answers is a device.

    Uses a short timeout, since a scan of all 256 addresses at the default
    half-second timeout would take over two minutes of mostly waiting.
    """
    hits: list[AddressHit] = []
    for address in addresses:
        response = bus.transaction(Frame(address, register), timeout=0.12)
        if response:
            hits.append(AddressHit(address, response.raw))
            report(
                f"  0x{address:02X} answered: {hexdump(response.raw)}"
                f"  (crc {'ok' if response.crc_ok else 'MISMATCH'})"
            )
    return hits


# -- register scan ---------------------------------------------------------


@dataclass
class RegisterHit:
    register: int
    response: bytes
    crc_ok: bool


def scan_registers(
    bus: DvsBus,
    address: int,
    registers: Iterable[int] = range(0x100),
    report: Reporter = print,
) -> list[RegisterHit]:
    """Read every register and record which ones answer, and with how much data.

    This is read-only: empty payload means read, so nothing here can change the
    fan's state.
    """
    hits: list[RegisterHit] = []
    for register in registers:
        response = bus.transaction(Frame(address, register), timeout=0.2)
        if response:
            hit = RegisterHit(register, response.raw, response.crc_ok)
            hits.append(hit)
            report(
                f"  reg 0x{register:02X}: {hexdump(response.raw)}"
                f"  ({len(response.raw)} bytes, crc {'ok' if hit.crc_ok else 'MISMATCH'})"
            )
    return hits


# -- setpoint hunt ---------------------------------------------------------


@dataclass
class SetpointCandidate:
    register: int
    low_speed: int | None
    high_speed: int | None

    @property
    def unreadable(self) -> bool:
        """True if either end failed to read, so `delta` means nothing."""
        return self.low_speed is None or self.high_speed is None

    @property
    def delta(self) -> int:
        if self.unreadable:
            return 0
        return abs(self.high_speed - self.low_speed)

    @property
    def moved(self) -> bool:
        """True if the reported speed changed, so this register affects speed."""
        return not self.unreadable and self.delta > 0


SPEED_READ_ATTEMPTS = 4
SPEED_READ_RETRY_DELAY = 0.25


def _speed_word_once(bus: DvsBus, speed_register: int) -> int | None:
    response = bus.read_register(speed_register)
    payload = response.payload
    if len(payload) >= 2:
        return int.from_bytes(payload[:2], "little")
    if len(payload) == 1:
        return payload[0]
    return None


def _speed_word(
    bus: DvsBus,
    speed_register: int,
    attempts: int = SPEED_READ_ATTEMPTS,
    retry_delay: float = SPEED_READ_RETRY_DELAY,
    report: Reporter | None = None,
) -> int | None:
    """Read the actual-speed register, retrying before accepting no answer.

    A single missed reply is common - the deadman shares the port, and a reply
    that arrives late or with the wrong checksum leaves an empty payload. Taking
    that at face value used to report `None -> 54`, which the setpoint hunt then
    scored as zero movement and ranked the real register below the noise. So a
    `None` is only believed after `attempts` consecutive empty reads. Only the
    give-up is reported - a read that succeeds on a later attempt is not worth
    a line of its own.
    """
    for attempt in range(1, attempts + 1):
        value = _speed_word_once(bus, speed_register)
        if value is not None:
            return value
        if attempt < attempts:
            time.sleep(retry_delay)
    if report is not None:
        report(f"    speed read: no reply after {attempts} attempts")
    return None


def _park_fan(
    bus: DvsBus, deadman: Deadman, register: int, settle: float, report: Reporter
) -> None:
    """Bring a candidate that moved the fan back to a standstill.

    Writing 0 to the register is not enough on its own: the deadman is still
    kicking 0x03 every interval, so the fan keeps spinning regardless. So the
    keepalive is stopped - which sends the regular stop command, 0x05 - the fan
    is given `settle` seconds to wind down, and only then does the keepalive come
    back for the next candidate. The zero write still goes out first so the fan
    can't resume at the high setpoint when the kicks restart.

    Settled on hardware: a stopped fan does *not* spin up from 0x03 kicks alone.
    Sixty seconds of kicking a parked fan with nothing else on the bus left it
    at zero throughout. So the park holds and candidates after the first mover
    do start from a standstill, which is what this routine needs.

    0x03 still keeps an already-turning fan turning - it prevents the spin-down
    without being able to start one.
    """
    bus.write_register(register, b"\x00")
    deadman.stop(send_stop=True)
    report(
        f"    zeroed 0x{register:02X} and sent stop (0x{STOP_REGISTER:02X}),"
        f" waiting {settle:g}s for the fan to wind down"
    )
    time.sleep(settle)
    deadman.start()


def hunt_setpoint(
    bus: DvsBus,
    address: int,
    candidates: Iterable[int] = range(0x00, 0x40),
    speed_register: int = 0x15,
    low: int = 0x20,
    high: int = 0xC0,
    settle: float = 6.0,
    speed_read_attempts: int = SPEED_READ_ATTEMPTS,
    report: Reporter = print,
) -> list[SetpointCandidate]:
    """Find the speed setpoint register by writing to candidates and watching 0x15.

    For each candidate register: write a low value, let the fan settle, read the
    actual-speed register; then write a high value, settle, read again. The
    setpoint register is the one where the reported speed moves a lot.

    Beware what this actually scores. "The speed moved" is not "the register
    controls the speed": writing *anything* to 0x11 runs the fan to maximum,
    which shows up here as a large delta and once got 0x11 saved as the setpoint
    register. Check a winner by writing two different non-zero values and
    confirming the speed lands somewhere different for each, rather than pinning
    to full both times. 0x15 is the real one.

    Each speed read is retried up to `speed_read_attempts` times, because one
    empty reply is not evidence the fan didn't move - a candidate with an
    unreadable end reports no movement at all and loses to noise.

    When a candidate moves the speed at all it has some control over it, so the
    fan is parked - zero written back, keepalive stopped, stop command sent, one
    `settle` to wind down - before the next candidate is tried. Without that the
    fan is left spinning and the next register's low reading is inherited.

    The deadman has to be running throughout or the fan won't spin at all, so
    this starts and stops it for you.

    Registers 0x04 (set address) and 0x05 (stop) are skipped - writing 0x04
    would move the fan to an address you don't know, and 0x05 would abort the
    run on every pass.
    """
    results: list[SetpointCandidate] = []
    deadman = Deadman(bus)
    deadman.start()
    report(f"deadman running ({bus.config.deadman_interval}s interval)")
    try:
        for register in candidates:
            if register in UNSAFE_TO_PROBE:
                report(f"  reg 0x{register:02X}: skipped (unsafe to probe)")
                continue
            if register == Deadman.REGISTER:
                continue

            bus.write_register(register, bytes([low]))
            time.sleep(settle)
            low_speed = _speed_word(
                bus, speed_register, attempts=speed_read_attempts, report=report
            )

            bus.write_register(register, bytes([high]))
            time.sleep(settle)
            high_speed = _speed_word(
                bus, speed_register, attempts=speed_read_attempts, report=report
            )

            candidate = SetpointCandidate(register, low_speed, high_speed)
            results.append(candidate)
            shown_low = "no reply" if low_speed is None else str(low_speed)
            shown_high = "no reply" if high_speed is None else str(high_speed)
            delta = "unknown" if candidate.unreadable else str(candidate.delta)
            report(
                f"  reg 0x{register:02X}: speed {shown_low} -> {shown_high}"
                f"  (delta {delta})"
            )

            if candidate.moved:
                _park_fan(bus, deadman, register, settle, report)
    finally:
        deadman.stop(send_stop=True)
        report("deadman stopped, fan told to stop")

    results.sort(key=lambda c: c.delta, reverse=True)
    return results
