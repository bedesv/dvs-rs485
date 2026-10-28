"""DVS bus frame format and register map.

Frame layout, per the reverse-engineering write-up:

    [address:1][register:1][payload:0..N][checksum:2]

A frame with an empty payload is a *read* — the fan answers with the register
contents. A frame with a payload is a *write*. The response length varies by
register and is not encoded anywhere in the frame, so the transport reads until
the bus goes idle rather than trying to read a known number of bytes.

Everything below marked UNKNOWN is genuinely undocumented in the source
material; don't treat the defaults as authoritative.
"""

from dataclasses import dataclass, field

from .crc import CrcSpec

# Wire settings. Baud and bit order are from the logic-analyser capture; the
# post never states the framing, and 8N1 is the near-universal default at 4800.
BAUD_RATE = 4800
BYTE_SIZE = 8
PARITY = "N"
STOP_BITS = 1

# UNKNOWN: the post says the first byte is an address and that the fan ignored
# every address but its own, but never says what that address was. 0x01 is a
# guess; use the address scan to find the real one.
DEFAULT_ADDRESS = 0x01


@dataclass(frozen=True)
class Register:
    number: int
    name: str
    description: str
    readable: bool = True
    writable: bool = False
    # Number of payload bytes a write takes, or None if unknown/variable.
    write_length: int | None = None
    # How to render a read response, if we know.
    decoder: str | None = None
    # Shown before the CLI will send it.
    warning: str | None = None


REGISTERS: dict[int, Register] = {
    r.number: r
    for r in (
        Register(
            0x03,
            "deadman",
            "Deadman kick - must be written every couple of seconds or the fan spins down",
            readable=True,
            writable=True,
            write_length=None,  # UNKNOWN: post doesn't say if it carries a payload
        ),
        Register(
            0x04,
            "set_address",
            "Set bus address of the addressed peripheral (one byte)",
            readable=False,
            writable=True,
            write_length=1,
            warning=(
                "This permanently changes the fan's bus address. If you get it wrong "
                "the fan stops answering on the old address and you'll have to scan "
                "the whole 0x00-0xFF range to find it again."
            ),
        ),
        Register(
            0x05,
            "stop",
            "Stop the fan",
            readable=False,
            writable=True,
            write_length=None,  # UNKNOWN: probably payload-less
        ),
        Register(
            0x10,
            "temperature",
            "Temperature at the fan; 3 bytes LSB MSB 0, u16le in 1/16 degC",
            readable=True,
            decoder="temperature",
        ),
        Register(
            0x11,
            "run_flat_out",
            "Writing anything here starts the fan and runs it to maximum, "
            "overriding the 0x15 demand. Purpose UNKNOWN. Answers with 3 "
            "bytes, e.g. 00 00 02, whose first two accumulate while the fan "
            "runs",
            readable=True,
            writable=True,
            write_length=1,
            warning=(
                "Writing this register runs the fan flat out no matter what "
                "value you send - 0x00 does it just as surely as 0xFF - and it "
                "ignores the speed demand in 0x15. It also keeps the fan alive "
                "on its own, so the fan will not wind down while anything is "
                "writing here. To set a speed, write 0x15 instead."
            ),
        ),
        Register(
            0x15,
            "fan_speed",
            "Speed demand (write, 1 byte) and actual speed (2-byte u16le reply) "
            "in the same units; the scale to RPM is still UNKNOWN",
            readable=True,
            writable=True,
            write_length=1,
            decoder="fan_speed",
        ),
    )
}

# What 0x15 does, measured on the hardware rather than inferred - see
# dvs/verify.py and the transcript in dvs_verify.jsonl:
#
#   * A 1-byte write is the speed demand; the fan answers it with 2 bytes,
#     u16le, which is the *actual* speed in the same units. Demand 40 settled at
#     40 and demand 56 at 56, exactly. An empty-payload read returns the same
#     two bytes, though the real controller never does one.
#   * It is a RATE, not a tacho period: stopped, it reads 00 00 rather than
#     saturating. What one count is worth in RPM is still UNKNOWN and needs the
#     calibration sweep.
#   * The impeller saturates at an actual of 80 on the test unit. Every demand
#     above roughly 85 gives 80, so the top of the demand range cannot be probed
#     through the speed reading.
#   * Lowest demand that will start a stopped fan: 12. That is a rising-ramp
#     starting threshold and is hysteretic - a turning fan keeps going below it.
#   * The demand is NOT 7-bit. Bit 7 is not masked: 0xC0 ran at the maximum
#     rather than dropping back to 0x40's speed. The 0x00-0x7F range in the bus
#     capture was the wall controller's dial range, not the register's.
#   * Settling is slow. Several ladder steps were still climbing after 45 s.
#
# The wall controller's boost mode - the timed one in its settings, not the
# speed dial - is just this register held at 0x7F. Captured switching on and
# off (dvs_boost_capture.log):
#
#   12:31:09.662  02 15 7F 0C 92   demand  73 -> 127   boost on
#   12:31:31.478  02 15 49 5A 07   demand 127 ->  73   boost off
#
# Byte for byte the frame the dial sends at Max, and the previous demand is
# restored when the timer ends. So the boost timer runs in the controller and
# puts nothing new on the bus.
#
# The write-up describes a speed *setpoint* register ("the fan has one register
# to control the speed setpoint, and a separate one that ... tells the fan to
# spin for a little while") without numbering it. There is no separate one:
# 0x15 is both, taking the demand as a 1-byte write and answering with the
# actual speed. Measured, not inferred - see the notes above.
SETPOINT_REGISTER: int | None = 0x15

# A caution about finding it that way: `discovery.hunt_setpoint` scores
# candidates on how far the speed moved, and 0x11 moves it a long way - to
# maximum - without being a setpoint at all. "Moved the speed" and "controls
# the speed" are not the same test, and only the second one is useful. A
# candidate that jumps to full whatever you write is the signature to watch for.

# The regular stop command. Never a scan target, but the way to actually park
# the fan - a zero setpoint doesn't do it while something is still kicking 0x03.
STOP_REGISTER = 0x05

# The keepalive register. `Deadman` and the bus monitor both need the number,
# so it lives here rather than only on the class that kicks it.
DEADMAN_REGISTER = 0x03

# Registers the discovery scans must never touch, because writing them has
# consequences you can't easily undo or that abort the scan.
UNSAFE_TO_PROBE = {0x04, STOP_REGISTER}


@dataclass
class Frame:
    address: int
    register: int
    payload: bytes = b""

    def encode(self, crc: CrcSpec) -> bytes:
        if not 0 <= self.address <= 0xFF:
            raise ValueError("address must be a single byte")
        if not 0 <= self.register <= 0xFF:
            raise ValueError("register must be a single byte")
        body = bytes([self.address, self.register]) + bytes(self.payload)
        return crc.append(body)

    @property
    def is_read(self) -> bool:
        return not self.payload


@dataclass
class Response:
    """A raw reply off the bus, plus whatever we can say about it."""

    raw: bytes
    crc: CrcSpec
    request: Frame | None = None
    notes: list[str] = field(default_factory=list)

    @property
    def crc_ok(self) -> bool:
        return self.crc.verify(self.raw)

    @property
    def body(self) -> bytes:
        """Frame contents with the checksum stripped."""
        return self.raw[:-2] if len(self.raw) >= 2 else b""

    @property
    def payload(self) -> bytes:
        """Body past the address and register bytes."""
        return self.body[2:] if len(self.body) >= 2 else b""

    def __bool__(self) -> bool:
        return bool(self.raw)


def hexdump(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data) if data else "(nothing)"


TEMPERATURE_COUNTS_PER_DEGREE = 16  # 12.4 fixed point; see decode_temperature


def decode_temperature(payload: bytes) -> str:
    """3 bytes, LSB MSB 0. The first two are a u16 count of 1/16 degC.

    Settled by reading the wall controller's display against the bus: 478 counts
    showed 30 degC on the panel, 325 showed 21, 303 showed 19, 292 showed 18.
    Only a scale between 15.3 and 16.5 counts/degC lets all four agree with an
    integer display, and 16 - the usual 12.4 fixed-point convention - is the only
    round number in that window (a least-squares fit through the four points
    gives 15.8). deci- and centi-degC are ruled out: they would make 478 read
    47.8 or 4.78.

    Corroborated by a register nobody was asking about. `0x12` answers with six
    bytes that are byte-identical in every capture taken so far - across
    different sessions and different temperatures, so they are constants, not
    readings - and as three u16le they are 640, -560, 1280. On this scale that is
    exactly +40.0, -35.0 and +80.0 degC: a sensor range and a threshold, all
    three landing on round numbers. Read as deci-degC they would be -56, 64 and
    128 degC, which is not a range a home ventilation unit's sensor has. Two
    registers agreeing on 1/16 degC is much better evidence than the panel
    readings alone.

    Panel readings sit 0.1-0.7 degC above raw/16, so there may be a small offset
    or display rounding on top - a scale of 16 with +4 counts fits all four
    panel points exactly. Deliberately not committed to: it is one parameter
    fitted to four points, it is worth 0.25 degC, and `0x12` needs no offset to
    come out round. This reports the plain quotient.

    UNKNOWN: what the third payload byte is for - it is 0 in every reply
    captured so far. Probably signed: the -560 in `0x12` is only a sensible
    number read as a signed 16-bit, and it shares these units. No reading below
    zero has been captured to confirm it, so `decode_temperature` still shows
    the unsigned value.
    """
    if len(payload) < 2:
        return "payload too short to decode"
    raw = int.from_bytes(payload[:2], "little")
    celsius = raw / TEMPERATURE_COUNTS_PER_DEGREE
    parts = [f"raw={raw} (0x{raw:04X})", f"{celsius:.2f} degC (raw/16)"]
    if len(payload) > 2 and any(payload[2:]):
        # UNKNOWN: always 0 so far, so anything else is worth seeing.
        parts.append(f"trailing bytes {hexdump(payload[2:])}")
    return "  ".join(parts)


def decode_fan_speed(payload: bytes) -> str:
    """One byte is a demand, two are an actual speed, u16le, same units.

    The `1/u16le` column this used to print is gone: it was there to make a
    tacho *period* obvious, and the hardware has since ruled that out - a
    stopped fan reads 00 00 rather than saturating. Little-endian is settled the
    same way (a demand of 56 answers 38 00, which is 14336 read big-endian).

    What a count is worth in RPM is still UNKNOWN, so the raw number is all this
    can honestly report.
    """
    if not payload:
        return "empty payload"
    parts = [f"bytes={hexdump(payload)}"]
    if len(payload) == 1:
        parts.append(f"demand={payload[0]}")
    elif len(payload) >= 2:
        parts.append(f"actual={int.from_bytes(payload[:2], 'little')} (u16le)")
    return "  ".join(parts)


DECODERS = {"temperature": decode_temperature, "fan_speed": decode_fan_speed}


def describe_payload(register: int, payload: bytes) -> str | None:
    reg = REGISTERS.get(register)
    if reg is None or reg.decoder is None:
        return None
    return DECODERS[reg.decoder](payload)
