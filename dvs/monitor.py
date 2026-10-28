"""Passive bus monitoring: capture the traffic first, decode it afterwards.

Wire the adapter onto the bus alongside the fan's own wall controller and this
records the conversation between them. That is the one experiment that answers
questions the fan can't be asked directly: what the controller sends, in what
order, and what the fan says back.

The two halves are deliberately separate:

  * `capture` listens - never transmits - and appends every burst of bytes it
    hears to a file, timestamped, one burst per line. It interprets *nothing*.
    If the CRC variant in the config turns out to be wrong, or a register's
    meaning gets settled next week, the capture is still good.
  * `Decoder` / `decode_file` read that file back, split bursts into frames and
    annotate each one with what it appears to be. All the guessing lives here,
    and re-running it costs nothing.

Nothing in this module is authoritative about meaning. A frame gets the best
comment the register map supports, and where the map says UNKNOWN so does the
comment.
"""

import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Iterable, Iterator

from .crc import CrcSpec
from .protocol import (
    DEADMAN_REGISTER,
    REGISTERS,
    STOP_REGISTER,
    describe_payload,
    hexdump,
)
from .transport import DvsBus

Reporter = Callable[[str], None]

# Address, register and two checksum bytes: the shortest frame that can exist.
MIN_FRAME = 4

TIME_FORMAT = "%Y-%m-%dT%H:%M:%S"


# -- capture file format ---------------------------------------------------


def format_stamp(at: float) -> str:
    """Millisecond-resolution timestamp - frames land ~2 ms apart at 4800 baud."""
    return f"{time.strftime(TIME_FORMAT, time.localtime(at))}.{int(at % 1 * 1000):03d}"


def format_line(at: float, data: bytes) -> str:
    """One captured burst as a line: timestamp then space-separated hex."""
    return f"{format_stamp(at)} {hexdump(data)}"


def parse_line(line: str) -> tuple[str, bytes] | None:
    """Inverse of `format_line`. Returns None for blanks and `#` comments."""
    line = line.strip()
    if not line or line.startswith("#"):
        return None
    stamp, _, rest = line.partition(" ")
    try:
        return stamp, bytes.fromhex(rest.replace(",", " "))
    except ValueError:
        return None


def capture(
    bus: DvsBus,
    path: Path,
    stop=None,
    on_burst: Callable[[float, bytes], None] | None = None,
) -> int:
    """Append every burst heard on the bus to `path`; returns the burst count.

    Each burst is flushed as it arrives rather than at the end, so a capture is
    readable - and decodable - while it is still running, and survives the
    process being killed.
    """
    count = 0
    with path.open("a", buffering=1) as fh:
        cfg = bus.config
        fh.write(
            f"# capture started {time.strftime(TIME_FORMAT)}"
            f" port={cfg.port} {cfg.baud} {cfg.byte_size}{cfg.parity}{cfg.stop_bits}"
            f" crc={cfg.crc}\n"
        )
        for at, data in bus.listen(stop=stop):
            fh.write(format_line(at, data) + "\n")
            count += 1
            if on_burst is not None:
                on_burst(at, data)
    return count


# -- framing ---------------------------------------------------------------


def split_frames(data: bytes, crc: CrcSpec) -> tuple[list[bytes], bytes]:
    """Split a burst into checksum-verified frames plus any trailing remainder.

    Nothing in a frame says how long it is, so the checksum is the only frame
    boundary there is: walk the prefix forward until it verifies, emit it, start
    again on what's left. Two frames back to back in one burst - a request and
    the reply that beat the idle gap - come out as two.

    Leftover bytes are returned rather than dropped. They usually mean the CRC
    variant is wrong, which is worth seeing.
    """
    frames: list[bytes] = []
    rest = data
    while len(rest) >= MIN_FRAME:
        for end in range(MIN_FRAME, len(rest) + 1):
            if crc.verify(rest[:end]):
                frames.append(rest[:end])
                rest = rest[end:]
                break
        else:
            break
    return frames, rest


# -- annotation ------------------------------------------------------------


@dataclass
class Message:
    """One decoded line: the bytes, and the best guess at what they mean."""

    stamp: str
    raw: bytes
    comment: str
    framed: bool = True

    def line(self, width: int = 30) -> str:
        return f"{self.stamp}  {hexdump(self.raw):<{width}}  # {self.comment}"


@dataclass
class Decoder:
    """Turns captured bursts into annotated messages.

    Stateful on purpose. A frame carrying a payload is either a write command or
    the answer to a read, and *nothing in the frame says which* - both directions
    use the same layout and there is no direction bit. The only evidence is what
    came immediately before: a payload frame on the address and register of the
    read that just went out is the reply to it. Anything else with a payload is
    read as a write.

    Ordering is the only evidence there is, so a capture that starts mid-exchange
    can get the first frame's direction wrong. UNKNOWN: whether the fan ever
    speaks unprompted, which would break the pairing too.
    """

    crc: CrcSpec = field(default_factory=CrcSpec)
    setpoint_register: int | None = None
    _pending: tuple[int, int] | None = field(default=None, init=False, repr=False)

    def feed(self, stamp: str, data: bytes) -> list[Message]:
        frames, rest = split_frames(data, self.crc)
        messages = [Message(stamp, f, self.describe(f)) for f in frames]
        if rest:
            messages.append(
                Message(
                    stamp,
                    rest,
                    f"{len(rest)} byte(s) that don't verify as a frame - almost always the "
                    f"wrong CRC variant, not corrupt data",
                    framed=False,
                )
            )
        return messages

    def describe(self, frame: bytes) -> str:
        address, register = frame[0], frame[1]
        payload = frame[2:-2]
        reg = REGISTERS.get(register)
        name = reg.name if reg else f"register 0x{register:02X}"
        return f"0x{address:02X}  {self._action(address, register, payload, reg, name)}"

    def _action(self, address, register, payload, reg, name) -> str:
        was_read = self._pending == (address, register)

        if not payload:
            # Empty payload means read - except for the payload-less commands,
            # where the register itself is the whole instruction.
            self._pending = (address, register)
            if register == DEADMAN_REGISTER:
                return "deadman kick - fan keeps spinning for another couple of seconds"
            if register == STOP_REGISTER:
                return "stop fan"
            if reg is not None and reg.writable and not reg.readable:
                return f"{name} command (0x{register:02X}, no payload)"
            return f"read {name} (0x{register:02X})"

        self._pending = None

        if was_read:
            described = describe_payload(register, payload)
            return f"{name} is {described}" if described else f"{name} = {hexdump(payload)}"

        if register == 0x04:
            return f"SET BUS ADDRESS to 0x{payload[0]:02X} - permanent"
        if register == DEADMAN_REGISTER:
            return f"deadman kick, payload {hexdump(payload)}"
        if register == self.setpoint_register:
            value = payload[0] if len(payload) == 1 else int.from_bytes(payload, "little")
            return f"set fan speed setpoint to {value} (0x{value:02X})"
        return f"write {name} (0x{register:02X}) = {hexdump(payload)}"


def decode(
    bursts: Iterable[tuple[str, bytes]],
    crc: CrcSpec | None = None,
    setpoint_register: int | None = None,
) -> Iterator[Message]:
    decoder = Decoder(CrcSpec() if crc is None else crc, setpoint_register)
    for stamp, data in bursts:
        yield from decoder.feed(stamp, data)


def read_capture(path: Path) -> Iterator[tuple[str, bytes]]:
    with path.open() as fh:
        for line in fh:
            parsed = parse_line(line)
            if parsed is not None:
                yield parsed


def decode_file(
    path: Path,
    crc: CrcSpec | None = None,
    setpoint_register: int | None = None,
    report: Reporter = print,
) -> list[Message]:
    """Decode a capture file, reporting each message as it goes."""
    messages = []
    for message in decode(read_capture(path), crc, setpoint_register):
        messages.append(message)
        report(message.line())
    return messages
