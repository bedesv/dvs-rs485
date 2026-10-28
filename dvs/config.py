"""Persisted settings, so you don't retype the port and CRC variant every run."""

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

from . import protocol
from .crc import DEFAULT_ALGORITHM, DEFAULT_BYTE_ORDER, CrcSpec

CONFIG_PATH = Path(__file__).resolve().parent.parent / "dvs_config.json"


@dataclass
class Config:
    port: str = ""
    baud: int = protocol.BAUD_RATE
    parity: str = protocol.PARITY
    stop_bits: int = protocol.STOP_BITS
    byte_size: int = protocol.BYTE_SIZE

    address: int = protocol.DEFAULT_ADDRESS

    crc_algorithm: str = DEFAULT_ALGORITHM
    crc_byte_order: str = DEFAULT_BYTE_ORDER

    # How long to wait for the first response byte, and how much dead air ends
    # a response. At 4800 baud one byte takes ~2.1 ms, so 15 ms of silence is
    # comfortably longer than any inter-byte gap inside a single frame.
    response_timeout: float = 0.5
    idle_gap: float = 0.015
    # Quiet time after a transaction before the next one, so the fan's UART and
    # the adapter's direction control both settle.
    turnaround: float = 0.02

    # Many cheap USB-RS485 adapters loop transmitted bytes back into RX.
    # Leave this on; it detects the echo by comparing against what we sent.
    strip_echo: bool = True

    # The write-up describes a separate speed *setpoint* register and never
    # numbers it. There isn't one: 0x15 takes the demand as a 1-byte write and
    # answers with the actual speed, measured on the hardware (verify.py T3,
    # demand 40 settled at 40 and 56 at 56).
    #
    # This used to default to 0x11, which the setpoint hunt picks out because
    # writing there does move the speed - straight to maximum, whatever value
    # you send, including zero. As a setpoint register that made menu '7' a
    # run-flat-out button. See REGISTERS[0x11].
    setpoint_register: int | None = 0x15
    deadman_payload: str = ""  # hex, e.g. "01"; empty means payload-less write
    deadman_interval: float = 0.5
    # Kicks are fire-and-forget: we don't know whether the fan acknowledges a
    # write to 0x03, and if it doesn't, waiting the full response_timeout on
    # every kick would both slow the keepalive below the fan's ~2s spin-down
    # and hog the port lock against foreground commands.
    deadman_response_timeout: float = 0.08

    # `transport.SpeedHold` re-asserts the demand on this timer. 0.25 s is what
    # the wall controller does - roughly four writes a second - and it is the
    # only thing holding the fan up while a sweep step settles, so it wants to
    # stay well inside the fan's ~2 s spin-down. The reply carries the actual
    # speed and arrives promptly; the timeout only bites when one goes missing,
    # and waiting the full response_timeout there would stall the hold.
    speed_hold_interval: float = 0.25
    speed_hold_response_timeout: float = 0.1

    log_path: str = "dvs_bus.log"
    # Where passive monitoring writes what it hears. Kept apart from log_path:
    # that one is a running commentary on what *we* did, this one is a raw
    # recording of the bus meant to be decoded later, possibly with a different
    # CRC variant than the one in force when it was taken.
    capture_path: str = "dvs_capture.log"
    # Structured transcript of a theory-test run: one JSON record per frame in
    # each direction, plus the verdicts. Separate from log_path for the same
    # reason capture_path is - this one is meant to be loaded and re-analysed,
    # not read as a running commentary.
    verify_path: str = "dvs_verify.jsonl"
    extra: dict = field(default_factory=dict)

    @property
    def crc(self) -> CrcSpec:
        return CrcSpec(self.crc_algorithm, self.crc_byte_order)

    @property
    def deadman_bytes(self) -> bytes:
        return bytes.fromhex(self.deadman_payload) if self.deadman_payload else b""

    def save(self, path: Path = CONFIG_PATH) -> None:
        path.write_text(json.dumps(asdict(self), indent=2) + "\n")

    @classmethod
    def load(cls, path: Path = CONFIG_PATH) -> "Config":
        if not path.exists():
            return cls()
        data = json.loads(path.read_text())
        known = {f for f in cls.__dataclass_fields__}
        return cls(**{k: v for k, v in data.items() if k in known})
