"""CRC-16 variants for the DVS bus.

The blog post says the checksum is "CRC16_CCITT", computed over every byte of
the frame except the two checksum bytes themselves. That name is ambiguous:
the sunshine2k tool the author used offers several mutually-incompatible
variants under CCITT-ish names, and the post doesn't say which one, nor which
byte order the two checksum bytes go out in.

So we implement all the plausible candidates and let the bus tell us which one
is right (see `identify`). Once you know, pin it in the config.
"""

from dataclasses import dataclass


def _reflect(value: int, width: int) -> int:
    out = 0
    for _ in range(width):
        out = (out << 1) | (value & 1)
        value >>= 1
    return out


@dataclass(frozen=True)
class CrcAlgorithm:
    name: str
    poly: int
    init: int
    refin: bool
    refout: bool
    xorout: int
    check: int  # CRC of b"123456789", from the CRC catalogue

    def __call__(self, data: bytes) -> int:
        crc = self.init
        for byte in data:
            crc ^= (_reflect(byte, 8) if self.refin else byte) << 8
            for _ in range(8):
                crc = ((crc << 1) ^ self.poly) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
        if self.refout:
            crc = _reflect(crc, 16)
        return crc ^ self.xorout


# Ordered most-likely-first for the DVS bus. The first four are all things
# people call "CRC16 CCITT"; MODBUS is here because RS485 + 2-byte trailing
# checksum is overwhelmingly Modbus RTU in the wild, and it's cheap to test.
ALGORITHMS: dict[str, CrcAlgorithm] = {
    a.name: a
    for a in (
        CrcAlgorithm("CCITT-FALSE", 0x1021, 0xFFFF, False, False, 0x0000, 0x29B1),
        CrcAlgorithm("XMODEM", 0x1021, 0x0000, False, False, 0x0000, 0x31C3),
        CrcAlgorithm("KERMIT", 0x1021, 0x0000, True, True, 0x0000, 0x2189),
        CrcAlgorithm("MCRF4XX", 0x1021, 0xFFFF, True, True, 0x0000, 0x6F91),
        CrcAlgorithm("X-25", 0x1021, 0xFFFF, True, True, 0xFFFF, 0x906E),
        CrcAlgorithm("GENIBUS", 0x1021, 0xFFFF, False, False, 0xFFFF, 0xD64E),
        CrcAlgorithm("MODBUS", 0x8005, 0xFFFF, True, True, 0x0000, 0x4B37),
        CrcAlgorithm("AUG-CCITT", 0x1021, 0x1D0F, False, False, 0x0000, 0xE5CC)
    )
}

BYTE_ORDERS = ("little", "big")

DEFAULT_ALGORITHM = "AUG-CCITT"
DEFAULT_BYTE_ORDER = "big"


@dataclass(frozen=True)
class CrcSpec:
    """A concrete choice of algorithm plus the wire order of the two bytes."""

    algorithm: str = DEFAULT_ALGORITHM
    byte_order: str = DEFAULT_BYTE_ORDER

    def __post_init__(self) -> None:
        if self.algorithm not in ALGORITHMS:
            raise ValueError(f"unknown CRC algorithm {self.algorithm!r}")
        if self.byte_order not in BYTE_ORDERS:
            raise ValueError(f"byte_order must be one of {BYTE_ORDERS}")

    def compute(self, data: bytes) -> int:
        return ALGORITHMS[self.algorithm](data)

    def trailer(self, data: bytes) -> bytes:
        """The two checksum bytes to append to `data`."""
        return self.compute(data).to_bytes(2, self.byte_order)

    def append(self, data: bytes) -> bytes:
        return bytes(data) + self.trailer(data)

    def verify(self, frame: bytes) -> bool:
        """True if `frame` (body + 2 checksum bytes) checks out."""
        if len(frame) < 3:
            return False
        return frame[-2:] == self.trailer(frame[:-2])

    def __str__(self) -> str:
        return f"{self.algorithm}/{self.byte_order}-endian"

    def __format__(self, format_spec: str) -> str:
        return format(str(self), format_spec)


def identify(frames: list[bytes]) -> list[CrcSpec]:
    """Return every CrcSpec under which *all* the given frames verify.

    Feed this known-good frames — ones captured off a real controller, or ones
    the fan actually answered. Two or three frames is normally enough to leave
    exactly one survivor. One short frame often leaves several, because
    different variants can coincide on tiny payloads.
    """
    survivors = []
    for name in ALGORITHMS:
        for order in BYTE_ORDERS:
            spec = CrcSpec(name, order)
            if frames and all(spec.verify(f) for f in frames):
                survivors.append(spec)
    return survivors
