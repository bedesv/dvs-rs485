"""Working out what register 0x15 actually means.

The write-up records "actual fan speed: units TODO". This module runs the
experiment that resolves it: sweep the setpoint over its range, and at each
step record both the raw register value and an independently measured RPM.
Then fit the two competing models and see which one the data prefers.

  * rate model:   raw = a * rpm + b      (raw is RPM, or RPM scaled)
  * period model: raw = k / rpm + c      (raw is a tacho period / timer count)

They look nothing alike once you have five or six points, and the difference
matters: a period register reads *large* when the fan is slow, and typically
saturates rather than reading zero when it's stopped.

See FAN_SPEED_UNITS_PLAN.md for the measurement rig and the full procedure.
"""

import csv
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

from .transport import DvsBus, SpeedHold


@dataclass
class SweepPoint:
    setpoint: int
    raw_bytes: str
    raw_u8: int | None
    raw_u16le: int | None
    raw_u16be: int | None
    measured_rpm: float | None = None
    note: str = ""


@dataclass
class Sweep:
    points: list[SweepPoint] = field(default_factory=list)

    def save_csv(self, path: Path) -> None:
        fields = list(SweepPoint.__dataclass_fields__)
        with Path(path).open("w", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=fields)
            writer.writeheader()
            for point in self.points:
                writer.writerow(asdict(point))

    @classmethod
    def load_csv(cls, path: Path) -> "Sweep":
        sweep = cls()
        with Path(path).open(newline="") as fh:
            for row in csv.DictReader(fh):
                sweep.points.append(
                    SweepPoint(
                        setpoint=int(row["setpoint"]),
                        raw_bytes=row["raw_bytes"],
                        raw_u8=_int_or_none(row["raw_u8"]),
                        raw_u16le=_int_or_none(row["raw_u16le"]),
                        raw_u16be=_int_or_none(row["raw_u16be"]),
                        measured_rpm=_float_or_none(row["measured_rpm"]),
                        note=row.get("note", ""),
                    )
                )
        return sweep


def _int_or_none(value: str) -> int | None:
    return int(value) if value not in ("", "None") else None


def _float_or_none(value: str) -> float | None:
    return float(value) if value not in ("", "None") else None


def run_sweep(
    bus: DvsBus,
    setpoint_register: int,
    setpoints: list[int],
    speed_register: int = 0x15,
    settle: float = 30.0,
    prompt_rpm=None,
    report=print,
) -> Sweep:
    """Step through setpoints, reading raw speed and asking for a measured RPM.

    `prompt_rpm(setpoint, raw_text)` should return the externally measured RPM
    as a float, or None to skip that point. Pass None to collect raw values
    only, which is still useful for a first look at the register's range.

    Settle time defaults to 30 s: a ducted centrifugal fan takes a good while to
    reach a steady speed, and a reading taken during the ramp will smear the
    fit. Don't shorten it below ~15 s.

    The fan is held up the way the wall controller does it - `SpeedHold`
    re-asserting the demand about four times a second - rather than by a
    `Deadman` kicking 0x03 with the demand written once per step. A sweep step
    is minutes of hands-off running with the operator watching the impeller
    rather than the bus, which is the worst case for the single-write approach:
    one lost frame there costs the whole step, and the fan coasting to a halt
    mid-measurement is exactly how it shows up.
    """
    sweep = Sweep()
    # Opens on the first setpoint rather than on zero, so the log doesn't start
    # with a demand the sweep never asked for.
    hold = SpeedHold(bus, register=setpoint_register, demand=setpoints[0] if setpoints else 0)
    hold.start()
    report(
        f"holding the demand on 0x{setpoint_register:02X} every {hold.interval}s; "
        f"sweeping {len(setpoints)} setpoints, {settle}s settle each"
    )
    try:
        for setpoint in setpoints:
            hold.set_demand(setpoint)
            report(f"\nsetpoint 0x{setpoint:02X} ({setpoint}) - settling {settle}s...")
            time.sleep(settle)

            response = bus.read_register(speed_register)
            payload = response.payload
            point = SweepPoint(
                setpoint=setpoint,
                raw_bytes=payload.hex(" ").upper(),
                raw_u8=payload[0] if len(payload) >= 1 else None,
                raw_u16le=int.from_bytes(payload[:2], "little") if len(payload) >= 2 else None,
                raw_u16be=int.from_bytes(payload[:2], "big") if len(payload) >= 2 else None,
            )
            report(
                f"  raw={point.raw_bytes or '(empty)'}"
                f"  u8={point.raw_u8}  u16le={point.raw_u16le}  u16be={point.raw_u16be}"
            )
            if prompt_rpm is not None:
                point.measured_rpm = prompt_rpm(setpoint, point.raw_bytes)
            sweep.points.append(point)
    finally:
        hold.stop(send_stop=True)
        report("\nspeed hold stopped, fan told to stop")
    return sweep


# -- model fitting ---------------------------------------------------------


@dataclass
class Fit:
    model: str
    slope: float
    intercept: float
    r_squared: float
    n: int

    def describe(self) -> str:
        if self.model == "rate":
            body = f"raw = {self.slope:.5g} * rpm + {self.intercept:.5g}"
            if self.slope != 0:
                body += f"   (rpm = raw / {self.slope:.5g}" + (
                    f" - {self.intercept / self.slope:.5g})" if self.intercept else ")"
                )
        else:
            body = f"raw = {self.slope:.6g} / rpm + {self.intercept:.5g}"
            if self.slope != 0:
                body += f"   (rpm = {self.slope:.6g} / (raw - {self.intercept:.5g}))"
        return f"{self.model:<7} R^2={self.r_squared:.5f}  n={self.n}  {body}"


def _least_squares(xs: list[float], ys: list[float]) -> tuple[float, float, float]:
    """Ordinary least squares fit of y = m*x + c. Returns (m, c, r_squared)."""
    n = len(xs)
    if n < 2:
        raise ValueError("need at least two points to fit")
    mean_x = sum(xs) / n
    mean_y = sum(ys) / n
    sxx = sum((x - mean_x) ** 2 for x in xs)
    sxy = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
    if sxx == 0:
        raise ValueError("no variation in x")
    slope = sxy / sxx
    intercept = mean_y - slope * mean_x
    syy = sum((y - mean_y) ** 2 for y in ys)
    if syy == 0:
        return slope, intercept, 1.0
    residual = sum((y - (slope * x + intercept)) ** 2 for x, y in zip(xs, ys))
    return slope, intercept, 1.0 - residual / syy


def fit_models(sweep: Sweep, raw_field: str = "raw_u16le") -> list[Fit]:
    """Fit both the rate and period models to the paired (raw, rpm) points.

    Returns the fits sorted best-first. Both are fitted with raw as the
    dependent variable so the R^2 values are directly comparable.
    """
    pairs = [
        (float(getattr(p, raw_field)), float(p.measured_rpm))
        for p in sweep.points
        if getattr(p, raw_field) is not None and p.measured_rpm
    ]
    if len(pairs) < 3:
        raise ValueError(
            f"need at least 3 points with both {raw_field} and a measured RPM; got {len(pairs)}"
        )

    fits: list[Fit] = []
    raws = [raw for raw, _ in pairs]

    slope, intercept, r2 = _least_squares([rpm for _, rpm in pairs], raws)
    fits.append(Fit("rate", slope, intercept, r2, len(pairs)))

    slope, intercept, r2 = _least_squares([1.0 / rpm for _, rpm in pairs], raws)
    fits.append(Fit("period", slope, intercept, r2, len(pairs)))

    fits.sort(key=lambda f: f.r_squared, reverse=True)
    return fits


def interpret(fits: list[Fit]) -> list[str]:
    """Turn the winning fit into a plain statement about the register's units."""
    best = fits[0]
    runner_up = fits[1] if len(fits) > 1 else None
    notes = []

    if runner_up is not None and abs(best.r_squared - runner_up.r_squared) < 0.02:
        notes.append(
            "The two models fit about equally well. That usually means the sweep "
            "covered too narrow a speed range - widen it, especially at the low end, "
            "where rate and period diverge most."
        )

    if best.model == "rate":
        scale = best.slope
        notes.append(f"Register looks like a RATE. Raw counts per RPM: {scale:.5g}.")
        for factor, meaning in (
            (1.0, "raw is RPM directly"),
            (1 / 60, "raw is Hz (revolutions per second)"),
            (0.5, "raw is RPM/2 - consistent with a 4-pole tacho counting 2 pulses/rev"),
            (2.0, "raw counts 2 pulses per revolution"),
            (10.0, "raw is RPM x 10"),
            (0.1, "raw is RPM / 10"),
        ):
            if factor and abs(scale - factor) / factor < 0.05:
                notes.append(f"  -> within 5% of {factor:g}: {meaning}.")
        if abs(best.intercept) > 0.05 * max(abs(scale) * 1000, 1):
            notes.append(
                f"  Non-zero intercept ({best.intercept:.5g}) - check the fan was fully "
                "settled at each step, or that the register isn't offset-encoded."
            )
    else:
        k = best.slope
        notes.append(
            f"Register looks like a PERIOD or timer count: raw ~ {k:.6g}/rpm. "
            "It reads large when the fan is slow."
        )
        notes.append(
            f"  If the timer runs at F Hz and the tacho gives P pulses per revolution, "
            f"then k = 60*F/P, so F/P = {k / 60:.6g} Hz. Compare that against plausible "
            "prescaled clocks (e.g. 1 MHz, 500 kHz, 62.5 kHz, 32.768 kHz) to pin down "
            "the tacho pulse count."
        )
        notes.append(
            "  Confirm with the stopped-fan test: a period register saturates "
            "(0xFFFF or similar) at zero speed rather than reading 0."
        )

    return notes
