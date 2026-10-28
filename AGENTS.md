# AGENTS.md

This file provides guidance to coding agents working with this repository.

## What this is

A menu-driven CLI for talking to a DVS air handling unit (ducted home ventilation
fan) over its RS485 bus. The protocol comes from [Ian Rees' reverse-engineering
write-up](https://ianrees.nz/tech/2018-07-11-reverse-engineering-the-dvs-bus.html),
which leaves several things undetermined — so this codebase is as much a
**discovery instrument** as a control tool. Half the menu exists to find out
facts the post never established.

## Commands

```bash
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt   # setup
.venv/bin/python -m dvs                        # run the interactive menu
.venv/bin/python -m dvs --list-ports           # find the USB-RS485 adapter
.venv/bin/python -m dvs --port /dev/cu.usbserial-X --address 0x02
.venv/bin/python -m pytest tests/ -q           # all tests, no hardware needed
.venv/bin/python -m pytest tests/test_dvs.py::test_crc_check_values -q   # one test
.venv/bin/python -m pytest tests/test_verify.py -q              # theory checks only
```

No linter or formatter is configured. Code is stdlib + `pyserial` only — the
least-squares fitting in `calibrate.py` is hand-rolled rather than pulling in
numpy, and new code should keep that dependency floor.

### Current test state

All 86 tests pass — `tests/test_dvs.py` (44), `tests/test_transport_pty.py` (15)
and `tests/test_verify.py` (27). The two pty files take ~23 s and ~50 s; both
sleep through real timing intervals, so the suite is about a minute and a half.

`tests/test_verify.py` is mostly fans built to *break* one claim each, asserting
the matching check reports `contradicted`. Keep it that way: a checker that can
only agree with the theory it came from is worthless, and these are the tests
that prove it can disagree. Its `TheoryFan.close` wakes the reader thread with a
byte before closing the pty master — without that, teardown deadlocks on macOS
for any test that leaves the serial side open.

`FakeFan` derives its `CrcSpec` from `Config()` (module-level `CRC = Config().crc`)
rather than hardcoding one, so pinning a new CRC default in `Config` doesn't
silence the fake fan and fail all 12 tests. Tests that deliberately mis-sign a
frame override the CRC on the *bus* and leave the fan on the default — keep it
that way round.

## Architecture

Layered bottom-up; each module depends only on the ones above it in this list.

- **`crc.py`** — eight CRC-16 variants (`ALGORITHMS`) × two byte orders
  (`BYTE_ORDERS`) = 16 candidate checksums. The post says "CRC16_CCITT", which
  names several mutually incompatible algorithms and doesn't state the wire
  order of the two bytes, so the ambiguity is modelled explicitly as `CrcSpec`
  and resolved empirically. `identify(frames)` narrows candidates from
  known-good captured frames.
- **`protocol.py`** — frame layout `[address:1][register:1][payload:0..N][checksum:2]`.
  **Empty payload = read, non-empty payload = write**; this is the single most
  important invariant in the codebase and is why the register scan is inherently
  safe. Holds the `REGISTERS` map, the `Frame`/`Response` dataclasses, the
  `DECODERS` for raw payloads, and `UNSAFE_TO_PROBE = {0x04, 0x05}`.
- **`transport.py`** — `DvsBus` (half-duplex serial I/O) and the two keepalive
  threads, `Deadman` (kicks `0x03`) and `SpeedHold` (re-asserts a demand on
  `0x15`). See the two transport quirks below.
- **`monitor.py`** — passive capture and offline decode, kept as two separate
  steps. `capture` transmits nothing and appends raw timestamped bursts to
  `capture_path`; `split_frames`/`Decoder`/`decode_file` turn a capture back
  into annotated lines. All interpretation lives on the decode side so a capture
  stays valid when the CRC variant or a register's meaning changes. See the two
  monitor rules below.
- **`discovery.py`** — the three scans that resolve the unknowns:
  `identify_crc_by_probe`, `scan_addresses`, `scan_registers`, `hunt_setpoint`.
  Pure functions over a `DvsBus`, each taking a `report: Reporter` callback so
  the CLI owns all formatting.
- **`verify.py`** — the theory-test tool. States six claims about `0x15` as
  `CHECKS` (T1–T6, ordered by how a run should go rather than by number), each a
  function `(Recorder, SpeedDriver, Plan, Reporter) -> Finding` returning
  `supported` / `contradicted` / `inconclusive`. `Recorder` sends every frame
  through `DvsBus.transaction` and appends it to a JSONL transcript, flushing per
  exchange so a Ctrl-C part way through a ten-minute run still leaves a record.
  `SpeedDriver` writes the demand on a timer the way the wall controller does.
  See the three rules below.
- **`calibrate.py`** — the fan-speed-units experiment: `run_sweep` collects
  (raw register value, externally measured RPM) pairs to CSV; `fit_models` fits
  the competing rate and period models; `interpret` turns the winner into prose.
- **`config.py`** — `Config` dataclass persisted to `dvs_config.json` at the
  repo root. `load()` filters unknown keys, so adding a field is
  backward-compatible with existing config files.
- **`cli.py`** — `App` class: menu dispatch table in `run()`, `MENU` list for
  display, one `do_*` method per key. Owns every `print`, prompt, and
  confirmation. Discovery routines that find something offer to save it to
  config.

### Two transport quirks that shape everything

1. **Reply lengths are unknown and vary by register**, and nothing in the frame
   encodes them. `_read_until_idle` reads until the bus has been quiet for
   `idle_gap` (15 ms; one byte at 4800 baud is ~2.1 ms). Never rewrite this to
   read a fixed byte count.
2. **Cheap USB-RS485 adapters echo transmitted bytes into RX.** Handled by
   comparing the head of the read buffer against what was just sent
   (`strip_echo`). Tests cover both the stripping and the disabled path.

### Two rules for the bus monitor

1. **A burst is not a frame.** `DvsBus.listen` yields whatever arrived between
   two idle gaps, which is frequently a request and its reply together. The
   checksum is the only frame boundary that exists, so `split_frames` walks
   forward until a prefix verifies. Bytes that never verify are returned and
   reported, not dropped — they're the signature of the wrong CRC variant.
2. **Capture stays dumb.** The capture file holds timestamps and hex, nothing
   else, so it survives every later change of mind about what the bytes mean.
   Don't move annotation into `capture`, and don't make the decoder discard
   what it can't explain.

Reply-vs-write is inferred from ordering (`Decoder._pending`), because both are
payload frames and nothing on the wire distinguishes them.

### Three rules for the theory checks

1. **A check must be able to say the theory is wrong.** Every check has a
   `contradicted` branch reached by real data, and `tests/test_verify.py` proves
   each one fires. `check_rate_not_period` discriminates by comparing the stopped
   reading against the *running* one rather than against a magic constant, so it
   works whatever the register's scale turns out to be. Don't replace a verdict
   with an assertion, and don't let `inconclusive` collapse into `supported`.
2. **The transcript stays dumb.** Hex, numbers and labels only — same reason the
   bus capture is. Interpretation belongs in `Finding` and `timeline`, so an old
   transcript survives changing our minds about the bytes. `load_transcript`
   filters unknown keys and kinds for the same reason `Config.load` does.
3. **`SpeedDriver`, `SpeedHold` and `Deadman` stay separate.** Two write `0x15`,
   one kicks `0x03`. T5 has now shown that *both registers* refresh the fan's
   timeout, so the classes are still not interchangeable: the `0x15` writers
   also carry the demand, and T5 needs to drive each register independently to
   keep testing them apart. Merging them would delete the experiment.
   `SpeedDriver` and `transport.SpeedHold` do overlap — same job, but
   `SpeedDriver` is wired to this module's JSONL `Recorder` and its T5
   instrumentation (pause/resume, lowest-seen tracking), and `transport` cannot
   depend on `verify`. If that duplication is ever worth removing, the direction
   is `SpeedDriver` growing on top of `SpeedHold`, not the sweep growing a
   transcript it has no use for.

4. **Design a check against the fan, not against the fake.** T2 and T5 both
   passed their fake-fan tests and then measured nothing useful on hardware — T2
   compared two demands a saturated impeller reads identically, T5 read a
   register that goes silent when the bus does. Before trusting a new check, ask
   what the fan does when it *isn't* being polled, and whether the quantity
   being compared still moves at the point being probed.

### The two ways of keeping the fan alive

The fan spins down ~2 s after the last frame that refreshes its timeout, so
anything needing the fan running has to be sending something. Either register
will do it, and the codebase has one thread for each:

- **`Deadman`** kicks `0x03` and carries no demand. Used by
  `discovery.hunt_setpoint` and by the menu's own keepalive (`App.deadman`).
- **`SpeedHold`** re-asserts the demand on `0x15` every
  `speed_hold_interval` (250 ms), which is what the wall controller does and
  nothing else. Used by `calibrate.run_sweep`.

Both share one port with the foreground menu under `DvsBus.lock` (an `RLock`),
both use a deliberately short response timeout (`deadman_response_timeout`
80 ms, `speed_hold_response_timeout` 100 ms) so a keepalive can't fall behind
the spin-down window or hog the lock, and both are started and stopped in a
`try/finally` with `send_stop=True`. The CLI stops its `Deadman` on exit and
Ctrl-C.

**The sweep uses `SpeedHold`, not `Deadman`, and that is a fix rather than a
preference.** It used to write the demand once per step and leave a `Deadman` to
hold the fan up; a real sweep died mid-step that way. A sweep step is minutes of
hands-off running with the operator watching the impeller instead of the bus,
which is the worst case for a write-once demand — one lost frame costs the whole
step, and the failure looks like the fan simply stopping. Re-asserting the
demand costs one interval instead. Don't quietly put the sweep back on `Deadman`.

Because the keepalive is what keeps the fan turning, **a zero setpoint does not
stop the fan while the deadman is running.** Parking it takes stopping the
keepalive and sending `STOP_REGISTER` (`discovery._park_fan`, `App.do_stop`).
`hunt_setpoint` does that between candidates whenever one moved the speed, then
restarts the keepalive, so each candidate starts from a standstill.

**`0x03` was contested and is now confirmed.** A passive capture of the wall
controller contains no `0x03` frames at all — it holds the fan up by writing the
demand to `0x15` about four times a second — so `verify.py` T5 proposed that
`0x03` did nothing. Hardware says otherwise: with `0x15` writes stopped and only
`0x03` kicks going out the fan held 56 for the whole window, while total silence
took it from 53 to 2. **Either register refreshes the timeout.** `Deadman` is
correct as written; the controller not using `0x03` was never evidence that it
does nothing, and reading it that way was the error.

## Working on this codebase

**Distinguish established protocol facts from guesses.** Anything genuinely
undocumented in the source material is marked `UNKNOWN` in comments and
docstrings. Don't quietly promote a guess to a fact, and don't delete an
`UNKNOWN` marker without evidence from the bus or a capture. When hardware
probing does settle something, pin it in `Config` defaults *and* update the
comment that called it unknown. Menu `t` (`verify.py`) is the instrument for the
`0x15` questions — run it before editing any of those comments, and cite the
transcript when you do.

Settled on hardware and written up next to `REGISTERS[0x15]` in `protocol.py`:
`0x15` takes a 1-byte demand and answers with the actual speed as u16le in the
same units; it is a rate, not a period; the impeller saturates at 80; the
starting floor is 12; the demand is wider than 7 bits. There is **no separate
setpoint register** — `0x15` is both, so `SETPOINT_REGISTER` is now `0x15`
rather than `None`. `0x03` kicks keep a turning fan turning but cannot start a
stopped one (60 s of kicking a parked fan left it at zero), which is the
`UNKNOWN` `discovery._park_fan` used to carry.

**`0x10` is 1/16 °C** — u16le, the 12.4 fixed-point convention. Settled by
reading the wall controller's panel against the bus (478↔30 °C, 325↔21, 303↔19,
292↔18); only a scale of 15.3–16.5 counts/°C makes all four agree with a
whole-degree display, and 16 is the only round number in that window.
Independently corroborated by **`0x12`**, which answers with three u16le
constants — identical in every capture — that come out as exactly +40.0, −35.0
and +80.0 °C on this scale and as an implausible −56/64/128 °C on any other. That
is why no offset is applied even though panel readings run 0.1–0.7 °C above
`raw/16`: `0x12` needs none. `tests/test_dvs.py` pins both the four panel pairs
and the `0x12` constants. Still `UNKNOWN`: the third payload byte (0 in every
reply so far). Probably signed, on the strength of `0x12`'s −560, but no
sub-zero reading has been captured.

**A moving temperature cannot pin the scale.** `dvs_temp_capture.log` is a
transient — 342→412→354 counts in 40 s — and it crosses the 24/25 °C display
boundary twice, so "the panel said 25" pairs with any raw from 389 to 412
depending on when you looked, and a filtered display lags a fast ramp anyway.
The 21 ↔ 325 panel pair, the one outlier of the four, is most likely this. Pair
panel readings against the bus only at a settled temperature.

Still open: **what one count is worth in RPM** (needs the calibration sweep with
a tachometer), what `0x11` is for, whether a `0x03` write
carries a payload, the true top of the demand range (hidden by saturation), and
the two-writes-per-cycle divergence in the capture — that last one is controller
behaviour, so it needs a passive capture, not `verify.py`.

**Two capture-derived claims turned out wrong, both the same way:** a *wall
controller policy* was read as a *fan property*. The dial's `0x00`–`0x7F` was
not the register's range, and the controller's use of `0x15` as a keepalive did
not mean `0x03` was dead. When the only evidence is a capture, it can show what
the controller chooses, never what the fan permits.

**Writing register `0x11` runs the fan flat out** — any value, including `0x00`,
starts a parked fan and drives it to maximum, overriding the demand in `0x15`.
It also keeps the fan alive on its own, so nothing winds down while something is
writing there. `config.setpoint_register` defaulted to `0x11`, which made menu
`7` ("set fan speed") a run-flat-out button; it is now `0x15`.

What `0x11` is *for* remains `UNKNOWN`, and the obvious guesses are now ruled
out. The wall controller never writes it — not across the full sweep of the
speed dial, and not switching its timed boost mode on and off. Boost turns out
to be `0x15` held at `0x7F`, the same frame the dial sends at Max. So `0x11` is
a register no normal operation of the unit touches; a service or factory
function is the remaining guess, and it is only a guess.

This is also a warning about `discovery.hunt_setpoint`: it ranks candidates on
how far the speed moved, and `0x11` moves it a long way without being a setpoint.
**"Moved the speed" is not "controls the speed."** Confirm a winner by writing
two different non-zero values and checking the speed lands somewhere different
for each, rather than pinning to full both times.

**Writing register `0x04` permanently changes the fan's bus address** and
getting it wrong means scanning all 256 addresses to find the unit again. It
carries a `warning` in `REGISTERS`, requires typing `yes`, and along with `0x05`
(stop) lives in `UNSAFE_TO_PROBE` so scans skip it. Keep all three protections
in place.

**Every byte in both directions is timestamped to `dvs_bus.log`.** When reverse
engineering, the log is the artefact — not the screen. Don't add I/O paths that
bypass `DvsBus.log`.

**A CRC mismatch on a reply almost always means the wrong CRC variant**, not
corrupt data. The CLI says so explicitly whenever it reports one; preserve that
hint rather than treating mismatches as errors.

**Register decoders print every plausible reading**, not one committed
interpretation — `decode_fan_speed` deliberately includes a `1/u16le` column
because a tacho *period* register reads large when the fan is slow. Keep that
shape until the units question is actually closed.

## Physical setup

4800 baud, 8N1 (the baud rate is confirmed by a logic-analyser capture; the
framing is an assumption and is adjustable in settings). The 5-pin 5.08mm
connector is A=Ground, B=RS485 B, C=RS485 A, D=+30 VDC, E=+12 VDC — wire A/B/C
only and leave the power pins alone. If nothing answers, swapping A and B is
harmless and often the fix.

`FAN_SPEED_UNITS_PLAN.md` holds the full experimental procedure for the units
question, including cheap no-instrument discriminators worth trying first (a
stopped-fan read of `0x15` settles rate-vs-period in about thirty seconds).
