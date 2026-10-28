# dvs

Claude Code and Codex were both used extensively for this project. They made it
possible for me to do this work when I wouldn't have had the time otherwise.

A menu-driven CLI for talking to a DVS air handling unit over its RS485 bus.

Protocol per [Ian Rees' reverse engineering write-up][post]. You pick a message
by pressing a key, enter values if the message needs them, and the tool builds
the frame, appends the checksum and sends it.

[post]: https://ianrees.nz/tech/2018-07-11-reverse-engineering-the-dvs-bus.html

## Install

```bash
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
```

## Run

```bash
.venv/bin/python -m dvs
```

Find your adapter first if you don't know its device path:

```bash
.venv/bin/python -m dvs --list-ports
```

Settings persist to `dvs_config.json`, so the port, address and CRC variant only
need setting once. Every byte in both directions is timestamped into
`dvs_bus.log`.

## Wiring

The 5-pin 5.08mm connector on the air handler:

| Pin | Signal |
|---|---|
| A | Ground |
| B | RS485 B |
| C | RS485 A |
| D | +30 VDC |
| E | +12 VDC |

Connect A, B and C to the adapter. Leave D and E alone — they power the DVS
controller, and your USB adapter is powered over USB. If nothing answers, swap
A and B; the labelling convention for RS485 pairs is not consistently honoured
across vendors and swapping them is harmless.

Bus settings are 4800 baud, 8N1. The post confirms the baud rate from a logic
analyser capture but never states the framing, so 8N1 is an assumption — it's
adjustable in settings if it turns out to be wrong.

## First run

The write-up leaves three things undetermined, and you need all three before
anything else works. In order:

1. **`c` — identify the CRC variant.** The post says "CRC16_CCITT", which names
   at least four mutually incompatible algorithms, and doesn't say which order
   the two checksum bytes go on the wire. This sends the same read with all 14
   combinations; only the right one gets a reply. If you have a logic analyser
   capture of the real controller, `d` identifies it offline from the captured
   bytes instead.

2. **`a` — scan for the address.** The post establishes that the first byte is
   an address and that the fan ignores everything else, but never says what the
   address actually was. The scan reads one register at each of 0x00–0xFF and
   reports what answers.

3. **`h` — hunt for the setpoint register.** This is the significant gap. The
   post's prose describes a speed setpoint register ("the fan has one register
   to control the speed setpoint, and a separate one that … tells the fan to
   spin for a little while") but the published register table doesn't include
   it. The hunt writes low and high values to each candidate register in turn
   and watches whether the reported speed at `0x15` moves.

Each of these offers to save its result to the config when it finds one.

## Menu

```
Read      1  actual fan speed (0x15)      Discover  a  scan addresses
          2  temperature (0x10)                     b  scan registers
          3  any register                           c  identify CRC by probing
Write     4  deadman kick (0x03)                    d  identify CRC from capture
          5  toggle deadman keepalive               h  hunt setpoint register
          6  stop fan (0x05)              Listen    m  monitor and capture the bus
          7  set speed setpoint                     n  decode a capture file
          8  any register                 Units     f  calibration sweep
          9  raw frame, no checksum                 g  analyse a sweep CSV
                                          Theory    t  test the 0x15 theory
                                                    r  review a transcript
                                          Other     s  settings
                                                    o  open/reopen port
                                                    q  quit
```

## Frame format

```
[address:1][register:1][payload:0..N][checksum:2]
```

Empty payload is a read; a payload makes it a write. Reply lengths vary by
register and aren't encoded anywhere in the frame, so the transport reads until
the bus has been quiet for 15 ms rather than expecting a fixed byte count.

Known registers:

| Reg | Meaning |
|---|---|
| 0x03 | Deadman — write every couple of seconds or the fan spins down |
| 0x04 | Set bus address (1 byte payload) |
| 0x05 | Stop |
| 0x10 | Temperature at fan, 3 bytes LSB MSB 0, **units unknown** |
| 0x15 | Actual fan speed, **length and units unknown** |
| ?    | Speed setpoint — **register number unknown**, use `h` |

## Listening to the real controller

Wire the adapter onto the bus with the fan's own wall controller still connected
and `m` records the conversation between them. It is the one experiment that
answers questions the fan can't be asked directly: what the controller sends,
how often, and what comes back.

Capture and decode are deliberately separate steps.

`m` **captures**. It transmits nothing, so neither the controller nor the fan
notices it's there. Every burst of bytes goes to `dvs_capture.log`
(`capture_path` in settings) as a timestamped line of hex, and appears on screen
decoded as it arrives. The file holds bytes and nothing else — no interpretation
to be invalidated later. Ctrl-C stops it.

```
2026-08-05T09:23:32.139 02 03 D2 C1
2026-08-05T09:23:32.758 02 15 A0 36 02 15 2C 01 18 41
```

`n` **decodes**, and can be re-run over the same capture as often as you like —
after pinning the CRC variant, after the setpoint register turns up, after a
register's units are settled:

```
2026-08-05T09:23:32.139  02 03 D2 C1              # 0x02  deadman kick - fan keeps spinning …
2026-08-05T09:23:32.758  02 15 A0 36              # 0x02  read fan_speed (0x15)
2026-08-05T09:23:32.758  02 15 2C 01 18 41        # 0x02  fan_speed is bytes=2C 01  u8=44  u16le=300 …
```

Two things are worth knowing about how that second line became two:

- **The checksum is the frame boundary.** A burst is whatever arrived between
  two 15 ms silences, which is often a request and its reply back to back.
  Nothing in a frame states its length, so the decoder walks forward until a
  prefix verifies, emits it, and starts again. Bytes it can't verify are
  reported rather than dropped — that's the signature of the wrong CRC variant.
- **Write and reply look identical on the wire.** Both are a payload frame;
  there is no direction bit. A payload frame on the address and register of the
  read that just went past is read as the answer to it (`fan_speed is …`);
  anything else is read as a write (`write fan_speed = …`). Ordering is the only
  evidence there is, so a capture that starts mid-exchange can get the first
  frame's direction wrong.

Stop the deadman before capturing, or the keepalive's own kicks end up in the
recording. `m` offers to do it for you.

## The deadman

The fan spins down a couple of seconds after the last write to `0x03`, so
anything that needs it running needs the keepalive going. Menu `5` starts a
background thread that kicks it every second; the header shows the kick count.
Kicks use a short timeout and share the port with foreground commands under a
lock, so the menu stays responsive while it runs.

The CLI stops the deadman and sends Stop on exit and on Ctrl-C. It does not
survive being killed with SIGKILL — if that happens the fan stops on its own
within a few seconds anyway, which is the point of a deadman.

Whether a write to `0x03` carries a payload is not documented. The default is a
payload-less write; `deadman_payload` in settings takes a hex string if it turns
out to need one.

## Fan speed units

See [FAN_SPEED_UNITS_PLAN.md](FAN_SPEED_UNITS_PLAN.md). **Phase 0 is done:** the
rate-vs-period question is settled — a stopped fan reads `00 00` rather than
saturating, so `0x15` is a rate (`raw ∝ rpm`). The period model and
`decode_fan_speed`'s `1/u16le` column are gone.

What's left is the scale: nobody knows yet what one count is worth in RPM. That
still needs menu `f`/`g` — the sweep against an independently measured RPM — and
the fit is now a one-model job. Useful anchors from the theory run: the demand
and the reading are in the same units, the impeller tops out at a reading of 80,
and it won't start below a demand of 12.

## Testing the 0x15 theory

A passive capture of the wall controller being walked from boost 0 to Max to −10
suggests a specific story for `0x15`, and parts of it disagree with what this
codebase assumes. Menu `t` drives each claim onto the bus and answers it with
**supported**, **contradicted** or **inconclusive**:

| | Claim | Result on hardware |
|---|---|---|
| T1 | a 1-byte write to `0x15` is answered with 2 payload bytes, LE | **supported** — and bare reads work too |
| T2 | the demand is 7-bit, `0x00`–`0x7F` | **contradicted** — bit 7 is not masked |
| T3 | the answer is the actual speed, same units as the demand | **supported** — demand 56 settles at exactly 56 |
| T4 | the answer is a rate, not a tacho period | **supported** — 0 stopped against 56 turning |
| T5 | `0x15` writes are the keepalive, not `0x03` kicks | **contradicted** — `0x03` is a keepalive too |
| T6 | there is a minimum runnable demand (~24) | **supported**, but the floor is 12 |

Both contradictions were the same mistake: reading a **wall controller policy**
as a **fan property**. The dial only ever sent `0x00`–`0x7F` and the controller
only ever used `0x15` writes to keep the fan alive, but neither was a limit of
the hardware. A capture shows what the controller chooses, never what the fan
permits — which is the whole argument for driving it directly.

The controller's **timed boost mode** — the one in its settings, not the speed
dial — turns out to be nothing new on the wire: it holds `0x15` at `0x7F` and
restores the previous demand when the timer ends, byte for byte the frame the
dial sends at Max. The timer runs in the controller.

```
12:31:09.662  02 15 7F 0C 92   demand  73 -> 127   boost on
12:31:31.478  02 15 49 5A 07   demand 127 ->  73   boost off
```

The measured behaviour is written up beside `REGISTERS[0x15]` in `protocol.py`.
The short version: `0x15` takes a 1-byte demand and answers with the actual
speed as `u16le` in the same units, it's a rate, the impeller saturates at 80,
the starting floor is 12, and both `0x15` writes and `0x03` kicks refresh the
keepalive timeout.

Every frame in both directions goes to `dvs_verify.jsonl` with a millisecond
timestamp, one JSON record each, alongside the verdicts:

```
{"kind": "exchange", "at": 0.031, "wall": "2026-08-05T10:25:26.252", "check": "T1",
 "phase": "1-byte write", "tx": "02 15 38 34 B1", "rx": "02 15 00 00 4B EB",
 "crc_ok": true, "register": 21, "demand": 56, "actual": 0}
```

Menu `r` reads that back as verdicts plus a demand/actual timeline, so a spin-up
or a coast-down is visible by eye. The file is deliberately dumb — hex, numbers
and labels, no prose — so it stays valid when we change our mind about what the
bytes mean, the same reason the bus capture is.

The run takes roughly ten minutes at the default settle times, spins the fan flat
out for T2, and deliberately stops and restarts it several times.

## Things that will bite you

- **Adapter echo.** Most cheap USB-RS485 adapters put transmitted bytes straight
  back into the receive buffer. Handled by default (`strip_echo`), and the CLI
  says so when it strips one.
- **CRC mismatch on a reply** almost always means the CRC variant is wrong, not
  that the data is corrupt. Run `c`.
- **Writing 0x04.** It permanently changes the fan's address, and getting it
  wrong means scanning all 256 addresses to find the unit again. The CLI makes
  you type `yes`. The register and setpoint scans skip it entirely.
- **Writing 0x11 runs the fan flat out** — any value, `0x00` included — ignoring
  the demand in 0x15, and holds it there as long as you keep writing. It is not
  the setpoint register, though the setpoint hunt will happily nominate it,
  because "moved the speed" and "controls the speed" aren't the same test. Set
  speed with 0x15. Nothing the wall controller does writes 0x11, so what it's
  for is still unknown.
- **Silence from everything.** Check A/B orientation, then that the fan has
  mains power — the RS485 transceiver in the unit is powered from the fan side.

## Tests

```bash
.venv/bin/python -m pytest tests/ -q
```

71 tests. `test_dvs.py` covers the CRC implementations against the published
catalogue check values, frame encoding, the model fitting against synthetic rate
and period data, and the monitor's frame splitting and annotation.
`test_transport_pty.py` runs a fake fan on a pty pair and covers the parts that
only misbehave on a real link: variable-length replies, the idle-gap terminator,
echo stripping, the deadman thread contending with foreground reads for the
port, and a passive capture proving it puts nothing on the bus.
`test_verify.py` points the theory checks at fake fans built to *break* one claim
each, and asserts the matching check says so — a checker that can only ever agree
with the theory it came from would pass a much easier test suite.

No hardware is needed for any of them.
