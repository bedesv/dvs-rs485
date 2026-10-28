"""Interactive menu for talking to a DVS air handler over RS485."""

import argparse
import sys
import termios
import tty
from pathlib import Path

import serial.tools.list_ports

from . import calibrate, discovery, monitor, protocol, verify
from .config import CONFIG_PATH, Config
from .crc import ALGORITHMS, BYTE_ORDERS, CrcSpec, identify
from .protocol import REGISTERS, Frame, hexdump
from .transport import BusError, Deadman, DvsBus

BOLD = "\033[1m"
DIM = "\033[2m"
RED = "\033[31m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
RESET = "\033[0m"


# -- input helpers ---------------------------------------------------------


def get_key(prompt: str = "select> ") -> str:
    """Read a single keypress without waiting for Enter."""
    print(prompt, end="", flush=True)
    if not sys.stdin.isatty():
        line = sys.stdin.readline()
        return line.strip()[:1].lower() if line else "q"
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        char = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
    if char in ("\x03", "\x04"):  # ctrl-c, ctrl-d
        print()
        raise KeyboardInterrupt
    print(char)
    return char.lower()


class Cancelled(Exception):
    """The user backed out of a prompt."""


def ask(prompt: str, default: str | None = None) -> str:
    suffix = f" [{default}]" if default is not None else ""
    try:
        value = input(f"{prompt}{suffix} (blank to cancel): ").strip()
    except EOFError:
        raise Cancelled
    if not value:
        if default is not None:
            return default
        raise Cancelled
    return value


def ask_int(prompt: str, default: int | None = None, lo: int = 0, hi: int = 0xFF) -> int:
    while True:
        text = ask(f"{prompt} (decimal, or 0x.. for hex)", None if default is None else str(default))
        try:
            value = int(text, 0)
        except ValueError:
            print(f"  {RED}not a number: {text!r}{RESET}")
            continue
        if not lo <= value <= hi:
            print(f"  {RED}must be between {lo} and {hi}{RESET}")
            continue
        return value


def ask_float(prompt: str, default: float | None = None) -> float:
    while True:
        text = ask(prompt, None if default is None else str(default))
        try:
            return float(text)
        except ValueError:
            print(f"  {RED}not a number: {text!r}{RESET}")


def ask_hex(prompt: str, allow_empty: bool = True) -> bytes:
    while True:
        try:
            text = input(f"{prompt} (hex bytes, e.g. '01 F4'): ").strip()
        except EOFError:
            raise Cancelled
        if not text:
            if allow_empty:
                return b""
            raise Cancelled
        try:
            return bytes.fromhex(text.replace(",", " "))
        except ValueError:
            print(f"  {RED}not valid hex: {text!r}{RESET}")


def confirm(prompt: str, word: str = "yes") -> bool:
    try:
        return input(f"{prompt} Type '{word}' to confirm: ").strip().lower() == word
    except EOFError:
        return False


# -- the app ---------------------------------------------------------------


class App:
    def __init__(self, config: Config, config_path: Path = CONFIG_PATH):
        self.config = config
        self.config_path = config_path
        self.bus = DvsBus(config)
        self.deadman = Deadman(self.bus)

    # -- output ------------------------------------------------------------

    def show_response(self, response, register: int | None = None) -> None:
        if not response:
            print(f"  {YELLOW}no response{RESET} (timed out after {self.config.response_timeout}s)")
            return
        status = f"{GREEN}crc ok{RESET}" if response.crc_ok else f"{RED}crc MISMATCH{RESET}"
        print(f"  rx {hexdump(response.raw)}  ({len(response.raw)} bytes, {status})")
        if not response.crc_ok:
            print(
                f"  {DIM}A mismatch here usually means the CRC variant is wrong, not that "
                f"the data is bad. Try menu item 'c'.{RESET}"
            )
        if response.payload:
            print(f"  payload {hexdump(response.payload)}")
        if register is not None:
            described = protocol.describe_payload(register, response.payload)
            if described:
                print(f"  {described}")
        for note in response.notes:
            print(f"  {DIM}note: {note}{RESET}")

    def require_port(self) -> bool:
        if self.bus.is_open:
            return True
        if not self.config.port:
            print(f"  {RED}No serial port set.{RESET} Use 's' (settings) to pick one.")
            return False
        try:
            self.bus.open()
            print(f"  opened {self.config.port}")
            return True
        except (BusError, OSError) as exc:
            print(f"  {RED}could not open {self.config.port}: {exc}{RESET}")
            return False

    # -- menu --------------------------------------------------------------

    def header(self) -> None:
        cfg = self.config
        port = cfg.port or "(unset)"
        state = f"{GREEN}open{RESET}" if self.bus.is_open else f"{DIM}closed{RESET}"
        dead = (
            f"{GREEN}running{RESET} ({self.deadman.kicks} kicks)"
            if self.deadman.running
            else f"{DIM}stopped{RESET}"
        )
        setpoint = (
            f"0x{cfg.setpoint_register:02X}"
            if cfg.setpoint_register is not None
            else f"{YELLOW}unknown{RESET}"
        )
        print()
        print(f"{BOLD}  DVS bus{RESET}")
        print(
            f"  port {port} [{state}]  {cfg.baud} {cfg.byte_size}{cfg.parity}{cfg.stop_bits}"
            f"   addr 0x{cfg.address:02X}   crc {cfg.crc}"
        )
        print(f"  deadman {dead}   setpoint register {setpoint}")

    MENU = [
        ("", "Read"),
        ("1", "Read actual fan speed        (0x15)"),
        ("2", "Read temperature at fan      (0x10)"),
        ("3", "Read any register"),
        ("", "Write"),
        ("4", "Deadman kick                 (0x03, one shot)"),
        ("5", "Toggle deadman keepalive"),
        ("6", "Stop fan                     (0x05)"),
        ("7", "Set fan speed setpoint"),
        ("8", "Write any register"),
        ("9", "Send a raw frame (no checksum added)"),
        ("", "Discover the undocumented bits"),
        ("a", "Scan for device addresses"),
        ("b", "Scan registers at this address"),
        ("c", "Identify CRC variant by probing the fan"),
        ("d", "Identify CRC variant from captured frames"),
        ("h", "Hunt for the speed setpoint register"),
        ("", "Listen to the real controller"),
        ("m", "Monitor the bus and capture it to a file"),
        ("n", "Decode a capture file"),
        ("", "Fan speed units"),
        ("f", "Run a calibration sweep"),
        ("g", "Analyse a sweep CSV"),
        ("", "Test the 0x15 speed-control theory"),
        ("t", "Run the theory checks against the fan"),
        ("r", "Review a saved test transcript"),
        ("", "Other"),
        ("s", "Settings"),
        ("o", "Open / reopen serial port"),
        ("q", "Quit"),
    ]

    def show_menu(self) -> None:
        print()
        for key, label in self.MENU:
            if not key:
                print(f"  {DIM}{label}{RESET}")
            else:
                print(f"   {BOLD}{key}{RESET}  {label}")
        print()

    def run(self) -> int:
        actions = {
            "1": self.do_read_speed,
            "2": self.do_read_temperature,
            "3": self.do_read_any,
            "4": self.do_deadman_kick,
            "5": self.do_toggle_deadman,
            "6": self.do_stop,
            "7": self.do_set_speed,
            "8": self.do_write_any,
            "9": self.do_raw,
            "a": self.do_scan_addresses,
            "b": self.do_scan_registers,
            "c": self.do_identify_crc_probe,
            "d": self.do_identify_crc_capture,
            "h": self.do_hunt_setpoint,
            "m": self.do_monitor,
            "n": self.do_decode_capture,
            "f": self.do_sweep,
            "g": self.do_analyse,
            "t": self.do_verify,
            "r": self.do_review,
            "s": self.do_settings,
            "o": self.do_open,
        }
        while True:
            self.header()
            self.show_menu()
            try:
                key = get_key()
            except KeyboardInterrupt:
                key = "q"
            if key == "q":
                break
            action = actions.get(key)
            if action is None:
                print(f"  {DIM}nothing bound to {key!r}{RESET}")
                continue
            try:
                action()
            except Cancelled:
                print(f"  {DIM}cancelled{RESET}")
            except KeyboardInterrupt:
                print(f"\n  {YELLOW}interrupted{RESET}")
            except (BusError, OSError, ValueError) as exc:
                print(f"  {RED}error: {exc}{RESET}")

        if self.deadman.running:
            print("  stopping deadman and the fan...")
            self.deadman.stop(send_stop=True)
        self.bus.close()
        return 0

    # -- read actions ------------------------------------------------------

    def _read(self, register: int) -> None:
        if not self.require_port():
            return
        reg = REGISTERS.get(register)
        name = f" ({reg.name})" if reg else ""
        print(f"  reading register 0x{register:02X}{name} from address 0x{self.config.address:02X}")
        self.show_response(self.bus.read_register(register), register)

    def do_read_speed(self) -> None:
        self._read(0x15)

    def do_read_temperature(self) -> None:
        self._read(0x10)

    def do_read_any(self) -> None:
        self._read(ask_int("register"))

    # -- write actions -----------------------------------------------------

    def do_deadman_kick(self) -> None:
        if not self.require_port():
            return
        payload = self.config.deadman_bytes
        print(f"  kicking deadman (0x03) payload={hexdump(payload) if payload else '(none)'}")
        # Full timeout here, unlike the background thread: a one-shot kick is a
        # diagnostic, so wait long enough to see whether the fan acknowledges.
        self.show_response(self.deadman.kick(timeout=self.config.response_timeout))

    def do_toggle_deadman(self) -> None:
        if self.deadman.running:
            self.deadman.stop()
            print("  deadman stopped - the fan will spin down in a couple of seconds")
            return
        if not self.require_port():
            return
        self.deadman.start()
        print(f"  deadman started, kicking 0x03 every {self.config.deadman_interval}s")

    def do_stop(self) -> None:
        if not self.require_port():
            return
        if self.deadman.running:
            self.deadman.stop()
            print("  (stopped the deadman keepalive too, or it would restart the fan)")
        self.show_response(self.bus.write_register(protocol.STOP_REGISTER))

    def do_set_speed(self) -> None:
        if self.config.setpoint_register is None:
            print(
                f"  {YELLOW}The setpoint register number is not known.{RESET}\n"
                f"  {DIM}The write-up describes it but never gives its number. Run 'h' to\n"
                f"  hunt for it, then set it in settings.{RESET}"
            )
            return
        if not self.require_port():
            return
        value = ask_int("setpoint value")
        register = self.config.setpoint_register
        print(f"  writing 0x{value:02X} to setpoint register 0x{register:02X}")
        self.show_response(self.bus.write_register(register, bytes([value])))
        if not self.deadman.running:
            print(
                f"  {YELLOW}The deadman isn't running, so the fan won't keep spinning.{RESET}"
                " Press '5' to start it."
            )

    def do_write_any(self) -> None:
        if not self.require_port():
            return
        register = ask_int("register")
        reg = REGISTERS.get(register)
        if reg and reg.warning:
            print(f"  {RED}{reg.warning}{RESET}")
            if not confirm("  Really write this register?"):
                raise Cancelled
        payload = ask_hex("  payload")
        print(f"  writing 0x{register:02X} payload={hexdump(payload) if payload else '(none)'}")
        self.show_response(self.bus.write_register(register, payload), register)

    def do_raw(self) -> None:
        if not self.require_port():
            return
        print(f"  {DIM}Bytes are sent verbatim - no checksum is appended.{RESET}")
        data = ask_hex("  frame", allow_empty=False)
        rx = self.bus.send_raw(data)
        print(f"  tx {hexdump(data)}")
        print(f"  rx {hexdump(rx)}" if rx else f"  {YELLOW}no response{RESET}")

    # -- discovery ---------------------------------------------------------

    def do_scan_addresses(self) -> None:
        if not self.require_port():
            return
        register = ask_int("probe which register", 0x15)
        lo = ask_int("first address", 0x00)
        hi = ask_int("last address", 0xFF)
        print(f"  scanning 0x{lo:02X}..0x{hi:02X}, this takes about {(hi - lo + 1) * 0.14:.0f}s")
        hits = discovery.scan_addresses(self.bus, register, range(lo, hi + 1))
        if not hits:
            print(
                f"  {YELLOW}nothing answered.{RESET} Either the CRC variant is wrong (try 'c'),\n"
                "  or A and B are swapped, or the fan isn't powered."
            )
            return
        print(f"  {GREEN}{len(hits)} device(s) answered{RESET}")
        if len(hits) == 1 and confirm(f"  Use address 0x{hits[0].address:02X}?"):
            self.config.address = hits[0].address
            self.config.save(self.config_path)
            print("  saved")

    def do_scan_registers(self) -> None:
        if not self.require_port():
            return
        lo = ask_int("first register", 0x00)
        hi = ask_int("last register", 0x40)
        print(f"  reading registers 0x{lo:02X}..0x{hi:02X} at address 0x{self.config.address:02X}")
        print(f"  {DIM}read-only: empty payloads can't change the fan's state{RESET}")
        hits = discovery.scan_registers(self.bus, self.config.address, range(lo, hi + 1))
        print(f"  {len(hits)} register(s) answered")

    def do_identify_crc_probe(self) -> None:
        if not self.require_port():
            return
        print(
            "  Sending the same read with every candidate checksum. Only the correct\n"
            "  one gets an answer."
        )
        address = ask_int("address to probe", self.config.address)
        register = ask_int("register to read", 0x15)
        winners = discovery.identify_crc_by_probe(self.bus, address, register)
        if not winners:
            print(
                f"  {YELLOW}No variant got a reply.{RESET} If you're sure the wiring and address\n"
                "  are right, the checksum may not be one of these seven - capture a real\n"
                "  controller frame and use 'd'."
            )
            return
        print(f"  {GREEN}replied under: {', '.join(str(w) for w in winners)}{RESET}")
        if len(winners) > 1:
            print(f"  {DIM}More than one matched - try a register with a longer reply.{RESET}")
        if confirm(f"  Save {winners[0]}?"):
            self.config.crc_algorithm = winners[0].algorithm
            self.config.crc_byte_order = winners[0].byte_order
            self.config.save(self.config_path)
            print("  saved")

    def do_identify_crc_capture(self) -> None:
        print(
            "  Paste known-good frames, one per line, as hex including the two checksum\n"
            "  bytes. Blank line when done. Two or three frames normally pin it down."
        )
        frames: list[bytes] = []
        while True:
            try:
                line = input("  frame> ").strip()
            except EOFError:
                break
            if not line:
                break
            try:
                frames.append(bytes.fromhex(line.replace(",", " ")))
            except ValueError:
                print(f"    {RED}not valid hex{RESET}")
        if not frames:
            raise Cancelled
        survivors = identify(frames)
        if not survivors:
            print(
                f"  {RED}No candidate matches all {len(frames)} frames.{RESET} Check you included\n"
                "  the checksum bytes and nothing else (no preamble, no trailing byte)."
            )
            return
        for spec in survivors:
            print(f"  {GREEN}match:{RESET} {spec}")
        if confirm(f"  Save {survivors[0]}?"):
            self.config.crc_algorithm = survivors[0].algorithm
            self.config.crc_byte_order = survivors[0].byte_order
            self.config.save(self.config_path)
            print("  saved")

    def do_hunt_setpoint(self) -> None:
        if not self.require_port():
            return
        print(
            f"  {BOLD}This spins the fan up and down.{RESET} For each candidate register it\n"
            "  writes a low value, waits, reads 0x15, then writes a high value and reads\n"
            "  again. The setpoint register is the one where reported speed moves most.\n"
            "  Any register that moved the speed is zeroed and the fan stopped for a\n"
            "  settle period before the next candidate, so nothing carries over.\n"
            f"  {DIM}0x04 (set address) and 0x05 (stop) are skipped.{RESET}"
        )
        lo = ask_int("first candidate register", 0x00)
        hi = ask_int("last candidate register", 0x3F)
        settle = ask_float("settle seconds per step", 6.0)
        count = hi - lo + 1
        print(f"  {YELLOW}~{count * 2 * settle / 60:.0f} minutes for {count} registers{RESET}")
        if not confirm("  Start the hunt?"):
            raise Cancelled
        results = discovery.hunt_setpoint(
            self.bus, self.config.address, range(lo, hi + 1), settle=settle
        )
        movers = [r for r in results if r.moved]
        unreadable = [r for r in results if r.unreadable]
        if unreadable:
            regs = ", ".join(f"0x{r.register:02X}" for r in unreadable)
            print(
                f"  {YELLOW}Speed read failed after {discovery.SPEED_READ_ATTEMPTS} tries"
                f" for:{RESET} {regs}\n"
                "  Those registers are unscored, not zero - re-run them before ruling them out."
            )
        if not movers:
            print(
                f"  {YELLOW}Nothing moved the speed reading.{RESET} Check the fan actually spins\n"
                "  while the deadman runs, and widen the candidate range."
            )
            return
        print(f"\n  {BOLD}Best candidates:{RESET}")
        for candidate in movers[:5]:
            print(
                f"    0x{candidate.register:02X}: {candidate.low_speed} -> "
                f"{candidate.high_speed} (delta {candidate.delta})"
            )
        best = movers[0]
        if confirm(f"  Save 0x{best.register:02X} as the setpoint register?"):
            self.config.setpoint_register = best.register
            self.config.save(self.config_path)
            print("  saved")

    # -- monitoring --------------------------------------------------------

    def do_monitor(self) -> None:
        if not self.require_port():
            return
        print(
            f"  {BOLD}Passive bus monitor.{RESET} Nothing is transmitted, so you can leave the\n"
            "  fan's own wall controller wired in and watch the two of them talk.\n"
            f"  {DIM}Each burst is appended to the capture file raw, and shown here decoded.\n"
            "  The file keeps the bytes only, so you can decode it again later under a\n"
            f"  different CRC variant. Ctrl-C stops.{RESET}"
        )
        if self.deadman.running:
            print(
                f"  {YELLOW}The deadman is running, so our own kicks will be in the capture.{RESET}"
            )
            if confirm("  Stop it first?"):
                self.deadman.stop()
                print("  deadman stopped")
        path = Path(ask("capture file", self.config.capture_path))
        decoder = monitor.Decoder(self.config.crc, self.config.setpoint_register)

        # Counted here rather than from capture()'s return value, because the
        # normal way out of this is Ctrl-C and there's no return value then.
        bursts = 0

        def show(at: float, data: bytes) -> None:
            nonlocal bursts
            bursts += 1
            for message in decoder.feed(monitor.format_stamp(at), data):
                colour = "" if message.framed else YELLOW
                print(f"  {colour}{message.line()}{RESET}")

        print(f"  {GREEN}listening{RESET} - appending to {path}")
        try:
            monitor.capture(self.bus, path, on_burst=show)
        except KeyboardInterrupt:
            pass
        print(f"\n  stopped after {bursts} burst(s); {path} is ready to decode with 'n'")

    def do_decode_capture(self) -> None:
        path = Path(ask("capture file", self.config.capture_path))
        if not path.exists():
            print(f"  {RED}no such file: {path}{RESET}")
            return
        print(
            f"  decoding under {self.config.crc}"
            + (
                f", setpoint register 0x{self.config.setpoint_register:02X}"
                if self.config.setpoint_register is not None
                else ", setpoint register unknown"
            )
        )
        out = ask("write decoded lines to (- for screen only)", "-")
        messages = list(
            monitor.decode(
                monitor.read_capture(path),
                self.config.crc,
                self.config.setpoint_register,
            )
        )
        for message in messages:
            colour = "" if message.framed else YELLOW
            print(f"  {colour}{message.line()}{RESET}")
        if out != "-":
            Path(out).write_text("\n".join(m.line() for m in messages) + "\n")
            print(f"  wrote {len(messages)} line(s) to {out}")
        unframed = sum(1 for m in messages if not m.framed)
        print(f"  {len(messages) - unframed} frame(s) decoded, {unframed} unrecognised")
        if unframed:
            print(
                f"  {DIM}Unrecognised bytes are nearly always the wrong CRC variant. Paste a\n"
                f"  captured frame into 'd' to pin the variant down, then decode again.{RESET}"
            )

    # -- calibration -------------------------------------------------------

    def do_sweep(self) -> None:
        if not self.require_port():
            return
        if self.config.setpoint_register is None:
            print(f"  {YELLOW}Set the setpoint register first (run 'h').{RESET}")
            return
        print(
            f"  {BOLD}Fan speed calibration sweep.{RESET}\n"
            "  At each step you'll be asked for an independently measured RPM. See\n"
            "  FAN_SPEED_UNITS_PLAN.md for how to measure it. Press Enter at the prompt\n"
            "  to record raw values only.\n"
            f"  The fan is held up by re-asserting the demand every "
            f"{self.config.speed_hold_interval}s, the way the wall\n"
            "  controller does - no 0x03 kicks are needed."
        )
        if self.deadman.running:
            print(
                f"  {YELLOW}The deadman is kicking 0x03 in the background.{RESET} The sweep holds\n"
                "  the fan up with 0x15 writes on its own, so the kicks are only extra\n"
                "  traffic competing for the port lock."
            )
            if confirm("  Stop the deadman first?"):
                self.deadman.stop()
                print("  deadman stopped")
        lo = ask_int("lowest setpoint", 0x20)
        hi = ask_int("highest setpoint", 0xFF)
        steps = ask_int("number of steps", 8, lo=2, hi=64)
        settle = ask_float("settle seconds per step", 30.0)
        out = Path(ask("output CSV", "sweep.csv"))

        span = (hi - lo) / (steps - 1)
        setpoints = [round(lo + span * i) for i in range(steps)]
        # Interleave up and down so a slow thermal drift shows up as scatter
        # rather than as a fake trend in the fit.
        setpoints = setpoints[::2] + setpoints[1::2][::-1]
        print(f"  order: {', '.join(f'0x{s:02X}' for s in setpoints)}")
        print(f"  {YELLOW}~{len(setpoints) * settle / 60:.0f} minutes{RESET}")
        if not confirm("  Start the sweep?"):
            raise Cancelled

        def prompt_rpm(setpoint: int, raw_text: str) -> float | None:
            while True:
                try:
                    text = input("  measured RPM (Enter to skip): ").strip()
                except EOFError:
                    return None
                if not text:
                    return None
                try:
                    return float(text)
                except ValueError:
                    print(f"    {RED}not a number{RESET}")

        sweep = calibrate.run_sweep(
            self.bus,
            self.config.setpoint_register,
            setpoints,
            settle=settle,
            prompt_rpm=prompt_rpm,
        )
        sweep.save_csv(out)
        print(f"  wrote {len(sweep.points)} points to {out}")
        self._report_fits(sweep)

    def do_analyse(self) -> None:
        path = Path(ask("sweep CSV", "sweep.csv"))
        if not path.exists():
            print(f"  {RED}no such file: {path}{RESET}")
            return
        self._report_fits(calibrate.Sweep.load_csv(path))

    def _report_fits(self, sweep: calibrate.Sweep) -> None:
        for field in ("raw_u16le", "raw_u16be", "raw_u8"):
            try:
                fits = calibrate.fit_models(sweep, field)
            except ValueError as exc:
                print(f"  {DIM}{field}: {exc}{RESET}")
                continue
            print(f"\n  {BOLD}{field}{RESET}")
            for fit in fits:
                print(f"    {fit.describe()}")
            for note in calibrate.interpret(fits):
                print(f"    {note}")

    # -- theory checks -----------------------------------------------------

    def do_verify(self) -> None:
        if not self.require_port():
            return
        print(
            f"  {BOLD}Test the 0x15 speed-control theory.{RESET}\n"
            "  Each claim below gets driven onto the bus and answered with a verdict of\n"
            "  supported, contradicted or inconclusive. Every frame in both directions is\n"
            "  written to a transcript you can review afterwards with 'r'.\n"
            f"  {YELLOW}This spins the fan, including flat out for the ceiling check, and\n"
            f"  deliberately lets it stop and restart several times.{RESET}"
        )
        for name, (title, _) in verify.CHECKS.items():
            print(f"    {BOLD}{name}{RESET}  {title}")

        chosen = ask("checks to run (comma separated, or 'all')", "all")
        which = None
        if chosen.lower() != "all":
            which = [c.strip().upper() for c in chosen.split(",") if c.strip()]
            unknown = [c for c in which if c not in verify.CHECKS]
            if unknown:
                print(f"  {RED}no such check(s): {', '.join(unknown)}{RESET}")
                return

        defaults = verify.Plan()
        plan = verify.Plan(
            settle=ask_float("settle seconds per demand", defaults.settle),
            spindown=ask_float("spin-down seconds when parking", defaults.spindown),
        )
        path = Path(ask("transcript file", self.config.verify_path))

        if self.deadman.running:
            print(
                f"  {YELLOW}The deadman is kicking 0x03 in the background.{RESET} That would\n"
                "  pollute the transcript and make T5 meaningless - it exists to find out\n"
                "  whether 0x03 does anything at all."
            )
            if not confirm("  Stop the deadman and continue?"):
                raise Cancelled
            self.deadman.stop()
            print("  deadman stopped")

        selected = list(verify.CHECKS) if which is None else which
        print(
            f"  {YELLOW}~{plan.estimated_seconds / 60:.0f} minutes for "
            f"{len(selected)} check(s){RESET}"
        )
        if not confirm("  Start the run?"):
            raise Cancelled

        findings, exchanges = verify.run_checks(
            self.bus, which=which, plan=plan, path=path
        )
        print(f"\n  {BOLD}Verdicts{RESET}")
        for line in verify.summarise(findings):
            print(f"  {line}")
        counts = {v: sum(1 for f in findings if f.verdict == v) for v in verify.VERDICTS}
        print(
            f"\n  {GREEN}{counts['supported']} supported{RESET}, "
            f"{RED}{counts['contradicted']} contradicted{RESET}, "
            f"{YELLOW}{counts['inconclusive']} inconclusive{RESET}"
            f"   {len(exchanges)} exchange(s) recorded in {path}"
        )
        if counts["contradicted"]:
            print(
                f"  {DIM}A contradicted claim is a result, not a failure - it's the thing\n"
                f"  worth writing down. Check the transcript before changing any code.{RESET}"
            )

    def do_review(self) -> None:
        path = Path(ask("transcript file", self.config.verify_path))
        if not path.exists():
            print(f"  {RED}no such file: {path}{RESET}")
            return
        transcript = verify.load_transcript(path)
        if not transcript.exchanges and not transcript.findings:
            print(f"  {YELLOW}nothing in {path}{RESET}")
            return
        for run in transcript.runs:
            print(
                f"  {DIM}run {run.get('wall', '?')}  crc {run.get('crc', '?')}"
                f"  address 0x{run.get('address', 0):02X}{RESET}"
            )
        if transcript.findings:
            print(f"\n  {BOLD}Verdicts{RESET}")
            for line in verify.summarise(transcript.findings):
                print(f"  {line}")
        print(f"\n  {BOLD}Exchanges{RESET} ({len(transcript.exchanges)})")
        lines = verify.timeline(transcript)
        out = ask("write the timeline to (- for screen)", "-")
        if out == "-":
            for line in lines:
                print(f"  {line}")
        else:
            Path(out).write_text("\n".join(lines) + "\n")
            print(f"  wrote {len(lines)} line(s) to {out}")

    # -- settings ----------------------------------------------------------

    def do_open(self) -> None:
        self.bus.close()
        if self.require_port():
            print(f"  {GREEN}port open{RESET}")

    def do_settings(self) -> None:
        cfg = self.config
        while True:
            print()
            print(f"  {BOLD}Settings{RESET}  {DIM}({self.config_path}){RESET}")
            print(f"   1  serial port ............. {cfg.port or '(unset)'}")
            print(f"   2  bus address ............. 0x{cfg.address:02X}")
            print(f"   3  CRC variant ............. {cfg.crc}")
            print(
                f"   4  setpoint register ....... "
                f"{'0x%02X' % cfg.setpoint_register if cfg.setpoint_register is not None else 'unknown'}"
            )
            print(f"   5  deadman payload / interval {cfg.deadman_payload or '(none)'} / {cfg.deadman_interval}s")
            print(f"   h  speed hold interval ..... {cfg.speed_hold_interval}s")
            print(f"   6  timeouts ................ resp {cfg.response_timeout}s, idle {cfg.idle_gap}s, turnaround {cfg.turnaround}s")
            print(f"   7  framing ................. {cfg.baud} {cfg.byte_size}{cfg.parity}{cfg.stop_bits}")
            print(f"   8  strip adapter echo ...... {cfg.strip_echo}")
            print(f"   9  log file ................ {cfg.log_path}")
            print(f"   0  bus capture file ....... {cfg.capture_path}")
            print(f"   v  theory transcript file . {cfg.verify_path}")
            print(f"   {BOLD}b{RESET}  back (saves)")
            try:
                key = get_key("  setting> ")
            except KeyboardInterrupt:
                break
            try:
                if key == "b":
                    break
                elif key == "1":
                    self._choose_port()
                elif key == "2":
                    cfg.address = ask_int("bus address", cfg.address)
                elif key == "3":
                    self._choose_crc()
                elif key == "4":
                    cfg.setpoint_register = ask_int("setpoint register", cfg.setpoint_register)
                elif key == "h":
                    cfg.speed_hold_interval = ask_float(
                        "speed hold interval (s)", cfg.speed_hold_interval
                    )
                elif key == "5":
                    cfg.deadman_payload = ask_hex("deadman payload").hex(" ").upper().replace(" ", "")
                    cfg.deadman_interval = ask_float("deadman interval (s)", cfg.deadman_interval)
                elif key == "6":
                    cfg.response_timeout = ask_float("response timeout (s)", cfg.response_timeout)
                    cfg.idle_gap = ask_float("idle gap ending a response (s)", cfg.idle_gap)
                    cfg.turnaround = ask_float("turnaround delay (s)", cfg.turnaround)
                elif key == "7":
                    cfg.baud = ask_int("baud", cfg.baud, lo=300, hi=1_000_000)
                    cfg.byte_size = ask_int("byte size", cfg.byte_size, lo=5, hi=8)
                    cfg.parity = ask("parity (N/E/O)", cfg.parity).upper()[:1]
                    cfg.stop_bits = ask_int("stop bits", cfg.stop_bits, lo=1, hi=2)
                elif key == "8":
                    cfg.strip_echo = not cfg.strip_echo
                elif key == "9":
                    cfg.log_path = ask("log file", cfg.log_path)
                elif key == "0":
                    cfg.capture_path = ask("bus capture file", cfg.capture_path)
                elif key == "v":
                    cfg.verify_path = ask("theory transcript file", cfg.verify_path)
            except Cancelled:
                print(f"  {DIM}unchanged{RESET}")

        cfg.save(self.config_path)
        self.bus.config = cfg
        print(f"  saved to {self.config_path}")

    def _choose_port(self) -> None:
        ports = list(serial.tools.list_ports.comports())
        if not ports:
            print(f"  {YELLOW}No serial ports found.{RESET} Is the RS485 adapter plugged in?")
        for index, port in enumerate(ports, 1):
            print(f"    {index}  {port.device}  {DIM}{port.description}{RESET}")
        choice = ask("port number, or a device path", self.config.port or None)
        if choice.isdigit() and 1 <= int(choice) <= len(ports):
            self.config.port = ports[int(choice) - 1].device
        else:
            self.config.port = choice
        self.bus.close()

    def _choose_crc(self) -> None:
        names = list(ALGORITHMS)
        for index, name in enumerate(names, 1):
            print(f"    {index}  {name}")
        choice = ask("algorithm number", str(names.index(self.config.crc_algorithm) + 1))
        if not (choice.isdigit() and 1 <= int(choice) <= len(names)):
            raise Cancelled
        algorithm = names[int(choice) - 1]
        order = ask(f"byte order {BYTE_ORDERS}", self.config.crc_byte_order)
        spec = CrcSpec(algorithm, order)  # validates
        self.config.crc_algorithm = spec.algorithm
        self.config.crc_byte_order = spec.byte_order


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Talk to a DVS air handler over RS485.")
    parser.add_argument("--port", help="serial device, e.g. /dev/tty.usbserial-A50285BI")
    parser.add_argument("--address", type=lambda s: int(s, 0), help="bus address, e.g. 0x01")
    parser.add_argument("--config", type=Path, default=CONFIG_PATH, help="config file path")
    parser.add_argument("--list-ports", action="store_true", help="list serial ports and exit")
    args = parser.parse_args(argv)

    if args.list_ports:
        for port in serial.tools.list_ports.comports():
            print(f"{port.device}\t{port.description}")
        return 0

    config = Config.load(args.config)
    if args.port:
        config.port = args.port
    if args.address is not None:
        config.address = args.address

    app = App(config, args.config)
    if config.port:
        try:
            app.bus.open()
        except (BusError, OSError) as exc:
            print(f"{YELLOW}could not open {config.port}: {exc}{RESET}")
    try:
        return app.run()
    except KeyboardInterrupt:
        if app.deadman.running:
            app.deadman.stop(send_stop=True)
        app.bus.close()
        return 130


if __name__ == "__main__":
    sys.exit(main())
