#!/usr/bin/env python3
"""
actuation_test.py — Jetson-side bench tool for the NUCLEO actuation board.

Speaks the register protocol from the technical proposal over I2C (slave 0x40):
  * status block read (0x00..0x19) with full decode
  * command frames   (reg 0x20): NOP/STANDBY/PUMP/ARM/ACTUATE/DISARM/...
  * heartbeat frames (reg 0x30): flight envelope @ 10 Hz (background thread)
  * corridor config  (reg 0x40)

The `sequence` subcommand runs the complete actuation test through the REAL
firmware interlocks (arming window, envelope permit, minimum holds) — nothing
is bypassed. ACTUATE requires typing FIRE (or --yes).

Usage examples (bus 7, adjust to yours):
  sudo python3 actuation_test.py --bus 7 status
  sudo python3 actuation_test.py --bus 7 monitor
  sudo python3 actuation_test.py --bus 7 cmd standby
  sudo python3 actuation_test.py --bus 7 config --alt-min 10 --alt-max 50 --spd-max 3
  sudo python3 actuation_test.py --bus 7 sequence --alt 35 --spd 1.2
  sudo python3 actuation_test.py --bus 7 shell --alt 35 --spd 1.2 \\
       --alt-min 10 --alt-max 50 --spd-max 3     # interactive, heartbeat kept alive
  python3 actuation_test.py selftest          # no hardware needed

Requires: pip3 install smbus2   (or apt install python3-smbus2)

SAFETY: `sequence` and `cmd fire` can drive the real hose-actuation output.
Bench-test with the nozzle depressurised or pointed somewhere safe.
Ctrl-C during `sequence` sends FORCE_FAILSAFE before exiting.
"""

import argparse
import sys
import threading
import time

# ---------------------------------------------------------------------------
# Protocol constants — mirror of App/Inc/proto.h (keep in sync!)
# ---------------------------------------------------------------------------
I2C_ADDR_DEFAULT = 0x40          # 7-bit address (on-wire write byte is 0x80)

REG_CMD        = 0x20
REG_HEARTBEAT  = 0x30
REG_CONFIG     = 0x40
STATUS_LEN     = 0x1A            # 26 bytes, registers 0x00..0x19

OPCODES = {
    "nop":        0x00,
    "standby":    0x10,
    "pump":       0x20,
    "arm":        0x30,
    "fire":       0x31,          # ACTUATE
    "disarm":     0x40,
    "clearfault": 0x50,
    "failsafe":   0xF0,
}

FSM_NAMES = ["BOOT", "STANDBY", "PUMPING", "READY",
             "ARMED", "ACTUATING", "FAILSAFE", "FAULT"]
TLM_NAMES = ["UNKNOWN", "CONN_VERIFY", "IDLE", "PUMP_RX", "READY", "ERROR"]
RES_NAMES = ["NONE", "ACK", "NACK", "ERR"]

PERMIT_BITS = ["HB_FRESH", "NAV_VALID", "ALT_OK", "SPD_OK",
               "MISSION", "ARMED", "TLM_READY", "ACTUATE_ALLOWED"]
FAULT_BITS = ["COMMS_TIMEOUT", "TLM_TIMEOUT", "POWER_ERROR", "CRC_ERROR",
              "SEQ_GAP", "ARM_TIMEOUT", "SENSOR_UNEXPECTED", "INTERNAL",
              "ENVELOPE_VIOLATION", "HB_STALE", "HB_CRC_ERROR", "CONFIG_MISSING",
              "PUMP_NOACK"]

HB_FLAG_NAV_VALID = 0x01
HB_FLAG_MISSION   = 0x02

ARM_BASE_MASK = 0x5F             # HB_FRESH|NAV_VALID|ALT_OK|SPD_OK|MISSION|TLM_READY
ACTUATE_KEY = 0x1825             # required ARG for ACTUATE (single-frame confirmation)
PERMIT_ACTUATE_ALLOWED = 0x80


# ---------------------------------------------------------------------------
# CRC-8/SMBus, poly 0x07, init 0x00 — verified against the firmware
# ---------------------------------------------------------------------------
def crc8(data):
    c = 0
    for b in data:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if (c & 0x80) else ((c << 1) & 0xFF)
    return c


def build_cmd(opcode, seq, arg=0):
    body = [opcode & 0xFF, seq & 0xFF, arg & 0xFF, (arg >> 8) & 0xFF]
    return [REG_CMD] + body + [crc8(body)]


def build_hb(hb_seq, alt_dm, spd_cms, flags):
    body = [hb_seq & 0xFF,
            alt_dm & 0xFF, (alt_dm >> 8) & 0xFF,
            spd_cms & 0xFF, (spd_cms >> 8) & 0xFF,
            flags & 0xFF]
    return [REG_HEARTBEAT] + body + [crc8(body)]


def build_cfg(alt_min_dm, alt_max_dm, spd_max_cms):
    body = [alt_min_dm & 0xFF, (alt_min_dm >> 8) & 0xFF,
            alt_max_dm & 0xFF, (alt_max_dm >> 8) & 0xFF,
            spd_max_cms & 0xFF, (spd_max_cms >> 8) & 0xFF]
    return [REG_CONFIG] + body + [crc8(body)]


def u16(lo, hi):
    return lo | (hi << 8)


def bits_str(value, names):
    on = [n for i, n in enumerate(names) if value & (1 << i)]
    return "|".join(on) if on else "-"


def parse_status(raw):
    return {
        "proto":     raw[0x00],
        "fw":        raw[0x01],
        "fsm":       raw[0x02],
        "tlm":       raw[0x03],
        "tlm_us":    u16(raw[0x04], raw[0x05]),
        "sensor":    raw[0x06],
        "result":    raw[0x07],
        "seq_echo":  raw[0x08],
        "arm_left":  raw[0x09] * 100,          # ms
        "ch1_us":    u16(raw[0x0A], raw[0x0B]),
        "ch2_us":    u16(raw[0x0C], raw[0x0D]),
        "fault":     u16(raw[0x0E], raw[0x0F]),
        "permit":    raw[0x10],
        "hb_age":    raw[0x11] * 10,           # ms
        "hb_seq":    raw[0x12],
        "alt_echo":  u16(raw[0x14], raw[0x15]) / 10.0,   # m
        "spd_echo":  u16(raw[0x16], raw[0x17]) / 100.0,  # m/s
        "btn_event": raw[0x18],
        "btn_seq":   raw[0x19],
    }


def fmt_status(st):
    fsm = FSM_NAMES[st["fsm"]] if st["fsm"] < len(FSM_NAMES) else str(st["fsm"])
    tlm = TLM_NAMES[st["tlm"]] if st["tlm"] < len(TLM_NAMES) else str(st["tlm"])
    res = RES_NAMES[st["result"]] if st["result"] < len(RES_NAMES) else str(st["result"])
    return (f"FSM={fsm} TLM={tlm}({st['tlm_us']}us) "
            f"CH1={st['ch1_us']}us CH2={st['ch2_us']}us "
            f"PERMIT=[{bits_str(st['permit'], PERMIT_BITS)}] "
            f"FAULT=[{bits_str(st['fault'], FAULT_BITS)}] "
            f"HBage={st['hb_age']}ms alt={st['alt_echo']:.1f}m spd={st['spd_echo']:.2f}m/s "
            f"last={res}(seq {st['seq_echo']}) armleft={st['arm_left']}ms "
            f"sensor={st['sensor']} btn={st['btn_event']}/{st['btn_seq']}")


# ---------------------------------------------------------------------------
# I2C link (thread-safe; heartbeat thread and main share one bus handle)
# ---------------------------------------------------------------------------
class Link:
    def __init__(self, bus_num, addr):
        from smbus2 import SMBus, i2c_msg   # lazy: selftest runs without smbus2
        self._i2c_msg = i2c_msg
        self._bus = SMBus(bus_num)
        self._addr = addr
        self._lock = threading.Lock()
        self._seq = 0

    def close(self):
        try:
            self._bus.close()
        except Exception:
            pass

    def _rdwr(self, *msgs):
        last = None
        for _ in range(3):                       # Tegra I2C occasionally EAGAINs
            try:
                with self._lock:
                    self._bus.i2c_rdwr(*msgs)
                return
            except OSError as e:
                last = e
                time.sleep(0.01)
        raise last

    def write_frame(self, payload):
        self._rdwr(self._i2c_msg.write(self._addr, payload))

    def read_status(self):
        wr = self._i2c_msg.write(self._addr, [0x00])
        rd = self._i2c_msg.read(self._addr, STATUS_LEN)
        self._rdwr(wr, rd)                       # repeated start between msgs
        return parse_status(list(rd))

    def next_seq(self):
        self._seq = (self._seq + 1) & 0xFF
        return self._seq

    def command(self, name, arg=0, wait_ack=True, timeout=0.5):
        """Send a command frame; poll SEQ_ECHO/LAST_RESULT for the ack."""
        seq = self.next_seq()
        self.write_frame(build_cmd(OPCODES[name], seq, arg))
        if not wait_ack:
            return None, seq
        deadline = time.time() + timeout
        while time.time() < deadline:
            st = self.read_status()
            if st["seq_echo"] == seq:
                return st, seq
            time.sleep(0.02)
        return None, seq


class Heartbeat(threading.Thread):
    """10 Hz flight-envelope heartbeat: keeps the interlock permit fresh.
    alt_dm/spd_cms/flags may be changed live; `paused` suspends sending
    (useful to provoke HB_STALE / auto-disarm on the bench)."""

    def __init__(self, link, alt_m, spd_ms, flags=HB_FLAG_NAV_VALID | HB_FLAG_MISSION):
        super().__init__(daemon=True)
        self.link = link
        self.alt_dm = int(round(alt_m * 10))
        self.spd_cms = int(round(spd_ms * 100))
        self.flags = flags
        self.paused = False
        self._stop_evt = threading.Event()
        self._seq = 0

    def run(self):
        while not self._stop_evt.is_set():
            if not self.paused:
                self._seq = (self._seq + 1) & 0xFF
                try:
                    self.link.write_frame(build_hb(self._seq, self.alt_dm,
                                                   self.spd_cms, self.flags))
                except OSError as e:
                    print(f"[hb] write failed: {e}", file=sys.stderr)
            self._stop_evt.wait(0.1)

    def stop(self):
        self._stop_evt.set()
        self.join(timeout=1.0)


# ---------------------------------------------------------------------------
# Helpers for the guided sequence
# ---------------------------------------------------------------------------
def wait_for(link, cond, timeout, what):
    """Poll status until cond(st) or timeout; print state transitions."""
    t0 = time.time()
    last_fsm = None
    while time.time() - t0 < timeout:
        st = link.read_status()
        if st["fsm"] != last_fsm:
            print(f"      FSM -> {FSM_NAMES[st['fsm']]}")
            last_fsm = st["fsm"]
        if cond(st):
            return st
        time.sleep(0.05)
    st = link.read_status()
    print(f"  TIMEOUT waiting for {what}")
    print(f"  {fmt_status(st)}")
    return None


def explain_arm_blockers(st):
    missing = [n for i, n in enumerate(PERMIT_BITS[:7])
               if (ARM_BASE_MASK & (1 << i)) and not (st["permit"] & (1 << i))]
    hints = {
        "HB_FRESH":  "heartbeat older than 300 ms — is the HB thread running?",
        "NAV_VALID": "HB_FLAGS bit0 not set in the heartbeat",
        "ALT_OK":    "altitude outside corridor — or corridor not set (CONFIG_MISSING)",
        "SPD_OK":    "speed above maximum — or corridor not set (CONFIG_MISSING)",
        "MISSION":   "HB_FLAGS bit1 (mission permit) not set",
        "TLM_READY": "pump stage not completed (FSM must be READY via PUMP) "
                     "or telemetry dead/ERROR",
    }
    for m in missing:
        print(f"      blocked by {m}: {hints.get(m, '')}")


def cmd_ack_line(st, seq):
    if st is None:
        return "no ack (seq echo never updated)"
    res = RES_NAMES[st["result"]] if st["result"] < len(RES_NAMES) else "?"
    return f"{res} (seq {seq})"


# ---------------------------------------------------------------------------
# Subcommands
# ---------------------------------------------------------------------------
def do_status(link, _args):
    print(fmt_status(link.read_status()))


def do_monitor(link, _args):
    print("Ctrl-C to stop.")
    try:
        while True:
            print(fmt_status(link.read_status()))
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass


def do_cmd(link, args):
    if args.name == "fire" and not args.yes:
        if input("Type FIRE to confirm actuation: ").strip() != "FIRE":
            print("aborted")
            return
    arg = ACTUATE_KEY if args.name == "fire" else 0
    st, seq = link.command(args.name, arg=arg)
    print(f"{args.name}: {cmd_ack_line(st, seq)}")
    if st:
        print(fmt_status(st))


def do_config(link, args):
    link.write_frame(build_cfg(int(round(args.alt_min * 10)),
                               int(round(args.alt_max * 10)),
                               int(round(args.spd_max * 100))))
    time.sleep(0.05)
    st = link.read_status()
    ok = "cleared" if not (st["fault"] & (1 << 11)) else "STILL SET (CRC?)"
    print(f"corridor set: alt [{args.alt_min}..{args.alt_max}] m, "
          f"spd <= {args.spd_max} m/s; CONFIG_MISSING {ok}")


def do_hb(link, args):
    flags = (HB_FLAG_NAV_VALID if args.nav else 0) | (HB_FLAG_MISSION if args.mission else 0)
    link.write_frame(build_hb(1, int(round(args.alt * 10)),
                              int(round(args.spd * 100)), flags))
    time.sleep(0.05)
    print(fmt_status(link.read_status()))


def do_sequence(link, args):
    hb = None
    try:
        print("=== ACTUATION SEQUENCE TEST (through real interlocks) ===")

        st0 = link.read_status()
        if st0["fsm"] == 7:                                   # FAULT
            print("[0/6] board is in FAULT — attempting CLEAR_FAULT")
            st, seq = link.command("clearfault")
            print(f"      {cmd_ack_line(st, seq)}")
            if st is None or st["result"] != 1:
                cur = link.read_status()
                print(f"      FAULT=[{bits_str(cur['fault'], FAULT_BITS)}] "
                      f"TLM={TLM_NAMES[cur['tlm']]}({cur['tlm_us']}us)")
                if cur["tlm"] == 5:                           # TLM_ERROR
                    print("      The actuation circuit itself reports ERROR (~2100 us) on Ch.4.")
                    print("      The firmware cannot clear this while the source persists. Check:")
                    print("        - 5 V supply to the circuit: it draws up to 500 mA pulses;")
                    print("          a USB-derived 5 V rail often sags. Use a solid supply.")
                    print("        - common GND between Nucleo, circuit, and supply")
                    print("        - PWM level: Nucleo drives 3.3 V; the spec window is 3-5 V,")
                    print("          so 3.3 V sits at the edge -> a 5 V level shifter may be needed")
                    print("        - then power-cycle the CIRCUIT (board running) and re-run")
                return 1
            print("      fault cleared")

        print(f"[1/6] heartbeat @10 Hz: alt={args.alt} m spd={args.spd} m/s NAV|MISSION")
        hb = Heartbeat(link, args.alt, args.spd)
        hb.start()
        time.sleep(0.3)

        print(f"[2/6] corridor: alt [{args.alt_min}..{args.alt_max}] m, spd <= {args.spd_max} m/s")
        link.write_frame(build_cfg(int(round(args.alt_min * 10)),
                                   int(round(args.alt_max * 10)),
                                   int(round(args.spd_max * 100))))

        print("[3/6] SET_STANDBY")
        st, seq = link.command("standby")
        print(f"      {cmd_ack_line(st, seq)}")
        st = wait_for(link, lambda s: s["fsm"] == 1, 2.0, "STANDBY")
        if st is None:
            return 1

        print("[4/6] PUMP (firmware holds Ch.1 = 1650 us until telemetry confirms; "
              "then AUTO-ARMS)")
        st, seq = link.command("pump")
        print(f"      {cmd_ack_line(st, seq)}")
        st = wait_for(link, lambda s: s["fsm"] == 4, args.pump_timeout, "ARMED (auto)")
        if st is None:
            s = link.read_status()
            if s["fault"] & (1 << 12):                      # PUMP_NOACK
                print("      firmware held Ch.1 = 1650 us for the full window, but the")
                print("      circuit's telemetry never confirmed (no PUMP_RX/READY).")
                print("      -> the circuit did not register the pump command: check Ch.1")
                print("         wiring and level margin (3.3 V into a 3-5 V input window),")
                print("         then 'cmd clearfault' and retry.")
            elif s["fsm"] == 2:
                print("      still PUMPING: telemetry has not reported READY (1400 us) yet.")
            return 1

        print("[5/6] permit check (firing gated by live ACTUATE_ALLOWED)")
        st = link.read_status()
        print(f"      PERMIT=[{bits_str(st['permit'], PERMIT_BITS)}]")
        if not (st["permit"] & PERMIT_ACTUATE_ALLOWED):
            explain_arm_blockers(st)
            return 1

        if not args.yes:
            print("      board is ARMED and will stay armed; a single ACTUATE fires.")
            if input("      Type FIRE to actuate: ").strip() != "FIRE":
                print("      aborted; sending SET_STANDBY (disarms)")
                link.command("standby")
                return 1

        print("[6/6] ACTUATE (Ch.2 = 1825 us, held >= 200 ms by firmware)")
        st, seq = link.command("fire", arg=ACTUATE_KEY)
        print(f"      {cmd_ack_line(st, seq)}")
        st = wait_for(link, lambda s: s["fsm"] in (5, 1, 6), 2.0, "ACTUATING/return")
        st = wait_for(link, lambda s: s["fsm"] in (1, 6), 2.0, "return to STANDBY")
        final = link.read_status()
        print("=== RESULT ===")
        print(fmt_status(final))
        print(f"sensor (Ch.3 ADS) triggered: {'YES' if final['sensor'] else 'no'}")
        if final["fault"]:
            print(f"faults set: {bits_str(final['fault'], FAULT_BITS)}")
        return 0

    except KeyboardInterrupt:
        print("\nABORT: sending FORCE_FAILSAFE")
        try:
            link.command("failsafe", wait_ack=False)
        except OSError:
            pass
        return 130
    finally:
        if hb:
            hb.stop()



SHELL_HELP = """commands:
  status                 one decoded status line
  standby|pump|fire|disarm|clearfault|failsafe|nop
                         send a command frame (fire asks for confirmation;
                         pump AUTO-ARMS on telemetry confirm — no arm step)
  arm                    probe only: ACK if already armed (auto-arm model)
  alt <m>  spd <m/s>     change the simulated envelope live
  flags <hex>            heartbeat flags (0x03 = NAV|MISSION)
  hb on|off              resume/pause the heartbeat (off provokes HB_STALE)
  config <amin> <amax> <smax>   set corridor (m, m, m/s)
  help                   this text
  quit                   exit (board enters FAILSAFE ~300 ms later, by design)
"""


def do_shell(link, args):
    """Interactive bench console with a persistent 10 Hz heartbeat."""
    hb = Heartbeat(link, args.alt, args.spd)
    hb.start()
    print(f"heartbeat running @10 Hz: alt={args.alt} m spd={args.spd} m/s NAV|MISSION")
    if args.alt_min is not None:
        link.write_frame(build_cfg(int(round(args.alt_min * 10)),
                                   int(round(args.alt_max * 10)),
                                   int(round(args.spd_max * 100))))
        print(f"corridor set: alt [{args.alt_min}..{args.alt_max}] m, "
              f"spd <= {args.spd_max} m/s")
    try:
        print(fmt_status(link.read_status()))
    except OSError as e:
        print(f"i2c error on first status read: {e}")
    print("type 'help' for commands, 'quit' to exit")
    try:
        while True:
            try:
                line = input("act> ").strip()
            except EOFError:
                break
            if not line:
                continue
            t = line.split()
            c = t[0].lower()
            try:
                if c in ("quit", "exit"):
                    break
                elif c == "help":
                    print(SHELL_HELP)
                elif c == "status":
                    print(fmt_status(link.read_status()))
                elif c in OPCODES:
                    if c == "fire":
                        if input("type FIRE to confirm: ").strip() != "FIRE":
                            print("aborted")
                            continue
                    st, seq = link.command(c, arg=(ACTUATE_KEY if c == "fire" else 0))
                    print(f"{c}: {cmd_ack_line(st, seq)}")
                elif c == "alt" and len(t) == 2:
                    hb.alt_dm = int(round(float(t[1]) * 10))
                    print(f"hb altitude -> {float(t[1])} m")
                elif c == "spd" and len(t) == 2:
                    hb.spd_cms = int(round(float(t[1]) * 100))
                    print(f"hb speed -> {float(t[1])} m/s")
                elif c == "flags" and len(t) == 2:
                    hb.flags = int(t[1], 0) & 0xFF
                    print(f"hb flags -> 0x{hb.flags:02X}")
                elif c == "hb" and len(t) == 2 and t[1] in ("on", "off"):
                    hb.paused = (t[1] == "off")
                    print(f"heartbeat {'paused' if hb.paused else 'resumed'}")
                elif c == "config" and len(t) == 4:
                    link.write_frame(build_cfg(int(round(float(t[1]) * 10)),
                                               int(round(float(t[2]) * 10)),
                                               int(round(float(t[3]) * 100))))
                    print("corridor written")
                else:
                    print("unknown command; type 'help'")
            except (ValueError, IndexError):
                print("bad arguments; type 'help'")
            except OSError as e:
                print(f"i2c error: {e}")
        print("exiting; heartbeat stops -> board enters FAILSAFE in ~300 ms (by design)")
        return 0
    except KeyboardInterrupt:
        print("\nABORT: sending FORCE_FAILSAFE")
        try:
            link.command("failsafe", wait_ack=False)
        except OSError:
            pass
        return 130
    finally:
        hb.stop()


def do_selftest(_link, _args):
    assert crc8([0x20, 0x07, 0x00, 0x00]) == 0xD8
    assert crc8([0x2A, 0x5E, 0x01, 0x78, 0x00, 0x03]) == 0xA7
    assert build_cmd(0x20, 7) == [0x20, 0x20, 0x07, 0x00, 0x00, 0xD8]
    assert build_hb(0x2A, 350, 120, 0x03) == \
        [0x30, 0x2A, 0x5E, 0x01, 0x78, 0x00, 0x03, 0xA7]
    cfg = build_cfg(100, 500, 300)
    assert cfg[0] == 0x40 and len(cfg) == 8 and cfg[-1] == crc8(cfg[1:-1])
    raw = list(range(STATUS_LEN))
    st = parse_status(raw)
    assert st["tlm_us"] == u16(4, 5) and st["fault"] == u16(0x0E, 0x0F)
    print("selftest OK (CRC vectors + frame builders + status parser)")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bus", type=int, default=7, help="I2C bus number (default 7)")
    ap.add_argument("--addr", type=lambda x: int(x, 0), default=I2C_ADDR_DEFAULT,
                    help="7-bit slave address (default 0x40; NOT 0x80)")
    sub = ap.add_subparsers(dest="sub", required=True)

    sub.add_parser("status", help="one decoded status read")
    sub.add_parser("monitor", help="poll status at 2 Hz")
    sub.add_parser("selftest", help="offline CRC/frame checks (no hardware)")

    p = sub.add_parser("cmd", help="send one command frame")
    p.add_argument("name", choices=sorted(OPCODES.keys()))
    p.add_argument("--yes", action="store_true", help="skip FIRE confirmation")

    p = sub.add_parser("config", help="set the flight-envelope corridor")
    p.add_argument("--alt-min", type=float, required=True, help="m")
    p.add_argument("--alt-max", type=float, required=True, help="m")
    p.add_argument("--spd-max", type=float, required=True, help="m/s")

    p = sub.add_parser("hb", help="send a single heartbeat frame")
    p.add_argument("--alt", type=float, required=True, help="m")
    p.add_argument("--spd", type=float, required=True, help="m/s")
    p.add_argument("--no-nav", dest="nav", action="store_false")
    p.add_argument("--no-mission", dest="mission", action="store_false")

    p = sub.add_parser("shell", help="interactive console with persistent heartbeat")
    p.add_argument("--alt", type=float, default=35.0, help="heartbeat altitude, m")
    p.add_argument("--spd", type=float, default=1.2, help="heartbeat speed, m/s")
    p.add_argument("--alt-min", type=float, default=None,
                   help="also set corridor on entry (needs all three)")
    p.add_argument("--alt-max", type=float, default=None)
    p.add_argument("--spd-max", type=float, default=None)

    p = sub.add_parser("sequence", help="full guided actuation test")
    p.add_argument("--alt", type=float, default=35.0, help="heartbeat altitude, m")
    p.add_argument("--spd", type=float, default=1.2, help="heartbeat speed, m/s")
    p.add_argument("--alt-min", type=float, default=10.0)
    p.add_argument("--alt-max", type=float, default=50.0)
    p.add_argument("--spd-max", type=float, default=3.0)
    p.add_argument("--pump-timeout", type=float, default=15.0,
                   help="s to wait for telemetry READY after PUMP")
    p.add_argument("--yes", action="store_true", help="skip FIRE confirmation")

    args = ap.parse_args()

    if args.sub == "selftest":
        return do_selftest(None, args) or 0

    link = Link(args.bus, args.addr)
    try:
        handler = {"status": do_status, "monitor": do_monitor, "cmd": do_cmd,
                   "config": do_config, "hb": do_hb, "sequence": do_sequence,
                   "shell": do_shell}[args.sub]
        return handler(link, args) or 0
    finally:
        link.close()


if __name__ == "__main__":
    sys.exit(main())
