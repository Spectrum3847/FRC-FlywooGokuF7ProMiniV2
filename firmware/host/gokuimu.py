#!/usr/bin/env python3
"""GokuIMU desktop test tool (needs pyserial).

  python gokuimu.py [--port COM6] monitor        live state
  python gokuimu.py info | config | dfu | reboot | cal | clear | yaw <deg>
  python gokuimu.py setcfg mount_yaw=90 trim_z=1.5 rate=200 ...
  python gokuimu.py bench <seconds>              stats over a window (keep still)
"""
import math
import struct
import sys
import time

import serial
import serial.tools.list_ports

STATE_FMT = "<II4f3f3f3f3ffHHHH3fI"
INFO_FMT = "<BBBB3IB3x24s"
CFG_FMT = "<6fIHH"
CFG_KEYS = ["mount_yaw", "mount_pitch", "mount_roll", "trim_x", "trim_y", "trim_z", "features", "rate"]
FLAGS = ["HW_FAULT", "BIAS_VALID", "MOVING", "GYRO_SAT", "ACCEL_SAT", "HSE_FAIL",
         "SAMPLE_MISSED", "BOOTING", "CFG_DEFAULT"]


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def frame(t, payload=b""):
    body = bytes([t, len(payload)]) + payload
    return b"\xA5\x5A" + body + struct.pack("<H", crc16(body))


class Dev:
    def __init__(self, port=None):
        if port is None:
            for p in serial.tools.list_ports.comports():
                if p.vid == 0x1209 and p.pid == 0x0001 or (p.product or "").startswith("GokuIMU"):
                    port = p.device
        if port is None:
            sys.exit("GokuIMU not found")
        self.s = serial.Serial(port, 115200, timeout=0.05)
        self.buf = bytearray()

    seq = 0

    def send(self, t, payload=b""):
        # Protocol v2: every command starts with a sequence byte echoed in the ACK.
        Dev.seq = (Dev.seq + 1) & 0xFF
        self.s.write(frame(t, bytes([Dev.seq]) + payload))

    def frames(self):
        self.buf += self.s.read(4096)
        while True:
            i = self.buf.find(b"\xA5\x5A")
            if i < 0:
                del self.buf[:-1]
                return
            del self.buf[:i]
            if len(self.buf) < 6:
                return
            n = self.buf[3]
            if len(self.buf) < n + 6:
                return
            body = bytes(self.buf[2:4 + n])
            (crc,) = struct.unpack_from("<H", self.buf, 4 + n)
            if crc != crc16(body):
                del self.buf[:2]
                continue
            del self.buf[:n + 6]
            yield body[0], body[2:]

    def wait(self, t, timeout=2.0):
        end = time.time() + timeout
        while time.time() < end:
            for ft, p in self.frames():
                if ft == t:
                    return p
        return None


def parse_state(p):
    v = struct.unpack(STATE_FMT, p)
    return dict(ts=v[0], seq=v[1], quat=v[2:6], yaw=v[6], pitch=v[7], roll=v[8],
                gyro=v[9:12], accel=v[12:15], accum=v[15:18], temp=v[18], nmc=v[19],
                flags=v[20], sticky=v[21], rate=v[22], bias=v[23:26], uptime=v[26])


def flagstr(f):
    return ",".join(n for i, n in enumerate(FLAGS) if f & (1 << i)) or "-"


def main():
    args = sys.argv[1:]
    port = None
    if "--port" in args:
        i = args.index("--port")
        port = args[i + 1]
        del args[i:i + 2]
    cmd = args[0] if args else "monitor"
    d = Dev(port)
    if cmd == "monitor":
        while True:
            for t, p in d.frames():
                if t == 1:
                    s = parse_state(p)
                    print(f"\ryaw {s['yaw']:9.3f} pitch {s['pitch']:7.2f} roll {s['roll']:7.2f} "
                          f"g [{s['gyro'][0]:7.2f} {s['gyro'][1]:7.2f} {s['gyro'][2]:7.2f}] "
                          f"a [{s['accel'][0]:5.2f} {s['accel'][1]:5.2f} {s['accel'][2]:5.2f}] "
                          f"T {s['temp']:4.1f} nm {s['nmc']} {s['rate']}Hz {flagstr(s['flags'])}   ",
                          end="", flush=True)
    elif cmd == "info":
        d.send(0x11)
        p = d.wait(2)
        v = struct.unpack(INFO_FMT, p)
        print(f"proto {v[0]} fw {v[1]}.{v[2]}.{v[3]} uid {v[4]:08X}{v[5]:08X}{v[6]:08X} "
              f"whoami 0x{v[7]:02X} board {v[8].rstrip(bytes(1)).decode()}")
    elif cmd == "config":
        d.send(0x12)
        print(dict(zip(CFG_KEYS, struct.unpack(CFG_FMT, d.wait(3))[:8])))
    elif cmd == "setcfg":
        d.send(0x12)
        cur = dict(zip(CFG_KEYS, struct.unpack(CFG_FMT, d.wait(3))[:8]))
        for kv in args[1:]:
            k, v = kv.split("=")
            cur[k] = int(v) if k in ("features", "rate") else float(v)
        d.send(0x13, struct.pack(CFG_FMT, *[cur[k] for k in CFG_KEYS], 0))
        print("ack", d.wait(4, 5.0).hex(), dict(zip(CFG_KEYS, struct.unpack(CFG_FMT, d.wait(3))[:8])))
    elif cmd == "yaw":
        d.send(0x10, struct.pack("<f", float(args[1])))
        print("ack", d.wait(4).hex())
    elif cmd in ("cal", "reboot", "dfu", "clear", "factory"):
        d.send({"cal": 0x15, "reboot": 0x16, "dfu": 0x17, "clear": 0x18, "factory": 0x19}[cmd])
        print("ack", (d.wait(4) or b"").hex())
    elif cmd == "bench":
        secs = float(args[1]) if len(args) > 1 else 10
        first = last = None
        n = 0
        end = time.time() + secs
        while time.time() < end:
            for t, p in d.frames():
                if t == 1:
                    s = parse_state(p)
                    first = first or s
                    last = s
                    n += 1
        dt = (last["ts"] - first["ts"]) / 1e6
        print(f"{n} frames in {secs}s ({n / secs:.1f}/s), seq gap {last['seq'] - first['seq'] + 1 - n}")
        print(f"yaw drift {(last['yaw'] - first['yaw']) / dt * 60:.4f} deg/min, "
              f"pitch {last['pitch']:.2f} roll {last['roll']:.2f} |a| {math.dist(last['accel'], (0, 0, 0)):.4f} g")
        print(f"bias {last['bias']}, temp {last['temp']:.1f}, nmc {last['nmc']}, rate {last['rate']}, "
              f"flags {flagstr(last['flags'])} sticky {flagstr(last['sticky'])}")


if __name__ == "__main__":
    main()
