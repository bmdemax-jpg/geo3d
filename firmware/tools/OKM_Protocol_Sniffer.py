#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OKM_Protocol_Sniffer -- transparent man-in-the-middle logger between a real
OKM CH05 device (paired over classic Bluetooth, with its own real COM port)
and OKM Visualizer 3D Studio, to reverse-engineer OKM's undocumented serial
protocol.

How it works: Visualizer 3D connects to a *virtual* COM port (one end of a
com0com pair) instead of the real device. This script forwards every byte
between that virtual port and the real device's COM port in both directions,
completely transparently, while logging a timestamped hex+ASCII dump of
everything that passes -- so Visualizer 3D behaves exactly as if it were
talking to the device directly, and we get a full capture to analyze.

Setup:
  1. Pair the real CH05 device in Windows Bluetooth settings as usual. Open
     Bluetooth settings -> the device -> "Ports COM" tab to find its real
     COM port number (e.g. COM6).
  2. Use com0com's Setup tool to create (or reuse) a virtual port pair, e.g.
     COM20 <-> COM21.
  3. Run this script, e.g.:
         python OKM_Protocol_Sniffer.py --real COM6 --bridge COM20
     (or just run it and answer the prompts)
  4. Point Visualizer 3D Studio at the *other* virtual port (COM21 in this
     example) -- NOT at COM6 directly.
  5. Use Visualizer 3D as usual: connect, press PULSE, CAL, change modes,
     etc. While it's running, type a short note + Enter in this console at
     any moment (e.g. "pressed PULSE") to stamp that action into the log so
     it's easy to correlate later.
  6. Stop with Ctrl+C. A full log is written to okm_capture_<timestamp>.log
     next to this script -- share that file (or paste snippets from it) to
     work out the exact packet format.
"""

import argparse
import subprocess
import sys
import threading
import time
from datetime import datetime


def ensure_dependency(module_name: str, pip_name: str) -> None:
    try:
        __import__(module_name)
    except ImportError:
        print(f"Installing required package '{pip_name}' (first run only)...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "--quiet", pip_name])


ensure_dependency("serial", "pyserial>=3.5")

import serial  # noqa: E402


def hexdump(data: bytes) -> str:
    hex_part = " ".join(f"{b:02X}" for b in data)
    ascii_part = "".join(chr(b) if 32 <= b < 127 else "." for b in data)
    return f"{hex_part}   |{ascii_part}|"


class LogFile:
    def __init__(self, path: str):
        self.path = path
        self.lock = threading.Lock()
        self.f = open(path, "a", encoding="utf-8")

    def write(self, line: str) -> None:
        with self.lock:
            self.f.write(line + "\n")
            self.f.flush()
        print(line)


def pump(src: serial.Serial, dst: serial.Serial, label: str, log: LogFile, stop_evt: threading.Event):
    while not stop_evt.is_set():
        try:
            data = src.read(256)
        except serial.SerialException as exc:
            log.write(f"[{ts()}] [!] {label}: {exc}")
            stop_evt.set()
            return
        if data:
            dst.write(data)
            log.write(f"[{ts()}] {label} ({len(data)}B): {hexdump(data)}")


def ts() -> str:
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


def notes_input(log: LogFile, stop_evt: threading.Event):
    print("Type a short note + Enter any time to stamp it into the log (e.g. 'pressed PULSE'). Ctrl+C to stop.")
    while not stop_evt.is_set():
        try:
            line = input()
        except EOFError:
            return
        if line.strip():
            log.write(f"[{ts()}] [NOTE] {line.strip()}")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--real", default=None, help="Real device COM port (e.g. COM6)")
    p.add_argument("--bridge", default=None, help="Bridge (virtual) COM port, e.g. COM20")
    p.add_argument("--baud", type=int, default=115200)
    args = p.parse_args()

    real_port = args.real or input("Real device COM port (e.g. COM6): ").strip()
    bridge_port = args.bridge or input("Bridge (virtual) COM port used by this script, e.g. COM20: ").strip()
    baud = args.baud
    log_path = f"okm_capture_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
    log = LogFile(log_path)
    log.write(f"[{ts()}] Sniffer starting. real={real_port} bridge={bridge_port} baud={baud}")
    log.write(f"[{ts()}] Point Visualizer 3D Studio at the OTHER end of the {bridge_port} com0com pair.")

    try:
        real = serial.Serial(real_port, baudrate=baud, timeout=0.2)
        bridge = serial.Serial(bridge_port, baudrate=baud, timeout=0.2)
    except Exception as exc:  # noqa: BLE001
        log.write(f"[{ts()}] [!] Failed to open ports: {exc}")
        input("Press Enter to close...")
        return

    stop_evt = threading.Event()
    t1 = threading.Thread(target=pump, args=(real, bridge, "DEVICE->APP", log, stop_evt), daemon=True)
    t2 = threading.Thread(target=pump, args=(bridge, real, "APP->DEVICE", log, stop_evt), daemon=True)
    t1.start()
    t2.start()

    try:
        notes_input(log, stop_evt)
    except KeyboardInterrupt:
        pass
    finally:
        stop_evt.set()
        time.sleep(0.3)
        for p in (real, bridge):
            try:
                p.close()
            except Exception:  # noqa: BLE001
                pass
        log.write(f"[{ts()}] Sniffer stopped. Log saved to {log_path}")


if __name__ == "__main__":
    main()
