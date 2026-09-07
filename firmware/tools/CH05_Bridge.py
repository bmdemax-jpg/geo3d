#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CH05_Bridge -- single-file BLE <-> virtual COM port bridge for the M5Stack
"CH05" device (GeoScan3D over BLE UART).

No manual dependency install needed: this file checks for bleak and
pyserial and installs them automatically on first run if missing.

Easiest usage (one click):
    Run Run_CH05_Bridge.bat next to this file -- it will ask for the COM
    port and start right away.

Command-line usage (optional):
    python CH05_Bridge.py --com COM11
    python CH05_Bridge.py --com COM11 --name CH05 --address AA:BB:CC:DD:EE:FF

Background: the ESP32-S3 inside M5Stack CoreS3 has no classic Bluetooth
(SPP), so the "CH05" BLE device can't open a real COM port on its own.
This script connects to the device over BLE UART (Nordic UART Service)
and forwards raw bytes between BLE and one end of a virtual COM port pair
created by com0com -- so the other end of the pair behaves exactly like a
real serial port to any program, such as Visualizer 3D.

Console output is kept in plain English on purpose: many Windows console
hosts (cmd.exe / legacy conhost) do not render right-to-left Arabic text
correctly, showing it reversed/garbled even with a UTF-8 code page. See
README.md for Arabic documentation.
"""

import argparse
import asyncio
import subprocess
import sys
from typing import Optional


def ensure_dependency(module_name: str, pip_name: str) -> None:
    try:
        __import__(module_name)
    except ImportError:
        print(f"[*] Installing required package '{pip_name}' (first run only)... please wait")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "--quiet", pip_name])


ensure_dependency("bleak", "bleak>=0.21")
ensure_dependency("serial", "pyserial>=3.5")

from bleak import BleakClient, BleakScanner  # noqa: E402
import serial  # noqa: E402

# Same UUIDs defined in firmware/GeoScan3D_v9/src/main.cpp (BLE_SERVICE_UUID/TX/RX)
NUS_TX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # notify: device -> computer
NUS_RX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # write: computer -> device
WRITE_CHUNK = 20  # safe regardless of negotiated MTU (default ATT MTU = 23 bytes, 20 payload)


async def find_device(name: str, timeout: float):
    print(f"[*] Scanning for BLE device named '{name}' (up to {timeout:.0f}s)...")
    device = await BleakScanner.find_device_by_filter(
        lambda d, adv: d.name == name, timeout=timeout
    )
    if device is None:
        raise RuntimeError(f"No BLE device named '{name}' found -- make sure it is powered on and nearby")
    return device


async def run_bridge(com_port: str, device_name: str, address: Optional[str],
                      baud: int, scan_timeout: float) -> None:
    target = address or await find_device(device_name, scan_timeout)
    ser = serial.Serial(com_port, baudrate=baud, timeout=0)
    print(f"[+] Opened COM port: {com_port}")

    async with BleakClient(target) as client:
        print(f"[+] Connected to BLE: {client.address}")

        def on_notify(_, data: bytearray) -> None:
            ser.write(bytes(data))

        await client.start_notify(NUS_TX_UUID, on_notify)
        print("[+] Bridge is running -- point Visualizer 3D at the other end of the com0com pair")
        print("[+] Leave this window open; press Ctrl+C to stop")

        loop = asyncio.get_event_loop()
        try:
            while True:
                data = await loop.run_in_executor(None, ser.read, 256)
                for i in range(0, len(data), WRITE_CHUNK):
                    await client.write_gatt_char(
                        NUS_RX_UUID, data[i:i + WRITE_CHUNK], response=False
                    )
                await asyncio.sleep(0.01)
        finally:
            await client.stop_notify(NUS_TX_UUID)
            ser.close()


def prompt_missing_args(args: argparse.Namespace) -> None:
    """Interactive mode: used when double-clicked with no command-line args."""
    if not args.com:
        args.com = input("Enter the COM port for this bridge (example: COM11): ").strip()
    if not args.name:
        name = input("BLE device name (leave blank to use default CH05): ").strip()
        args.name = name or "CH05"


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--com", default=None,
                   help="COM port this bridge will use (give the other end of the pair to Visualizer 3D)")
    p.add_argument("--name", default=None, help="Advertised BLE device name (default: CH05)")
    p.add_argument("--address", default=None,
                   help="Fixed MAC address of the device (optional, skips scanning by name)")
    p.add_argument("--baud", type=int, default=115200,
                   help="Baud rate to open the local COM port with (irrelevant over BLE, only for pyserial)")
    p.add_argument("--scan-timeout", type=float, default=10.0,
                   help="Device scan timeout in seconds")
    args = p.parse_args()

    prompt_missing_args(args)

    try:
        asyncio.run(run_bridge(args.com, args.name, args.address, args.baud, args.scan_timeout))
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"[!] Error: {exc}")
        input("Press Enter to close...")


if __name__ == "__main__":
    main()
