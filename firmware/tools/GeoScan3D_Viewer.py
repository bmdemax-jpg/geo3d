#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GeoScan3D_Viewer -- open-source desktop viewer for the M5Stack "CH05"
(GeoScan3D) BLE device. Talks directly to the device's own protocol over
BLE (no COM port, no bridge, no third-party software) -- so it never runs
into OKM Visualizer 3D's undocumented proprietary protocol at all.

Single file, no manual dependency install: on first run it installs
`bleak` automatically if missing (tkinter ships with standard Python on
Windows, so nothing else is required).

Usage: double-click Run_GeoScan3D_Viewer.bat, or:
    python GeoScan3D_Viewer.py
"""

import asyncio
import queue
import struct
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk


def ensure_dependency(module_name: str, pip_name: str) -> None:
    try:
        __import__(module_name)
    except ImportError:
        print(f"Installing required package '{pip_name}' (first run only)...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "--quiet", pip_name])


ensure_dependency("bleak", "bleak>=0.21")

from bleak import BleakClient, BleakScanner  # noqa: E402

# Same UUIDs and packet layout as firmware/GeoScan3D_v9/src/main.cpp
NUS_TX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # notify: device -> host
NUS_RX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # write: host -> device

# struct BlePacket (packed): header0,header1,uint32 ts,float ut1,ut2,grad,gradKf,depthCm,
#                            uint8 detect,mode,float yaw,uint16 crc16  (34 bytes, v9.1+)
FMT_34 = "<BBIfffffBBfH"
FMT_30 = "<BBIfffffBBH"  # legacy packet without yaw (v9.0)


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def parse_packet(data: bytes):
    for fmt in (FMT_34, FMT_30):
        size = struct.calcsize(fmt)
        if len(data) != size:
            continue
        fields = struct.unpack(fmt, data)
        crc_received = fields[-1]
        if crc16_ccitt(data[:-2]) != crc_received:
            continue
        if fmt == FMT_34:
            _, _, ts, ut1, ut2, grad, gradKf, depthCm, detect, mode, yaw, _ = fields
        else:
            _, _, ts, ut1, ut2, grad, gradKf, depthCm, detect, mode, _ = fields
            yaw = None
        return {
            "ts": ts, "ut1": ut1, "ut2": ut2, "grad": grad, "gradKf": gradKf,
            "depthCm": depthCm, "detect": detect, "mode": mode, "yaw": yaw,
        }
    return None


class BleWorker:
    """Runs bleak on a background asyncio loop; talks to the GUI via queues."""

    def __init__(self, events: "queue.Queue"):
        self.events = events
        self.commands: "queue.Queue" = queue.Queue()
        self.loop = None
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.client = None
        self._stop = threading.Event()

    def start(self):
        self.thread.start()

    def _run_loop(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def connect(self, name: str, address: str, scan_timeout: float):
        asyncio.run_coroutine_threadsafe(self._connect(name, address, scan_timeout), self.loop)

    def disconnect(self):
        asyncio.run_coroutine_threadsafe(self._disconnect(), self.loop)

    def send_command(self, text: str):
        asyncio.run_coroutine_threadsafe(self._send(text), self.loop)

    async def _connect(self, name, address, scan_timeout):
        try:
            self.events.put(("status", f"Scanning for '{name}'..."))
            target = address
            if not target:
                device = await BleakScanner.find_device_by_filter(
                    lambda d, adv: d.name == name, timeout=scan_timeout
                )
                if device is None:
                    self.events.put(("error", f"Device '{name}' not found"))
                    return
                target = device
            # Windows sometimes hands back a stale/incomplete cached GATT
            # service table right after connect (especially just after an
            # OS-level pairing was removed), so start_notify fails with
            # "Characteristic ... was not found!" even though the connect
            # itself succeeded. A full disconnect+reconnect cycle clears it;
            # retry that a few times instead of making the user click
            # Connect again by hand.
            last_exc = None
            for attempt in range(4):
                self.client = BleakClient(target)
                try:
                    await self.client.connect()
                    await asyncio.sleep(0.5)  # let Windows finish resolving services
                    await self.client.start_notify(NUS_TX_UUID, self._on_notify)
                    last_exc = None
                    break
                except Exception as exc:  # noqa: BLE001
                    last_exc = exc
                    try:
                        await self.client.disconnect()
                    except Exception:  # noqa: BLE001
                        pass
                    await asyncio.sleep(1.5)
            if last_exc is not None:
                raise last_exc
            self.events.put(("connected", self.client.address))
        except Exception as exc:  # noqa: BLE001
            self.events.put(("error", str(exc)))

    def _on_notify(self, _handle, data: bytearray):
        pkt = parse_packet(bytes(data))
        if pkt is not None:
            self.events.put(("packet", pkt))

    async def _disconnect(self):
        try:
            if self.client is not None:
                await self.client.disconnect()
        except Exception:  # noqa: BLE001
            pass
        self.client = None
        self.events.put(("disconnected", None))

    async def _send(self, text: str):
        try:
            if self.client is not None and self.client.is_connected:
                await self.client.write_gatt_char(NUS_RX_UUID, text.encode("ascii"), response=False)
                self.events.put(("sent", text))
            else:
                self.events.put(("error", "Not connected"))
        except Exception as exc:  # noqa: BLE001
            self.events.put(("error", str(exc)))


MODE_NAMES = {0: "LIVE", 1: "SCAN", 2: "REVIEW"}


class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        root.title("GeoScan3D Viewer")
        root.geometry("760x560")

        self.events: "queue.Queue" = queue.Queue()
        self.worker = BleWorker(self.events)
        self.worker.start()

        self.history = []  # gradKf history for the strip chart
        self.connected = False

        self._build_ui()
        self.root.after(50, self._poll_events)

    # ---- UI ----------------------------------------------------------
    def _build_ui(self):
        top = ttk.Frame(self.root, padding=8)
        top.pack(fill="x")

        ttk.Label(top, text="Device name:").pack(side="left")
        self.name_var = tk.StringVar(value="CH05")
        ttk.Entry(top, textvariable=self.name_var, width=12).pack(side="left", padx=4)

        self.connect_btn = ttk.Button(top, text="Connect", command=self._on_connect_click)
        self.connect_btn.pack(side="left", padx=8)

        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(top, textvariable=self.status_var, foreground="#b00020").pack(side="left", padx=8)

        # Live readout
        grid = ttk.LabelFrame(self.root, text="Live data", padding=8)
        grid.pack(fill="x", padx=8, pady=4)
        self.value_vars = {}
        fields = [("ts", "Time (ms)"), ("ut1", "S1 (uT)"), ("ut2", "S2 (uT)"),
                  ("grad", "Grad (uT)"), ("gradKf", "GradKf (uT)"), ("depthCm", "Depth (cm)"),
                  ("detect", "Detect"), ("mode", "Mode"), ("yaw", "Yaw (deg)")]
        for i, (key, label) in enumerate(fields):
            r, c = divmod(i, 3)
            ttk.Label(grid, text=label + ":").grid(row=r, column=c * 2, sticky="e", padx=4, pady=2)
            var = tk.StringVar(value="--")
            self.value_vars[key] = var
            ttk.Label(grid, textvariable=var, width=12).grid(row=r, column=c * 2 + 1, sticky="w")

        # Strip chart
        chart_frame = ttk.LabelFrame(self.root, text="GradKf live chart", padding=4)
        chart_frame.pack(fill="both", expand=True, padx=8, pady=4)
        self.canvas = tk.Canvas(chart_frame, bg="white", height=180)
        self.canvas.pack(fill="both", expand=True)

        # Controls
        ctrl = ttk.LabelFrame(self.root, text="Commands", padding=8)
        ctrl.pack(fill="x", padx=8, pady=4)

        ttk.Button(ctrl, text="CAL", command=lambda: self._send("CAL")).pack(side="left", padx=3)
        ttk.Button(ctrl, text="MAGCAL", command=lambda: self._send("MAGCAL")).pack(side="left", padx=3)
        ttk.Button(ctrl, text="PULSE / STEP", command=lambda: self._send("STEP")).pack(side="left", padx=3)
        ttk.Button(ctrl, text="START", command=lambda: self._send("START")).pack(side="left", padx=3)
        ttk.Button(ctrl, text="STOP", command=lambda: self._send("STOP")).pack(side="left", padx=3)

        ttk.Label(ctrl, text="  THR (uT):").pack(side="left", padx=(12, 2))
        self.thr_var = tk.StringVar(value="5.0")
        ttk.Entry(ctrl, textvariable=self.thr_var, width=6).pack(side="left")
        ttk.Button(ctrl, text="Set", command=self._on_set_thr).pack(side="left", padx=3)

        ttk.Label(ctrl, text="  Grid W,H:").pack(side="left", padx=(12, 2))
        self.grid_w_var = tk.StringVar(value="8")
        self.grid_h_var = tk.StringVar(value="8")
        ttk.Entry(ctrl, textvariable=self.grid_w_var, width=4).pack(side="left")
        ttk.Entry(ctrl, textvariable=self.grid_h_var, width=4).pack(side="left")
        ttk.Button(ctrl, text="Set", command=self._on_set_grid).pack(side="left", padx=3)

        # Log
        log_frame = ttk.LabelFrame(self.root, text="Log", padding=4)
        log_frame.pack(fill="both", expand=False, padx=8, pady=(0, 8))
        self.log = tk.Text(log_frame, height=6, state="disabled")
        self.log.pack(fill="both", expand=True)

    # ---- actions -------------------------------------------------------
    def _on_connect_click(self):
        if self.connected:
            self.worker.disconnect()
            self.connect_btn.config(text="Connect")
        else:
            self._log(f"Connecting to '{self.name_var.get()}'...")
            self.worker.connect(self.name_var.get().strip() or "CH05", None, 10.0)

    def _on_set_thr(self):
        try:
            val = float(self.thr_var.get())
        except ValueError:
            self._log("Invalid THR value")
            return
        self._send(f"THR,{val}")

    def _on_set_grid(self):
        try:
            w = int(self.grid_w_var.get())
            h = int(self.grid_h_var.get())
        except ValueError:
            self._log("Invalid grid size")
            return
        self._send(f"GRID {w} {h}")

    def _send(self, text: str):
        self.worker.send_command(text)

    def _log(self, text: str):
        self.log.config(state="normal")
        self.log.insert("end", f"{time.strftime('%H:%M:%S')}  {text}\n")
        self.log.see("end")
        self.log.config(state="disabled")

    # ---- event pump ------------------------------------------------------
    def _poll_events(self):
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "status":
                    self.status_var.set(payload)
                elif kind == "connected":
                    self.connected = True
                    self.status_var.set(f"Connected: {payload}")
                    self.connect_btn.config(text="Disconnect")
                    self._log(f"Connected to {payload}")
                elif kind == "disconnected":
                    self.connected = False
                    self.status_var.set("Disconnected")
                    self.connect_btn.config(text="Connect")
                    self._log("Disconnected")
                elif kind == "error":
                    self.status_var.set(f"Error: {payload}")
                    self._log(f"Error: {payload}")
                elif kind == "sent":
                    self._log(f"Sent: {payload}")
                elif kind == "packet":
                    self._update_packet(payload)
        except queue.Empty:
            pass
        self.root.after(50, self._poll_events)

    def _update_packet(self, pkt: dict):
        self.value_vars["ts"].set(str(pkt["ts"]))
        self.value_vars["ut1"].set(f"{pkt['ut1']:.2f}")
        self.value_vars["ut2"].set(f"{pkt['ut2']:.2f}")
        self.value_vars["grad"].set(f"{pkt['grad']:.2f}")
        self.value_vars["gradKf"].set(f"{pkt['gradKf']:.2f}")
        self.value_vars["depthCm"].set(f"{pkt['depthCm']:.1f}")
        self.value_vars["detect"].set("YES" if pkt["detect"] else "no")
        self.value_vars["mode"].set(MODE_NAMES.get(pkt["mode"], str(pkt["mode"])))
        self.value_vars["yaw"].set("--" if pkt["yaw"] is None else f"{pkt['yaw']:.1f}")

        self.history.append(pkt["gradKf"])
        if len(self.history) > 200:
            self.history.pop(0)
        self._redraw_chart()

    def _redraw_chart(self):
        self.canvas.delete("all")
        w = self.canvas.winfo_width() or 700
        h = self.canvas.winfo_height() or 180
        if len(self.history) < 2:
            return
        lo, hi = min(self.history), max(self.history)
        if hi - lo < 1e-6:
            hi = lo + 1.0
        n = len(self.history)
        points = []
        for i, v in enumerate(self.history):
            x = i / (n - 1) * (w - 10) + 5
            y = h - 5 - (v - lo) / (hi - lo) * (h - 10)
            points.extend([x, y])
        self.canvas.create_line(*points, fill="#1a73e8", width=2)


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
