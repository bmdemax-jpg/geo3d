#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GeoScan3D_Viewer_USB -- open-source desktop viewer for the M5Stack
GeoScan3D device over a plain USB cable (no BLE, no com0com, no bridge).

Requires firmware v9.6+ (adds Serial command handling and an always-on
text mirror of the live GRAD/DEPTH/DET/MODE line -- see main.cpp
serviceSerialCommands() / sendBleTextLine()).

Use this only as a fallback when BLE keeps failing on a given Windows
machine (WinRT/GATT quirks); for real field use the device still needs
BLE since the whole point is a wireless connection. This tool exists to
prove the firmware/protocol work at all, independent of Bluetooth.

Usage: double-click Run_GeoScan3D_Viewer_USB.bat, or:
    python GeoScan3D_Viewer_USB.py
"""

import queue
import re
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


ensure_dependency("serial", "pyserial>=3.5")

import serial  # noqa: E402
import serial.tools.list_ports  # noqa: E402

# Matches the firmware's text line: GRAD,<f>,DEPTH,<f>,DET,<0|1>,MODE,<L|S|R>
LINE_RE = re.compile(
    r"GRAD,(?P<grad>-?\d+\.?\d*),DEPTH,(?P<depth>-?\d+\.?\d*),"
    r"DET,(?P<det>\d+),MODE,(?P<mode>[LSR])"
)
MODE_NAMES = {"L": "LIVE", "S": "SCAN", "R": "REVIEW"}


class SerialWorker:
    """Reads lines from the COM port and writes commands to it, on a background thread."""

    def __init__(self, events: "queue.Queue"):
        self.events = events
        self.commands: "queue.Queue" = queue.Queue()
        self.ser = None
        self.thread = None
        self._stop = threading.Event()

    def connect(self, port: str, baud: int):
        self._stop.clear()
        self.thread = threading.Thread(target=self._run, args=(port, baud), daemon=True)
        self.thread.start()

    def disconnect(self):
        self._stop.set()

    def send_command(self, text: str):
        self.commands.put(text)

    def _run(self, port, baud):
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.2)
        except Exception as exc:  # noqa: BLE001
            self.events.put(("error", str(exc)))
            return
        self.events.put(("connected", port))
        buf = b""
        try:
            while not self._stop.is_set():
                while not self.commands.empty():
                    cmd = self.commands.get_nowait()
                    self.ser.write((cmd + "\n").encode("ascii"))
                    self.events.put(("sent", cmd))
                data = self.ser.read(256)
                if data:
                    buf += data
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        text = line.decode("ascii", errors="ignore").strip()
                        if text:
                            self._handle_line(text)
        finally:
            try:
                self.ser.close()
            except Exception:  # noqa: BLE001
                pass
            self.events.put(("disconnected", None))

    def _handle_line(self, text: str):
        m = LINE_RE.search(text)
        if m:
            self.events.put(("packet", {
                "grad": float(m.group("grad")),
                "depth": float(m.group("depth")),
                "det": int(m.group("det")),
                "mode": MODE_NAMES.get(m.group("mode"), m.group("mode")),
            }))
        else:
            self.events.put(("log", text))


class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        root.title("GeoScan3D Viewer (USB)")
        root.geometry("760x560")

        self.events: "queue.Queue" = queue.Queue()
        self.worker = SerialWorker(self.events)
        self.history = []
        self.connected = False

        self._build_ui()
        self._refresh_ports()
        self.root.after(50, self._poll_events)

    def _build_ui(self):
        top = ttk.Frame(self.root, padding=8)
        top.pack(fill="x")

        ttk.Label(top, text="COM port:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=10, state="readonly")
        self.port_combo.pack(side="left", padx=4)
        ttk.Button(top, text="Refresh", command=self._refresh_ports).pack(side="left")

        self.connect_btn = ttk.Button(top, text="Connect", command=self._on_connect_click)
        self.connect_btn.pack(side="left", padx=8)

        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(top, textvariable=self.status_var, foreground="#b00020").pack(side="left", padx=8)

        grid = ttk.LabelFrame(self.root, text="Live data", padding=8)
        grid.pack(fill="x", padx=8, pady=4)
        self.value_vars = {}
        for i, (key, label) in enumerate([("grad", "Grad (uT)"), ("depth", "Depth (cm)"),
                                           ("det", "Detect"), ("mode", "Mode")]):
            ttk.Label(grid, text=label + ":").grid(row=0, column=i * 2, sticky="e", padx=4, pady=2)
            var = tk.StringVar(value="--")
            self.value_vars[key] = var
            ttk.Label(grid, textvariable=var, width=10).grid(row=0, column=i * 2 + 1, sticky="w")

        chart_frame = ttk.LabelFrame(self.root, text="Grad live chart", padding=4)
        chart_frame.pack(fill="both", expand=True, padx=8, pady=4)
        self.canvas = tk.Canvas(chart_frame, bg="white", height=180)
        self.canvas.pack(fill="both", expand=True)

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

        log_frame = ttk.LabelFrame(self.root, text="Log", padding=4)
        log_frame.pack(fill="both", expand=False, padx=8, pady=(0, 8))
        self.log = tk.Text(log_frame, height=6, state="disabled")
        self.log.pack(fill="both", expand=True)

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _on_connect_click(self):
        if self.connected:
            self.worker.disconnect()
            self.connect_btn.config(text="Connect")
        else:
            port = self.port_var.get()
            if not port:
                self._log("Pick a COM port first")
                return
            self._log(f"Connecting to {port}...")
            self.worker.connect(port, 115200)

    def _on_set_thr(self):
        try:
            val = float(self.thr_var.get())
        except ValueError:
            self._log("Invalid THR value")
            return
        self._send(f"THR,{val}")

    def _send(self, text: str):
        self.worker.send_command(text)

    def _log(self, text: str):
        self.log.config(state="normal")
        self.log.insert("end", f"{time.strftime('%H:%M:%S')}  {text}\n")
        self.log.see("end")
        self.log.config(state="disabled")

    def _poll_events(self):
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "connected":
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
                elif kind == "log":
                    self._log(payload)
                elif kind == "packet":
                    self._update_packet(payload)
        except queue.Empty:
            pass
        self.root.after(50, self._poll_events)

    def _update_packet(self, pkt: dict):
        self.value_vars["grad"].set(f"{pkt['grad']:.2f}")
        self.value_vars["depth"].set(f"{pkt['depth']:.1f}")
        self.value_vars["det"].set("YES" if pkt["det"] else "no")
        self.value_vars["mode"].set(pkt["mode"])

        self.history.append(pkt["grad"])
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
