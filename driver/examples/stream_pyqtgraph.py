#!/usr/bin/env python3
"""
Minimal PyQt6 + pyqtgraph live plot for MONOMOD.

Uses the SAME plot logic as our GUI (shift-register buffer, per-channel DC,
uniform y_scale) but with NO tab widget, splitter, or other wrapping.

If this works → the issue in the GUI is specifically about the nested layout.
If this also fails → our plot code has a fundamental bug.

Usage:
    python3 stream_pyqtgraph.py 192.168.0.96
"""

import sys
import time

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QApplication

sys.path.insert(0, "src")
from monomod import Device


NUM_CHANNELS = 3
WINDOW_SEC = 5.0
DISPLAY_HZ = 200
TICK_MS = 33
CHANNEL_SPACING = 100.0
CHANNEL_COLORS = [(200, 200, 200), (255, 180, 0), (0, 180, 255)]


class LivePlot(pg.PlotWidget):
    """Minimal plot — copy of axonCtrl's PlotArea pattern."""

    def __init__(self):
        super().__init__()
        self.setBackground("k")
        self.showGrid(x=True, y=True, alpha=0.3)
        self.setLabel("bottom", "Time", units="s")
        self.setLabel("left", "Stacked channels")

        self.setDownsampling(auto=True, mode="peak")
        self.setClipToView(True)

        # Curves — axonCtrl pattern
        self.curves = []
        for i in range(NUM_CHANNELS):
            c = pg.PlotCurveItem(
                pen=pg.mkPen(CHANNEL_COLORS[i], width=1)
            )
            self.addItem(c)
            self.curves.append(c)

        # Buffers
        n = int(WINDOW_SEC * DISPLAY_HZ)
        self._x = np.linspace(0, WINDOW_SEC, n, dtype=np.float32)
        self._y = np.zeros((n, NUM_CHANNELS), dtype=np.float32)
        self._dc = np.zeros(NUM_CHANNELS, dtype=np.float32)
        self.y_scale = 1.0
        self._samples_written = 0
        self._scaled = False

        self.setLimits(xMin=0.0)
        self.setXRange(0.0, WINDOW_SEC, padding=0)
        self.setYRange(-CHANNEL_SPACING * 0.5,
                       CHANNEL_SPACING * NUM_CHANNELS * 1.2, padding=0)

    def push(self, arr):
        if arr.size == 0:
            return
        arr = arr.astype(np.float32, copy=False)
        dn = arr.shape[0]
        n = self._y.shape[0]
        nch = min(arr.shape[1], NUM_CHANNELS)
        if dn >= n:
            self._y[:, :nch] = arr[-n:, :nch]
        else:
            self._y[:-dn, :nch] = self._y[dn:, :nch]
            self._y[-dn:, :nch] = arr[:, :nch]
        self._samples_written += dn
        self._x += dn / float(DISPLAY_HZ)  # axonCtrl: X grows over time

    def redraw(self):
        if self._samples_written < 10:
            return

        if not self._scaled and self._samples_written >= DISPLAY_HZ:
            pp_list = []
            for i in range(NUM_CHANNELS):
                self._dc[i] = float(np.mean(self._y[:, i]))
                pp = float(np.ptp(self._y[:, i] - self._dc[i]))
                if pp > 0:
                    pp_list.append(pp)
            if pp_list:
                self.y_scale = CHANNEL_SPACING / (3.0 * float(np.median(pp_list)))
                print(f"[plot] y_scale={self.y_scale:.3e} dc={self._dc.tolist()}")
                self._scaled = True

        ys = float(self.y_scale)
        ds = max(1, len(self._x) // 2000)
        x = self._x[::ds]
        for i in range(NUM_CHANNELS):
            y = self._y[::ds, i]
            self.curves[i].setData(x, (y - self._dc[i]) * ys + (i * CHANNEL_SPACING))

        # Follow X (axonCtrl pattern)
        xmin, xmax = float(self._x[0]), float(self._x[-1])
        xmin = round(xmin * 10.0) / 10.0
        xmax = round(xmax * 10.0) / 10.0
        self.setXRange(xmin, xmax, padding=0)


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 stream_pyqtgraph.py <device_ip>")
        return 1

    ip = sys.argv[1]

    app = QApplication(sys.argv)
    plot = LivePlot()
    plot.resize(1000, 600)
    plot.setWindowTitle(f"MONOMOD pyqtgraph test @ {ip}")
    plot.show()

    device = Device()
    if not device.connect(ip):
        print(f"ERROR: Failed to connect to {ip}")
        return 1

    print(f"Connected: {device.info}")

    def on_data(samples, ts_us, seq):
        pass  # noop — tick will drain queue

    device.set_data_callback(on_data)
    if not device.start(sample_rate=3200):
        print("ERROR: start failed")
        return 1
    print("Streaming started.")

    last_stat = time.monotonic()
    last_total = [0]

    def tick():
        nonlocal last_stat
        arr, _ = device.drain()
        if arr is None or arr.size == 0:
            return
        # Decimate to DISPLAY_HZ
        sr = 3200
        decim = max(1, round(sr / DISPLAY_HZ))
        arr_ds = arr[::decim, :NUM_CHANNELS].astype(np.float32)
        plot.push(arr_ds)
        plot.redraw()

        now = time.monotonic()
        if now - last_stat >= 1.0:
            last_stat = now
            print(f"[tick] buffer={plot._samples_written} "
                  f"last_raw={arr[-1, :].tolist()}")

    timer = QTimer()
    timer.timeout.connect(tick)
    timer.start(TICK_MS)

    try:
        sys.exit(app.exec())
    finally:
        device.stop()
        device.disconnect()


if __name__ == "__main__":
    sys.exit(main())
