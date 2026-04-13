#!/usr/bin/env python3
"""
Minimal matplotlib-based live plot for MONOMOD.

Usage:
    python3 stream_matplotlib.py 192.168.0.96
"""

import sys
import time
import threading
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

sys.path.insert(0, "src")
from monomod import Device


NUM_CHANNELS = 3
BUFFER_SECONDS = 5.0
SAMPLE_RATE_HZ = 3200       # expected SPS from device
BUFFER_SIZE = int(BUFFER_SECONDS * SAMPLE_RATE_HZ)  # ~16000 samples
REFRESH_HZ = 30              # plot refresh rate


class LivePlot:
    def __init__(self, ip: str):
        self.ip = ip
        self.device = Device()

        # Ring buffer (numpy circular)
        self._buf = np.zeros((BUFFER_SIZE, NUM_CHANNELS), dtype=np.float32)
        self._write_pos = 0
        self._total_written = 0
        self._lock = threading.Lock()

        # Stats for terminal output
        self._last_stat_time = 0
        self._last_total = 0

    def connect(self) -> bool:
        if not self.device.connect(self.ip):
            print(f"ERROR: Failed to connect to {self.ip}")
            return False
        info = self.device.info
        if info:
            print(f"Connected: FW {info['fw_version']}  {info['num_channels']}ch  "
                  f"max {info['max_sample_rate']} Hz")
        return True

    def _on_data(self, samples: np.ndarray, ts_us: int, seq: int):
        """Called from UDP receiver thread."""
        n = samples.shape[0]
        if n == 0:
            return
        with self._lock:
            # Write to circular buffer
            for i in range(n):
                self._buf[self._write_pos] = samples[i, :NUM_CHANNELS]
                self._write_pos = (self._write_pos + 1) % BUFFER_SIZE
            self._total_written += n

    def start_stream(self):
        self.device.set_data_callback(self._on_data)
        if not self.device.start(sample_rate=SAMPLE_RATE_HZ):
            print("ERROR: start failed")
            return False
        print("Streaming...")
        return True

    def stop(self):
        self.device.stop()
        self.device.disconnect()

    def get_latest(self):
        """Return the last BUFFER_SIZE samples in chronological order."""
        with self._lock:
            total = self._total_written
            if total == 0:
                return None
            n_valid = min(total, BUFFER_SIZE)
            if total < BUFFER_SIZE:
                # Buffer not yet full — data is at [0:total]
                return self._buf[:total].copy()
            else:
                # Buffer is full — reorder so oldest first
                out = np.empty((BUFFER_SIZE, NUM_CHANNELS), dtype=np.float32)
                tail = self._buf[self._write_pos:]
                head = self._buf[:self._write_pos]
                out[:len(tail)] = tail
                out[len(tail):] = head
                return out


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 stream_matplotlib.py <device_ip>")
        return 1

    ip = sys.argv[1]
    app = LivePlot(ip)
    if not app.connect():
        return 1
    if not app.start_stream():
        app.stop()
        return 1

    # Setup matplotlib figure
    plt.style.use("dark_background")
    fig, axes = plt.subplots(NUM_CHANNELS, 1, figsize=(10, 7), sharex=True)
    if NUM_CHANNELS == 1:
        axes = [axes]
    colors = ["white", "orange", "cyan"]
    lines = []
    for i, ax in enumerate(axes):
        (line,) = ax.plot([], [], color=colors[i], lw=1)
        ax.set_ylabel(f"CH{i}")
        ax.grid(alpha=0.3)
        lines.append(line)
    axes[-1].set_xlabel("Time (s)")
    fig.suptitle(f"MONOMOD @ {ip}")

    stats_time = time.monotonic()
    last_total = 0

    def update(frame):
        nonlocal stats_time, last_total

        data = app.get_latest()
        if data is None or data.shape[0] < 2:
            return lines

        n = data.shape[0]
        t = np.linspace(-n / SAMPLE_RATE_HZ, 0, n)  # time axis ending at 0 (now)

        # Per-channel DC subtraction so the Y scale is meaningful
        for i in range(NUM_CHANNELS):
            y = data[:, i] - np.mean(data[:, i])
            lines[i].set_data(t, y)
            axes[i].set_xlim(t[0], t[-1])
            # Autoscale Y based on actual signal
            ymax = float(np.max(np.abs(y)))
            if ymax < 1:
                ymax = 1
            axes[i].set_ylim(-ymax * 1.2, ymax * 1.2)

        # Terminal stats every ~1 s
        now = time.monotonic()
        if now - stats_time >= 1.0:
            total = app._total_written
            sps = (total - last_total) / (now - stats_time)
            last3 = data[-1, :]
            first3 = data[0, :]
            print(f"[mpl] buffer={n}  sps={sps:.0f}  "
                  f"ch0(last)={last3[0]:+.0f}  ch1(last)={last3[1]:+.0f}  "
                  f"ch2(last)={last3[2]:+.0f}")
            stats_time = now
            last_total = total

        return lines

    anim = FuncAnimation(
        fig, update,
        interval=int(1000 / REFRESH_HZ),
        blit=False, cache_frame_data=False,
    )

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        app.stop()
        print("Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
