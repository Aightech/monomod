#!/usr/bin/env python3
"""
MONOMOD GUI — Multi-device EMG streaming with one unified plot.

Connect to N devices, stream all EMG channels onto a single stacked plot,
record everything to one long-format CSV.
"""

import sys
import time
import threading
from collections import deque

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import QTimer, Qt, QThread, pyqtSignal
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QLineEdit, QFileDialog, QStatusBar, QMessageBox,
    QListWidget, QListWidgetItem, QInputDialog, QFrame, QComboBox, QCheckBox,
)

from monomod import Device, discover_devices
from monomod.config import (load_config, get_sample_rate, load_raw,
                            iter_session_devices, get_recording,
                            get_experiment, get_gui)
from monomod.protocol import ADC_TYPE_ADS1293, ADC_TYPE_INA
from lsl_streamer import LslManager
from recorder import make_recorder, HAVE_H5PY


# =============================================================================
# Constants
# =============================================================================
MAX_CHANNELS_PER_DEVICE = 3   # upper bound for buffer allocation (ADS1293 has 3, INA has 1)
WINDOW_SEC = 5.0
DISPLAY_HZ = 200
TICK_MS = 33
CHANNEL_SPACING = 100.0
GAP_SEC = 0.25   # a data gap longer than this is drawn as a break (outage)

# IMU buffer sizing (ICM-20948 @ 100 Hz, 5 s window = 500 samples)
IMU_RATE_HZ = 100
IMU_BUFFER_N = int(WINDOW_SEC * IMU_RATE_HZ)

# Selectable rates for the LIS3DH node (INA). EMG values are 1000/N so the 1 ms
# FreeRTOS tick can hit them exactly; the node clamps to its capabilities.
EMG_RATE_CHOICES = [100, 200, 250, 500, 1000]
IMU_RATE_CHOICES = [25, 50, 100, 200]

# Base channel colors for EMG channels (ch0/ch1/ch2)
BASE_HUES = [
    (200, 200, 200),  # white-ish (ch0)
    (255, 180, 0),    # orange    (ch1)
    (0, 180, 255),    # cyan      (ch2)
]
# IMU channel colors (X/Y/Z — RGB convention)
IMU_HUES = [
    (255, 100, 100),  # X - red
    (100, 255, 100),  # Y - green
    (100, 150, 255),  # Z - blue
]
# Device tints applied multiplicatively
DEVICE_TINTS = [
    (1.00, 1.00, 1.00),  # dev0
    (0.70, 1.00, 0.70),  # dev1 (greener)
    (1.00, 0.70, 0.70),  # dev2 (redder)
    (0.70, 0.70, 1.00),  # dev3 (bluer)
    (1.00, 1.00, 0.60),  # dev4 (yellower)
]


# Default marker map (digit key -> label); overridable from experiment.markers
DEFAULT_MARKERS = {
    "1": "trial_start",
    "2": "contraction_start",
    "3": "contraction_stop",
    "4": "trial_end",
    "0": "artifact",
}


def _tinted(base: tuple, device_idx: int) -> tuple:
    t = DEVICE_TINTS[device_idx % len(DEVICE_TINTS)]
    return tuple(int(max(0, min(255, b * tt))) for b, tt in zip(base, t))


def pen_for_emg(device_idx: int, channel_idx: int) -> pg.mkPen:
    return pg.mkPen(_tinted(BASE_HUES[channel_idx % 3], device_idx), width=1)


def pen_for_imu(device_idx: int, axis_idx: int) -> pg.mkPen:
    return pg.mkPen(_tinted(IMU_HUES[axis_idx % 3], device_idx), width=1)


# =============================================================================
# DeviceSlot — one connected device with its own buffer
# =============================================================================
class DeviceSlot:
    """Wraps a monomod.Device + a local shift-register buffer for plotting."""

    # Shared monotonic epoch so every device's time axis is on ONE wall clock.
    # (Sample times are anchored to this; a per-device sample counter would leave
    # a reconnected device behind.)
    _epoch = None

    @staticmethod
    def _now() -> float:
        if DeviceSlot._epoch is None:
            DeviceSlot._epoch = time.monotonic()
        return time.monotonic() - DeviceSlot._epoch

    def __init__(self, ip: str, device_idx: int):
        self.ip = ip
        self.device_idx = device_idx
        self.device = Device()
        self.name = f"dev{device_idx}"

        # num_channels is set after connect() based on device.info. Default 3.
        self.num_channels = 3

        n = int(WINDOW_SEC * DISPLAY_HZ)
        self._x = np.linspace(0, WINDOW_SEC, n, dtype=np.float32)
        # NaN = "no data yet" — unfilled buffer isn't drawn (connect='finite')
        self._y = np.full((n, MAX_CHANNELS_PER_DEVICE), np.nan, dtype=np.float32)
        self._dc = np.zeros(MAX_CHANNELS_PER_DEVICE, dtype=np.float32)
        self._samples_written = 0
        self._sample_rate = 3200
        self._imu_rate = IMU_RATE_HZ
        self._latest_raw = np.zeros(MAX_CHANNELS_PER_DEVICE, dtype=np.int32)
        self._imu_text = "no data yet"

        # IMU buffer: shape (IMU_BUFFER_N, 6) → accel_x/y/z (g), gyro_x/y/z (dps)
        self._imu_x = np.linspace(0, WINDOW_SEC, IMU_BUFFER_N, dtype=np.float32)
        self._imu_y = np.full((IMU_BUFFER_N, 6), np.nan, dtype=np.float32)
        self._imu_written = 0
        # Thread-safe handoff: RX thread appends, GUI thread drains (drain_imu)
        self._imu_queue = deque(maxlen=4096)

        # Stats
        self.info = None
        self.imu_count = 0
        self.data_count = 0
        self._last_data_t = 0.0   # time.monotonic() of last EMG data (stall detect)

        # Auto-reconnect intent/state (managed by MainWindow)
        self._want_stream = False     # user intends this device to be streaming
        self._reconnecting = False    # a background reconnect is in flight
        self._next_reconnect_t = 0.0  # monotonic throttle for retry attempts

        # LSL outlets for this device (set by LslManager when LSL is enabled)
        self._lsl = None

    def _wire(self):
        """Restore device info + RX callbacks after a (re)connect."""
        self.info = self.device.info
        if self.info:
            self.num_channels = min(int(self.info.get("num_channels", 3)),
                                    MAX_CHANNELS_PER_DEVICE)
            # Honor the device-reported sample rate so display decimation,
            # recorded timestamps, and the LSL nominal rate all match reality
            # (e.g. the INA/LIS3DH node reports ~1000 Hz, not the 3200 default).
            rate = int(self.info.get("max_sample_rate", 0) or 0)
            if rate > 0:
                self._sample_rate = rate
        self.device.set_data_callback(self._on_data)
        self.device.set_imu_callback(self._on_imu)

    def connect(self) -> bool:
        if not self.device.connect(self.ip):
            return False
        self._wire()
        print(f"[{self.name}] connected to {self.ip}, "
              f"adc_type={self.info.get('adc_type') if self.info else '?'}, "
              f"channels={self.num_channels}")
        return True

    def reconnect_network(self, emg_rate=None, imu_rate=None,
                          timeout: float = 2.0) -> bool:
        """Network-only reconnect (safe to run off the GUI thread): no numpy
        buffer or Qt access. Re-wires callbacks, restarts the stream, re-syncs
        the clock. Plot buffers are shift-registers, so they need no reset."""
        self.device.disconnect()
        if not self.device.connect(self.ip, timeout=timeout):
            return False
        self._wire()
        if emg_rate:
            self._sample_rate = emg_rate
        if imu_rate:
            self._imu_rate = imu_rate
        ok = self.device.start(sample_rate=self._sample_rate)
        if ok:
            div = max(1, round(self._sample_rate / max(1, self._imu_rate)))
            try:
                self.device.set_imu(True, div)
            except Exception:
                pass
            try:
                self.device.sync_clock()
            except Exception:
                pass
        return ok

    def disconnect(self):
        if self.device.is_connected:
            if self.device.is_streaming:
                self.device.stop()
            self.device.disconnect()

    def start(self) -> bool:
        if not self.device.is_connected:
            return False
        # reset EMG buffer (NaN = no data yet)
        self._y.fill(np.nan)
        self._dc.fill(0)
        self._x = np.linspace(0, WINDOW_SEC, len(self._x), dtype=np.float32)
        self._samples_written = 0
        # reset IMU buffer
        self._imu_y.fill(np.nan)
        self._imu_x = np.linspace(0, WINDOW_SEC, IMU_BUFFER_N, dtype=np.float32)
        self._imu_written = 0
        ok = self.device.start(sample_rate=self._sample_rate)
        if ok:
            self._want_stream = True   # intent — drives auto-reconnect
            # IMU decimation: emit 1 accel sample per (emg_rate / imu_rate) EMG samples
            div = max(1, round(self._sample_rate / max(1, self._imu_rate)))
            try:
                self.device.set_imu(True, div)
            except Exception as e:
                print(f"[{self.name}] set_imu failed: {e!r}")
            # NTP-style clock sync so multi-device recordings share a timeline
            try:
                self.device.sync_clock()
            except Exception as e:
                print(f"[{self.name}] clock sync failed: {e!r}")
        return ok

    def stop(self) -> bool:
        self._want_stream = False   # user asked to stop — don't auto-reconnect
        return self.device.stop()

    @property
    def is_connected(self) -> bool:
        return self.device.is_connected

    @property
    def is_streaming(self) -> bool:
        return self.device.is_streaming

    def _on_data(self, samples: np.ndarray, ts_us: int, seq: int):
        """Called from UDP receiver thread."""
        if samples.size == 0:
            return
        n = min(samples.shape[1], self.num_channels)
        self._latest_raw[:n] = samples[-1, :n]
        self.data_count += 1

    def _on_imu(self, imu, ts_us):
        """Called from the UDP receiver thread — only enqueue (no shared-state
        writes). The GUI thread consumes the queue in drain_imu()."""
        self.imu_count += 1
        a = imu.get("accel_g", [0.0, 0.0, 0.0])
        g = imu.get("gyro_dps", [0.0, 0.0, 0.0])
        # deque.append is atomic under the GIL — safe across threads.
        self._imu_queue.append((a[0], a[1], a[2], g[0], g[1], g[2], ts_us))

    def drain_imu(self):
        """Called on the GUI thread: pop queued IMU rows, update the plot buffer
        and status text, and return the rows (for recording + LSL).

        Each row is (ax, ay, az, gx, gy, gz, ts_us); accel in g, gyro in dps.
        """
        if not self._imu_queue:
            return []
        rows = []
        while self._imu_queue:
            rows.append(self._imu_queue.popleft())
        # Single batched shift instead of one full-array roll per row.
        arr = np.asarray([r[:6] for r in rows], dtype=np.float32)   # (m, 6)
        m = arr.shape[0]
        n = self._imu_y.shape[0]
        now = DeviceSlot._now()
        t_new = now - np.arange(m - 1, -1, -1, dtype=np.float32) / IMU_RATE_HZ
        gap = (now - float(self._imu_x[-1])) > GAP_SEC
        if m >= n:
            self._imu_y[:] = arr[-n:]
            self._imu_x[:] = t_new[-n:]
        else:
            self._imu_y[:-m] = self._imu_y[m:]
            self._imu_y[-m:] = arr
            self._imu_x[:-m] = self._imu_x[m:]
            self._imu_x[-m:] = t_new
            if gap:
                self._imu_y[-m, :] = np.nan   # NaN break => drawn as a gap
        self._imu_written += m
        last = rows[-1]
        self._imu_text = (f"Acc {last[0]:+.2f}/{last[1]:+.2f}/{last[2]:+.2f}g "
                          f"Gyr {last[3]:+.0f}/{last[4]:+.0f}/{last[5]:+.0f}dps")
        return rows

    def drain_push(self, recorder=None):
        """Called in GUI thread at tick rate. Drains device data and updates buffer.
        Returns the drained array for recording (or None)."""
        arr, _ = self.device.drain()
        if arr is None or arr.size == 0:
            return None
        self._last_data_t = time.monotonic()

        # Decimate for display
        decim = max(1, round(self._sample_rate / DISPLAY_HZ))
        nch = self.num_channels
        arr_ds = arr[::decim, :nch].astype(np.float32)

        dn = arr_ds.shape[0]
        n = self._y.shape[0]
        now = DeviceSlot._now()
        # Real per-sample times for this batch (contiguous, ending at "now").
        t_new = now - np.arange(dn - 1, -1, -1, dtype=np.float32) / DISPLAY_HZ
        gap = (now - float(self._x[-1])) > GAP_SEC   # outage since last sample
        if dn >= n:
            self._y[:, :nch] = arr_ds[-n:, :]
            self._x[:] = t_new[-n:]
        else:
            self._y[:-dn, :nch] = self._y[dn:, :nch]
            self._y[-dn:, :nch] = arr_ds
            self._x[:-dn] = self._x[dn:]
            self._x[-dn:] = t_new
            if gap:
                self._y[-dn, :nch] = np.nan   # NaN break => drawn as a gap
        self._samples_written += dn

        return arr  # full-resolution data for recording

    def compute_dc(self):
        """Recompute per-channel DC from full buffer (called by plot auto-scale)."""
        for i in range(self.num_channels):
            col = self._y[:, i]
            self._dc[i] = float(np.nanmean(col)) if np.isfinite(col).any() else 0.0


# =============================================================================
# MultiDevicePlot — shared plot for N devices
# =============================================================================
class MultiDevicePlot(pg.PlotWidget):
    """Stacked multi-device plot. Total lanes = len(slots) * 3."""

    def __init__(self):
        super().__init__()
        self.setBackground("k")
        self.showGrid(x=True, y=True, alpha=0.3)
        self.setLabel("bottom", "Time", units="s")
        self.setLabel("left", "Stacked channels")
        self.setDownsampling(auto=True, mode="peak")
        self.setClipToView(True)
        self.setLimits(xMin=0.0)
        self.setXRange(0.0, WINDOW_SEC, padding=0)

        self.slots = []            # list[DeviceSlot]
        self.y_scale = 1.0
        self._scaled = False

        self._update_y_range()

    def _update_y_range(self):
        total_lanes = max(1, sum(s.num_channels for s in self.slots))
        self.setYRange(-CHANNEL_SPACING * 0.5,
                       CHANNEL_SPACING * total_lanes * 1.1, padding=0)

    def add_slot(self, slot: DeviceSlot):
        self.slots.append(slot)
        # Each slot stores its own curves list so we can have variable channel
        # counts per device (ADS1293 = 3, INA = 1)
        slot.curves = []
        for ch in range(slot.num_channels):
            c = pg.PlotCurveItem(pen=pen_for_emg(slot.device_idx, ch))
            self.addItem(c)
            slot.curves.append(c)
        self._update_y_range()
        self._scaled = False

    def remove_slot(self, slot: DeviceSlot):
        if slot not in self.slots:
            return
        for c in getattr(slot, "curves", []):
            self.removeItem(c)
        slot.curves = []
        self.slots.remove(slot)
        self._update_y_range()

    def clear_data(self):
        for slot in self.slots:
            slot._y.fill(np.nan)
            slot._dc.fill(0)
            slot._x = np.linspace(0, WINDOW_SEC, len(slot._x), dtype=np.float32)
            slot._samples_written = 0
        self._scaled = False

    def manual_rescale(self):
        self._scaled = False

    def redraw(self):
        if not self.slots:
            return

        # Auto-scale once enough data has arrived on at least one slot
        if not self._scaled:
            ready = any(s._samples_written >= DISPLAY_HZ for s in self.slots)
            if ready:
                pp_list = []
                for slot in self.slots:
                    for i in range(slot.num_channels):
                        col = slot._y[:, i]
                        if not np.isfinite(col).any():   # all-NaN (gap) — skip
                            continue
                        slot._dc[i] = float(np.nanmean(col))
                        pp = float(np.nanmax(col) - np.nanmin(col))
                        if pp > 0:
                            pp_list.append(pp)
                if pp_list:
                    self.y_scale = CHANNEL_SPACING / (3.0 * float(np.median(pp_list)))
                    print(f"[plot] auto-scaled: y_scale={self.y_scale:.3e}  "
                          f"median_pp={np.median(pp_list):.0f}")
                    self._scaled = True

        ys = float(self.y_scale)

        # Each slot stacks its own channels into the next lanes
        lane_idx = 0
        x_min_global, x_max_global = None, None
        for slot in self.slots:
            n = slot.num_channels
            if slot._samples_written < 2:
                lane_idx += n
                continue
            ds = max(1, len(slot._x) // 2000)
            x = slot._x[::ds]
            for ch in range(n):
                y = slot._y[::ds, ch]
                slot.curves[ch].setData(
                    x, (y - slot._dc[ch]) * ys + (lane_idx * CHANNEL_SPACING),
                    connect="finite")   # NaN samples => visible gap
                lane_idx += 1

            if x_min_global is None or x[0] < x_min_global:
                x_min_global = float(x[0])
            if x_max_global is None or x[-1] > x_max_global:
                x_max_global = float(x[-1])

        # Right-align to the most-recent sample across devices, fixed width — a
        # lagging (just-reconnected) device stays time-aligned, not stretched.
        if x_max_global is not None:
            self.setXRange(x_max_global - WINDOW_SEC, x_max_global, padding=0)


# =============================================================================
# IMUPlot — one plot showing accelerometer OR gyroscope for N devices
# =============================================================================
class IMUPlot(pg.PlotWidget):
    """One plot for 3-axis IMU data across N devices.

    signal_type: 'accel' (data in g, range ±2) or 'gyro' (dps, range ±250)
    """

    def __init__(self, signal_type: str):
        super().__init__()
        self.signal_type = signal_type  # 'accel' or 'gyro'
        self.setBackground("k")
        self.showGrid(x=True, y=True, alpha=0.3)
        self.setDownsampling(auto=True, mode="peak")
        self.setClipToView(True)
        self.setLimits(xMin=0.0)
        self.setXRange(0.0, WINDOW_SEC, padding=0)

        if signal_type == "accel":
            self.setLabel("left", "Accel", units="g")
            self.setYRange(-2.0, 2.0, padding=0)
            self._col_offset = 0           # cols 0,1,2 in _imu_y
        else:  # gyro
            self.setLabel("left", "Gyro", units="dps")
            self.setYRange(-300.0, 300.0, padding=0)
            self._col_offset = 3           # cols 3,4,5 in _imu_y

        self.setLabel("bottom", "Time", units="s")

        self.slots = []
        self._curves = []   # flat: [slot0_X, slot0_Y, slot0_Z, slot1_X, ...]

    def add_slot(self, slot: DeviceSlot):
        self.slots.append(slot)
        for axis in range(3):
            c = pg.PlotCurveItem(pen=pen_for_imu(slot.device_idx, axis))
            self.addItem(c)
            self._curves.append(c)

    def remove_slot(self, slot: DeviceSlot):
        if slot not in self.slots:
            return
        idx = self.slots.index(slot)
        for _ in range(3):
            c = self._curves.pop(idx * 3)
            self.removeItem(c)
        self.slots.pop(idx)

    def redraw(self):
        if not self.slots:
            return
        x_min, x_max = None, None
        for i, slot in enumerate(self.slots):
            if slot._imu_written < 2:
                continue
            ds = max(1, len(slot._imu_x) // 2000)
            x = slot._imu_x[::ds]
            for axis in range(3):
                col = self._col_offset + axis
                y = slot._imu_y[::ds, col]
                curve = self._curves[i * 3 + axis]
                curve.setData(x, y, connect="finite")   # NaN => visible gap
            if x_max is None or x[-1] > x_max:
                x_max = float(x[-1])
        if x_max is not None:
            self.setXRange(x_max - WINDOW_SEC, x_max, padding=0)


# =============================================================================
# Discovery worker — runs the blocking mDNS scan off the GUI thread
# =============================================================================
class DiscoverWorker(QThread):
    done = pyqtSignal(list)

    def __init__(self, timeout_s: float = 2.0):
        super().__init__()
        self._timeout = timeout_s

    def run(self):
        try:
            devs = discover_devices(timeout_s=self._timeout, verbose=False)
        except Exception as e:
            print(f"[scan] error: {e!r}")
            devs = []
        self.done.emit(devs)


# =============================================================================
# Main Window
# =============================================================================
class MainWindow(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("MONOMOD EMG — Multi-Device")
        self.resize(1400, 800)

        # Recording state (CSV or HDF5 via recorder backend; all recorder access
        # is on the GUI thread, so no lock is needed)
        self._recording = False
        self._recorder = None
        self._last_stat_time = 0

        # Defaults overridable from a loaded session config
        self._marker_map = dict(DEFAULT_MARKERS)
        self._rec_format = "csv"
        self._include_imu = True
        self._experiment = {}   # subject/session/operator/notes (from Load Session)

        # LSL streaming (per device: one EMG + one 6-axis IMU outlet)
        self._lsl = LslManager()

        # Build UI
        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(5, 5, 5, 5)
        root.setSpacing(5)

        # --- Top control bar ---
        ctrl = QHBoxLayout()
        ctrl.addWidget(QLabel("IP:"))
        self.ip_edit = QLineEdit()
        self.ip_edit.setPlaceholderText("192.168.x.x")
        self.ip_edit.setMaximumWidth(140)
        ctrl.addWidget(self.ip_edit)

        self.btn_add = QPushButton("Add Device")
        self.btn_add.clicked.connect(self._on_add_device)
        ctrl.addWidget(self.btn_add)

        self.btn_scan = QPushButton("Scan")
        self.btn_scan.clicked.connect(self._on_scan)
        ctrl.addWidget(self.btn_scan)

        ctrl.addSpacing(20)
        ctrl.addWidget(QLabel("EMG Hz:"))
        self.emg_rate_select = QComboBox()
        self.emg_rate_select.addItems([str(r) for r in EMG_RATE_CHOICES])
        self.emg_rate_select.setCurrentText("1000")
        self.emg_rate_select.setToolTip("EMG sample rate (LIS3DH node) — applied on Start")
        ctrl.addWidget(self.emg_rate_select)

        ctrl.addWidget(QLabel("IMU Hz:"))
        self.imu_rate_select = QComboBox()
        self.imu_rate_select.addItems([str(r) for r in IMU_RATE_CHOICES])
        self.imu_rate_select.setCurrentText("100")
        self.imu_rate_select.setToolTip("Accel/IMU sample rate (LIS3DH node) — applied on Start")
        ctrl.addWidget(self.imu_rate_select)

        self.btn_start = QPushButton("Start All")
        self.btn_start.clicked.connect(self._on_start_all)
        ctrl.addWidget(self.btn_start)

        self.btn_stop = QPushButton("Stop All")
        self.btn_stop.clicked.connect(self._on_stop_all)
        ctrl.addWidget(self.btn_stop)

        ctrl.addSpacing(20)
        ctrl.addWidget(QLabel("View:"))
        self.view_select = QComboBox()
        self.view_select.addItems(["EMG only", "Accel only", "Gyro only", "All"])
        self.view_select.setCurrentIndex(0)  # default: EMG only
        self.view_select.currentIndexChanged.connect(self._on_view_changed)
        ctrl.addWidget(self.view_select)

        ctrl.addSpacing(20)
        self.btn_auto = QPushButton("Auto Scale")
        self.btn_auto.clicked.connect(self._on_auto_scale)
        ctrl.addWidget(self.btn_auto)

        self.btn_yaml = QPushButton("Load YAML…")
        self.btn_yaml.clicked.connect(self._on_load_yaml)
        ctrl.addWidget(self.btn_yaml)

        self.btn_session = QPushButton("Load Session…")
        self.btn_session.setToolTip("Connect to all session.devices and apply "
                                    "names, per-device EMG, markers, prefs")
        self.btn_session.clicked.connect(self._on_load_session)
        ctrl.addWidget(self.btn_session)

        self.rec_format = QComboBox()
        self.rec_format.addItems(["CSV", "HDF5"] if HAVE_H5PY else ["CSV"])
        self.rec_format.setToolTip("Recording format" if HAVE_H5PY
                                   else "Recording format (install h5py for HDF5)")
        ctrl.addWidget(self.rec_format)

        self.btn_record = QPushButton("Record")
        self.btn_record.clicked.connect(self._on_record)
        ctrl.addWidget(self.btn_record)

        self.chk_autoreconnect = QCheckBox("Auto-reconnect")
        self.chk_autoreconnect.setToolTip("Reconnect a streaming device that "
                                          "stalls (no data > 3 s)")
        ctrl.addWidget(self.chk_autoreconnect)

        self.btn_lsl = QPushButton("LSL")
        self.btn_lsl.setCheckable(True)
        self.btn_lsl.setToolTip("Stream every device to LSL "
                                "(one EMG + one 6-axis IMU outlet each)")
        self.btn_lsl.clicked.connect(self._on_lsl_toggled)
        ctrl.addWidget(self.btn_lsl)

        ctrl.addStretch(1)
        root.addLayout(ctrl)

        # --- Marker bar (host-side event annotation) ---
        self.marker_bar = QWidget()
        self._marker_layout = QHBoxLayout(self.marker_bar)
        self._marker_layout.setContentsMargins(5, 0, 5, 0)
        self._rebuild_marker_bar()
        root.addWidget(self.marker_bar)

        # --- Main area: device list (left) + plot (right) ---
        main_row = QHBoxLayout()
        main_row.setSpacing(5)

        # Device list panel
        left_panel = QVBoxLayout()
        left_panel.setContentsMargins(0, 0, 0, 0)
        lbl_devs = QLabel("Devices")
        lbl_devs.setStyleSheet("font-weight: bold; padding: 2px;")
        left_panel.addWidget(lbl_devs)
        self.dev_list = QListWidget()
        self.dev_list.setFixedWidth(280)
        left_panel.addWidget(self.dev_list, stretch=1)

        # Per-device controls (act on the selected device)
        dev_btns = QHBoxLayout()
        self.btn_dev_start = QPushButton("Start")
        self.btn_dev_start.clicked.connect(self._on_start_selected)
        self.btn_dev_stop = QPushButton("Stop")
        self.btn_dev_stop.clicked.connect(self._on_stop_selected)
        self.btn_dev_reconnect = QPushButton("Reconnect")
        self.btn_dev_reconnect.clicked.connect(self._on_reconnect_selected)
        for b in (self.btn_dev_start, self.btn_dev_stop, self.btn_dev_reconnect):
            dev_btns.addWidget(b)
        left_panel.addLayout(dev_btns)

        self.btn_remove = QPushButton("Remove selected")
        self.btn_remove.clicked.connect(self._on_remove_device)
        left_panel.addWidget(self.btn_remove)

        left_widget = QWidget()
        left_widget.setLayout(left_panel)
        left_widget.setFixedWidth(290)
        main_row.addWidget(left_widget)

        # Right side: 3 stacked plots (EMG / Accel / Gyro)
        plots_layout = QVBoxLayout()
        plots_layout.setSpacing(2)
        self.plot = MultiDevicePlot()          # EMG
        self.plot_accel = IMUPlot("accel")
        self.plot_gyro = IMUPlot("gyro")
        plots_layout.addWidget(self.plot, stretch=3)
        plots_layout.addWidget(self.plot_accel, stretch=1)
        plots_layout.addWidget(self.plot_gyro, stretch=1)
        plots_widget = QWidget()
        plots_widget.setLayout(plots_layout)
        main_row.addWidget(plots_widget, stretch=1)

        main_row_widget = QWidget()
        main_row_widget.setLayout(main_row)
        root.addWidget(main_row_widget, stretch=1)

        self.setCentralWidget(central)

        # Apply default view (EMG only)
        self._on_view_changed(self.view_select.currentIndex())

        # Status bar
        self.status = QStatusBar()
        self.setStatusBar(self.status)
        self.status.showMessage("Ready")

        # Tick timer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(TICK_MS)

    # ---- Device list helpers ----
    def _refresh_device_list(self):
        self.dev_list.clear()
        now_m = time.monotonic()
        for slot in self.plot.slots:
            if slot.is_streaming:
                stalled = (now_m - slot._last_data_t) > 2.0
                status = "STALLED (no data)" if stalled else "streaming"
            elif slot.is_connected:
                status = "connected"
            else:
                status = "disconnected"
            rx = slot.device.packets_received
            raw = slot._latest_raw
            # Receiver-level IMU packet count (before our _on_imu callback)
            rx = slot.device._rx if hasattr(slot.device, "_rx") else None
            rx_imu_ok = rx.imu_ok if rx else 0
            rx_imu_fail = rx.imu_fail if rx else 0
            if slot.imu_count > 0:
                imu_line = slot._imu_text
            elif slot.is_streaming:
                if rx_imu_ok > 0 or rx_imu_fail > 0:
                    imu_line = f"packets at receiver: ok={rx_imu_ok} fail={rx_imu_fail} (not reaching slot!)"
                else:
                    imu_line = "waiting for IMU packets…"
            else:
                imu_line = "start streaming first"
            last_str = " ".join(f"{int(raw[i]):+d}" for i in range(slot.num_channels))
            st = slot.device.latest_status
            if st:
                health = f"  health: RSSI {st['rssi']}dBm  dropped={st['packets_dropped']}"
                if st.get("battery_pct"):
                    health += f"  bat={st['battery_pct']}%"
            else:
                health = "  health: (no status yet)"
            if slot.device.clock_offset_s is not None:
                health += f"  sync±{slot.device.clock_rtt_s * 500:.1f}ms"
            text = (f"{slot.name}  {slot.ip}  ({slot.num_channels}ch)\n"
                    f"  {status}  data={slot.data_count}  imu={slot.imu_count}\n"
                    f"  last=[{last_str}]\n"
                    f"  IMU: {imu_line}\n"
                    f"{health}")
            item = QListWidgetItem(text)
            self.dev_list.addItem(item)

    # ---- UI event handlers ----
    def _on_add_device(self):
        ip = self.ip_edit.text().strip()
        if not ip:
            return
        # Check duplicates
        for slot in self.plot.slots:
            if slot.ip == ip:
                QMessageBox.information(self, "Already connected", f"{ip} is already in the list")
                return
        device_idx = len(self.plot.slots)
        slot = DeviceSlot(ip, device_idx)
        self.status.showMessage(f"Connecting to {ip}…")
        QApplication.processEvents()
        if not slot.connect():
            QMessageBox.warning(self, "Connection failed", f"Could not connect to {ip}")
            return
        self.plot.add_slot(slot)
        self.plot_accel.add_slot(slot)
        self.plot_gyro.add_slot(slot)
        self.status.showMessage(f"Connected {ip} as {slot.name}")
        self.ip_edit.clear()
        self._refresh_device_list()

    def _on_remove_device(self):
        row = self.dev_list.currentRow()
        if row < 0 or row >= len(self.plot.slots):
            return
        slot = self.plot.slots[row]
        self._lsl.remove(slot)
        slot.disconnect()
        self.plot.remove_slot(slot)
        self.plot_accel.remove_slot(slot)
        self.plot_gyro.remove_slot(slot)
        self._refresh_device_list()

    def _selected_slot(self):
        row = self.dev_list.currentRow()
        if 0 <= row < len(self.plot.slots):
            return self.plot.slots[row]
        return None

    def _apply_rates(self, slot):
        """Apply the GUI rate selectors to a node (INA) device before Start.
        ADS1293 boards keep their YAML/info-derived rate (firmware ignores it)."""
        if (slot.info or {}).get("adc_type") == ADC_TYPE_INA:
            slot._sample_rate = int(self.emg_rate_select.currentText())
            slot._imu_rate = int(self.imu_rate_select.currentText())

    def _on_start_selected(self):
        slot = self._selected_slot()
        if slot and slot.is_connected and not slot.is_streaming:
            self._apply_rates(slot)
            slot.start()
            self._refresh_device_list()

    def _on_stop_selected(self):
        slot = self._selected_slot()
        if slot and slot.is_streaming:
            slot.stop()
            self._refresh_device_list()

    def _reconnect_slot(self, slot) -> bool:
        """Reconnect one slot (used by the button and by auto-reconnect)."""
        was_streaming = slot.is_streaming
        self._lsl.remove(slot)        # drop stale LSL outlets; recreated on demand
        slot.disconnect()             # clears the (possibly stale) clock offset
        ok = slot.connect()
        if ok:
            if was_streaming:
                slot.start()          # start() re-syncs the clock
            else:
                try:
                    slot.device.sync_clock()   # idle reconnect: still re-sync
                except Exception:
                    pass
        return ok

    def _spawn_reconnect(self, slot):
        """Reconnect a slot on a background thread (network ops only) so the GUI
        never stalls. Retries are driven by intent in _tick."""
        slot._reconnecting = True
        emg = imu = None
        if (slot.info or {}).get("adc_type") == ADC_TYPE_INA:
            emg = int(self.emg_rate_select.currentText())
            imu = int(self.imu_rate_select.currentText())

        def work():
            try:
                slot.reconnect_network(emg, imu, timeout=2.0)
            except Exception as e:
                print(f"[{slot.name}] reconnect error: {e!r}")
            finally:
                slot._reconnecting = False

        threading.Thread(target=work, daemon=True).start()

    def _on_reconnect_selected(self):
        slot = self._selected_slot()
        if not slot:
            return
        self.status.showMessage(f"Reconnecting {slot.ip}…")
        QApplication.processEvents()
        ok = self._reconnect_slot(slot)
        self.status.showMessage(f"Reconnected {slot.ip}" if ok
                                else f"Reconnect failed: {slot.ip}")
        self._refresh_device_list()

    def _on_scan(self):
        if getattr(self, "_scan_worker", None) and self._scan_worker.isRunning():
            return
        self.status.showMessage("Scanning…")
        self.btn_scan.setEnabled(False)
        self._scan_worker = DiscoverWorker(2.0)
        self._scan_worker.done.connect(self._on_scan_done)
        self._scan_worker.start()

    def _on_scan_done(self, devs: list):
        self.btn_scan.setEnabled(True)
        if not devs:
            self.status.showMessage("No devices found")
            return
        ips = [d["ip"] for d in devs]
        connected = {s.ip for s in self.plot.slots}
        new_ips = [ip for ip in ips if ip not in connected]
        if not new_ips:
            self.status.showMessage(f"Found {len(devs)} device(s), all already connected")
            return
        if len(new_ips) == 1:
            self.ip_edit.setText(new_ips[0])
            self.status.showMessage(f"Found {new_ips[0]} — click Add Device")
        else:
            # Offer to connect to all
            reply = QMessageBox.question(
                self, "Multiple devices found",
                f"Found {len(new_ips)} new devices:\n" + "\n".join(new_ips) +
                "\n\nConnect to all?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
            )
            if reply == QMessageBox.StandardButton.Yes:
                for ip in new_ips:
                    self.ip_edit.setText(ip)
                    self._on_add_device()

    def _on_start_all(self):
        started = 0
        for slot in self.plot.slots:
            if slot.is_connected and not slot.is_streaming:
                self._apply_rates(slot)
                if slot.start():
                    started += 1
        self.plot.clear_data()
        self.status.showMessage(f"Started {started} device(s)")

    def _on_stop_all(self):
        for slot in self.plot.slots:
            if slot.is_streaming:
                slot.stop()
        self.status.showMessage("Stopped all")
        self._stop_recording()

    def _on_view_changed(self, idx: int):
        """EMG / Accel / Gyro / All"""
        show_emg   = idx in (0, 3)
        show_accel = idx in (1, 3)
        show_gyro  = idx in (2, 3)
        self.plot.setVisible(show_emg)
        self.plot_accel.setVisible(show_accel)
        self.plot_gyro.setVisible(show_gyro)

    def _on_auto_scale(self):
        self.plot.manual_rescale()

    def _on_load_yaml(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Load Config", "", "YAML (*.yaml *.yml)"
        )
        if not path:
            return
        try:
            cfg = load_config(path)
            emg = cfg.get("emg", {})
            sr = int(get_sample_rate(emg))
            # Apply to all connected devices
            for slot in self.plot.slots:
                slot._sample_rate = sr
                if slot.is_connected:
                    slot.device._apply_emg_config(emg)
            self.status.showMessage(f"Config loaded → all devices: {sr} Hz")
        except Exception as e:
            QMessageBox.critical(self, "Config error", str(e))

    def _on_load_session(self):
        """Connect to every session.devices entry and apply names, per-device
        EMG (ADS1293 only), markers, recording prefs, and the default view."""
        path, _ = QFileDialog.getOpenFileName(
            self, "Load Session", "", "YAML (*.yaml *.yml)")
        if not path:
            return
        try:
            cfg = load_raw(path)
        except Exception as e:
            QMessageBox.critical(self, "Config error", str(e))
            return

        # Recording prefs
        rec = get_recording(cfg)
        self._rec_format = rec["format"]
        self._include_imu = rec["include_imu"]
        want = "HDF5" if self._rec_format.lower() in ("hdf5", "h5") else "CSV"
        i = self.rec_format.findText(want)
        if i >= 0:
            self.rec_format.setCurrentIndex(i)

        # Experiment metadata (saved into recordings) + markers
        self._experiment = get_experiment(cfg)
        markers = self._experiment.get("markers")
        if isinstance(markers, dict) and markers:
            self._marker_map = {str(k): str(v) for k, v in markers.items()}
            self._rebuild_marker_bar()

        # GUI default view (window_sec/display_hz/channel_spacing are
        # construction-time constants — not reconfigured live)
        view = get_gui(cfg)["default_view"]
        vi = self.view_select.findText(view)
        if vi >= 0:
            self.view_select.setCurrentIndex(vi)

        # Connect to each device and apply per-device settings
        connected = 0
        for d in iter_session_devices(cfg):
            ip = d["ip"]
            slot = next((s for s in self.plot.slots if s.ip == ip), None)
            if slot is None:
                slot = DeviceSlot(ip, len(self.plot.slots))
                self.status.showMessage(f"Connecting {ip}…")
                QApplication.processEvents()
                if not slot.connect():
                    self.status.showMessage(f"Failed to connect {ip}")
                    continue
                self.plot.add_slot(slot)
                self.plot_accel.add_slot(slot)
                self.plot_gyro.add_slot(slot)
                connected += 1
            if d.get("name"):
                slot.name = d["name"]
            # EMG register config + configured rate only apply to ADS1293 boards;
            # INA/LIS3DH nodes keep their device-reported rate from connect().
            if (slot.info or {}).get("adc_type") == ADC_TYPE_ADS1293:
                emg = d["emg"]
                slot._sample_rate = int(get_sample_rate(emg))
                try:
                    slot.device._apply_emg_config(emg)
                except Exception as e:
                    print(f"[session] EMG apply failed for {ip}: {e!r}")

        self._refresh_device_list()
        self.status.showMessage(
            f"Session loaded: {connected} device(s) connected")

    # ---- Markers (host-side annotation) ----
    def _rebuild_marker_bar(self):
        """(Re)populate the marker button bar from self._marker_map."""
        while self._marker_layout.count():
            item = self._marker_layout.takeAt(0)
            w = item.widget()
            if w:
                w.deleteLater()
        self._marker_layout.addWidget(QLabel("Markers:"))
        for key in sorted(self._marker_map):
            label = self._marker_map[key]
            btn = QPushButton(f"{key}: {label}")
            btn.setToolTip(f"Press '{key}' to mark “{label}”")
            btn.clicked.connect(
                lambda _=False, l=label, k=key: self._fire_marker(l, k))
            self._marker_layout.addWidget(btn)
        self._marker_layout.addStretch(1)

    def _fire_marker(self, label: str, key: str = ""):
        """Stamp one host timestamp and fan the marker to recording + LSL."""
        host_time = time.time()
        sinks = []
        if self._recording and self._recorder is not None:
            self._recorder.write_marker(host_time, label, key)
            sinks.append("rec")
        if self._lsl.enabled:
            self._lsl.push_marker(label)
            sinks.append("LSL")
        where = "+".join(sinks) if sinks else "no active sink (record or LSL first)"
        self.status.showMessage(f"Marker “{label}” → {where}", 1500)

    def keyPressEvent(self, event):
        # Digit keys fire markers — but only when a text field doesn't have focus
        # (QLineEdit consumes digits, so typing an IP won't trigger a marker).
        t = event.text()
        if t in self._marker_map:
            self._fire_marker(self._marker_map[t], t)
        else:
            super().keyPressEvent(event)

    def _on_lsl_toggled(self, checked: bool):
        if checked:
            if not self._lsl.available:
                QMessageBox.warning(
                    self, "LSL unavailable",
                    "pylsl is not installed.\n\nInstall it with:\n"
                    "    pip install pylsl")
                self.btn_lsl.setChecked(False)
                return
            self._lsl.enable()
            # Outlets are created lazily in _tick for streaming devices.
            n = sum(1 for s in self.plot.slots if s.is_streaming)
            self.status.showMessage(
                f"LSL on — streaming {n} device(s) "
                f"(EMG + IMU outlets). Start devices to add more.")
        else:
            self._lsl.disable()
            self.status.showMessage("LSL off")

    def _on_record(self):
        if self._recording:
            self._stop_recording()
            return
        fmt = self.rec_format.currentText().lower()   # "csv" | "hdf5"
        ext = "h5" if fmt == "hdf5" else "csv"
        filt = "HDF5 (*.h5)" if fmt == "hdf5" else "CSV (*.csv)"
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Recording",
            f"monomod_session_{int(time.time())}.{ext}", filt)
        if not path:
            return
        meta = {k: v for k, v in self._experiment.items()
                if k in ("subject_id", "session_id", "operator", "notes")}
        meta["created_s"] = time.time()
        meta["devices"] = ",".join(s.name for s in self.plot.slots)
        try:
            self._recorder = make_recorder(fmt, path, self.plot.slots, meta)
        except Exception as e:
            QMessageBox.critical(self, "Recording error", str(e))
            return
        self._recording = True
        self.btn_record.setText("Stop Recording")
        self.status.showMessage(f"Recording ({fmt}) → {path}")

    def _stop_recording(self):
        if self._recorder is not None:
            n = self._recorder.count
            try:
                self._recorder.close()
            except Exception as e:
                print(f"[rec] close error: {e!r}")
            self._recorder = None
            self.status.showMessage(f"Saved {n} rows")
        self._recording = False
        self.btn_record.setText("Record")

    # ---- Tick ----
    def _tick(self):
        # Drain all devices and push to plot buffers
        host_time = time.time()
        any_data = False
        for slot in self.plot.slots:
            if not slot.is_streaming:
                continue
            full_arr = slot.drain_push()           # EMG chunk (GUI thread)
            imu_rows = slot.drain_imu()             # IMU rows  (GUI thread)
            any_data = any_data or (full_arr is not None)

            # LSL: ensure outlets, then push EMG chunk + each IMU sample.
            if self._lsl.enabled:
                self._lsl.ensure(slot)
                if slot._lsl is not None:
                    if full_arr is not None:
                        slot._lsl.push_emg(full_arr, slot.num_channels)
                    for r in imu_rows:
                        slot._lsl.push_imu(r[:3], r[3:6])

            # Recording
            if self._recording and self._recorder is not None:
                if full_arr is not None:
                    self._recorder.write_emg(slot, full_arr, host_time)
                if imu_rows and self._include_imu:
                    self._recorder.write_imu(slot, imu_rows, host_time)

        if any_data:
            self.plot.redraw()

        # IMU plots redraw every tick (cheap — ≤500 pts per device)
        if any(s.is_streaming for s in self.plot.slots):
            self.plot_accel.redraw()
            self.plot_gyro.redraw()

        # Per-second status update
        now = time.monotonic()
        if now - self._last_stat_time >= 1.0:
            self._last_stat_time = now

            # Auto-reconnect (opt-in): retry by INTENT, off the GUI thread, until
            # the device is back — survives a power-pull even if it rejoins slowly.
            if self.chk_autoreconnect.isChecked():
                for slot in list(self.plot.slots):
                    if not slot._want_stream or slot._reconnecting:
                        continue
                    down = (not slot.is_streaming
                            or (now - slot._last_data_t) > 3.0)
                    if down and now >= slot._next_reconnect_t:
                        slot._next_reconnect_t = now + 2.0   # throttle attempts
                        self.status.showMessage(f"Auto-reconnecting {slot.ip}…")
                        self._spawn_reconnect(slot)

            # Re-sync clocks for streaming devices that lost sync (e.g. after a
            # reconnect/reboot). Guarded by recent data + not-reconnecting so we
            # never block on a dead device or race the reconnect worker.
            for slot in self.plot.slots:
                if (slot.is_streaming and not slot._reconnecting
                        and slot.device.clock_offset_s is None
                        and (now - slot._last_data_t) < 2.0):
                    try:
                        slot.device.sync_clock()
                    except Exception as e:
                        print(f"[{slot.name}] re-sync failed: {e!r}")

            n_conn = sum(1 for s in self.plot.slots if s.is_connected)
            n_stream = sum(1 for s in self.plot.slots if s.is_streaming)
            rec = (f" | REC {self._recorder.count}"
                   if self._recording and self._recorder else "")
            self.status.showMessage(f"Devices: {n_conn} connected, {n_stream} streaming{rec}")
            self._refresh_device_list()

    def closeEvent(self, event):
        self._stop_recording()
        self._lsl.disable()
        for slot in self.plot.slots:
            slot.disconnect()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
