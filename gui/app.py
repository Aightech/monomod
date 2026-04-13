#!/usr/bin/env python3
"""
MONOMOD GUI — Multi-device EMG streaming with one unified plot.

Connect to N devices, stream all EMG channels onto a single stacked plot,
record everything to one long-format CSV.
"""

import sys
import time
import csv
import threading

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QLineEdit, QFileDialog, QStatusBar, QMessageBox,
    QListWidget, QListWidgetItem, QInputDialog, QFrame, QComboBox,
)

from monomod import Device, discover_devices
from monomod.config import load_config, get_sample_rate


# =============================================================================
# Constants
# =============================================================================
MAX_CHANNELS_PER_DEVICE = 3   # upper bound for buffer allocation (ADS1293 has 3, INA has 1)
WINDOW_SEC = 5.0
DISPLAY_HZ = 200
TICK_MS = 33
CHANNEL_SPACING = 100.0

# IMU buffer sizing (ICM-20948 @ 100 Hz, 5 s window = 500 samples)
IMU_RATE_HZ = 100
IMU_BUFFER_N = int(WINDOW_SEC * IMU_RATE_HZ)

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

    def __init__(self, ip: str, device_idx: int):
        self.ip = ip
        self.device_idx = device_idx
        self.device = Device()
        self.name = f"dev{device_idx}"

        # num_channels is set after connect() based on device.info. Default 3.
        self.num_channels = 3

        n = int(WINDOW_SEC * DISPLAY_HZ)
        self._x = np.linspace(0, WINDOW_SEC, n, dtype=np.float32)
        self._y = np.zeros((n, MAX_CHANNELS_PER_DEVICE), dtype=np.float32)
        self._dc = np.zeros(MAX_CHANNELS_PER_DEVICE, dtype=np.float32)
        self._samples_written = 0
        self._sample_rate = 3200
        self._latest_raw = np.zeros(MAX_CHANNELS_PER_DEVICE, dtype=np.int32)
        self._imu_text = "no data yet"

        # IMU buffer: shape (IMU_BUFFER_N, 6) → accel_x/y/z (g), gyro_x/y/z (dps)
        self._imu_x = np.linspace(0, WINDOW_SEC, IMU_BUFFER_N, dtype=np.float32)
        self._imu_y = np.zeros((IMU_BUFFER_N, 6), dtype=np.float32)
        self._imu_written = 0

        # Stats
        self.info = None
        self.imu_count = 0
        self.data_count = 0

    def connect(self) -> bool:
        if not self.device.connect(self.ip):
            return False
        self.info = self.device.info
        if self.info:
            self.num_channels = min(int(self.info.get("num_channels", 3)),
                                    MAX_CHANNELS_PER_DEVICE)
        self.device.set_data_callback(self._on_data)
        self.device.set_imu_callback(self._on_imu)
        print(f"[{self.name}] connected to {self.ip}, "
              f"adc_type={self.info.get('adc_type') if self.info else '?'}, "
              f"channels={self.num_channels}")
        return True

    def disconnect(self):
        if self.device.is_connected:
            if self.device.is_streaming:
                self.device.stop()
            self.device.disconnect()

    def start(self) -> bool:
        if not self.device.is_connected:
            return False
        # reset EMG buffer
        self._y.fill(0)
        self._dc.fill(0)
        self._x = np.linspace(0, WINDOW_SEC, len(self._x), dtype=np.float32)
        self._samples_written = 0
        # reset IMU buffer
        self._imu_y.fill(0)
        self._imu_x = np.linspace(0, WINDOW_SEC, IMU_BUFFER_N, dtype=np.float32)
        self._imu_written = 0
        return self.device.start(sample_rate=self._sample_rate)

    def stop(self) -> bool:
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
        """Called from UDP receiver thread for each IMU packet."""
        try:
            self.imu_count += 1
            if self.imu_count == 1 or self.imu_count % 200 == 0:
                print(f"[{self.name}] IMU packet #{self.imu_count}: {imu}")
            a = imu.get("accel_g", [0.0, 0.0, 0.0])
            g = imu.get("gyro_dps", [0.0, 0.0, 0.0])
            self._imu_text = (f"Acc {a[0]:+.2f}/{a[1]:+.2f}/{a[2]:+.2f}g "
                              f"Gyr {g[0]:+.0f}/{g[1]:+.0f}/{g[2]:+.0f}dps")
            # Shift-register append (accel_g × 3, gyro_dps × 3)
            row = np.array([a[0], a[1], a[2], g[0], g[1], g[2]], dtype=np.float32)
            self._imu_y[:-1] = self._imu_y[1:]
            self._imu_y[-1] = row
            self._imu_x += 1.0 / float(IMU_RATE_HZ)
            self._imu_written += 1
        except Exception as e:
            print(f"[{self.name}] _on_imu ERROR: {e!r}")
            import traceback; traceback.print_exc()

    def drain_push(self, recorder=None):
        """Called in GUI thread at tick rate. Drains device data and updates buffer.
        Returns the drained array for recording (or None)."""
        arr, _ = self.device.drain()
        if arr is None or arr.size == 0:
            return None

        # Decimate for display
        decim = max(1, round(self._sample_rate / DISPLAY_HZ))
        nch = self.num_channels
        arr_ds = arr[::decim, :nch].astype(np.float32)

        dn = arr_ds.shape[0]
        n = self._y.shape[0]
        if dn >= n:
            self._y[:, :nch] = arr_ds[-n:, :]
            self._x[:] = np.linspace(0, WINDOW_SEC, n, dtype=np.float32)
        else:
            self._y[:-dn, :nch] = self._y[dn:, :nch]
            self._y[-dn:, :nch] = arr_ds
            self._x += dn / float(DISPLAY_HZ)
        self._samples_written += dn

        return arr  # full-resolution data for recording

    def compute_dc(self):
        """Recompute per-channel DC from full buffer (called by plot auto-scale)."""
        for i in range(self.num_channels):
            self._dc[i] = float(np.mean(self._y[:, i]))


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
            slot._y.fill(0)
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
                        slot._dc[i] = float(np.mean(slot._y[:, i]))
                        pp = float(np.ptp(slot._y[:, i] - slot._dc[i]))
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
                    x, (y - slot._dc[ch]) * ys + (lane_idx * CHANNEL_SPACING))
                lane_idx += 1

            if x_min_global is None or x[0] < x_min_global:
                x_min_global = float(x[0])
            if x_max_global is None or x[-1] > x_max_global:
                x_max_global = float(x[-1])

        # Follow the fastest-advancing device's X range
        if x_min_global is not None:
            xmin = round(x_min_global * 10.0) / 10.0
            xmax = round(x_max_global * 10.0) / 10.0
            self.setXRange(xmin, xmax, padding=0)


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
                curve.setData(x, y)
            if x_min is None or x[0] < x_min:
                x_min = float(x[0])
            if x_max is None or x[-1] > x_max:
                x_max = float(x[-1])
        if x_min is not None:
            self.setXRange(round(x_min * 10) / 10, round(x_max * 10) / 10, padding=0)


# =============================================================================
# Main Window
# =============================================================================
class MainWindow(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("MONOMOD EMG — Multi-Device")
        self.resize(1400, 800)

        # Recording state
        self._recording = False
        self._csv_writer = None
        self._csv_file = None
        self._rec_count = 0
        self._rec_lock = threading.Lock()
        self._last_stat_time = 0

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

        self.btn_record = QPushButton("Record")
        self.btn_record.clicked.connect(self._on_record)
        ctrl.addWidget(self.btn_record)

        ctrl.addStretch(1)
        root.addLayout(ctrl)

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
        for slot in self.plot.slots:
            status = "streaming" if slot.is_streaming else (
                "connected" if slot.is_connected else "disconnected"
            )
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
            text = (f"{slot.name}  {slot.ip}  ({slot.num_channels}ch)\n"
                    f"  {status}  data={slot.data_count}  imu={slot.imu_count}\n"
                    f"  last=[{last_str}]\n"
                    f"  IMU: {imu_line}")
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
        slot.disconnect()
        self.plot.remove_slot(slot)
        self.plot_accel.remove_slot(slot)
        self.plot_gyro.remove_slot(slot)
        self._refresh_device_list()

    def _on_scan(self):
        self.status.showMessage("Scanning…")
        QApplication.processEvents()
        devs = discover_devices(timeout_s=2.0)
        if not devs:
            self.status.showMessage("No devices found")
            return
        # Show all found devices in a dialog; user picks one or all
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

    def _on_record(self):
        if self._recording:
            self._stop_recording()
        else:
            path, _ = QFileDialog.getSaveFileName(
                self, "Save Recording",
                f"monomod_session_{int(time.time())}.csv",
                "CSV (*.csv)"
            )
            if path:
                self._csv_file = open(path, "w", newline="")
                self._csv_writer = csv.writer(self._csv_file)
                # Fixed-width CSV with up to MAX_CHANNELS_PER_DEVICE channels.
                # Empty trailing channels are written as blanks (e.g. INA has only ch0).
                header = ["host_time_s", "device", "adc_type", "device_ts_us"]
                header += [f"ch{i}" for i in range(MAX_CHANNELS_PER_DEVICE)]
                self._csv_writer.writerow(header)
                self._recording = True
                self._rec_count = 0
                self.btn_record.setText("Stop Recording")
                self.status.showMessage(f"Recording → {path}")

    def _stop_recording(self):
        with self._rec_lock:
            if self._csv_file:
                self._csv_file.close()
                self._csv_file = None
            self._csv_writer = None
            self._recording = False
        self.btn_record.setText("Record")
        if self._rec_count:
            self.status.showMessage(f"Saved {self._rec_count} samples")

    # ---- Tick ----
    def _tick(self):
        # Drain all devices and push to plot buffers
        host_time = time.time()
        any_data = False
        for slot in self.plot.slots:
            if not slot.is_streaming:
                continue
            full_arr = slot.drain_push()
            any_data = any_data or (full_arr is not None)

            # Recording (write full-resolution data)
            if full_arr is not None and self._recording:
                with self._rec_lock:
                    if self._csv_writer:
                        sr = max(1, slot._sample_rate)
                        n = full_arr.shape[0]
                        nch = slot.num_channels
                        adc_type = (slot.info or {}).get("adc_type", "")
                        for k in range(n):
                            t = host_time - (n - 1 - k) / sr
                            row = [f"{t:.6f}", slot.name, adc_type, ""]
                            for c in range(MAX_CHANNELS_PER_DEVICE):
                                row.append(int(full_arr[k, c]) if c < nch else "")
                            self._csv_writer.writerow(row)
                            self._rec_count += 1

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
            n_conn = sum(1 for s in self.plot.slots if s.is_connected)
            n_stream = sum(1 for s in self.plot.slots if s.is_streaming)
            rec = f" | REC {self._rec_count}" if self._recording else ""
            self.status.showMessage(f"Devices: {n_conn} connected, {n_stream} streaming{rec}")
            self._refresh_device_list()

    def closeEvent(self, event):
        self._stop_recording()
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
