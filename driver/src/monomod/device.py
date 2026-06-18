"""
High-level Device class for MONOMOD wireless EMG modules.

Usage:
    from monomod import Device, discover_devices

    devices = discover_devices()
    dev = Device()
    dev.connect(devices[0]["ip"])
    dev.set_data_callback(lambda samples, ts, seq: print(samples.shape))
    dev.start()
    ...
    dev.stop()
    dev.disconnect()
"""

import threading
from collections import deque
from typing import Callable, Optional

import numpy as np

from .control import TcpControl
from . import receiver
from .receiver import ReceiverHandle
from .discovery import discover_devices as _discover
from .config import load_config, get_sample_rate
from . import protocol as proto
from . import ads1293


class Device:
    """High-level interface to a single MONOMOD device."""

    def __init__(self):
        self._ctrl = TcpControl()
        self._rx: Optional[ReceiverHandle] = None
        self._ip: Optional[str] = None
        self._info: Optional[dict] = None
        self._streaming = False

        # Callbacks
        self._data_cb: Optional[Callable] = None
        self._imu_cb: Optional[Callable] = None
        self._status_cb: Optional[Callable] = None
        self.latest_status: Optional[dict] = None

        # Clock sync: host_wall ≈ device_ts_us*1e-6 + clock_offset_s
        self.clock_offset_s: Optional[float] = None
        self.clock_rtt_s: Optional[float] = None

        # Thread-safe sample queue for GUI consumption
        self._queue: deque = deque(maxlen=8192)
        self._queue_lock = threading.Lock()

        # Stats
        self.packets_received = 0
        self.packets_lost = 0
        self._last_seq = -1

        # ADC max for conversion
        self._fast_adc_max = 0x8000
        self._precise_adc_max = [0x800000] * 3

    @property
    def packets_invalid(self) -> int:
        """UDP packets that arrived but failed magic/CRC check."""
        return self._rx.packets_invalid if self._rx else 0

    @property
    def packets_raw(self) -> int:
        """Total UDP packets received by the OS socket (valid + invalid)."""
        return self._rx.packets_received + self.packets_invalid if self._rx else 0

    # ---- Discovery ----

    @staticmethod
    def discover(timeout_s: float = 3.0) -> list[dict]:
        """Find MONOMOD devices on the network."""
        return _discover(timeout_s)

    # ---- Connection ----

    def connect(self, ip: str, timeout: float = 5.0) -> bool:
        """Connect to a device by IP address."""
        if not self._ctrl.connect(ip, timeout=timeout):
            return False

        # Register with the shared UDP receiver (demuxes by this device's IP)
        self._ip = ip
        self._rx = receiver.register(ip, on_data=self._on_data,
                                     on_imu=self._on_imu,
                                     on_status=self._on_status)

        # Fetch device info
        self._info = self._ctrl.get_info()
        return True

    def disconnect(self):
        """Disconnect from the device."""
        if self._streaming:
            self.stop()
        if self._rx:
            receiver.unregister(self._ip)
            self._rx = None
        self._ctrl.close()
        self._info = None
        # Drop the clock offset — the device may reboot (esp_timer resets to 0),
        # so a stale offset would be wrong. Re-synced on the next connect/start.
        self.clock_offset_s = None
        self.clock_rtt_s = None

    @property
    def is_connected(self) -> bool:
        return self._ctrl.is_connected

    @property
    def is_streaming(self) -> bool:
        return self._streaming

    @property
    def info(self) -> dict | None:
        return self._info

    # ---- Configuration ----

    def configure_from_yaml(self, path: str):
        """Load YAML config and apply to device via register writes."""
        cfg = load_config(path)
        emg = cfg.get("emg", {})
        self._apply_emg_config(emg)

    def _apply_emg_config(self, emg: dict):
        """Apply EMG config by writing ADS1293 registers via TCP.

        Currently applied:
          - local ADC max values (for host-side voltage conversion)
          - RLD_CN (0x0C) via WRITE_REGISTER so the firmware routes the
            right-leg-drive to the configured input pin
        """
        # Update local ADC max values
        f = emg.get("filters", {})
        r2 = f.get("R2", 4)
        r3 = f.get("R3", 4)
        if isinstance(r3, list):
            r3 = r3[0]
        self._fast_adc_max, pm = ads1293.compute_adc_max(r2, r3)
        self._precise_adc_max = [pm] * 3

        # Right-Leg Drive routing — sent to device via WRITE_REGISTER
        rld = emg.get("rld", {})
        route     = int(rld.get("route", 0))
        bw_high   = bool(rld.get("bw_high", False))
        cap_drive = int(rld.get("cap_drive", 0))
        try:
            rld_cn = ads1293.compute_rld_cn(route, bw_high, cap_drive)
        except ValueError as e:
            print(f"[monomod] RLD config invalid: {e}")
            return
        if self.is_connected:
            ok = self.write_register(ads1293.RLD_CN_REG, rld_cn)
            human = f"IN{route}" if route else "OFF"
            print(f"[monomod] RLD -> {human}  (RLD_CN=0x{rld_cn:02X}, "
                  f"bw_high={bw_high}, cap_drive={cap_drive})  "
                  f"{'ok' if ok else 'FAILED'}")

    # ---- Streaming ----

    def start(self, sample_rate: int = 3200, ch_mask: int = 0x07) -> bool:
        """Start EMG data streaming."""
        if not self._ctrl.is_connected:
            return False
        self._last_seq = -1
        self.packets_received = 0
        self.packets_lost = 0
        ok = self._ctrl.start(sample_rate, ch_mask)
        if ok:
            self._streaming = True
        return ok

    def stop(self) -> bool:
        """Stop EMG data streaming."""
        ok = self._ctrl.stop()
        self._streaming = False
        return ok

    def enable_imu(self, rate_hz: int = 100) -> bool:
        """Enable IMU streaming at rate_hz (assumes a 1 kHz base divider)."""
        rate_div = max(1, 1000 // rate_hz)
        return self._ctrl.set_imu(True, rate_div)

    def disable_imu(self) -> bool:
        """Disable IMU streaming."""
        return self._ctrl.set_imu(False)

    def set_imu(self, enable: bool = True, rate_div: int = 1) -> bool:
        """Low-level SET_IMU: on the LIS3DH node, rate_div = number of EMG samples
        per emitted accel sample (i.e. accel_rate = emg_rate / rate_div)."""
        return self._ctrl.set_imu(enable, max(1, int(rate_div)))

    # ---- Callbacks ----

    def set_data_callback(self, cb: Callable):
        """Set callback: cb(samples: np.ndarray[n,3], timestamp_us: int, seq: int)"""
        self._data_cb = cb

    def set_imu_callback(self, cb: Callable):
        """Set callback: cb(imu_data: dict, timestamp_us: int)"""
        self._imu_cb = cb

    def set_status_callback(self, cb: Callable):
        """Set callback: cb(status: dict, timestamp_us: int)"""
        self._status_cb = cb

    # ---- Queue-based access (for GUI) ----

    def drain(self) -> tuple[Optional[np.ndarray], float]:
        """Drain all queued samples. Returns (samples[N,3], latency_ms).

        Returns (None, 0) if no data available.
        """
        with self._queue_lock:
            if not self._queue:
                return None, 0.0
            items = list(self._queue)
            self._queue.clear()

        arrays = [item[0] for item in items]
        combined = np.vstack(arrays)
        latency = items[-1][1]  # latest latency
        return combined, latency

    # ---- Register access ----

    def read_register(self, addr: int) -> int | None:
        return self._ctrl.read_register(addr)

    def write_register(self, addr: int, value: int) -> bool:
        return self._ctrl.write_register(addr, value)

    # ---- WiFi ----

    def set_wifi(self, ssid: str, password: str) -> bool:
        return self._ctrl.set_wifi(ssid, password)

    def reboot(self):
        self._ctrl.reboot()

    def ping(self) -> float | None:
        return self._ctrl.ping()

    def sync_clock(self, n: int = 5) -> bool:
        """Estimate the device→host clock offset using the lowest-RTT of n pings
        (NTP-style). Sets clock_offset_s / clock_rtt_s. Returns True on success."""
        import time
        best_rtt = None
        for _ in range(max(1, n)):
            res = self._ctrl.ping_clock()
            if res is None:
                continue
            rtt, dev_ts = res
            if best_rtt is None or rtt < best_rtt:
                best_rtt = rtt
                # device stamped ~rtt/2 before this reply arrived
                host_at_device = time.time() - rtt / 2.0
                self.clock_offset_s = host_at_device - dev_ts * 1e-6
        if best_rtt is None:
            return False
        self.clock_rtt_s = best_rtt
        return True

    def device_to_host(self, ts_us: int) -> float | None:
        """Map a device µs timestamp to host wall-clock seconds (None if unsynced).
        Note: device ts is 32-bit µs (wraps ~71 min); valid near the sync time."""
        if self.clock_offset_s is None:
            return None
        return ts_us * 1e-6 + self.clock_offset_s

    # ---- Internal callbacks ----

    def _on_data(self, samples: np.ndarray, timestamp_us: int, seq: int):
        """Called by UdpReceiver thread when data arrives."""
        self.packets_received += 1

        # Track packet loss
        if self._last_seq >= 0:
            expected = (self._last_seq + 1) & 0xFFFF
            if seq != expected:
                gap = (seq - expected) & 0xFFFF
                self.packets_lost += gap
        self._last_seq = seq

        # Push to queue for GUI drain
        import time
        latency_ms = time.monotonic() * 1000  # placeholder
        with self._queue_lock:
            self._queue.append((samples, latency_ms))

        # User callback
        if self._data_cb:
            self._data_cb(samples, timestamp_us, seq)

    def _on_imu(self, imu_data: dict, timestamp_us: int):
        """Called by the RX thread when IMU data arrives."""
        if self._imu_cb:
            self._imu_cb(imu_data, timestamp_us)

    def _on_status(self, status: dict, timestamp_us: int):
        """Called by the RX thread when a STATUS packet arrives."""
        self.latest_status = status
        if self._status_cb:
            self._status_cb(status, timestamp_us)
