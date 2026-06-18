"""
LSL (Lab Streaming Layer) bridge for the MONOMOD GUI.

Per connected device it opens two outlets:
  - <name>_EMG : N channels (raw ADC counts, float32) at the device sample rate
  - <name>_IMU : 6 channels (accel x/y/z in g, gyro x/y/z in dps) at the IMU rate

LIS3DH-only nodes have no gyroscope, so the gyro channels stream as zeros.

pylsl is an optional dependency (`pip install pylsl`); if it isn't installed,
LslManager.available is False and the GUI keeps working without LSL.

Threading: EMG is pushed from the GUI thread (drained chunks), IMU from the UDP
receiver thread (one sample per packet). Each outlet is therefore written by a
single thread, which is the supported usage.
"""

from __future__ import annotations

try:
    from pylsl import StreamInfo, StreamOutlet, local_clock
    HAVE_LSL = True
except ImportError:  # pylsl not installed
    HAVE_LSL = False


IMU_RATE_HZ = 100  # nominal IMU rate advertised on the LSL stream


class _DeviceStreams:
    """The pair of LSL outlets backing one device."""

    def __init__(self, name: str, ip: str, num_channels: int, emg_rate: float):
        self.emg = self._make_emg(name, ip, num_channels, emg_rate)
        self.imu = self._make_imu(name, ip)

    @staticmethod
    def _make_emg(name, ip, nch, rate):
        info = StreamInfo(f"MONOMOD_{name}_EMG", "EMG", nch, float(rate),
                          "float32", f"monomod-{ip}-emg")
        chans = info.desc().append_child("channels")
        for i in range(nch):
            c = chans.append_child("channel")
            c.append_child_value("label", f"{name}_emg{i}")
            c.append_child_value("unit", "raw")      # raw ADC counts
            c.append_child_value("type", "EMG")
        return StreamOutlet(info)

    @staticmethod
    def _make_imu(name, ip):
        info = StreamInfo(f"MONOMOD_{name}_IMU", "IMU", 6, float(IMU_RATE_HZ),
                          "float32", f"monomod-{ip}-imu")
        chans = info.desc().append_child("channels")
        meta = [("accel_x", "g", "ACC"), ("accel_y", "g", "ACC"),
                ("accel_z", "g", "ACC"), ("gyro_x", "dps", "GYR"),
                ("gyro_y", "dps", "GYR"), ("gyro_z", "dps", "GYR")]
        for label, unit, typ in meta:
            c = chans.append_child("channel")
            c.append_child_value("label", f"{name}_{label}")
            c.append_child_value("unit", unit)
            c.append_child_value("type", typ)
        return StreamOutlet(info)

    def push_emg(self, arr, nch: int):
        """arr: np.ndarray[n_samples, >=nch] of ADC counts."""
        if arr is None or arr.size == 0:
            return
        # One timestamp for the chunk; consumers interpolate via the nominal
        # rate. tolist() keeps us compatible with pylsl builds lacking numpy.
        self.emg.push_chunk(arr[:, :nch].astype("float32").tolist(), local_clock())

    def push_imu(self, accel_g, gyro_dps):
        row = [float(accel_g[0]), float(accel_g[1]), float(accel_g[2]),
               float(gyro_dps[0]), float(gyro_dps[1]), float(gyro_dps[2])]
        self.imu.push_sample(row, local_clock())


class LslManager:
    """Owns the per-device outlets and the on/off state for the whole GUI."""

    def __init__(self):
        self._on = False
        self._slots = set()   # DeviceSlots that currently have outlets
        self._markers = None  # session-wide marker outlet

    @property
    def available(self) -> bool:
        return HAVE_LSL

    @property
    def enabled(self) -> bool:
        return self._on

    def enable(self):
        self._on = HAVE_LSL
        if self._on and self._markers is None:
            # Session-wide marker stream: 1 string channel, irregular rate.
            info = StreamInfo("MONOMOD_markers", "Markers", 1, 0.0,
                              "string", "monomod-markers")
            self._markers = StreamOutlet(info)

    def disable(self):
        self._on = False
        for slot in self._slots:
            slot._lsl = None      # dropping the outlets closes the streams
        self._slots.clear()
        self._markers = None      # close the marker stream

    def push_marker(self, label: str):
        """Push a marker onto the session-wide LSL marker stream (if enabled)."""
        if self._markers is not None:
            self._markers.push_sample([str(label)], local_clock())

    def ensure(self, slot):
        """Create this slot's outlets if streaming and not already present."""
        if not self._on:
            return
        if getattr(slot, "_lsl", None) is None:
            slot._lsl = _DeviceStreams(slot.name, slot.ip,
                                       slot.num_channels, slot._sample_rate)
            self._slots.add(slot)

    def remove(self, slot):
        """Tear down a slot's outlets (on disconnect/removal/stop)."""
        if slot in self._slots:
            slot._lsl = None
            self._slots.discard(slot)
