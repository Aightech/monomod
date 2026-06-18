"""
Recording backends for the MONOMOD GUI.

Two formats behind one interface:
  - CsvRecorder  : one unified long-format CSV (EMG + IMU + marker rows)
  - Hdf5Recorder : per-device groups with resizable emg/imu datasets + markers

Common interface:
    rec = make_recorder(fmt, path, slots, meta)
    rec.write_emg(slot, arr, host_time)     # arr: int32 [n_samples, >=nch]
    rec.write_imu(slot, rows, host_time)    # rows: [(ax,ay,az,gx,gy,gz,ts_us), ...]
    rec.write_marker(host_time, label, key) # session-wide annotation
    rec.close()
    rec.count                               # rows/samples written

host_time is the wall-clock time (time.time()) of the most recent sample in the
batch; per-sample times are back-filled from the device sample rate / IMU rate so
all streams share one clock.
"""

import csv
import queue
import threading
import time

try:
    import h5py
    HAVE_H5PY = True
except ImportError:
    HAVE_H5PY = False

IMU_RATE_HZ = 100.0
_MAX_CH = 3  # ch0..ch2 columns in the CSV (INA uses ch0 only)


class Recorder:
    def __init__(self):
        self.count = 0

    def write_emg(self, slot, arr, host_time): ...
    def write_imu(self, slot, rows, host_time): ...
    def write_marker(self, host_time, label, key=""): ...
    def flush(self): ...
    def close(self): ...


class CsvRecorder(Recorder):
    """Single long-format CSV. `stream` column distinguishes emg/imu/marker rows."""

    HEADER = (["host_time_s", "device", "stream", "device_ts_us"]
              + [f"ch{i}" for i in range(_MAX_CH)]
              + ["ax", "ay", "az", "gx", "gy", "gz", "marker"])

    def __init__(self, path, meta=None):
        super().__init__()
        self._f = open(path, "w", newline="")
        for k, v in (meta or {}).items():       # experiment metadata as # comments
            self._f.write(f"# {k}: {v}\n")
        self._w = csv.writer(self._f)
        self._w.writerow(self.HEADER)

    def write_emg(self, slot, arr, host_time):
        n = arr.shape[0]
        nch = slot.num_channels
        sr = max(1, slot._sample_rate)
        for k in range(n):
            t = host_time - (n - 1 - k) / sr
            row = [f"{t:.6f}", slot.name, "emg", ""]
            row += [int(arr[k, c]) if c < nch else "" for c in range(_MAX_CH)]
            row += ["", "", "", "", "", "", ""]   # ax..gz, marker
            self._w.writerow(row)
            self.count += 1

    def write_imu(self, slot, rows, host_time):
        m = len(rows)
        for j, r in enumerate(rows):
            t = host_time - (m - 1 - j) / IMU_RATE_HZ
            row = [f"{t:.6f}", slot.name, "imu", int(r[6])]
            row += ["", "", ""]                    # ch0..ch2
            row += [f"{r[0]:.6f}", f"{r[1]:.6f}", f"{r[2]:.6f}",
                    f"{r[3]:.4f}", f"{r[4]:.4f}", f"{r[5]:.4f}", ""]
            self._w.writerow(row)
            self.count += 1

    def write_marker(self, host_time, label, key=""):
        row = [f"{host_time:.6f}", "ALL", "marker", ""]
        row += ["", "", "", "", "", "", "", "", ""]   # ch0..gz
        row += [label]
        self._w.writerow(row)
        self.count += 1

    def flush(self):
        if self._f:
            self._f.flush()

    def close(self):
        if self._f:
            self._f.close()
            self._f = None


class Hdf5Recorder(Recorder):
    """Per-device groups: /<name>/emg [N,nch], /<name>/imu [N,6], /markers."""

    def __init__(self, path, meta=None):
        super().__init__()
        self._f = h5py.File(path, "w")
        for k, v in (meta or {}).items():
            try:
                self._f.attrs[k] = v
            except (TypeError, ValueError):
                self._f.attrs[k] = str(v)
        self._groups = {}   # slot.name -> h5py.Group
        self._str_dt = h5py.string_dtype(encoding="utf-8")
        self._mk_t = None   # marker datasets created lazily
        self._mk_label = None

    def _group(self, slot):
        g = self._groups.get(slot.name)
        if g is not None:
            return g
        g = self._f.create_group(slot.name)
        nch = slot.num_channels
        g.create_dataset("emg", shape=(0, nch), maxshape=(None, nch),
                         dtype="int32", chunks=(1024, nch))
        g.create_dataset("emg_t", shape=(0,), maxshape=(None,),
                         dtype="f8", chunks=(1024,))
        g.create_dataset("imu", shape=(0, 6), maxshape=(None, 6),
                         dtype="f4", chunks=(256, 6))
        g.create_dataset("imu_t", shape=(0,), maxshape=(None,),
                         dtype="f8", chunks=(256,))
        g.attrs["ip"] = slot.ip
        g.attrs["adc_type"] = (slot.info or {}).get("adc_type", 0)
        g.attrs["sample_rate"] = slot._sample_rate
        g.attrs["num_channels"] = nch
        # Clock-sync offset (host_wall ≈ device_ts_us*1e-6 + offset); 0 if unsynced
        g.attrs["clock_offset_s"] = float(getattr(slot.device, "clock_offset_s", None) or 0.0)
        self._groups[slot.name] = g
        return g

    @staticmethod
    def _append(ds, block):
        old = ds.shape[0]
        ds.resize(old + len(block), axis=0)
        ds[old:] = block

    def write_emg(self, slot, arr, host_time):
        g = self._group(slot)
        nch = slot.num_channels
        n = arr.shape[0]
        sr = max(1, slot._sample_rate)
        self._append(g["emg"], arr[:, :nch].astype("int32"))
        self._append(g["emg_t"], [host_time - (n - 1 - k) / sr for k in range(n)])
        self.count += n

    def write_imu(self, slot, rows, host_time):
        if not rows:
            return
        g = self._group(slot)
        m = len(rows)
        self._append(g["imu"], [r[:6] for r in rows])
        self._append(g["imu_t"], [host_time - (m - 1 - j) / IMU_RATE_HZ
                                  for j in range(m)])
        self.count += m

    def write_marker(self, host_time, label, key=""):
        if self._mk_t is None:
            mg = self._f.create_group("markers")
            self._mk_t = mg.create_dataset("t", shape=(0,), maxshape=(None,),
                                           dtype="f8", chunks=(64,))
            self._mk_label = mg.create_dataset("label", shape=(0,),
                                               maxshape=(None,),
                                               dtype=self._str_dt, chunks=(64,))
        self._append(self._mk_t, [host_time])
        self._append(self._mk_label, [label])
        self.count += 1

    def flush(self):
        if self._f:
            self._f.flush()

    def close(self):
        if self._f:
            self._f.close()
            self._f = None


class ThreadedRecorder:
    """Wraps a Recorder so writes happen on a background thread (off the GUI
    thread), with periodic flush. The GUI only enqueues drained chunks."""

    def __init__(self, inner, flush_interval=1.0):
        self._inner = inner
        self._flush_interval = flush_interval
        self._q = queue.Queue(maxsize=20000)
        self.dropped = 0
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    @property
    def count(self):
        return self._inner.count

    def _put(self, item):
        try:
            self._q.put_nowait(item)
        except queue.Full:
            self.dropped += 1

    def write_emg(self, slot, arr, host_time): self._put(("emg", slot, arr, host_time))
    def write_imu(self, slot, rows, host_time): self._put(("imu", slot, rows, host_time))
    def write_marker(self, host_time, label, key=""): self._put(("marker", host_time, label, key))

    def _apply(self, item):
        kind = item[0]
        if kind == "emg":
            self._inner.write_emg(item[1], item[2], item[3])
        elif kind == "imu":
            self._inner.write_imu(item[1], item[2], item[3])
        elif kind == "marker":
            self._inner.write_marker(item[1], item[2], item[3])

    def _run(self):
        last_flush = time.monotonic()
        while True:
            try:
                item = self._q.get(timeout=0.2)
            except queue.Empty:
                item = None
            if item is not None:
                if item[0] == "__stop__":
                    break
                try:
                    self._apply(item)
                except Exception as e:
                    print(f"[rec] write error: {e!r}")
            now = time.monotonic()
            if now - last_flush > self._flush_interval:
                last_flush = now
                try:
                    self._inner.flush()
                except Exception:
                    pass
        # Drain anything queued before stop, then close.
        while True:
            try:
                item = self._q.get_nowait()
            except queue.Empty:
                break
            if item[0] == "__stop__":
                continue
            try:
                self._apply(item)
            except Exception as e:
                print(f"[rec] write error (drain): {e!r}")
        try:
            self._inner.close()
        except Exception as e:
            print(f"[rec] close error: {e!r}")

    def close(self):
        self._put(("__stop__",))
        self._t.join(timeout=5.0)


def make_recorder(fmt, path, slots=None, meta=None, threaded=True):
    """Create a recorder for `fmt` ('csv' or 'hdf5'). Wrapped in a background
    writer thread unless threaded=False."""
    fmt = (fmt or "csv").lower()
    if fmt in ("hdf5", "h5"):
        if not HAVE_H5PY:
            raise RuntimeError("h5py is not installed (pip install h5py)")
        inner = Hdf5Recorder(path, meta)
    else:
        inner = CsvRecorder(path, meta)
    return ThreadedRecorder(inner) if threaded else inner
