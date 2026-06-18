"""Recorder backends: unified CSV + HDF5 (h5py optional)."""
import os
import tempfile

import numpy as np
import pytest

import recorder


class _Dev:
    clock_offset_s = 0.0


class _Slot:
    name = "d0"
    ip = "1.1.1.1"
    num_channels = 1
    _sample_rate = 1000
    info = {"adc_type": 4}
    device = _Dev()


def test_csv_unified():
    p = tempfile.mktemp(suffix=".csv")
    r = recorder.make_recorder("csv", p, [_Slot()], {"subject_id": "S01"},
                               threaded=False)
    r.write_emg(_Slot(), np.array([[7], [8]], dtype=np.int32), 1.0)
    r.write_imu(_Slot(), [(0.1, 0.2, 0.98, 0, 0, 0, 111)], 1.0)
    r.write_marker(1.0, "go", "1")
    r.close()
    txt = open(p).read()
    os.remove(p)
    assert "# subject_id: S01" in txt
    assert ",emg," in txt and ",imu," in txt and ",marker," in txt
    assert "go" in txt
    assert r.count == 4   # 2 emg + 1 imu + 1 marker


def test_threaded_csv():
    p = tempfile.mktemp(suffix=".csv")
    r = recorder.make_recorder("csv", p, [_Slot()], None)   # threaded=True default
    r.write_emg(_Slot(), np.array([[1]], dtype=np.int32), 1.0)
    r.close()   # joins worker, flushes, closes
    assert os.path.exists(p)
    os.remove(p)


@pytest.mark.skipif(not recorder.HAVE_H5PY, reason="h5py not installed")
def test_hdf5():
    import h5py
    p = tempfile.mktemp(suffix=".h5")
    r = recorder.make_recorder("hdf5", p, [_Slot()], {"subject_id": "S01"},
                               threaded=False)
    r.write_emg(_Slot(), np.array([[1], [2], [3]], dtype=np.int32), 1.0)
    r.write_marker(1.0, "go", "1")
    r.close()
    with h5py.File(p) as f:
        assert f["d0"]["emg"].shape == (3, 1)
        assert "markers" in f
        assert f.attrs["subject_id"] == "S01"
    os.remove(p)
