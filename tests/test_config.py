"""Config merge, session iteration, and accessor defaults."""
from monomod import config


def test_merge_keeps_defaults():
    m = config._merge_emg({"filters": {"R3": 8}})
    assert m["filters"]["R3"] == 8
    assert m["filters"]["R1"] == 2   # default preserved


def test_iter_session_devices_merges_overrides():
    raw = {
        "emg": {"filters": {"R2": 5}},
        "session": {"devices": [
            {"ip": "1.2.3.4", "name": "a", "emg": {"filters": {"R3": 16}}},
            {"name": "no-ip"},   # skipped (no ip)
        ]},
    }
    devs = list(config.iter_session_devices(raw))
    assert len(devs) == 1
    assert devs[0]["ip"] == "1.2.3.4" and devs[0]["name"] == "a"
    assert devs[0]["emg"]["filters"]["R2"] == 5      # from top-level
    assert devs[0]["emg"]["filters"]["R3"] == 16     # per-device override


def test_accessor_defaults():
    assert config.get_recording({})["format"] == "csv"
    assert config.get_recording({})["include_imu"] is True
    assert config.get_gui({})["default_view"] == "EMG only"
    assert config.get_experiment({}) == {}
