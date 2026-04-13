"""
YAML configuration loader for MONOMOD devices.

Loads device config from YAML, validates, and translates
to ADS1293 register values.
"""

import copy
import yaml
from . import ads1293


def load_config(path: str) -> dict:
    """Load and validate a MONOMOD YAML config file."""
    with open(path) as f:
        cfg = yaml.safe_load(f)

    # Merge with defaults
    result = {
        "device": cfg.get("device", {}),
        "wifi": cfg.get("wifi", {}),
        "emg": _merge_emg(cfg.get("emg", {})),
        "imu": cfg.get("imu", {"enabled": True, "rate_hz": 100}),
    }
    return result


def _merge_emg(emg: dict) -> dict:
    """Merge EMG config with defaults (deep copy so nested dicts are safe)."""
    defaults = copy.deepcopy(ads1293.DEFAULT_CONFIG)

    if "channels" in emg:
        for key in ("ch0", "ch1", "ch2"):
            if key in emg["channels"]:
                ch_cfg = emg["channels"][key]
                defaults["channels"][key] = {
                    "enabled": ch_cfg.get("enabled", True),
                    "pos_input": ch_cfg.get("pos_input", 1),
                    "neg_input": ch_cfg.get("neg_input", 2),
                }

    if "high_res" in emg:
        defaults["high_res"] = emg["high_res"]
    if "high_freq" in emg:
        defaults["high_freq"] = emg["high_freq"]

    if "filters" in emg:
        f = emg["filters"]
        for k in ("R1", "R2", "R3"):
            if k in f:
                defaults["filters"][k] = f[k]

    if "clock" in emg:
        defaults["clock"] = emg["clock"]

    if "rld" in emg:
        r = emg["rld"]
        for k in ("route", "bw_high", "cap_drive"):
            if k in r:
                defaults["rld"][k] = r[k]

    return defaults


def get_sample_rate(emg_config: dict) -> float:
    """Compute sample rate from EMG config."""
    f = emg_config.get("filters", {})
    r1 = f.get("R1", 2)
    r2 = f.get("R2", 4)
    r3 = f.get("R3", 4)
    hf = emg_config.get("high_freq", False)
    # R1/R3 can be per-channel (list) or global (int)
    if isinstance(r1, list):
        r1 = r1[0]
    if isinstance(r3, list):
        r3 = r3[0]
    return ads1293.compute_sample_rate(r1, r2, r3, hf)


def load_session(path: str) -> dict:
    """Load a multi-device session YAML config."""
    with open(path) as f:
        cfg = yaml.safe_load(f)
    return cfg.get("session", cfg)
