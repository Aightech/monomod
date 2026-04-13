#!/usr/bin/env python3
"""
MONOMOD WiFi provisioning helper.

Reads WiFi credentials from a YAML file and sends them to a MONOMOD device
over USB serial. The firmware stores them in NVS and auto-connects on every
boot afterward.

The device accepts line-based commands over its USB CDC serial port:
    WIFI+ADD:<ssid>,<password>\\n   -> save credentials + connect
    WIFI+STATUS\\n                  -> print current WiFi status

This script wraps those commands so you don't have to type them by hand.

Usage:
    python provision_wifi.py                      # uses ./wifi_config.yaml
    python provision_wifi.py -c my.yaml           # custom config
    python provision_wifi.py -p /dev/ttyACM0      # override port
    python provision_wifi.py --status             # just print WIFI+STATUS
    python provision_wifi.py --ssid Foo --pass Bar   # skip the YAML entirely

Requirements:
    pip install pyserial pyyaml
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

try:
    import serial
except ImportError:
    sys.exit("error: pyserial is not installed — run `pip install pyserial`")

try:
    import yaml
except ImportError:
    sys.exit("error: pyyaml is not installed — run `pip install pyyaml`")


DEFAULT_CONFIG = Path(__file__).with_name("wifi_config.yaml")


def load_config(path: Path) -> dict:
    if not path.exists():
        sys.exit(f"error: config file not found: {path}")
    with path.open() as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        sys.exit(f"error: {path} is not a YAML mapping")
    return data


def open_serial(port: str, baudrate: int) -> serial.Serial:
    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=1.0)
    except serial.SerialException as e:
        sys.exit(f"error: could not open {port}: {e}")
    return ser


def drain_output(ser: serial.Serial, duration_s: float, prefix: str = "  [dev] ") -> None:
    """Read whatever the device prints for up to `duration_s` seconds."""
    deadline = time.time() + duration_s
    buf = b""
    while time.time() < deadline:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
            # Print complete lines as we see them
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode(errors="replace").rstrip("\r")
                if text:
                    print(prefix + text)
        else:
            # No data for a short window — assume device is idle
            if buf:
                text = buf.decode(errors="replace").rstrip()
                if text:
                    print(prefix + text)
                buf = b""
    # Flush whatever is left
    if buf:
        text = buf.decode(errors="replace").rstrip()
        if text:
            print(prefix + text)


def send_line(ser: serial.Serial, line: str) -> None:
    print(f"  -> {line}")
    ser.write((line + "\n").encode())
    ser.flush()


def provision(port: str, baudrate: int, boot_wait_s: float,
              networks: list[dict], show_status: bool) -> None:
    print(f"Opening {port} @ {baudrate} baud...")
    ser = open_serial(port, baudrate)

    try:
        # Let the device finish booting / printing any startup logs.
        if boot_wait_s > 0:
            print(f"Waiting {boot_wait_s:.1f}s for device...")
            drain_output(ser, boot_wait_s)

        # Nudge the device with a status query so we see a response even
        # if it's already running a session.
        send_line(ser, "WIFI+STATUS")
        drain_output(ser, 0.5)

        for i, net in enumerate(networks, start=1):
            ssid = str(net.get("ssid", "")).strip()
            password = str(net.get("password", ""))
            if not ssid:
                print(f"  (skipping network #{i}: empty ssid)")
                continue
            print(f"\n[{i}/{len(networks)}] Provisioning '{ssid}'...")
            send_line(ser, f"WIFI+ADD:{ssid},{password}")
            # Give the device time to save to NVS and attempt the connection.
            # Association typically takes 2-5 seconds on WPA2 networks.
            drain_output(ser, 8.0)

        if show_status:
            print("\nFinal status:")
            send_line(ser, "WIFI+STATUS")
            drain_output(ser, 1.5)

    finally:
        ser.close()
        print("\nDone. Device will auto-connect to the last network on every boot.")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("-c", "--config", type=Path, default=DEFAULT_CONFIG,
                    help=f"YAML config file (default: {DEFAULT_CONFIG.name})")
    ap.add_argument("-p", "--port", help="serial port (overrides YAML)")
    ap.add_argument("-b", "--baudrate", type=int, help="baud rate (overrides YAML)")
    ap.add_argument("--ssid", help="inline SSID (skip YAML networks)")
    ap.add_argument("--password", "--pass", dest="password", default="",
                    help="inline password (used with --ssid)")
    ap.add_argument("--status", action="store_true",
                    help="only query current WIFI+STATUS, don't send creds")
    ap.add_argument("--no-status", action="store_true",
                    help="skip the final WIFI+STATUS readback")
    args = ap.parse_args()

    cfg = load_config(args.config) if args.config.exists() else {}

    port = args.port or cfg.get("port") or os.environ.get("ESPPORT") or "/dev/ttyACM0"
    baudrate = args.baudrate or int(cfg.get("baudrate", 115200))
    boot_wait_s = float(cfg.get("boot_wait_s", 2.0))

    if args.status:
        networks = []
    elif args.ssid:
        networks = [{"ssid": args.ssid, "password": args.password}]
    else:
        networks = cfg.get("networks") or []
        if not networks:
            sys.exit("error: no networks in config and no --ssid given")

    provision(port, baudrate, boot_wait_s, networks,
              show_status=not args.no_status)
    return 0


if __name__ == "__main__":
    sys.exit(main())
