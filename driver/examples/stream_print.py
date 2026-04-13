#!/usr/bin/env python3
"""
Minimal example: discover a MONOMOD device, connect, stream EMG, print samples.

Usage:
    python stream_print.py [device_ip]
"""

import sys
import time
sys.path.insert(0, "src")

from monomod import Device, discover_devices


def on_data(samples, ts_us, seq):
    """Called for each received data packet."""
    n = samples.shape[0]
    ch0 = samples[0, 0] if n > 0 else 0
    print(f"seq={seq:5d}  ts={ts_us:10d}  n={n:3d}  ch0={ch0:8d}")


def main():
    # Discover or use provided IP
    if len(sys.argv) > 1:
        ip = sys.argv[1]
        print(f"Connecting to {ip}...")
    else:
        print("Scanning for MONOMOD devices...")
        devices = discover_devices(timeout_s=3.0)
        if not devices:
            print("No devices found. Pass IP as argument: python stream_print.py 192.168.1.x")
            return
        ip = devices[0]["ip"]
        print(f"Found: {devices[0]}")

    dev = Device()
    if not dev.connect(ip):
        print(f"Failed to connect to {ip}")
        return

    info = dev.info
    if info:
        print(f"Device: FW {info['fw_version']}, {info['num_channels']}ch, "
              f"max {info['max_sample_rate']} Hz")

    ping = dev.ping()
    print(f"Ping: {ping:.1f} ms" if ping else "Ping failed")

    dev.set_data_callback(on_data)
    print("Starting stream...")
    dev.start(sample_rate=3200, ch_mask=0x07)

    try:
        while True:
            time.sleep(1)
            rx = dev._rx
            print(f"  raw={dev.packets_raw}  "
                  f"data_ok={rx.data_ok} data_fail={rx.data_fail}  "
                  f"imu_ok={rx.imu_ok} imu_fail={rx.imu_fail}  "
                  f"hdr_bad={rx.packets_invalid}  "
                  f"max_size={rx.max_size_seen}")
    except KeyboardInterrupt:
        print("\nStopping...")

    dev.stop()
    dev.disconnect()
    print("Done.")


if __name__ == "__main__":
    main()
