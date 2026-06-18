"""
Shared UDP data receiver for MONOMOD.

All devices stream to the host on the same UDP port (5000). A single socket is
therefore bound once and incoming datagrams are demultiplexed by **sender IP** to
the matching device. This fixes multi-device routing: previously each Device bound
its own socket to 0.0.0.0:5000, so the OS delivered every device's packets to just
one of them.

Usage (internal — Device does this):
    handle = register(ip, on_data=cb_data, on_imu=cb_imu)
    ...
    unregister(ip)

Each `register()` returns a `ReceiverHandle` carrying this device's callbacks and
its own packet counters (`packets_received`, `packets_invalid`, `imu_ok`, ...).
"""

import socket
import threading
import numpy as np
from typing import Callable, Optional

from . import protocol as proto


class ReceiverHandle:
    """Per-device callbacks + counters. One per registered IP."""

    def __init__(self, ip: str, on_data: Optional[Callable],
                 on_imu: Optional[Callable], on_status: Optional[Callable] = None):
        self.ip = ip
        self.on_data = on_data
        self.on_imu = on_imu
        self.on_status = on_status
        # Counters (names kept stable — the GUI reads these via device._rx)
        self.packets_received = 0   # successful header parses (any type)
        self.packets_invalid = 0    # failed magic/CRC
        self.data_ok = 0
        self.data_fail = 0
        self.imu_ok = 0
        self.imu_fail = 0
        self.status_ok = 0
        self.other = 0              # unknown packet types
        self.max_size_seen = 0      # largest packet observed

    def dispatch(self, data: bytes):
        """Parse one datagram from this device and fire the right callback."""
        n = len(data)
        if n > self.max_size_seen:
            self.max_size_seen = n

        pkt = proto.parse_header(data)
        if pkt is None:
            self.packets_invalid += 1
            return
        self.packets_received += 1

        if pkt["type"] == proto.TYPE_DATA_RAW:
            parsed = proto.parse_data_packet(pkt["payload"])
            if parsed:
                self.data_ok += 1
                if self.on_data:
                    # parsed["samples"] is already an int32 ndarray (vectorized)
                    self.on_data(parsed["samples"], pkt["timestamp"], pkt["seq"])
            else:
                self.data_fail += 1
        elif pkt["type"] == proto.TYPE_IMU:
            imu = proto.parse_imu_packet(pkt["payload"])
            if imu:
                self.imu_ok += 1
                if self.on_imu:
                    self.on_imu(imu, pkt["timestamp"])
            else:
                self.imu_fail += 1
        elif pkt["type"] == proto.TYPE_STATUS:
            status = proto.parse_status(pkt["payload"])
            if status:
                self.status_ok += 1
                if self.on_status:
                    self.on_status(status, pkt["timestamp"])
        else:
            self.other += 1


class SharedReceiver(threading.Thread):
    """One socket bound to UDP :5000, demuxing datagrams by sender IP."""

    def __init__(self, port: int = proto.UDP_DATA_PORT):
        super().__init__(daemon=True)
        self.port = port
        self._stop_evt = threading.Event()
        self._sock: Optional[socket.socket] = None
        self._handles: dict[str, ReceiverHandle] = {}
        self._lock = threading.Lock()
        self.unmatched = 0          # datagrams from unregistered IPs

    # ---- registry ----
    def add(self, ip: str, on_data, on_imu, on_status=None) -> ReceiverHandle:
        handle = ReceiverHandle(ip, on_data, on_imu, on_status)
        with self._lock:
            self._handles[ip] = handle
        return handle

    def remove(self, ip: str) -> bool:
        """Remove a device; returns True if no devices remain."""
        with self._lock:
            self._handles.pop(ip, None)
            return not self._handles

    def stop(self):
        self._stop_evt.set()
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass

    def run(self):
        while not self._stop_evt.is_set():
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
                sock.bind(("0.0.0.0", self.port))
                sock.settimeout(0.5)
                self._sock = sock
            except OSError:
                import time
                time.sleep(0.25)
                continue

            buf = bytearray(2000)
            try:
                while not self._stop_evt.is_set():
                    try:
                        n, addr = sock.recvfrom_into(buf)
                    except socket.timeout:
                        continue
                    except OSError:
                        break

                    with self._lock:
                        handle = self._handles.get(addr[0])
                    if handle is None:
                        self.unmatched += 1
                        continue
                    handle.dispatch(bytes(buf[:n]))
            finally:
                try:
                    sock.close()
                except OSError:
                    pass
                self._sock = None


# ---- module-level singleton + register/unregister --------------------------
_shared: Optional[SharedReceiver] = None
_shared_lock = threading.Lock()


def register(ip: str, on_data: Optional[Callable] = None,
             on_imu: Optional[Callable] = None,
             on_status: Optional[Callable] = None,
             port: int = proto.UDP_DATA_PORT) -> ReceiverHandle:
    """Start the shared receiver (if needed) and register a device by IP."""
    global _shared
    with _shared_lock:
        if _shared is None or not _shared.is_alive():
            _shared = SharedReceiver(port)
            _shared.start()
        return _shared.add(ip, on_data, on_imu, on_status)


def unregister(ip: str):
    """Unregister a device; stop the shared receiver when the last one leaves."""
    global _shared
    with _shared_lock:
        if _shared is None:
            return
        if _shared.remove(ip):
            _shared.stop()
            _shared.join(timeout=1.0)
            _shared = None
