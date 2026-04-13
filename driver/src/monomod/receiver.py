"""
UDP data receiver thread for MONOMOD.

Listens for axon_protocol packets on UDP port 5000, decodes DATA_RAW
and IMU packets, and dispatches via callbacks.
"""

import socket
import threading
import numpy as np
from typing import Callable, Optional

from . import protocol as proto


class UdpReceiver(threading.Thread):
    """
    Background thread that receives UDP data packets from a MONOMOD device.

    Callbacks:
        on_data(samples: np.ndarray, timestamp_us: int, seq: int)
            samples shape: (n_samples, n_channels), dtype=int32
        on_imu(imu_data: dict, timestamp_us: int)
            imu_data: {quat, accel, gyro, accuracy}
    """

    def __init__(self, port: int = proto.UDP_DATA_PORT):
        super().__init__(daemon=True)
        self.port = port
        self.on_data: Optional[Callable] = None
        self.on_imu: Optional[Callable] = None
        self._stop_evt = threading.Event()
        self._sock: Optional[socket.socket] = None
        self.packets_received = 0   # total successful header parses (any type)
        self.packets_invalid = 0    # failed magic/CRC
        # Per-type counters
        self.data_ok = 0
        self.data_fail = 0
        self.imu_ok = 0
        self.imu_fail = 0
        self.other = 0              # unknown packet types
        self.max_size_seen = 0      # largest packet observed

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
                        n, _ = sock.recvfrom_into(buf)
                    except socket.timeout:
                        continue
                    except OSError:
                        break

                    data = bytes(buf[:n])
                    if n > self.max_size_seen:
                        self.max_size_seen = n
                    pkt = proto.parse_header(data)
                    if pkt is None:
                        self.packets_invalid += 1
                        continue

                    self.packets_received += 1

                    if pkt["type"] == proto.TYPE_DATA_RAW:
                        parsed = proto.parse_data_packet(pkt["payload"])
                        if parsed:
                            self.data_ok += 1
                            if self.on_data:
                                samples = np.array(parsed["samples"], dtype=np.int32)
                                self.on_data(samples, pkt["timestamp"], pkt["seq"])
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
                    else:
                        self.other += 1

            finally:
                try:
                    sock.close()
                except OSError:
                    pass
                self._sock = None
