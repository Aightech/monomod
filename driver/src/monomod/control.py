"""
TCP control channel for MONOMOD devices.

Sends command packets over TCP, receives ACK responses.
"""

import socket
import struct
import time
from typing import Optional

from . import protocol as proto


class TcpControl:
    """Synchronous TCP command channel to a MONOMOD device."""

    def __init__(self):
        self._sock: Optional[socket.socket] = None
        self._seq = 0
        self.ip: Optional[str] = None

    def connect(self, ip: str, port: int = proto.TCP_CTRL_PORT,
                timeout: float = 5.0) -> bool:
        """Connect to a device's TCP control port."""
        self.ip = ip
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(timeout)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.connect((ip, port))
            self._sock = sock
            return True
        except OSError as e:
            print(f"TCP connect failed: {e}")
            return False

    def close(self):
        """Close the TCP connection."""
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    @property
    def is_connected(self) -> bool:
        return self._sock is not None

    def send_command(self, cmd_id: int, params: bytes = b"",
                     timeout: float = 5.0) -> dict | None:
        """Send a command and wait for ACK response.

        Returns parsed ACK dict or None on failure.
        """
        if not self._sock:
            return None

        self._seq = (self._seq + 1) & 0xFFFF
        pkt = proto.build_command(cmd_id, params, self._seq)

        try:
            self._sock.settimeout(timeout)
            self._sock.sendall(pkt)

            # Receive response
            resp = self._sock.recv(512)
            if not resp:
                return None

            parsed = proto.parse_header(resp)
            if parsed is None:
                return None

            if parsed["type"] == proto.TYPE_ACK:
                return proto.parse_ack(parsed["payload"])
            return None

        except (OSError, socket.timeout) as e:
            print(f"Command 0x{cmd_id:02X} failed: {e}")
            return None

    # ---- Convenience methods ----

    def ping(self) -> float | None:
        """Ping device. Returns roundtrip time in ms, or None."""
        t0 = time.monotonic()
        ack = self.send_command(proto.CMD_PING, timeout=2.0)
        if ack and ack["status"] == proto.ACK_SUCCESS:
            return (time.monotonic() - t0) * 1000.0
        return None

    def get_info(self) -> dict | None:
        """Get device info."""
        ack = self.send_command(proto.CMD_GET_INFO)
        if ack and ack["status"] == proto.ACK_SUCCESS:
            return proto.parse_device_info(ack["data"])
        return None

    def start(self, sample_rate: int = 3200, ch_mask: int = 0x07) -> bool:
        """Start EMG streaming."""
        params = struct.pack("<II", sample_rate, ch_mask)
        ack = self.send_command(proto.CMD_START, params)
        return ack is not None and ack["status"] == proto.ACK_SUCCESS

    def stop(self) -> bool:
        """Stop EMG streaming."""
        ack = self.send_command(proto.CMD_STOP)
        return ack is not None and ack["status"] == proto.ACK_SUCCESS

    def set_imu(self, enable: bool, rate_div: int = 1) -> bool:
        """Enable/disable IMU streaming."""
        params = struct.pack("BB", int(enable), rate_div)
        ack = self.send_command(proto.CMD_SET_IMU, params)
        return ack is not None and ack["status"] == proto.ACK_SUCCESS

    def read_register(self, addr: int) -> int | None:
        """Read an ADS1293 register."""
        ack = self.send_command(proto.CMD_READ_REGISTER, struct.pack("B", addr))
        if ack and ack["status"] == proto.ACK_SUCCESS and len(ack["data"]) >= 2:
            return ack["data"][1]
        return None

    def write_register(self, addr: int, value: int) -> bool:
        """Write an ADS1293 register."""
        params = struct.pack("BB", addr, value)
        ack = self.send_command(proto.CMD_WRITE_REGISTER, params)
        return ack is not None and ack["status"] == proto.ACK_SUCCESS

    def set_wifi(self, ssid: str, password: str) -> bool:
        """Set WiFi credentials on device."""
        params = struct.pack("BB", len(ssid), len(password))
        params += ssid.encode() + password.encode()
        ack = self.send_command(proto.CMD_SET_WIFI, params)
        return ack is not None and ack["status"] == proto.ACK_SUCCESS

    def reboot(self):
        """Reboot the device."""
        self.send_command(proto.CMD_REBOOT, timeout=1.0)
        self.close()
