"""
TCP control channel for MONOMOD devices.

Sends command packets over TCP, receives ACK responses.
"""

import socket
import struct
import threading
import time
from typing import Optional

from . import protocol as proto


class TcpControl:
    """Synchronous TCP command channel to a MONOMOD device."""

    def __init__(self):
        self._sock: Optional[socket.socket] = None
        self._seq = 0
        self.ip: Optional[str] = None
        self._lock = threading.Lock()   # serialize command transactions

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

    def reconnect(self, timeout: float = 5.0) -> bool:
        """Re-establish the TCP control connection (after a drop)."""
        if self.ip is None:
            return False
        self.close()
        return self.connect(self.ip, timeout=timeout)

    def _recv_response(self, timeout: float) -> dict | None:
        """Read until a complete, CRC-valid packet is parsed (responses arrive
        one at a time, but may be split across TCP segments)."""
        buf = b""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                chunk = self._sock.recv(512)
            except socket.timeout:
                break
            if not chunk:
                break
            buf += chunk
            parsed = proto.parse_header(buf)
            if parsed is not None:
                return parsed
            if len(buf) > 4096:        # runaway guard
                break
        return None

    def _txn(self, cmd_id: int, params: bytes = b"",
             timeout: float = 5.0) -> dict | None:
        """Send a command, return the full parsed response header (or None).
        Serialized so concurrent callers can't interleave on the socket."""
        if not self._sock:
            return None
        with self._lock:
            self._seq = (self._seq + 1) & 0xFFFF
            pkt = proto.build_command(cmd_id, params, self._seq)
            try:
                self._sock.settimeout(timeout)
                self._sock.sendall(pkt)
                return self._recv_response(timeout)
            except (OSError, socket.timeout) as e:
                print(f"Command 0x{cmd_id:02X} failed: {e}")
                return None

    def send_command(self, cmd_id: int, params: bytes = b"",
                     timeout: float = 5.0) -> dict | None:
        """Send a command and wait for its ACK. Returns parsed ACK dict or None."""
        parsed = self._txn(cmd_id, params, timeout)
        if parsed is None:
            return None
        if parsed["type"] == proto.TYPE_ACK:
            return proto.parse_ack(parsed["payload"])
        return None

    # ---- Convenience methods ----

    def ping(self) -> float | None:
        """Ping device. Returns roundtrip time in ms, or None."""
        t0 = time.monotonic()
        ack = self.send_command(proto.CMD_PING, timeout=2.0)
        if ack and ack["status"] == proto.ACK_SUCCESS:
            return (time.monotonic() - t0) * 1000.0
        return None

    def ping_clock(self, timeout: float = 1.0) -> tuple[float, int] | None:
        """Ping for clock sync. Returns (rtt_s, device_ts_us) where device_ts_us
        is the device's monotonic µs timestamp from the ACK header, or None."""
        t0 = time.monotonic()
        parsed = self._txn(proto.CMD_PING, timeout=timeout)
        rtt = time.monotonic() - t0
        if parsed is None or parsed["type"] != proto.TYPE_ACK:
            return None
        return (rtt, parsed["timestamp"])

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
