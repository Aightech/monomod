"""
Axon protocol packet building and parsing for MONOMOD.

Packet format: [MAGIC:2][TYPE:1][SEQ:2][TS_US:4][PAYLOAD:N][CRC16:2]
"""

import struct

# Protocol constants
MAGIC = 0xAE01
HEADER_SIZE = 9       # 2+1+2+4
CRC_SIZE = 2
HEADER_FMT = "<HBHI"  # magic(u16), type(u8), seq(u16), timestamp(u32)

# Message types
TYPE_DATA_RAW   = 0x01
TYPE_IMU        = 0x03
TYPE_STATUS     = 0x10
TYPE_CMD        = 0x20
TYPE_ACK        = 0x21
TYPE_NACK       = 0x22
TYPE_DISCOVER   = 0xFE

# Command IDs
CMD_PING            = 0x00
CMD_GET_INFO        = 0x01
CMD_GET_STATUS      = 0x02
CMD_START           = 0x10
CMD_STOP            = 0x11
CMD_SET_IMU         = 0x15
CMD_READ_REGISTER   = 0x16
CMD_WRITE_REGISTER  = 0x17
CMD_SEND_MARKER     = 0x20
CMD_SET_WIFI        = 0x40
CMD_REBOOT          = 0xF0

# ADC types
ADC_TYPE_ADS1293    = 0x03
ADS1293_SAMPLE_SIZE = 3   # 24-bit
ADS1293_NUM_CHANNELS = 3

ADC_TYPE_INA        = 0x04
INA_SAMPLE_SIZE     = 2   # 16-bit
INA_NUM_CHANNELS    = 1

# ACK status
ACK_SUCCESS         = 0x00
ACK_INVALID_PARAM   = 0x01
ACK_NOT_SUPPORTED   = 0x02
ACK_BUSY            = 0x03
ACK_HW_ERROR        = 0x04

# Ports
UDP_DATA_PORT = 5000
TCP_CTRL_PORT = 5001


def _crc16_ccitt(data: bytes) -> int:
    """CRC-16-CCITT (poly 0x1021, init 0xFFFF)."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


def build_packet(msg_type: int, payload: bytes = b"", seq: int = 0) -> bytes:
    """Build a complete packet with header + payload + CRC."""
    header = struct.pack(HEADER_FMT, MAGIC, msg_type, seq, 0)
    data = header + payload
    crc = _crc16_ccitt(data)
    return data + struct.pack("<H", crc)


def build_command(cmd_id: int, params: bytes = b"", seq: int = 0) -> bytes:
    """Build a command packet."""
    payload = struct.pack("B", cmd_id) + params
    return build_packet(TYPE_CMD, payload, seq)


def parse_header(data: bytes) -> dict | None:
    """Parse packet header. Returns None if invalid."""
    if len(data) < HEADER_SIZE + CRC_SIZE:
        return None
    magic, msg_type, seq, timestamp = struct.unpack_from(HEADER_FMT, data, 0)
    if magic != MAGIC:
        return None
    # Verify CRC
    calc_crc = _crc16_ccitt(data[:-CRC_SIZE])
    rx_crc = struct.unpack_from("<H", data, len(data) - CRC_SIZE)[0]
    if calc_crc != rx_crc:
        return None
    return {
        "type": msg_type,
        "seq": seq,
        "timestamp": timestamp,
        "payload": data[HEADER_SIZE:-CRC_SIZE],
    }


def parse_ack(payload: bytes) -> dict:
    """Parse ACK payload (2+ bytes)."""
    if len(payload) < 2:
        return {"cmd_id": 0, "status": 0xFF, "data": b""}
    return {
        "cmd_id": payload[0],
        "status": payload[1],
        "data": payload[2:],
    }


def parse_data_packet(payload: bytes) -> dict | None:
    """Parse DATA_RAW payload → (ch_mask, n_samples, adc_type, samples).

    Returns dict with:
        ch_mask: int
        n_samples: int
        adc_type: int
        samples: list of list[int] — [n_samples][n_channels]
    """
    if len(payload) < 6:
        return None
    ch_mask, n_samples, adc_type = struct.unpack_from("<IBB", payload, 0)
    n_channels = bin(ch_mask).count("1")

    if adc_type == ADC_TYPE_ADS1293:
        sample_size = ADS1293_SAMPLE_SIZE
    elif adc_type == ADC_TYPE_INA:
        sample_size = INA_SAMPLE_SIZE
    else:
        return None  # unsupported

    data = payload[6:]
    expected = n_channels * n_samples * sample_size
    if len(data) < expected:
        return None

    samples = []
    offset = 0
    for _ in range(n_samples):
        frame = []
        for _ in range(n_channels):
            if sample_size == 3:
                # 24-bit big-endian, sign-extend
                b0, b1, b2 = data[offset], data[offset + 1], data[offset + 2]
                val = (b0 << 16) | (b1 << 8) | b2
                if val & 0x800000:
                    val |= ~0xFFFFFF
            elif sample_size == 2:
                # 16-bit big-endian, sign-extend
                b0, b1 = data[offset], data[offset + 1]
                val = (b0 << 8) | b1
                if val & 0x8000:
                    val |= ~0xFFFF
            else:
                val = 0
            frame.append(val)
            offset += sample_size
        samples.append(frame)
    return {
        "ch_mask": ch_mask,
        "n_samples": n_samples,
        "adc_type": adc_type,
        "samples": samples,
    }


def parse_imu_packet(payload: bytes) -> dict | None:
    """Parse ICM-20948 IMU payload (12 bytes: 6× int16, raw ADC counts).

    Returns raw int16 accel + gyro. Conversion to g / dps depends on
    the configured range on the device (default ±2g, ±250 dps):
        accel_g   = raw / 16384.0    (at ±2g)
        gyro_dps  = raw / 131.0      (at ±250 dps)
    """
    if len(payload) < 12:
        return None
    ax, ay, az, gx, gy, gz = struct.unpack_from("<6h", payload, 0)
    return {
        "accel_raw": [ax, ay, az],
        "gyro_raw":  [gx, gy, gz],
        # Convenience: convert assuming default ±2g / ±250 dps
        "accel_g":   [ax / 16384.0, ay / 16384.0, az / 16384.0],
        "gyro_dps":  [gx / 131.0,  gy / 131.0,  gz / 131.0],
    }


def parse_device_info(data: bytes) -> dict | None:
    """Parse device info from GET_INFO ACK response.

    C struct (packed, 30 bytes):
        [0..3]  fw major/minor/patch/build (u8 x4)
        [4..5]  hw_rev (u16)
        [6..17] serial (char[12])
        [18]    adc_type (u8)
        [19]    num_channels (u8)
        [20..23] features (u32)
        [24..25] sample_size (u16)
        [26..29] max_sample_rate (u32)
    """
    if len(data) < 30:
        return None
    fmt = "<BBBBH12sBBIHI"
    (fw_maj, fw_min, fw_patch, fw_build, hw_rev, serial_bytes,
     adc_type, num_ch, features, sample_size, max_sr) = struct.unpack_from(fmt, data, 0)
    return {
        "fw_version": f"{fw_maj}.{fw_min}.{fw_patch}",
        "hw_rev": hw_rev,
        "serial": serial_bytes.decode("ascii", errors="replace").rstrip("\x00"),
        "adc_type": adc_type,
        "num_channels": num_ch,
        "features": features,
        "sample_size": sample_size,
        "max_sample_rate": max_sr,
    }


def parse_status(payload: bytes) -> dict | None:
    """Parse STATUS payload (23 bytes)."""
    if len(payload) < 23:
        return None
    (batt_pct, batt_mv, flags, temp, pkt_sent, pkt_drop, rssi,
     state, sr, ch_mask) = struct.unpack_from("<BHBBIIbBII", payload, 0)
    return {
        "battery_pct": batt_pct,
        "battery_mv": batt_mv,
        "flags": flags,
        "temp_c": temp,
        "packets_sent": pkt_sent,
        "packets_dropped": pkt_drop,
        "rssi": rssi,
        "state": state,
        "sample_rate": sr,
        "channel_mask": ch_mask,
    }
