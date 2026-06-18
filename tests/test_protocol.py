"""Protocol build/parse round-trips, CRC, sign-extension, and wire-size drift."""
import struct

from monomod import protocol as proto


def test_crc_known_vector():
    # CRC-16/CCITT-FALSE check value for "123456789" is 0x29B1.
    assert proto._crc16_ccitt(b"123456789") == 0x29B1


def test_command_roundtrip():
    pkt = proto.build_command(proto.CMD_START, struct.pack("<II", 1000, 1), seq=5)
    p = proto.parse_header(pkt)
    assert p is not None
    assert p["type"] == proto.TYPE_CMD and p["seq"] == 5
    assert p["payload"][0] == proto.CMD_START


def test_bad_crc_rejected():
    pkt = bytearray(proto.build_command(proto.CMD_PING))
    pkt[-1] ^= 0xFF
    assert proto.parse_header(bytes(pkt)) is None


def test_data_ina_signext():
    pl = struct.pack("<IBB", 0x1, 3, proto.ADC_TYPE_INA) + struct.pack(">hhh", 1, -1, -32768)
    r = proto.parse_data_packet(pl)
    assert r["samples"].tolist() == [[1], [-1], [-32768]]
    assert str(r["samples"].dtype) == "int32"


def test_data_ads24_signext():
    def be24(v):
        v &= 0xFFFFFF
        return bytes([(v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF])
    pl = (struct.pack("<IBB", 0x7, 1, proto.ADC_TYPE_ADS1293)
          + be24(8388607) + be24(-8388608) + be24(0))
    r = proto.parse_data_packet(pl)
    assert r["samples"].tolist() == [[8388607, -8388608, 0]]


def test_imu_roundtrip():
    r = proto.parse_imu_packet(struct.pack("<6h", 1, 2, 3, -1, -2, -3))
    assert r["accel_raw"] == [1, 2, 3] and r["gyro_raw"] == [-1, -2, -3]


def test_status_roundtrip():
    pl = struct.pack("<BHBBIIbBII", 50, 3700, 0xA4, 25, 100, 2, -60, 3, 1000, 1)
    r = proto.parse_status(pl)
    assert r["battery_pct"] == 50 and r["rssi"] == -60 and r["sample_rate"] == 1000


# These sizes are mirrored by static_assert in packet_types.h — keep in sync.
def test_wire_sizes():
    assert struct.calcsize(proto.HEADER_FMT) == 9
    assert struct.calcsize("<IBB") == 6              # data header
    assert struct.calcsize("<6h") == 12             # imu payload
    assert struct.calcsize("<BHBBIIbBII") == 23     # status payload
    assert struct.calcsize("<BBBBH12sBBIHI") == 30  # device info
