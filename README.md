# monomod

Standalone wireless EMG modules for student lab experiments.

Each module is a XIAO ESP32-C3 + ADS1293 (3-ch 24-bit EMG AFE) + ICM-20948
(9-axis IMU), streaming over WiFi. A single PC runs a PyQt6 GUI that
connects to 3-5 modules at once and records everything to one file.

## Layout

```
monomod/
  firmware/    ESP-IDF firmware for the XIAO ESP32-C3
  driver/      Python package (UDP/TCP client, packet parser, config loader)
  gui/         PyQt6 multi-device GUI (plots + recording)
  provision/   YAML-driven WiFi provisioning helper (USB serial)
```

## Quick start

### 1. Flash the firmware

```bash
cd firmware
# one-time ESP-IDF setup: source ~/esp/esp-idf/export.sh
idf.py set-target esp32c3
idf.py -p /dev/ttyACM0 build flash monitor
```

### 2. Push WiFi credentials

```bash
cd provision
pip install -r requirements.txt
$EDITOR wifi_config.yaml          # set ssid / password
python3 provision_wifi.py         # pushes creds over USB, saves to NVS
```

The device auto-connects on every subsequent boot.

### 3. Run the GUI

```bash
cd gui
pip install -r requirements.txt
./run_gui.sh                      # or: python3 app.py
```

Scan for devices, connect, hit **Start** — EMG + IMU plot live and record
to a single CSV across all connected modules.

## Hardware

- **Board**: Seeed XIAO ESP32-C3
- **EMG AFE**: TI ADS1293 (3-ch, 24-bit, SPI). Optional fallback: INA331 +
  LIS3DH module, or bare-ADC mode on D0.
- **IMU**: Adafruit ICM-20948 (I²C, 9-axis)
- **Pinout**:
  | Function       | Silk | GPIO |
  |----------------|------|------|
  | SPI MOSI       | D9   | 9    |
  | SPI MISO       | D10  | 10   |
  | SPI SCLK       | D8   | 8    |
  | SPI CS         | D7   | 20   |
  | I²C SDA        | D4   | 6    |
  | I²C SCL        | D5   | 7    |
  | Analog EMG in  | D0   | 2    |

## Protocol

Binary packet format, UDP for data (port 5000), TCP for commands (port 5001),
mDNS discovery (`_monomod._udp.local.`).

```
[MAGIC 0xAE01 : 2][TYPE : 1][SEQ : 2][TS_US : 4][PAYLOAD : N][CRC16 : 2]
```

See `firmware/components/monomod_protocol/include/packet_types.h` and
`driver/src/monomod/protocol.py` for the full definition.

## License

TBD.
