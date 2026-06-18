# MONOMOD LIS3DH node

Firmware for a XIAO ESP32-C3 + single LIS3DH module that streams **accelerometer
+ EMG** to the MONOMOD GUI over WiFi. The LIS3DH is the only sensor: 3-axis
accel plus the amplified EMG signal wired to its auxiliary **ADC1** input, all
read over one SPI bus by a single acquisition task (no bus contention).

Speaks the stock MONOMOD protocol (UDP data :5000, TCP control :5001, mDNS
`_monomod._udp`), so the existing `driver/` + `gui/` discover and stream from it
with no changes.

| Stream | Source            | Protocol mapping                          |
|--------|-------------------|-------------------------------------------|
| EMG    | LIS3DH ADC1 (SPI) | `DATA_RAW` / `MONOMOD_ADC_TYPE_INA`, 1ch 16-bit |
| Accel  | LIS3DH (SPI)      | `MONOMOD_TYPE_IMU` (accel filled, gyro = 0)  |

Rates are set from the GUI (EMG Hz / IMU Hz selectors, applied on Start): EMG
100–1000 Hz, accel 25–200 Hz. The firmware applies the EMG rate (`START`) by
setting the acq period and a matching LIS3DH ODR, and the IMU rate (`SET_IMU`) as
an EMG-sample decimation. Compile-time defaults (`*_RATE_HZ` in `main/main.cpp`)
are EMG 1 kHz / accel 100 Hz. 1 kHz is the practical EMG ceiling (LIS3DH aux ADC).

## Wiring (by silk label) and board selection

Pins are referenced by their printed `D`-label; the GPIO depends on the carrier
board (`monomod_board` component). The board **defaults from the build target**
(XIAO ESP32-S3 for `esp32s3`, XIAO ESP32-C3 otherwise); override with
`idf.py menuconfig` → **MONOMOD Board** (e.g. for the C3 Super Mini).

| Signal      | Silk | XIAO C3 | C3 Super Mini | XIAO S3 |
|-------------|------|---------|---------------|---------|
| MOSI / PICO | D10  | 10      | 4             | 9       |
| MISO / POCI | D9   | 9       | 3             | 8       |
| SCLK        | D8   | 8       | 2             | 7       |
| CS          | D7   | 20      | 1             | 44      |
| NeoPixel    | D0   | 2       | 5             | 1       |

NeoPixel status: red = no sensor, blue = waiting for WiFi, dim cyan = idle/ready,
green pulse = streaming.

## Build, provision, run

```bash
source ~/esp/esp-idf/export.sh
cd firmware/lis3dh_node

# XIAO ESP32-C3 (board auto-defaults to XIAO C3)
idf.py set-target esp32c3
idf.py -p /dev/ttyACM0 flash monitor

# XIAO ESP32-S3 (board auto-defaults to XIAO S3)
idf.py set-target esp32s3
idf.py -p /dev/ttyACM0 flash monitor

# Push WiFi creds (device stores them in NVS; persists across reboots):
#   WIFI+ADD:<ssid>,<password>   typed in the monitor, or via ../../provision
```

Identify which port is which chip before flashing:
`python -m esptool -p /dev/ttyACMx chip_id` → reports ESP32-C3 vs ESP32-S3.

To keep both targets built at once (no re-`set-target`), use separate build dirs:
`idf.py -B build_s3 -DSDKCONFIG=sdkconfig.s3 set-target esp32s3` (and likewise
`build_c3`/`sdkconfig.c3`), then build/flash with the same `-B`/`-DSDKCONFIG`.

Then in the GUI: scan/connect (or add the device IP) and hit Start — EMG plots
as a 1-channel trace, accel on the IMU plot.
