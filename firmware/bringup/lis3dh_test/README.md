# LIS3DH SPI bring-up test

Minimal standalone firmware to verify a LIS3DH is soldered correctly and
detected on the SPI bus of a XIAO ESP32-C3 module. Reuses the `lis3dh` driver
component from `../../components/`.

## What it does

- **NeoPixel blue** → probing
- **NeoPixel red (blinking)** → LIS3DH not detected; the probe retries every
  second so you can rework the joint and watch it turn green without reflashing
- **NeoPixel green** → `WHO_AM_I == 0x33`; then it streams live accel readings
  over the USB serial console (tilt the board — the values should change)

## Wiring (XIAO ESP32-C3 silk → GPIO)

| Signal     | Silk | GPIO |
|------------|------|------|
| MOSI / PICO| D10  | 10   |
| MISO / POCI| D9   | 9    |
| SCLK       | D8   | 8    |
| CS         | D7   | 20   |
| NeoPixel   | D0   | 2    |

The amplified EMG signal on the LIS3DH ADC1 input is **not** used by this test.
Pin assignments live at the top of `main/main.c`.

## Build & flash

```bash
source ~/esp/esp-idf/export.sh   # once per shell
cd firmware/bringup/lis3dh_test
idf.py set-target esp32c3        # once
idf.py -p /dev/ttyACM0 flash monitor
```

First build downloads the `espressif/led_strip` managed component.
