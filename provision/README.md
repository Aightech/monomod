# MONOMOD WiFi provisioning

Small helper to push WiFi credentials to a MONOMOD device over USB serial,
driven by a YAML config file — so you don't have to type `WIFI+ADD:...` by
hand every time you (re)flash a board.

## Install

```bash
pip install -r requirements.txt
```

## Use

1. Edit `wifi_config.yaml` with your network(s).
2. Plug the device in and note its serial port (default `/dev/ttyACM0`).
3. Run:

   ```bash
   python provision_wifi.py
   ```

   The script opens the port, waits for the device to boot, sends a
   `WIFI+ADD:<ssid>,<password>` line for each network in the config, and
   prints the device's log output. Credentials are stored in NVS on the
   device, so they survive reboots.

## Quick one-liners

```bash
# Override the port
python provision_wifi.py -p /dev/ttyACM1

# Skip the YAML entirely
python provision_wifi.py --ssid MyNet --password hunter2

# Just query the current status
python provision_wifi.py --status

# Use a different config file
python provision_wifi.py -c lab_wifi.yaml
```

## How it works

The firmware listens on its USB CDC serial port for two line-based commands:

| Command                            | Effect                                  |
|------------------------------------|-----------------------------------------|
| `WIFI+ADD:<ssid>,<password>\n`     | Save to NVS and connect immediately     |
| `WIFI+STATUS\n`                    | Print current SSID / IP / RSSI          |

This script just wraps those.
