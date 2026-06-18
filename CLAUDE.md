# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

`monomod` is a wireless EMG module stack: each module is a Seeed XIAO ESP32-C3 +
ADS1293 (3-ch, 24-bit EMG AFE) + ICM-20948 (9-axis IMU) that streams biosignals
over WiFi. One PC runs a PyQt6 GUI connecting to 3–5 modules at once, recording
everything to a single file.

Four cooperating subsystems, each in its own top-level directory:

- `firmware/` — ESP-IDF C/C++ firmware for the XIAO ESP32-C3
- `driver/`   — `monomod` Python package: UDP/TCP client, packet parser, config loader
- `gui/`      — PyQt6 multi-device GUI (live plots + recording)
- `provision/`— YAML-driven WiFi provisioning over USB serial

## Commands

### Firmware (ESP-IDF, target esp32c3)
There are three ESP-IDF projects under `firmware/`, each its own project root
(run `idf.py` from inside it). They share the components in `firmware/components/`
via `EXTRA_COMPONENT_DIRS`:
- `firmware/` — the main ADS1293-centric `monomod` app
- `firmware/lis3dh_node/` — LIS3DH-only board: streams accel + EMG (EMG on the
  LIS3DH's ADC1 input) to the GUI via the same MONOMOD protocol
- `firmware/bringup/lis3dh_test/` — standalone LIS3DH SPI detection test

```bash
cd firmware                           # or firmware/lis3dh_node, etc.
source ~/esp/esp-idf/export.sh        # one-time per shell
idf.py set-target esp32c3             # one-time per checkout
idf.py build
idf.py -p /dev/ttyACM0 flash monitor  # XIAO C3 enumerates as /dev/ttyACM0 over USB-Serial/JTAG
idf.py menuconfig                     # edit sdkconfig (defaults live in sdkconfig.defaults)
```

### Driver (Python package)
```bash
cd driver
pip install -e .                      # core: numpy, pyyaml
pip install -e ".[gui,discovery]"     # extras: pyqt6/pyqtgraph, zeroconf, h5py, pylsl
python examples/stream_print.py       # minimal usage examples live in driver/examples/
```

### GUI
```bash
cd gui
./run_gui.sh                          # creates .venv, installs reqs + editable driver, runs app.py
# or, in an existing env: python3 app.py
```
`run_gui.sh` pip-installs the driver from `../driver` in editable mode — the GUI
depends on the local `monomod` package, not a published one.

### WiFi provisioning
```bash
cd provision
pip install -r requirements.txt
$EDITOR wifi_config.yaml
python3 provision_wifi.py                       # push creds from YAML over USB
python3 provision_wifi.py --ssid X --password Y # bypass YAML
python3 provision_wifi.py --status              # query current SSID/IP/RSSI
```

There is no automated test suite. Validation is hardware-in-the-loop: flash a
board, run a `driver/examples/` script or the GUI, and watch the serial monitor.

## Architecture

### The shared wire protocol is the contract
The binary "MONOMOD" protocol (v1.3) is defined **twice** and the two definitions
must stay byte-compatible:
- `firmware/components/monomod_protocol/include/packet_types.h` (C structs, `#pragma pack(1)`)
- `driver/src/monomod/protocol.py` (`struct` format strings)

Packet layout: `[MAGIC 0xAE01 : 2][TYPE : 1][SEQ : 2][TS_US : 4][PAYLOAD : N][CRC16 : 2]`
(9-byte header, CRC-16-CCITT poly 0x1021 init 0xFFFF). **When you change a packet
struct, command ID, or field on one side, change the other side too** — there is
no codegen or schema keeping them in sync, and a mismatch silently corrupts parsing.

Transport split: **UDP port 5000 for data** (one-way device→host stream, lossy by
design — host tracks gaps via the `seq` field), **TCP port 5001 for commands/ACKs**
(`TcpControl`), **mDNS `_monomod._udp.local.` for discovery**. Multi-byte ADC
samples are packed **big-endian** in the payload and sign-extended on the host;
everything else in the header/structs is little-endian.

### Firmware data flow (single-core C3, FreeRTOS)
`main.cpp` `app_main()` does: USB-Serial/JTAG console setup → NVS init → **EMG
module auto-detect** → I2C/IMU init → WiFi connect from NVS → start NetworkManager.
Then it loops forever handling line-based serial commands (`WIFI+ADD:`, `WIFI+STATUS`)
and late WiFi connects.

Streaming is a three-task pipeline created on the `START` command and torn down on `STOP`:
- **acq_task** (`tasks/acq_task.cpp`, prio 22) — esp_timer ISR reads the ADC, pushes
  `SampleFrame`s into a `MonomodRingBuffer`, signals the tx semaphore.
- **tx_task** (`tasks/tx_task.cpp`, prio 18) — drains the ring, batches ~30 ms of
  samples, splits into UDP packets capped at 32 samples (~307 B) to dodge WiFi MTU
  drops, sends via NetworkManager.
- **imu_task** (`tasks/imu_task.cpp`, prio 10) — independent IMU sampling/send loop.

WiFi/LWIP runs at prio 23 (above acq). Commands are dispatched by
`handle_command()` in `main.cpp`, called back from `NetworkManager`'s TCP server.

**EMG module auto-detect** (`main.cpp` step 2) probes the SPI bus in order and sets
`g_emg_module`, which changes the ADC type/channel count reported by `GET_INFO` and
which acq path runs:
1. ADS1293 (REVID) → `EMG_MODULE_ADS1293`, 3-ch 24-bit
2. else LIS3DH (WHO_AM_I) → `EMG_MODULE_INA_LIS`, INA331 + LIS3DH combo
3. else bare-ADC fallback → 1-ch analog on GPIO2 (D0) via INA path

The same `monomod_adc_type_t` byte travels in every data packet so the host parser
picks the right sample size (3 B for ADS1293, 2 B for INA) without prior knowledge.

### Firmware components
`firmware/components/` holds reusable ESP-IDF components, each with its own
`CMakeLists.txt`: hardware drivers (`ads1293`, `ina_emg`, `lis3dh`, `icm20948`),
`monomod_protocol` (the shared header above), `monomod_core` (`ring_buffer.hpp`,
`timestamp.hpp`), `monomod_drivers` (WiFi provisioning), `monomod_net` (the
`NetworkManager` — UDP/TCP/mDNS server, shared by all firmware projects; it's
self-contained and does **not** depend on any project's `config.hpp`), and
`monomod_board` (header-only: maps silk `D`-labels → GPIOs per carrier board).
Each project's `main/CMakeLists.txt` lists the components it needs under `REQUIRES`.

**Board portability:** firmware refers to silk labels (`PIN_D0`..`PIN_D10` from
`monomod_board/include/board_pins.h`), not raw GPIOs, so the same code builds for
the Seeed XIAO ESP32-C3 (default) or the ESP32-C3 Super Mini. Pick via
`idf.py menuconfig` → "MONOMOD Board" (or `CONFIG_MONOMOD_BOARD_SUPERMINI_C3=y`).
The `bringup/lis3dh_test` project lists individual component dirs in
`EXTRA_COMPONENT_DIRS` (not the whole `components/`) so it doesn't pull the
networking components, which need the managed `mdns` dependency.
Board pinout, task priorities, stack sizes, and network ports for the main app
are in `firmware/main/config.hpp` (each other project keeps its own pin defines
at the top of its `main.cpp`).

The `lis3dh_node` firmware shows the minimal contract for a new board. Acquisition
is decoupled from transmission via lock-free SPSC rings (`monomod_core`): an
**acq task** (prio 22) is the sole SPI owner and only pushes samples to rings
(`push()` drop-on-full); a **tx task** (prio 18) drains and emits MONOMOD packets
(`DATA_RAW`/`MONOMOD_ADC_TYPE_INA` EMG, `MONOMOD_TYPE_IMU` accel) — so a slow/blocked
WiFi send never stalls or silently drops sampling. It also emits `MONOMOD_TYPE_STATUS`
(~1 Hz: RSSI, ring-overflow drops, state), detects WiFi loss to stop+auto-reconnect,
and registers the tx task with the task WDT (panic on hang). The host/GUI are
reused unchanged; no host-side code is board-specific.

Flash: 4 MB, custom partition table `firmware/partitions.csv` (note: this CSV is
intentionally committed — `.gitignore` excludes recording CSVs but keeps it).
FreeRTOS is `UNICORE` (C3 is single-core), tick rate 1000 Hz.

### Driver layering
`Device` (`device.py`) is the high-level facade composing `TcpControl`
(`control.py`, sync command/ACK), the shared UDP receiver (`receiver.py`), and
`discover_devices` (`discovery.py`, mDNS). Data reaches the app two ways: a
registered `set_data_callback` (called from the RX thread) **and** an internal
thread-safe `deque` you `drain()` (used by the GUI poll loop). `Device` also
exposes `set_status_callback`/`latest_status` (parsed `MONOMOD_TYPE_STATUS`
telemetry) and `sync_clock()` — an NTP-style min-RTT PING that sets
`clock_offset_s` so multi-device recordings can share a host timeline.

**Multi-device demux:** all devices stream to the host on UDP :5000, so
`receiver.py` binds **one** `SharedReceiver` socket and routes each datagram to a
per-device `ReceiverHandle` by **sender IP**. `Device.connect/disconnect` call
`receiver.register(ip, ...)`/`unregister(ip)` (ref-counted: the socket starts on
the first device and stops after the last). Never bind a second socket to :5000.

`config.py` loads the YAML and exposes session/recording/experiment/gui accessors
(`iter_session_devices`, `get_recording`, etc.); `ads1293.py` holds register math
(sample-rate from R1/R2/R3 decimation, RLD routing, ADC-max for voltage conversion).

### GUI
`gui/` is a PyQt6 app across three modules:
- `app.py` — `MainWindow` owns a list of `DeviceSlot` (one per module, each
  wrapping a driver `Device`), feeding `MultiDevicePlot` (EMG) and `IMUPlot`
  (accel/gyro). A Qt timer `_tick()` (GUI thread) drains each slot's EMG queue
  **and** IMU queue (`drain_imu`), updates plots, pushes LSL, and records. The RX
  thread only enqueues — all shared-buffer writes happen on the GUI thread.
- `recorder.py` — `CsvRecorder` (one unified long CSV: EMG + IMU + marker rows
  via a `stream` column) and `Hdf5Recorder` (per-device groups, resizable
  emg/imu datasets, `/markers`); pick via `make_recorder(fmt, ...)`. By default
  wrapped in a `ThreadedRecorder` so writes happen off the GUI thread (queue +
  periodic flush). h5py optional.
- `lsl_streamer.py` — `LslManager` opens per-device EMG + 6-axis IMU outlets plus
  one session-wide string marker stream. pylsl optional (guarded import).

Markers are **host-side** (digit keys / button bar from `experiment.markers`):
one host timestamp fans into the recorder and the LSL marker stream — no device
round-trip. `gui/monomod_config.yaml` is the canonical config schema; "Load YAML"
applies EMG config to connected devices, while "Load Session" connects to every
`session.devices` entry and applies names, per-device EMG (ADS1293 only), markers,
recording prefs, and the default view.

## Conventions & gotchas

- Firmware protocol symbols are prefixed `MONOMOD_` (macros/enums) and `monomod_`
  (structs/types); the Python side uses unprefixed `proto.*` constants. The wire
  protocol is "MONOMOD Protocol v1.3" (originally derived from AxonCtrl; all
  `axon`/`AXON_` naming was renamed to `monomod`/`MONOMOD_`).
- Many config fields are documented-but-not-yet-wired (marked in
  `monomod_config.yaml` and with `TODO` in `main.cpp`, e.g. `START` does not yet
  apply sample_rate/ch_mask to the ADC). Don't assume a YAML knob reaches hardware
  without checking the firmware path.
- The XIAO C3 has no UART-to-USB chip; console + flashing go over the built-in
  USB-Serial/JTAG. `main.cpp` installs that driver and sets stdin non-blocking so
  `getchar()` (and `provision_wifi.py`) actually receive bytes.
- `idf.py monitor` and `provision_wifi.py` both hold `/dev/ttyACM0` — only one can
  own the port at a time.
