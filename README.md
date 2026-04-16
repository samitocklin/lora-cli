# lora-cli

Command-line tool for the **LR1120 / LR1121** LoRa radio chip on Raspberry Pi.
Targets the Waveshare LR1121 HAT but the pin mapping is adjustable in `cli/board.c`.

Supports LoRa and GFSK modulation on both sub-GHz (430–928 MHz) and 2.4 GHz bands.

---

## Features

| Mode | Description |
|---|---|
| `--status` | Read chip version, system status and error registers |
| `--rx` | Continuous receive — prints each packet with RSSI/SNR and hex dump |
| `--tx` | Transmit one or more packets at configurable power and rate |
| `--scan` | Fast CAD-based frequency scan with live ANSI spectrum display |
| `--txtest` | RF test: unmodulated CW or infinite preamble for spectrum analysers |
| `--mavlink` | Bidirectional MAVLink telemetry bridge over 2.4 GHz LoRa (TDMA) |

Both LoRa and GFSK modems are supported on `--rx`, `--tx`, and `--txtest`.

---

## Hardware

### LR1121 HAT GPIO mapping (Waveshare / generic)

| Signal | GPIO (BCM) | Notes |
|---|---|---|
| RESET | 25 | Active low |
| BUSY | 5 | High while chip is processing |
| DIO8 | 26 | (unused in current firmware) |
| IRQ (DIO9) | 23 | Rising edge on TX\_DONE / RX\_DONE |
| LED TX | 4 | |
| LED RX | 17 | |
| LED HB | 27 | Heartbeat / activity |
| SPI | `/dev/spidev0.0` | SCLK=11, MOSI=10, MISO=9, CS0=8 |

RF switch: SKY13588 controlled via DIO5 (RFSW0) / DIO6 (RFSW1).
TCXO: 3.3 V supply, 10 ms startup time.

---

## Dependencies

```bash
sudo apt install gcc libgpiod-dev pkg-config
```

Requires libgpiod **v2** (Raspberry Pi OS Bookworm and later include it).
Check: `pkg-config --modversion libgpiod` — needs 2.x.

---

## Build

```bash
make            # native build on Pi
make install    # copy to /usr/local/bin/lora-cli
```

Cross-compile from an x86 host (requires arm-linux-gnueabihf toolchain and sysroot):

```bash
make CROSS=1 SYSROOT=/path/to/pi-sysroot
```

The Semtech SWDR001 driver sources are vendored in `src/` and compiled in.
No external LoRa library dependency.

---

## Pi hardware setup

### Enable SPI

```
# /boot/firmware/config.txt  (or /boot/config.txt on older images)
dtparam=spi=on
```

Reboot, then verify: `ls /dev/spidev0.*`

### Permissions

Run as root or add your user to the `spi` and `gpio` groups:

```bash
sudo usermod -aG spi,gpio $USER
# log out and back in
```

In practice `sudo lora-cli ...` is the simplest approach.

---

## Modes

All modes require `--spi spi0` (or `spi1`). This is always the first argument.

```
sudo lora-cli --spi spi0 <MODE> [options]
```

---

### `--status` — chip info

Reads and prints chip ID, hardware revision, firmware version, system status,
error flags, raw VBAT and temperature readings. Useful to verify the SPI
connection is working before attempting radio operations.

```bash
sudo lora-cli --spi spi0 --status
```

---

### `--rx` — continuous receive

Listens on the configured frequency and prints each received packet as a
hex/ASCII dump with signal statistics (RSSI, SNR, signal RSSI for LoRa;
RSSI-sync and RSSI-avg for GFSK).

```bash
# LoRa RX, 868.3 MHz, SF9 BW125
sudo lora-cli --spi spi0 --rx --freq 868300000 --sf 9 --bw 125

# LoRa RX, 2.44 GHz
sudo lora-cli --spi spi0 --rx --freq 2440000000 --sf 7 --bw 800

# Meshtastic EU868 Long Fast (SF11 BW250, sync 0x2B)
sudo lora-cli --spi spi0 --rx --freq 869525000 --sf 11 --bw 250 \
              --sync-word 2B --preamble 16

# GFSK RX, 868 MHz, 50 kbps
sudo lora-cli --spi spi0 --rx --fsk --freq 868000000 --bitrate 50000

# With boosted LNA for better sensitivity (sub-GHz only)
sudo lora-cli --spi spi0 --rx --freq 868100000 --bw-boosted
```

Runs until **Ctrl+C**.

**LoRa flags:** `--freq`, `--sf`, `--bw`, `--cr`, `--preamble`, `--sync-word`,
`--no-crc`, `--implicit`, `--payload-len`, `--ldro`, `--invert-iq`, `--bw-boosted`

**GFSK flags:** `--fsk`, `--bitrate`, `--fdev`

---

### `--tx` — transmit packets

Transmits a payload one or more times with a configurable inter-packet delay.

```bash
# Send "hello world" once at 14 dBm, 868.3 MHz SF7
sudo lora-cli --spi spi0 --tx --freq 868300000 --payload "hello world"

# Send 10 packets, 1 second apart
sudo lora-cli --spi spi0 --tx --freq 868300000 --payload "ping" \
              --count 10 --interval 1000

# Flood continuously (no delay)
sudo lora-cli --spi spi0 --tx --freq 868300000 --payload "flood" \
              --count 0 --interval 0

# Sub-GHz HP PA, 22 dBm
sudo lora-cli --spi spi0 --tx --freq 868000000 --power 22 --hp-pa \
              --payload "high power"

# Hex payload
sudo lora-cli --spi spi0 --tx --freq 868000000 --payload-hex deadbeef01020304

# GFSK TX, 50 kbps
sudo lora-cli --spi spi0 --tx --fsk --freq 868000000 --bitrate 50000 \
              --payload "gfsk packet"
```

**LoRa flags:** `--freq`, `--sf`, `--bw`, `--cr`, `--preamble`, `--sync-word`,
`--no-crc`, `--implicit`, `--ldro`, `--invert-iq`

**GFSK flags:** `--fsk`, `--bitrate`, `--fdev`

**TX-specific:** `--power`, `--hp-pa`, `--payload`, `--payload-hex`, `--count`, `--interval`

#### PA selection

| Band | Flag | PA | Supply | Max power |
|---|---|---|---|---|
| sub-GHz | (default) | LP | VREG | +15 dBm |
| sub-GHz | `--hp-pa` or `--power > 15` | HP | VBAT | +22 dBm |
| 2.4 GHz | (automatic) | HF | VREG | +13 dBm |

---

### `--scan` — CAD frequency scan

Performs rapid Channel Activity Detection (CAD) sweeps across a frequency range
and displays a live ANSI spectrum with colour-coded activity bars.

Colour scale is absolute (fraction of CAD passes that detected activity):
gray (0%) → dark green → green → yellow → orange → red (≥80%).

```bash
# Default EU868 band scan
sudo lora-cli --spi spi0 --scan

# Custom range with 100 kHz steps, continuous
sudo lora-cli --spi spi0 --scan \
              --freq-start 863000000 --freq-end 928000000 \
              --freq-step 100000 --continuous

# Tune sensitivity (lower peak/min = more sensitive, more false positives)
sudo lora-cli --spi spi0 --scan --cad-symb 2 --cad-peak 26 --cad-min 10
```

Default CAD thresholds per SF (conservative, suppress thermal noise):

| SF | detect\_peak | detect\_min |
|---|---|---|
| ≤6 | 30 | 14 |
| 7–8 | 28 | 12 |
| ≥9 | 24 | 10 |

If the display shows all red, raise `--cad-peak` (e.g. `--cad-peak 32`) or
switch to 2-symbol CAD (`--cad-symb 2`).

**Flags:** `--freq-start`, `--freq-end`, `--freq-step`, `--sf`, `--bw`,
`--cad-symb`, `--cad-peak`, `--cad-min`, `--continuous`

---

### `--txtest` — RF transmitter test

Locks the transmitter on for use with a spectrum analyser or power meter.
The radio transmits continuously until **Ctrl+C**.

> **Warning:** continuous transmission — ensure you are authorised to transmit
> on the selected frequency before using this mode.

Two sub-modes:

#### CW — unmodulated carrier (`--cw`)

Pure sine wave at exactly the configured frequency. Use this to:
- Measure output power at the antenna port
- Check PA harmonic content
- Verify TCXO frequency accuracy

```bash
# Sub-GHz CW, 14 dBm LP PA
sudo lora-cli --spi spi0 --txtest --cw --freq 868000000 --power 14

# Sub-GHz CW, 22 dBm HP PA (via VBAT rail)
sudo lora-cli --spi spi0 --txtest --cw --freq 868000000 --power 22 --hp-pa

# 2.4 GHz CW, 13 dBm HF PA
sudo lora-cli --spi spi0 --txtest --cw --freq 2440000000 --power 13
```

#### Infinite preamble — modulated (default)

Continuous modulated preamble at 100% duty cycle. Use this to:
- Measure occupied bandwidth
- Verify spectral mask compliance
- Test PA linearity under a realistic envelope

```bash
# LoRa preamble, SF7 BW125 at 868 MHz
sudo lora-cli --spi spi0 --txtest --freq 868000000 --sf 7 --bw 125

# LoRa preamble, 2.4 GHz SF7 BW800
sudo lora-cli --spi spi0 --txtest --freq 2440000000 --sf 7 --bw 800

# GFSK preamble, 50 kbps, 25 kHz deviation
sudo lora-cli --spi spi0 --txtest --fsk --freq 868000000 \
              --bitrate 50000 --fdev 25000

# GFSK preamble, 2.4 GHz, 100 kbps
sudo lora-cli --spi spi0 --txtest --fsk --freq 2440000000 --bitrate 100000
```

**Flags:** `--freq`, `--power`, `--hp-pa`, `--cw`, `--sf`, `--bw` (LoRa preamble),
`--fsk`, `--bitrate`, `--fdev` (GFSK preamble)

---

### `--mavlink` — MAVLink telemetry bridge

Bridges a MAVLink serial stream over a 2.4 GHz LoRa link between two Pi Zeros.
Designed for drone telemetry where one Pi is on the aircraft and one is at the
ground station.

#### Protocol: fixed TDMA ping-pong

The air-side Pi (drone) is the TDMA master. It transmits one LoRa packet every
`slot_ms` milliseconds regardless of queue depth, then immediately listens for
the ground station's response. The ground side listens, responds after a short
guard interval, and the cycle repeats.

```
← slot_ms (200 ms default) ──────────────────────────────────→
  [AIR TX ~62 ms] [10 ms guard] [GND TX ~62 ms] [~66 ms idle]
```

At SF7/BW800 each 250-byte LoRa packet takes ≈62 ms on air, giving ≈5 Hz
in each direction — sufficient for MAVLink telemetry and GCS commands.

#### Air side (drone FC connected via UART)

```bash
sudo lora-cli --spi spi0 --mavlink --air \
              --freq 2440000000 --sf 7 --bw 800 \
              --uart /dev/serial0 --baud 57600
```

#### Ground side (TCP server for mavproxy on laptop)

```bash
sudo lora-cli --spi spi0 --mavlink \
              --freq 2440000000 --sf 7 --bw 800 \
              --tcp-port 5760
```

Then on your laptop:

```bash
mavproxy.py --master tcp:<ground-pi-ip>:5760
```

#### UART setup on Pi Zero W

The hardware UART must be freed from Bluetooth. Add to `/boot/firmware/config.txt`:

```
enable_uart=1
dtoverlay=disable-bt
```

Reboot. `/dev/serial0` → `/dev/ttyAMA0` is then the hardware UART on GPIO pins
14 (TX) and 15 (RX).

Typical FC baud rates: 57600 (ArduPilot default), 115200, 921600.

#### TDMA tuning

At the defaults (SF7, BW800, slot 200 ms) each direction gets ≈5 Hz and
≈17 kbps of capacity. Adjust with:

| Flag | Default | Notes |
|---|---|---|
| `--slot-ms` | 200 | Lower = faster, must be > AIR tx + guard + GND tx (~134 ms min for SF7/BW800 full packets) |
| `--guard-ms` | 10 | Reduce to 5 if processing is fast; raise if you see packet collisions |
| `--sf` | 7 | Higher SF = more range, slower data rate, larger slot needed |
| `--bw` | 800 | 2.4 GHz only: 200/400/800 kHz |
| `--power` | 13 | HF PA max is +13 dBm at 2.4 GHz |

#### Testing with PTY (single machine)

```bash
sudo lora-cli --spi spi0 --mavlink --air --freq 2440000000 \
              --pty /tmp/ttyLORA0
mavproxy.py --master /tmp/ttyLORA0
```

**Flags:** `--air`, `--uart`, `--baud`, `--tcp-port`, `--pty`, `--slot-ms`, `--guard-ms`,
plus standard modem flags `--freq`, `--sf`, `--bw`, `--cr`, `--preamble`, `--power`

---

## GFSK modem

Add `--fsk` to any `--rx`, `--tx`, or `--txtest` command to use GFSK instead of LoRa.

Fixed parameters (not currently exposed as flags):
- Pulse shape: GFSK BT=0.5
- Preamble: 5 bytes (40 bits of 0xAA)
- Preamble detector: 16-bit minimum
- Sync word: 4 bytes `0x2D 0xD4 0x2D 0xD4` (same as SX127x default)
- Header: variable length
- CRC: 2-byte CCITT
- Whitening: IBM LFSR (on)
- RX bandwidth: auto-calculated as 1.5 × Carson BW = 1.5 × 2 × (fdev + bitrate/2)

GFSK is compatible at the packet level with SX1276/SX1278 devices using the same
sync word and settings.

---

## Sync words (LoRa)

The sync word is a single byte that the hardware uses to filter packets before
the CPU sees them. Mismatched sync words are discarded silently.

| Value | Network |
|---|---|
| `0x12` | Private LoRa / satellite (default) |
| `0x34` | LoRaWAN public |
| `0x2B` | Meshtastic |

```bash
--sync-word 12   # private (default, can be omitted)
--sync-word 34   # LoRaWAN
--sync-word 2B   # Meshtastic
--sync-word AB   # any custom byte
```

---

## Project structure

```
lora-cli/
├── Makefile
├── cli/
│   ├── board.h / board.c      platform HAL: GPIO (libgpiod v2), SPI (spidev)
│   ├── radio.h / radio.c      param structs, enum converters, shared modem setup
│   ├── mode_status.c          --status
│   ├── mode_rx.c              --rx
│   ├── mode_tx.c              --tx
│   ├── mode_scan.c            --scan
│   ├── mode_txtest.c          --txtest
│   ├── mode_mavlink.c         --mavlink
│   └── main.c                 argument parser and dispatch
└── src/
    └── lr11xx_*.c / .h        Semtech SWDR001 driver (BSD-licensed, vendored)
```
