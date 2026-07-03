# E-Lotto Slave — GCP Measurement Node (ESP32-P4)

Slave firmware for the [E-Lotto GCP project](https://github.com/hpheuer/elotto). A second
ESP32-P4 that runs the **identical GCP engine** as the master (same TRNG register
`0x501101A4`, same 32,000 × 200-bit Z-score math) but nothing else — no Ethernet, no
webserver, no lottery logic. It waits on UART1 for commands from the master and measures on
its **own independent TRNG**, giving the combined system a √2 SNR boost
(`z = (z_master + z_slave) / √2`).

Full architecture, wiring, protocol timing and robustness details:
see the master repo's **[Dual-ESP: Master & Slave](https://github.com/hpheuer/elotto#dual-esp-master--slave)** section.

## Wiring (UART1 crossover, 460800 baud 8N1)

| Master | | Slave |
|---|:---:|---|
| GPIO14 (TX) | → | GPIO15 (RX) |
| GPIO15 (RX) | ← | GPIO14 (TX) |
| GND | ↔ | GND |

## Protocol (ASCII, line-based; slave only ever answers)

| Command | Reply | Meaning |
|---|---|---|
| `P\n` | `OK\n` | Ping (startup detection) |
| `B<n>\n` | `OK\n` after n runs | Baseline calibration, stores own baseline mean |
| `M\n` | `Z:<float>\n` | One measurement, returns baseline-corrected Z |
| `A\n` | `OK\n` | Abort (also polled mid-run every 8000 segments) |

## ⚠ Keep in sync with the master

`UART_BAUD` must equal `SLAVE_BAUD` in the master's `sensor.c`, and the **yield cadence in
`gcp_zscore_raw()` (`seg % 8000`) must match the master's** — per-run wall time is
max(master, slave), so a mismatch slows every measurement to the slower device.

## Build & Flash

```powershell
# IDF terminal (ESP-IDF v6.0.1, target esp32p4)
cd D:\E-Lotto\elotto_slave
idf.py build
idf.py -p COM6 flash
```
