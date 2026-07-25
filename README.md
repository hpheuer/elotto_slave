# E-Lotto Slave — GCP Measurement Node (ESP32-P4)

Slave firmware for the [E-Lotto GCP project](https://github.com/hpheuer/elotto). A second
ESP32-P4 that runs the **identical GCP engine** as the master (same Z-score math, same
extraction code) but nothing else — no Ethernet, no webserver, no lottery logic. It waits on
UART1 for commands from the master and measures on its **own independent noise source**, giving
the combined system a √2 SNR boost (`z = (z_master + z_slave) / √2`).

**Its own source, never a shared one.** This node has its **own OV5647 camera** on its CSI
connector, capped and in the dark; entropy is photon shot + read noise from non-overlapping
frame pairs. Sharing one camera between the two nodes would make the two measurements identical
by construction and the √2 gain fictional. The on-chip TRNG (register `0x501101A4`) remains as
the alternative source.

Full architecture, wiring, protocol timing, camera physics and robustness details:
see the master repo's **[Dual-ESP: Master & Slave](https://github.com/hpheuer/elotto#dual-esp-master--slave)**
and **[Camera Entropy](https://github.com/hpheuer/elotto#camera-entropy-ov5647-dark-frame)** sections.

## ⚠ The camera component comes from the master repo

`CMakeLists.txt` pulls it in with `EXTRA_COMPONENT_DIRS=../elotto/components`, so:

- **The two repos must sit next to each other on disk** (`…/elotto` and `…/elotto_slave`).
  There is no copy here — one source of truth means byte-identical extraction on both nodes.
- **A change to `components/elotto_camera/` affects both nodes** — build and flash both, and
  commit both repos together.

## Wiring (UART1 crossover, 460800 baud 8N1)

| Master | | Slave |
|---|:---:|---|
| GPIO14 (TX) | → | GPIO15 (RX) |
| GPIO15 (RX) | ← | GPIO14 (TX) |
| GND | ↔ | GND |

Camera: OV5647 on the CSI connector, SCCB on GPIO8/7, XCLK unwired (the RPi-style module clocks
itself). PSRAM is mandatory when the camera source is used.

## Protocol (ASCII, line-based; slave only ever answers)

| Command | Reply | Meaning |
|---|---|---|
| `P\n` | `OK\n` | Ping (startup detection) |
| `B<n>\n` | `OK\n` after n runs | Baseline calibration, stores own baseline mean; **re-arms the camera** (marks a session start) |
| `M\n` | `Z:<float>,<C\|T>\n` | One measurement, baseline-corrected Z + the source it actually used (**C**amera / **T**RNG) |
| `D\n` | `D:<ready>,<bias>,<σ>,<Mbit/s>,<stalls>,<stuck>,<C\|T>\n` | Camera diagnostics; the master asks once per loop for its `/loops` table |
| `A\n` | `OK\n` | Abort (also polled mid-run) |

The source tag sits **after** the float so `atof()` still parses the number. Reporting `T`
during a camera session makes the **master abort**: mixing sources mid-session would change the
physics being measured with no record of which runs were affected. Re-arming on `B` keeps one
transient stall from latching this node to TRNG until power-cycle.

## ⚠ Keep in sync with the master

- `UART_BAUD` must equal `SLAVE_BAUD` in the master's `sensor.c`.
- `CAM_SEGMENTS` / `TRNG_SEGMENTS` must match the master's, or the two nodes integrate over
  different run lengths.
- The **yield/abort-poll cadence in `gcp_zscore_raw()`** must match the master's — per-run wall
  time is max(master, slave), so a mismatch slows every measurement to the slower device.
- **Task priority is load-bearing.** `app_main` *is* the entropy consumer here, and IDF hardcodes
  its priority to 1 — below the camera extraction task. It therefore calls
  `vTaskPrioritySet(NULL, ELOTTO_CAM_TASK_PRIO + 1)` at startup. Without it the producer starves
  the consumer and a run takes 5.1 s instead of 0.47 s (signature: ring `drops` huge,
  `waits == 0`).

## Build & Flash

```powershell
# IDF terminal (ESP-IDF v6.0.1, target esp32p4)
cd D:\E-Lotto\elotto_slave
idf.py build
idf.py -p COM6 flash
```

`sdkconfig.defaults` must keep `CONFIG_ESP32P4_SELECTS_REV_LESS_V3=y` **before**
`CONFIG_ESP32P4_REV_MIN_0=y` — the latter depends on it, and without it the choice silently
falls back to rev v3.1 and the binary refuses to boot on these v1.3 boards.
