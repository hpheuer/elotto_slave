# E-Lotto Slave — GCP Measurement Node (ESP32-P4)

Slave firmware for the [E-Lotto GCP project](https://github.com/hpheuer/elotto). A second
ESP32-P4 that runs the **identical GCP engine** as the master (same Z-score math, same
extraction code) and no lottery logic. It waits on **UDP port 5000** for commands from the
master and measures on its **own independent noise source**, giving the combined system a √2
SNR boost (`z = (z_master + z_slave) / √2`).

**Its own source, never a shared one.** This node has its **own OV5647 camera** on its CSI
connector, capped and in the dark; entropy is photon shot + read noise from non-overlapping
frame pairs. Sharing one camera between the two nodes would make the two measurements identical
by construction and the √2 gain fictional. The on-chip TRNG (register `0x501101A4`) remains as
the alternative source.

Full architecture, protocol timing, camera physics and robustness details: see the master
repo's **[Dual-ESP: Master & Slave](https://github.com/hpheuer/elotto#dual-esp-master--slave)**
and **[Camera Entropy](https://github.com/hpheuer/elotto#camera-entropy-ov5647-dark-frame)**
sections, plus `docs/PLAN_NETWORK.md` for the transport.

## Ethernet, not UART (Phase C)

The UART1 crossover (GPIO14/15, 460800 baud) is gone. This node is a plain Ethernet node on the
switch: the master triggers it with a **broadcast datagram** — one packet, so at n nodes they
all start within microseconds instead of after n sequential UART writes — and it answers
unicast.

It also runs a small **HTTP server**, and that is not a convenience. With
`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y`, an image that cannot be reached over the network is
reverted on the next boot *by design*, so a slave without a webserver cannot be installed at
all. That is why this node ran the recovery updater from Phase A until Phase C gave it one.

| endpoint | |
|---|---|
| `GET /` | what this node is |
| `GET /diag` | camera health, active source, firmware identity |
| `GET /otainfo` | image version / slot / state |
| `POST /update` | push firmware; **409** while a measurement is running |

## ⚠ Three components come from the master repo

`CMakeLists.txt` pulls them in with `EXTRA_COMPONENT_DIRS=../elotto/components`, so:

- **The two repos must sit next to each other on disk** (`…/elotto` and `…/elotto_slave`).
  There is no copy here — one source of truth means byte-identical extraction on both nodes
  (`elotto_camera`), a wire format the two ends cannot disagree about (`elotto_link`), and one
  implementation of the logic that decides whether a node is still reachable (`elotto_ota`).
- **A change to any of them affects several nodes** — build and flash all of them, and commit
  the repos together.

Camera: OV5647 on the CSI connector, SCCB on GPIO8/7, XCLK unwired (the RPi-style module clocks
itself). PSRAM is mandatory when the camera source is used.

## Protocol (ASCII over UDP; the slave only ever answers)

Every datagram is one frame: `EL1 <seq> <payload>`. The payload is unchanged from the UART era
— swapping the transport was deliberately kept separate from changing what is measured.

| Command | Reply | Meaning |
|---|---|---|
| `P` | `OK` | Discovery (broadcast; replaces the wired ping) |
| `B<n>` | `OK` after n runs | Baseline calibration, stores own baseline mean; **re-arms the camera** (marks a session start) |
| `M` | `Z:<float>,<C\|T>` | One measurement, baseline-corrected Z + the source it actually used (**C**amera / **T**RNG) |
| `D` | `D:<ready>,<bias>,<σ>,<Mbit/s>,<stalls>,<stuck>,<C\|T>` | Camera diagnostics; the master asks once per loop for its `/loops` table |
| `A` | `OK` | Abort (also polled mid-run) |

The source tag sits **after** the float so `atof()` still parses the number. Reporting `T`
during a camera session makes the **master abort**: mixing sources mid-session would change the
physics being measured with no record of which runs were affected. Re-arming on `B` keeps one
transient stall from latching this node to TRNG until power-cycle.

**Why the sequence number.** UART was lossless and ordered, so a reply could only belong to the
command just sent. UDP guarantees neither. A late reply, accepted blindly, would pair `z_slave`
of run *k* with `z_master` of run *k+1* — a correlation bug that looks exactly like physics. So
mismatched frames are dropped and counted (`net_stale` in the master's `/status`), never used.
The master resends under the **same** sequence number and this node answers a repeat of a
completed command from a one-entry cache, so a lost reply costs a round trip rather than a
second measurement.

## ⚠ Keep in sync with the master

- The wire format lives in `components/elotto_link/` and is compiled into both ends — there is
  nothing to keep in sync by hand, which is the point.
- `CAM_SEGMENTS` / `TRNG_SEGMENTS` must match the master's, or the two nodes integrate over
  different run lengths.
- The **yield/abort-poll cadence in `gcp_zscore_raw()`** must match the master's — per-run wall
  time is max(master, slave), so a mismatch slows every measurement to the slower device.
- **Task priority is load-bearing.** The entropy consumer here is the `link` task, created at
  `ELOTTO_CAM_TASK_PRIO + 1`. It must not live in `app_main`, whose priority IDF hardcodes to 1
  — below the camera extraction task. Without the higher priority the producer starves the
  consumer and a run takes 5.1 s instead of 0.47 s (signature: ring `drops` huge, `waits == 0`).

## Build & Flash

Flashing is **over Ethernet**, like every other node:

```powershell
cd D:\E-Lotto\elotto
.\build.ps1 -C ../elotto_slave build
curl http://192.168.178.103/update --data-binary @../elotto_slave/build/elotto_slave.bin
```

The node writes the inactive slot, reboots, and marks itself valid only once its webserver
answers, so a failed transfer or a dead image cannot strand it. USB (COM6) is only for the
bootloader, the partition table, or a board whose recovery updater is gone.

`sdkconfig.defaults` must keep `CONFIG_ESP32P4_SELECTS_REV_LESS_V3=y` **before**
`CONFIG_ESP32P4_REV_MIN_0=y` — the latter depends on it, and without it the choice silently
falls back to rev v3.1 and the binary refuses to boot on these v1.3 boards. It must also point
`CONFIG_PARTITION_TABLE_CUSTOM_FILENAME` at `../elotto/partitions.csv`: all three projects share
one table, or a board flashed by one cannot be updated by another.
