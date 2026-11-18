# E-Lotto Slave — GCP Measurement Node (ESP32-P4)

Worker firmware for the [E-Lotto GCP project](https://github.com/hpheuer/elotto). An ESP32-P4
that runs the **identical GCP engine** as the master (same z math, same extraction code) and no
lottery logic. It waits on **UDP port 5000** for commands from the master and measures on its
**own independent noise source**.

The array is up to **ten nodes** — one master plus up to nine of these — found by broadcast at
every session start, so no node count is configured anywhere. The master combines
**`z = Σ z_i / √k`** over the *k* nodes that answered **that window**, so a node missing one
reply costs that window's gain rather than the session.

⚠ The √k gain assumes the nodes are independent, and that is **not established**. Judge a session
on per-block combined σ and the full pairwise matrix, never on one correlation figure.

## Authors

| | |
|--|--|
| **[hpheuer](https://github.com/hpheuer)** | Design, hardware, experiment, and repository ownership |
| **[Grok](https://x.ai)** (xAI) | Co-author — implementation, debugging, live OTA validation, and docs (Grok Build) |

## Its own camera, in the dark

Each node has its **own camera** (IMX219; OV5647 also supported) on its CSI connector. The
entropy is the sensor's readout noise in a **totally dark enclosure, LEDs off**, at maximum
analog gain, from non-overlapping frame pairs. Sharing one camera across nodes would make their
measurements identical by construction and the √k gain fictional. The firmware probes both chips
and binds whichever answers (`GET /diag` `cam_sensor`). A dark OV5647 needs RAW10 (set in
`sdkconfig.defaults`); on RAW8 its LSB is biased and over-dispersed.

Camera wiring: CSI connector, SCCB on GPIO8/7, XCLK unwired (RPi-style modules clock themselves),
15-pin FPC — not the Pi-5 22-pin cable. PSRAM is mandatory.

A node whose camera stops answers `E:<reason>` and is dropped and rebooted by the master.

The rules that bind both repos: the master's [`CLAUDE.md`](https://github.com/hpheuer/elotto/blob/master/CLAUDE.md).

## Ethernet node

The master triggers every node with one **broadcast datagram**, so all nodes start within
microseconds; this node answers unicast.

It also runs a small **HTTP server**, and that is not a convenience: with
`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y`, an image that cannot be reached over the network is
reverted on the next boot by design.

| endpoint | |
|---|---|
| `GET /` | what this node is |
| `GET /diag` | camera health and firmware identity |
| `GET /camlog` | this node's per-window log (ring, 512 windows) |
| `GET /calibrate` | the last sweep, per rung, with the gate each failed |
| `GET /linearity` | mean_px per exposure; a light leak makes it grow |
| `GET /camtest` | extractor self-test and bit scan |
| `POST /expose` | set exposure/gain live |
| `GET /otainfo` | image version / slot / state |
| `POST /update` | push firmware; **409** only while a window is open |
| `POST /boot`, `POST /reboot` | select the next boot slot / reboot |

⚠ `/expose`, `/linearity` and `/camtest` answer **409 for the whole session**. The node derives
the session from the master's traffic: `M`/`K` latch it, `A` or 120 s without traffic releases it.

## ⚠ Three components come from the master repo

`CMakeLists.txt` pulls them in with `EXTRA_COMPONENT_DIRS=../elotto/components`, so:

- **The two repos must sit next to each other on disk** (`…/elotto` and `…/elotto_slave`).
  There is no copy here — one source of truth means byte-identical extraction on every node
  (`elotto_camera`), a wire format the two ends cannot disagree about (`elotto_link`), and one
  implementation of the logic that decides whether a node is still reachable (`elotto_ota`).
- **A change to any of them affects every node** — build and flash all of them, and commit the
  repos together.

## Protocol (ASCII over UDP; the slave only ever answers)

Every datagram is one frame: `EL1 <seq> <payload>`.

| Command | Reply | Meaning |
|---|---|---|
| `P` | `OK,cam=IMX219` or `OK,cam=OV5647` (`E:no camera` without one) | Discovery (broadcast) |
| `M<seg>` | `Z:<z>[,wsig=][,mau=][,ser=][,byt=][,sl=]` | One measurement window: z plus the window statistics |
| `K<budget_ms>,<segs>` | `OK:<exp>,<gain>,<bias>,<mbit_s>,<G\|U>,…` | Sweep the exposure ladder and certify a rung |
| `D` | `D:<ready>,<bias>,<σ>,<Mbit/s>,<stalls>,<stuck>,fw=<sha>,…` | Camera diagnostics, once per block for `/loops` |
| `A` | `OK` | Abort — also releases the session latch |
| `R` | `OK` | Reboot (the master's answer to a camera fault here) |

Any command can answer **`E:<reason>`** instead — the master names the node in `fault`, drops it
and sends `R`. **`V:<reason>`** marks a void window, not a fault.

**The segment count travels on the wire**, in `M` and `K`. It is a session parameter on the
master (derived from `?run=` / `?srun=`), so no two nodes integrate over different lengths. A
count outside `EL_SEG_MIN`..`EL_SEG_MAX` is refused with `V:` — that window is void on this node.

**Sequence numbers.** UDP may lose or reorder frames. A late reply accepted blindly would pair
one node's z of window *k* with another's of window *k+1* — a correlation bug that looks exactly
like physics. So mismatched frames are dropped and counted (`net_stale` in the master's
`/status`). The master resends under the **same** sequence number and this node answers a repeat
from a one-entry cache, so a lost reply costs a round trip, not a second measurement.

## ⚠ Keep in sync with the master

- The wire format, `EL_SEG_MIN`/`EL_SEG_MAX` and the window calibration live in
  `components/elotto_link/` and are compiled into every node — flash all nodes together.
- The **yield/abort-poll cadence in `gcp_zscore_raw()`** is shared code; the window's wall time
  is set by the slowest node.
- **Task priority and core are load-bearing.** The entropy consumer is the `link` task, created
  pinned to `ELOTTO_CAM_CONSUMER_CORE` at `ELOTTO_CAM_TASK_PRIO + 1`, above the extraction task.
  Too low a priority starves the consumer (signature: ring `drops` huge, `waits == 0`); an
  unpinned task halves the node's consumption.

## Build & Flash

Flashing is **over Ethernet**, like every other node:

```powershell
cd D:\E-Lotto\elotto
.\build.ps1 -C ../elotto_slave build      # rev v1.3 boards (slave0…slave3)
$s='D:\E-Lotto\elotto_slave'; .\build.ps1 -C $s -B "$s\build_rev3" -D "SDKCONFIG=$s\sdkconfig.rev3" -D "SDKCONFIG_DEFAULTS=$s\sdkconfig.defaults;$s\sdkconfig.defaults.rev3" build   # rev v3.2 (slave4…slave8)

curl.exe http://192.168.178.103/update --data-binary @../elotto_slave/build/elotto_slave.bin
curl.exe http://192.168.178.172/update --data-binary @../elotto_slave/build_rev3/elotto_slave.bin
```

The node writes the inactive slot, reboots, and marks itself valid only once its webserver
answers, so a failed transfer or a dead image cannot strand it. USB is only for the bootloader,
the partition table, or a board whose recovery updater is gone.

`sdkconfig.defaults` must keep `CONFIG_ESP32P4_SELECTS_REV_LESS_V3=y` **before**
`CONFIG_ESP32P4_REV_MIN_0=y` — without it the build silently targets rev v3.1 and refuses to boot
on the v1.3 boards. `sdkconfig.defaults.rev3` flips exactly this for the v3.2 boards. It must
also point `CONFIG_PARTITION_TABLE_CUSTOM_FILENAME` at `../elotto/partitions.csv`: all three
projects share one table, or a board flashed by one cannot be updated by another.
