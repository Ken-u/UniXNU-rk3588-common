# UniXNU RK3588 support

Shared Rockchip RK3588 support for [UniXNU](https://unixnu.com): CPU topology,
GICv3 (GIC-600), the 16550-compatible debug UART, PSCI restart, secondary-core
bring-up from EL2, and the personality of the DesignWare MSHC eMMC host.

`soc.toml` selects `ArmCpu` and `ArmDTPlatform` from the UniXNU component
catalog. Device repositories supply memory layout, peripherals, firmware,
partition layout, and packaging. [`UniXNU-evb10`](https://github.com/Ken-u/UniXNU-evb10)
is the first device integration.

Build through the [UniXNU CLI](https://unixnu.com/building).
Read the [device repository specification](https://unixnu.com/porting/device-repository),
[device-tree specification](https://unixnu.com/porting/device-tree),
and [component catalog](https://unixnu.com/reference/components).

## Contents

| Path | Contents |
| --- | --- |
| `soc.toml` | SoC profile and component selection |
| `configs/rk3588.config` | Kernel configuration fragment |
| `dts/rk3588.dtsi` | Shared hardware resources: CPUs (the Cortex-A76 cores first), PSCI, GIC, eMMC, UART2, Mali-G610 |
| `patches/xnu/series` | Ordered XNU patches |
| `scripts/apply-armcpu-patches.py` | Archive step: applies the ArmCpu fixes in `scripts/data/` |
| `scripts/build-rk3588-dvfs.py`, `scripts/data/Rk3588Dvfs/` | Archive step and sources of the frequency and voltage management (see [Frequency management](#frequency-management)) |
| `scripts/build-rk3588-xhci.py`, `scripts/data/Rk3588Xhci/` | Archive step and sources of the polled xHCI host driver for the DWC3 controllers (see [USB 3 (xHCI)](#usb-3-xhci)) |
| `scripts/build-rk3588-vdec.py`, `scripts/data/Rk3588Vdec/` | Archive step and sources of the rkvdec2 video decoder bring-up and its self-test (see [Video decoder (rkvdec2)](#video-decoder-rkvdec2)) |
| `kits/` | Reserved for shared platform operations |
| `tools/board/` | Host-side bring-up tools shared by RK3588 boards: serial capture with times, U-Boot console, USB helpers, reset controller. Not part of the kernel build; see its [README](tools/board/README.md) |

## XNU patches

| Patch | Purpose |
| --- | --- |
| `rk3588-pe-serial-16550` | Console driver for the `/arm-io/uart16550` node |
| `rk3588-psci-restart` | Restart and halt through PSCI |
| `rk3588-smp-el2` | Secondary cores start at EL2 and drop to EL1 |
| `rk3588-boot-cpu-mpidr` | The boot CPU is the `/cpus` node whose `reg` is the MPIDR affinity of the core that parses the tree, so the nodes can be listed in any order |
| `rk3588-slow-cores` | `sched-slowcpus=<mask>` keeps threads of the interactive and user-initiated classes off the listed (slow) logical CPUs while a fast core is idle or can be preempted; `sched-slowpri` sets the base priority from which a thread counts, `sched-slowstat=<seconds>` prints the placement counters |
| `rk3588-sctlr-uct` | EL0 access to `CTR_EL0` for the GPU user-space stack |
| `rk3588-emmc-personality` | Matches `rockchip,rk3588-dwcmshc` to the eMMC driver |
| `rk3588-root-trust` | Lets the eMMC driver vouch for the owner's read-only root volume |
| `rk3588-cpu-idle` | `rk3588_cpu_idle_mach(cpu)`: a CPU's idle time from XNU's recount, which the governor of `scripts/data/Rk3588Dvfs` (a kernel extension that cannot see `struct processor`) turns into a busy share |

ArmCpu has no patch channel of its own. `scripts/data/ArmCpu-rk3588-*.patch`
fixes inline assembly (the Darwin assembler treats `;` as a comment), adds
an optional redistributor-based IPI path, and lets the boot core be any
logical core (the secondaries that queue before it registers are finished
from slot 0). A device repository lists
`apply-armcpu-patches.py` in `[kernel] archives` ahead of `component:ArmCpu`.
The patches apply to the ArmCpu revision of the UniXNU `components.toml`:
`93a5030` (UniXNU `e2fdae4`, 2026-11-06) and what follows it. They do not
apply to the older `9066b53` (UniXNU `066c8d8`); the series for that revision
is the one before the commit that rebased it.

## Frequency management

`scripts/data/Rk3588Dvfs/` is a kernel extension the device builds with `build-rk3588-dvfs.py` (same compiler record as the kernel, `-Werror`,
Mach-O audit) and force-loads. The device lists `"build-rk3588-dvfs.py <path>"` in `[kernel] archives`, naming one source of its own that defines the
rails (`Rk3588DvfsBoard.cpp`: which regulator feeds which cluster, its limits and ramp, and the board's input-activity counter; the table and
the contract are in `Rk3588Dvfs.h`), and calls `rk3588_dvfs_start()` from a probe once the kernel is up. What is shared:

| File | Contents |
| --- | --- |
| `Rk3588Scmi` | The firmware's clock service: SCMI over SMC (smc-id `0x82000010`, shared memory `0x10f000`). Clock ids: CPUL 0, DSU 1, CPUB01 2, CPUB23 3, DDR 4, GPU 5, NPU 6 |
| `Rk3588Pmic` | Polled drivers for the regulators of the reference design: RK806 on SPI2 (the Cortex-A55 rail is its DCDC2) and RK8602/RK8603 on I2C (the Cortex-A76 rails), reading the firmware's setup and leaving it in place |
| `Rk3588Domain` | The operating points (voltage per rate) of the Cortex-A55 and Cortex-A76 clusters, and the change from one to another: voltage first when going up, clock first when going down |
| `Rk3588Governor` | The CPU governor (below), including the over-temperature limit |
| `Rk3588Thermal` | The seven TSADC sensors (`0xfec00000`), polled; brings the sampling up when the firmware left it off |
| `Rk3588Gpu` | The Mali-G610's clock and activity: what the GPU runs at (its cycle counter), a busy share from `GPU_STATUS`, and the GPU governor (below) |
| `Rk3588Dvfs` | Boot arguments, and the staged probes used to bring it up (modes 1 to 5 and 8 to 10) |

Why the voltage is part of it: above 600 MHz the firmware clocks the cores from a PVTPLL, a ring oscillator whose speed follows the supply, and the
rate the service reports is only the oscillator's setting. At the 1.0 V the firmware leaves on the Cortex-A76 rails every setting from 1608 MHz up
runs at about 2230 MHz and 816 MHz at about 1220 MHz; the nominal clock comes only at the operating point's own voltage (measured with a dependent add
chain on a bound thread: at the table's voltage the clusters run within 5 % of the rate, from 400 MHz to 2.4 GHz). The oscillator also makes a too low
voltage slow instead of unsafe.

The DSU (the cache every cluster's traffic passes through) takes the Cortex-A55 rate whenever that is set, and a rate of its own afterwards; it sits on
the Cortex-A55 rail. A load from the L3 takes 18 ns with the DSU at 1800 MHz and 52 ns at 400 MHz, from memory 120 and 181 ns, so the governor holds
the DSU at three quarters of the faster Cortex-A76 cluster's rate.

Boot arguments: `rk3588-dvfs=N` (0 off; 1 report the firmware's clocks; 2 and 3 sweep the rates and probe the regulators read-only; 4 sweep the operating
points with their voltages; 5 measure the DSU; **6 run the governors**; 7 the same windows and temperatures without any change, the baseline; 8 sweep the GPU
clock service's rates; 9 watch the GPU's busy share; 10 sweep the GPU's voltage), `rk3588-dvfs-min=MHz`, `rk3588-dvfs-boost-ms=N`, `rk3588-dvfs-tmax=C`,
`rk3588-dvfs-report=N`, `rk3588-gpu=0`, `rk3588-gpu-min=MHz`, `rk3588-gpu-max=MHz`, `rk3588-gpu-boost-ms=N`.

The CPU governor (mode 6) looks at each cluster every 16 ms: the lowest point at or above 1.25 x busy x the current rate, the top point from 90 % busy, up at
once and down after four windows that asked for less; the Cortex-A76 clusters stay at or above half their top rate while there is work and go to the
top for `rk3588-dvfs-boost-ms` (default 300) when the board's activity counter moves (a touch). Every eight windows it reads the sensors (`Rk3588Thermal`):
at or above `rk3588-dvfs-tmax` (default 85 C, 0 = off) the highest point every cluster and the GPU may use goes down one step, and up again 2 C below. It
prints each cluster's changes, mean rate, busy share, voltage and time at each point every ten seconds, with the mean temperature and a model of the cores'
dynamic power (the vendor device tree's `dynamic-power-coefficient`, P = C f V^2 per busy core, against the same work at the top points). A cluster whose
clock service or regulator fails five times is put back at its top point and left alone. The board has no power meter and the temperature sensors resolve
0.93 C a step, so the power figure is a model, not a measurement.

The GPU is not a PVTPLL clock only: the clock service puts the GPU on a PVTPLL for the settings 300 to 900 MHz (bit 14 of `CLKSEL_CON(158)`; the 800 and 900
settings share one ring and differ by the supply only) and on GPLL / 6 for 200 MHz. As with the CPU the ring follows the supply: the cycle counter measures
the setting's nominal rate at the vendor's voltage for it (300 at 675 mV 310 MHz, 600 at 675 mV 598, 700 at 700 mV 719, 800 at 750 mV 852, 900 at 800 mV 957),
about 2.1 to 2.3 MHz per millivolt, and a lower voltage is slower, not unsafe. The service ignores a request for 1000 MHz. The Mali driver sets CPLL / 2 (750 MHz)
itself and never goes through the service. The GPU governor (a thread of its own, started with mode 6) samples `GPU_STATUS` about every millisecond (every 8 ms
after a second without work) and every 25 ms takes the top point at 40 % busy or more, otherwise the lowest that would keep the GPU about 30 % busy, up
at once and down after three windows; a touch holds it at the top for `rk3588-gpu-boost-ms` (default 200); `rk3588-gpu-max` (default 800) caps the setting. Voltage
first on the way up, last on the way down, from the vendor's table (rk3588s.dtsi `gpu_opp_table`, typical die).
Not done: the DDR clock (clock id 4: the service has no rate list for it), and an idle state that powers a cluster down.

## USB 3 (xHCI)

`scripts/data/Rk3588Xhci/` is a polled xHCI host driver for the DWC3 blocks of the RK3588 (`GSNPSID 0x5533300a`; the firmware powers the PHYs and
puts each core into host mode, the driver resets the xHC and takes it from there). The device builds it with `build-rk3588-xhci.py <path>` (same compiler
record as the kernel, `-Werror`, Mach-O audit; `"build-rk3588-xhci.py <path>"` in `[kernel] archives`, naming one source of its own) and force-loads it,
and calls `rk3588_xhci_start()` from a probe once the kernel is up. The board source defines `rk3588_xhci_controllers[]` (name, register base, a DMA
window below 4 GiB for each controller, mapped as normal non-cacheable memory) and `rk3588_xhci_hid_sink` (where decoded keyboard and mouse reports go).

| File | Contents |
| --- | --- |
| `XhciRegs.h`, `XhciCore.h` | Register and context layouts; the controller core: reset and start, command / event / transfer rings, slots, Address Device, Configure Endpoint (with the hub fields), Evaluate Context, control transfers, interrupt and bulk TRBs, endpoint recovery, port reset (hot, warm for an Inactive USB 3 link). A computation over a register accessor and one DMA window, no kernel include |
| `UsbDesc.h`, `HidReport.h` | Descriptor constants and walks, the endpoint context of an endpoint, the layout of a mouse report read from its HID report descriptor |
| `UsbHost.h` | Root port scan, enumeration (route strings, transaction translators, full speed packet size), hubs (USB 2 and SuperSpeed, one TT or one per port, nested), HID boot keyboards and mice |
| `Rk3588Xhci.cpp` | The kernel glue: boot argument, mappings, the register accessor, the thread |
| `tests/` | `XhciModel.h` (a strict model of an xHC and of the devices on its ports) and `test-xhci.cpp`; `tests/run.sh` builds and runs it with the sanitizers |

Boot argument `rk3588-xhci=N`: 0 off (default); 1 print the registers as the firmware left them; 2 also reset and start the controllers and watch the root
ports; 3 also enumerate what is plugged in (descriptors); 4 also drive hubs; 5 also configure and poll HID boot keyboards and mice.

The test runs the real driver code against the model: enumeration and HID at every speed, 32 and 64 byte contexts, scratchpads, hubs behind hubs, unplugging,
stalls and a device that never answers, ring and event-ring wraps, slot exhaustion, short data stages. Mutating the driver (the cycle bit at the link TRB,
PORTSC write hygiene, the TT port, the route string, warm reset, Set TR Dequeue, the residual of a short data stage) makes it fail. That the model agrees with
a real controller is a separate question: the board runs answer it (a device repository's evidence).

## Video decoder (rkvdec2)

`scripts/data/Rk3588Vdec/` brings up the RK3588 video decoder block (rkvdec2, "vdpu34x" in Rockchip's MPP; RKVDEC0 at `0xfdc30000`) from the kernel and replays recorded decode jobs
on it. It is the first step towards hardware H.264 / H.265 decoding for iPadOS and **has no user yet**: nothing in userland talks to it, and the only caller is a self-test. The device
builds it with `build-rk3588-vdec.py <path>` (the kernel's compile record, `-Werror`, Mach-O audit; the archive embeds `traces/*.vdt`), force-loads it and calls `rk3588_vdec_start()` from a probe.
The board source defines `rk3588_vdec_config` (the physical address and size of a DMA window below 4 GiB; the block ran in IOMMU bypass, so buffers are addressed physically).

| File | Contents |
| --- | --- |
| `VdecTrace.h` | The trace format (`VDT1`), its parser and the player: upload the CPU-written pages, copy the register image with the buffer addresses relocated into the window, clear the output region, write the cache registers, the register runs, then the start register, poll the interrupt register, compare the CRC-32 of the output. Plain C++ over a register accessor and one DMA window |
| `Rk3588Vdec.cpp`, `Rk3588Vdec.h` | The kernel glue: PMU power domains (VCODEC, VDPU, RKVDEC0 in the vendor driver's order), the clocks (nearest PLL divisors at or below the vendor device tree's 800 / 600 / 1000 MHz), the resets, the mappings (the window through IOKit: the kernel's I/O submap is 8 MiB), the self-test thread |
| `traces/` | Recorded jobs of ten streams, 10 pictures each: H.264 at 320x240 (CABAC, CAVLC, with B pictures) and 1280x720 (CABAC); H.265 at 320x240 (plain, with a B pyramid and non-reference B pictures, with default scaling matrices, deblocking off, WPP entry points) and 1280x720 |
| `tools/` | How the traces are made: `trace-shim.c` (an `LD_PRELOAD` shim that makes unmodified MPP believe it runs on an RK3588 and records every job's registers, buffer relocations and CPU-written pages), `make-trace.py`, `gen-yuv.py`, `make-h264-trace.sh`, `make-h265-trace.sh`, `check-hevc-sei.py` |
| `tests/` | `test-vdec.cpp` (a strict model of the decoder's registers; parser, player and every committed trace) and `run.sh` (builds and runs it with the sanitizers) |

The register images are MPP's own: its H.264 / H.265 parsers and `vdpu34x` register generators run on an x86 host against the shim (MPP at `9b56348`, not modified), and the reference pictures come from libavc
(AOSP `external/libavc` at `4275faf`: `avcenc` writes the stream and its reconstruction, `avcdec` decodes it again and `make-h264-trace.sh` insists they are byte-equal) and libhevc (AOSP `external/libhevc` at `48cb531`:
`hevcenc` / `hevcdec`; its decoder does not check the picture hash SEI, so `check-hevc-sei.py` compares the decoded pictures with the checksum SEI the encoder wrote, and `make-h265-trace.sh` runs it). So a trace holds
what the hardware is told to do and the picture it must produce; replaying it needs no parser on the board. MPP is multi-threaded, so recording a stream twice gives equivalent traces, not byte-identical ones (the
CRCs match); the shim holds a lock around each `ioctl` batch because for H.265 MPP's parser and HAL threads submit at the same time, and it must answer the hardware-ID query with the block's real ID (`0x53813f05`):
MPP takes its vdpu34x path for H.265 only for that id. libhevc's encoder needs a complete config file (`make-h265-trace.sh` patches libhevc's sample), only the CRC and checksum hash SEI, one IDR, no tiles, slice segment mode off.

Boot argument `rk3588-vdec=N`: 0 off (default); 1 look at the power domains and PLL rates (nothing is written); 2 also power the domains, set the clocks, reset, read the ID and IOMMU registers; 3 also replay
every embedded trace once and compare; 4 repeat them `rk3588-vdec-passes=N` times (default 20). `rk3588-vdec-dump=1` hex-dumps the picture of the first job that mismatches.

Not done: the userland side (AppleD5500 / AppleAVDUserClient on iPadOS: a kernel user client and the parser that makes the register images on the board), H.265 Main10, several slices or tiles (libhevc's encoder
cannot make them), CRA / open GOP, H.264 interlaced, multi-slice and High profile streams (not in the traces), interrupt-driven completion (the replay polls), the IOMMU, the second core and the core combiner, DDR
bandwidth management, the vendor clock rates.

## Core order

XNU numbers the cores in the order of the `/cpus` nodes, and the scheduler of this
kernel sees every core as one cluster of one type: a new or woken thread goes to
the lowest-numbered idle core. The tree therefore lists the four Cortex-A76 cores
first (logical CPUs 0 to 3) and the Cortex-A55 cores after them. The core the
firmware starts is a Cortex-A55 and is the fifth node; `rk3588-boot-cpu-mpidr` and
the ArmCpu boot-core patch allow that. The `cpumask` boot argument counts logical
CPUs, so `0x1f` leaves the four Cortex-A76 cores and the boot core.

## Status

The patches and the device tree were carried over from a working kernel on
RK3588 EVB10 hardware. They have not been rebuilt against the format 2
layout yet; see the device repository's `porting.toml` for per-area results.
