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
| `Rk3588Governor` | The CPU governor (below) |
| `Rk3588Dvfs` | Boot arguments, and the staged probes used to bring it up (modes 1 to 5) |

Why the voltage is part of it: above 600 MHz the firmware clocks the cores from a PVTPLL, a ring oscillator whose speed follows the supply, and the
rate the service reports is only the oscillator's setting. At the 1.0 V the firmware leaves on the Cortex-A76 rails every setting from 1608 MHz up
runs at about 2230 MHz and 816 MHz at about 1220 MHz; the nominal clock comes only at the operating point's own voltage (measured with a dependent add
chain on a bound thread: at the table's voltage the clusters run within 5 % of the rate, from 400 MHz to 2.4 GHz). The oscillator also makes a too low
voltage slow instead of unsafe.

The DSU (the cache every cluster's traffic passes through) takes the Cortex-A55 rate whenever that is set, and a rate of its own afterwards; it sits on
the Cortex-A55 rail. A load from the L3 takes 18 ns with the DSU at 1800 MHz and 52 ns at 400 MHz, from memory 120 and 181 ns, so the governor holds
the DSU at three quarters of the faster Cortex-A76 cluster's rate.

Boot arguments: `rk3588-dvfs=N` (0 off; 1 report the firmware's clocks; 2 and 3 sweep the rates and probe the regulators read-only; 4 sweep the operating
points with their voltages; 5 measure the DSU; **6 run the governor**), `rk3588-dvfs-min=MHz`, `rk3588-dvfs-boost-ms=N`, `rk3588-dvfs-report=N`.

The governor (mode 6) looks at each cluster every 16 ms: the lowest point at or above 1.25 x busy x the current rate, the top point from 90 % busy, up at
once and down after four windows that asked for less; the Cortex-A76 clusters stay at or above half their top rate while there is work and go to the
top for `rk3588-dvfs-boost-ms` when the board's activity counter moves (a touch). It prints each cluster's changes, mean rate, busy share, voltage and
the time at each point every ten seconds. A cluster whose clock service or regulator fails five times is put back at its top point and left alone.
Not done: thermal limits, the GPU and memory clocks (clock ids 5 and 4; the Mali driver sets its own), and an idle state that powers a cluster down.

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
