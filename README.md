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
| `kits/` | Reserved for shared platform operations |

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

ArmCpu has no patch channel of its own. `scripts/data/ArmCpu-rk3588-*.patch`
fixes inline assembly (the Darwin assembler treats `;` as a comment), adds
an optional redistributor-based IPI path, and lets the boot core be any
logical core (the secondaries that queue before it registers are finished
from slot 0). A device repository lists
`apply-armcpu-patches.py` in `[kernel] archives` ahead of `component:ArmCpu`.

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
