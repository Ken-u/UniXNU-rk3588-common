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
| `scripts/build-rk3588-rga.py`, `scripts/data/Rk3588Rga/` | Archive step and sources of the RGA3 2D copy block and its IOMMU, used by the video decoder to deliver pictures (see [2D copy (RGA3)](#2d-copy-rga3)) |
| `scripts/build-rk3588-venc.py`, `scripts/data/Rk3588Venc/` | Archive step and sources of the RKVENC0 video encoder bring-up and its H.264 / H.265 encode-job self-test (see [Video encoder (rkvenc)](#video-encoder-rkvenc)) |
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
on it, and decodes embedded H.264 and H.265 streams with Rockchip's own parsers compiled into the kernel. With `rk3588-vdec=7` it also presents the kernel service `AppleD5500` that iPadOS's own
VideoToolbox H.264 hardware decoder opens, so **the only user is a test tool for now** (`usertest/`; no App). Both of VideoToolbox's hardware decoders, H.264 and H.265, are served. The device builds it with `build-rk3588-vdec.py <path>` (the kernel's compile record, `-Werror`, Mach-O audit; the archive embeds
`traces/*.vdt` and `streams/*.h264`, `*.h265`), force-loads it and calls `rk3588_vdec_start()` from a probe.
The board source defines `rk3588_vdec_config` (the physical address and size of a DMA window below 4 GiB; the block runs in IOMMU bypass, so buffers are addressed physically). The Orange Pi 5 gives it 64 MiB: a 1080p High profile stream with several reference pictures peaks at 27 MiB.

| File | Contents |
| --- | --- |
| `VdecTrace.h` | The trace format (`VDT1`), its parser and the player: upload the CPU-written pages, copy the register image with the buffer addresses relocated into the window, clear the output region, write the cache registers, the register runs, then the start register, poll the interrupt register, compare the CRC-32 of the output. Plain C++ over a register accessor and one DMA window |
| `Rk3588Vdec.cpp`, `Rk3588Vdec.h` | The kernel glue: PMU power domains (VCODEC, VDPU, RKVDEC0 in the vendor driver's order), the clocks (nearest PLL divisors at or below the vendor device tree's 800 / 600 / 1000 MHz), the resets (CRU SOFTRST_CON(40) bits 2 to 9; the vendor driver's recovery pulses five of them), the mappings (the window through IOKit: the kernel's I/O submap is 8 MiB), the self-test thread |
| `Rk3588VdecD5500.cpp` | The `AppleD5500` IOService and its user client (type 1, the eight selectors of the original kext's client: CreateDecoder, DestroyDecoder, DecodeFrameFig, MapPixelBuffer, UnmapPixelBuffer, SetCryptSession, one unnamed, SetCallback): the ring buffer and the output pictures are IOSurfaces found by id in `IOCoreSurfaceRoot`; the parameter sets (the plug-in writes them length-prefixed for H.264 and H.265; avcC and hvcC records and Annex B are read as well) are taken from the ring's first bytes and tell the codec; the service sets the IORegistry property `HEVCSupported`, which the plug-in's `AppleD5500CheckPlatform` reads before it registers its H.265 decoders (`rk3588-vdec-hevc=0` turns H.265 off); each call carries one access unit, which the service ends with an access unit delimiter so that the core's cutter emits it at once; frames are queued to a thread of the client's own that runs the core one job at a time and copies each NV12 picture into the output slot's surface; completion is `sendAsyncResult64({sequence, status, record index})` plus a 0x410-byte result record in the ring. The interface is described in `UniXNU-orangepi5/docs/media.md` |
| `D5500Stream.h` | What the `AppleD5500` service reads out of the plug-in's bytes, header-only so a host test can run it: the SPS parsers of both codecs (profile, level, size, bit depths, reference count: the numbers of the CreateDecoder answer), `detectCodec`, `parseExtradata` (avcC, hvcC, length-prefixed or Annex B parameter sets, one output format), `frameAccessUnit` (a length-prefixed or Annex B access unit to what the core takes, with the parameter sets in front of the first one and the delimiter behind) |
| `usertest/` | `vt_test.c` (a VideoToolbox test: a hardware-required `VTDecompressionSession` per embedded H.264 or H.265 stream, the CRC-32 of every returned NV12 picture compared with the reference list; 16 passes, run as launchd's job `com.apple.ptpd` with `evb10-exec=2`) and `build.sh` (clang, ad-hoc signed; its cdhash goes into the owner trust bundle) |
| `core/` | The decoder core, plain C: MPP's H.264 and H.265 parsers (`h264d`, `h265d`) and their `vdpu34x` register generators (41 files, **not copied**: `fetch_mpp.py` gets the commit pinned in `mpp.lock` and checks a tree hash) behind `MppShim.c` (a replacement for MPP's OS layer: memory, a first-fit buffer pool over the DMA window, the device ioctls as capture of the register image) and `VdecCore.c` (MPP's no-thread decode loop written out linearly). `VdecCore.h` is the API: `vdec_core_create(codec)` (`kVdecH264` / `kVdecH265`), `vdec_core_decode(bytes, eos, sink)` calls a function with each NV12 picture and a `run` callback with each job. H.265 streams are cut into access units by `VdecCore.c` itself (MPP's rule, `h265d_find_frame_end`) and the parser runs with `split_parse = 0`; MPP's own splitter is linked but cannot start (`mpp_ring_get` fails in `MppShim.c`): it needs a doubly mapped ring and hands out frames in place, then overwrites their first bytes with the carried start code when an input piece ends inside the next picture's start code (`tests/run-core.sh` found it with 1000-byte pieces). `kernel/` has the libc shim headers and `KernelLibc.c` for the kernel build; the host build is `build-host.sh` |
| `streams/` | The 17 H.264 streams (`*.h264`) and the 6 H.265 streams (`*.h265`, libhevc's encoder: `t320_hevc` I and P, `_b` B pyramid, `_sc` default scaling matrices, `_nodbk` deblocking off, `_wpp` entry points, `t720_hevc`) the traces were recorded from, with the CRC-32 of each correct picture (display order; `NAME.coded.crc` also for the two cropped streams, the whole coded picture a job writes): what the in-kernel self-test decodes. Main profile from libavc's encoder (`t320_cabac`, `t320_cavlc`, `t320_bframes` 320x240, `t720_cabac`), High profile from x264 (`t320_high`, `_cavlc`, `_cqm` JVT scaling matrices, `_slices` four slices, `_mbaff`, `_mbaff_slices` MBAFF, `_fakeil` frame_mbs_only_flag 0 without MBAFF, `_refs` six references and B pyramid, `_p`, `_idr`, `t720_high`, `t320c_high` coded 320x240 shown as 312x234, `t1080_high` coded 1920x1088 shown as 1920x1080) |
| `traces/` | Recorded jobs of 23 streams, 10 pictures each (6 for 1080p): the 17 H.264 streams above; H.265 at 320x240 (plain, with a B pyramid and non-reference B pictures, with default scaling matrices, deblocking off, WPP entry points) and 1280x720 |
| `tools/` | How the traces are made: `trace-shim.c` (an `LD_PRELOAD` shim that makes unmodified MPP believe it runs on an RK3588 and records every job's registers, buffer relocations and CPU-written pages), `make-trace.py`, `gen-yuv.py`, `make-h264-trace.sh` (libavc's encoder, or x264 with `X264=`; `CROP=` for a stream that signals a crop), `record-h264-streams.sh` (the x264 set, each recorded with MPP's threaded and no-thread tests), `make-h265-trace.sh`, `record-h265-streams.sh` (the six libhevc streams, threaded and no-thread recordings), `check-hevc-sei.py` |
| `tests/` | `test-vdec.cpp` (a strict model of the decoder's registers; parser, player and every committed trace) and `run.sh` (builds and runs it with the sanitizers); `test-core.cpp` and `run-core.sh` (the core against `traces/mpp-nt/`, MPP's own recording of the same streams: register images, address registers, CPU-written buffers, picture order, each stream fed whole and in pieces); `test-d5500.cpp` and `run-d5500.sh` (`D5500Stream.h` against every stream in each of five extradata layouts and three access-unit framings, through the real core with a block that accepts every job: exactly one decoded picture and the right pts per call, the sizes the answer is built from, edge cases) |

The register images are MPP's own: its H.264 / H.265 parsers and `vdpu34x` register generators run on an x86 host against the shim (MPP at `9b56348`, not modified), and the reference pictures come from libavc
(AOSP `external/libavc` at `4275faf`: `avcenc` writes the stream and its reconstruction, `avcdec` decodes it again and `make-h264-trace.sh` insists they are byte-equal) and libhevc (AOSP `external/libhevc` at `48cb531`:
`hevcenc` / `hevcdec`; its decoder does not check the picture hash SEI, so `check-hevc-sei.py` compares the decoded pictures with the checksum SEI the encoder wrote, and `make-h265-trace.sh` runs it). So a trace holds
what the hardware is told to do and the picture it must produce; replaying it needs no parser on the board. MPP is multi-threaded, so recording a stream twice gives equivalent traces, not byte-identical ones (the
CRCs match); the shim holds a lock around each `ioctl` batch because for H.265 MPP's parser and HAL threads submit at the same time, and it must answer the hardware-ID query with the block's real ID (`0x53813f05`):
MPP takes its vdpu34x path for H.265 only for that id. libhevc's encoder needs a complete config file (`make-h265-trace.sh` patches libhevc's sample), only the CRC and checksum hash SEI, one IDR, no tiles, slice segment mode off.

Boot argument `rk3588-vdec=N`: 0 off (default); 1 look at the power domains and PLL rates (nothing is written); 2 also power the domains, set the clocks, reset, read the ID and IOMMU registers; 3 also replay
every embedded trace once and compare; 4 repeat them `rk3588-vdec-passes=N` times (default 20); 5 as 3, then every embedded H.264 and H.265 stream once through the decoder core and compare each picture's CRC-32;
6 only the decoder core, `rk3588-vdec-passes=N` times, `rk3588-vdec-chunk=N` feeds the streams in pieces of N bytes; 7 registers the `AppleD5500` service (`rk3588-vdec-d5500-log=N`: how much traffic is printed; 2 and up also reads every picture twice a byte at a time to check its CRC-32, which costs about 100 ns a byte: 25 ms a frame at 320x240, 300 ms at 720p).
`rk3588-vdec-dump=1` hex-dumps the picture of the first job that mismatches; `rk3588-vdec-mpp-log=N` sets MPP's own log level; `rk3588-vdec-reset-test=1` runs, after everything else, a job whose expected CRC is wrong on purpose and then every trace again (the recovery works if they still pass). An error status of the block (error, timeout, empty buffer, collocated-MV reference error) or a wrong picture makes the driver
pulse the five soft resets after that job, as the vendor driver does (`rkvdec2_reset` without the PMU idle request, which the RK3588 device tree skips). **The first version pulsed SOFTRST_CON(20) bits 2, 3, 6, 7 and 8, from reset ids of another numbering that the vendor header does not use (ids 321 to 328, which are not `16 * 20 + bit`): those are the DDR controller's lines and asserting them stopped the board until its watchdog reset it. The block's lines are SOFTRST_CON(40) bits 2 to 9 (the vendor ids 642 to 649).** On the board the corrected pulse leaves the block working: the reset test fails a job on purpose and the next 23 of 23 traces pass.

`patches/xnu/rk3588-wc-batch-flush.patch` (in the shared series, so every RK3588 board gets it) came out of this work: the pmap's batch path for cache attribute changes
(`pmap_batch_set_cache_attributes_internal`, used by `vm_object_iopl_request`) never cleaned or invalidated the data cache when a page went from cacheable to non-cacheable, so a write-back of a dirty line of a fresh
IOSurface page (zero fill, a previous owner's data) overwrote one 64-byte line of what the user had stored through the write-combined mapping. Minutes after boot that showed as zeroed extradata or frame bytes in the decoder's
ring buffer; with the patch 248 of 248 VideoToolbox sessions were right (two boots, up to 16 passes and 727 s each) (`UniXNU-orangepi5/evidence/device-vdec-d5500-20261107.txt`). Not run on the EVB10.

What a frame costs through VideoToolbox (log level 1, the default; `UniXNU-orangepi5/evidence/device-vdec-perf-20261107.txt`): the service prints a two-line profile when a decoder is destroyed. At 320x240 about 0.3 ms a frame
in the service (the block 0.07 ms from start to interrupt, the job runner 0.11, the CPU copy of the picture into the surface 0.08), at 1280x720 about 2.2 ms (the block 0.65 ms, the copy 1.2 ms: the copy is now the larger part).
With the RGA3 doing the copy (`rk3588-rga=5`, [2D copy (RGA3)](#2d-copy-rga3)) a 720p frame is 1.3 ms in the service (the copy 0.34 ms) and a 320x240 frame 0.26 ms.
The 25 to 300 ms of the first runs were the log-level-2 CRC check above, which used to be on by default. A whole session of ten pictures (first frame in to the last picture out, as the test tool sees it) takes 49 ms at 320x240
and 147 ms at 720p; the test tool's own copy and CRC of each picture are 8 ms of the first and 82 ms of the second, the rest (the first picture comes 9 to 13 ms after the first frame; the serial log lines; VideoToolbox itself) was not split further. A decoder is reset before it is destroyed, so MPP's `mpp_buf_slot` no longer asserts at the end of the B-frame stream.

Not done: H.265 Main10, several slices or tiles (libhevc's encoder
cannot make them; the access unit cutter in `VdecCore.c` is only exercised with single-slice pictures, suffix SEI and parameter sets at the start), CRA / open GOP, H.264 PAFF (field pictures; x264's `--interlaced` makes MBAFF), High 10 / 4:2:2, interrupt-driven completion (the replay polls), the IOMMU, the second core and the core combiner, DDR
bandwidth management, the vendor clock rates.

## 2D copy (RGA3)

`scripts/data/Rk3588Rga/` drives RGA3 core 0 (`0xfdb60000`) as a picture copier for the video decoder. The decoder leaves a picture in its DMA window and the `AppleD5500` service has to put it into an
IOSurface whose pages are scattered over all of memory (above 4 GiB, so beyond the 32-bit reach of the older RGA2); the CPU did that through uncached memory in 1.1 to 1.2 ms for a 720p picture. RGA3's IOMMU (Rockchip
IOMMU v2: 40-bit table words) reaches every page of an IOSurface, so the driver owns a page table in a small DMA window of the board's, maps the decoder's window (read-only) and each IOSurface (at `MapPixelBuffer`) into the block's 32-bit
IO address space, and runs one copy job per picture in slave mode (the CPU writes the 48 command registers, starts the job and polls the raw interrupt register; the interrupt is not used).
The block only does NV12 to NV12 raster copies with a crop; anything outside its limits (odd offset or size, pitch not a multiple of 16, under 68x2, a plane not 16-byte aligned) is refused with a reason (printed for the first eight) and the caller copies on the CPU, as it
does when the block reports an error (a page fault, a timeout; the driver then resets the block and brings the IOMMU back with the same tables, and gives the block up after three failures in a row).
The device builds it with `build-rk3588-rga.py <path>` (compile record of the kernel, `-Werror`, Mach-O audit), force-loads the archive and calls `rk3588_rga_start()` from its probe; its source file defines `rk3588_rga_config` (a window below 4 GiB: the page tables in
the first 512 KiB, a scratch page that guards the end of every mapping, and the self-test's pictures from 1 MiB on; 8 MiB at least). `Rk3588Vdec` exports `rk3588_vdec_power_pending()`: the RGA3 domain hangs under VDPU and VCODEC, so this driver waits until the decoder's thread has powered them.

| File | Contents |
| --- | --- |
| `RkIommu.h` | Header-only, no kernel types: the table-word encoding (address bits 39:36 and 35:32 in bits 7:4 and 11:8), the two-level `PageTables` (directory, then page tables from a pool; refuses double maps, misalignment, addresses over 40 bits), and the register sequences the vendor driver uses (stall, directory address with a pattern readback, force reset, enable paging, interrupt mask, auto-gating bit 31, zap) |
| `Rga3.h` | Header-only: the register image of a 1:1 NV12 copy (window 0 raster read with scaling bypassed, window 1 off, overlap as a straight copy, write-back with the vendor's outstanding-request limit), the limits check, the job runner (start, poll, clear; give up after a timeout) and the soft reset |
| `Rk3588Rga.cpp`, `Rk3588Rga.h` | The kernel glue: PMU power domains (VCODEC, VDPU, RGA30), the core clock (nearest PLL divisor not above the target), the resets, the first-fit allocator of IO addresses (every range is followed by four guard pages that map the scratch page), `rk3588_rga_map_physical` / `rk3588_rga_map_memory` / `rk3588_rga_unmap` / `rk3588_rga_copy_nv12`, and the self-tests |
| `tests/` | `test-rga.cpp` (strict models: memory with regions at address bits 32, 35, 36 and 39..36, the IOMMU with its stall rules, a translation cache that only a zap empties, read-only pages and page faults, and RGA3 reading and writing row by row through it; encoding, tables, the built registers, rejects, copies of six picture shapes into scattered pages, the remap-needs-a-zap rule, a fault, an error bit together with frame-done, a hang) and `run.sh` (4357 checks, ASan and UBSan); `mutate.sh` breaks the headers 32 ways, one at a time, and requires the test to fail each time (all 32 are killed; the first version of the test let two live and could not stop one that hung forever) |

Boot argument `rk3588-rga=N`: 0 off (default); 1 look at the power domains, clocks and PLL rates (nothing is written); 2 also power the block, set its clock, release its resets and read its version and the IOMMU's registers;
3 also copy test pictures with the IOMMU off (physical addresses inside the window); 4 bring the IOMMU up and copy the test pictures into 64 KiB pieces of kernel memory (physically separate pages) `rk3588-rga-passes=N` times (default 10);
5 bring it up and leave it ready for the video service; 6 as 4 with one pass, then ready. `rk3588-rga-mhz=N` is the core clock (default 750, cpll / 2, the setting found at boot), `rk3588-rga-fault=1` adds a copy to an unmapped address to mode 4.
The video service uses the block whenever it is ready (`rk3588-rga=5` or 6) and `rk3588-vdec-d5500-rga=0` is not given; its per-decoder profile then ends with a line `copies: N by the RGA3 (the block ran X us per picture), M by the CPU, K the RGA3 failed on`.
Measured on the Orange Pi 5 (`UniXNU-orangepi5/evidence/device-rga-20261107.txt`): all six test shapes (320x240, a 304x224 crop with wider pitches, 720p, 720p with padded pitches, 1080p, 68x64) are byte-identical to the model with the IOMMU off and on,
a copy to an unmapped address stops the block with an error bit and an IOMMU page fault, and one reset later the next copy is right. The block takes about 3.8 pixels per clock: 720p in 615 us at 396 MHz, 411 at 594 and 326 at 750; 1080p 1358, 906 and 720 us.
Through VideoToolbox a 720p frame's time in the service fell from 2.24 to 1.34 ms (the copy from 1.23 ms to 0.34), 320x240 from 0.30 to 0.27 ms, and the pictures stayed right (128 of 128 sessions with the block at 396 MHz and 128 of 128 in the final configuration at 750 MHz; one other 750 MHz boot lost one session to `videocodecd` being killed by a SIGKILL
0.3 s after the first session was created, before any picture, which the block had no part in).
Not done: the second RGA3 core and RGA2, other formats, scaling, rotation and colour conversion (the block can do them; nothing asks), completion by interrupt (SPI 114, shared with the IOMMU), master (command list) mode, quality-of-service settings, a copy that runs while the decoder works on the next frame, other boards.

## Video encoder (rkvenc)

`scripts/data/Rk3588Venc/` brings up the RK3588 video encoder block (RKVENC0, "vepu580" in Rockchip's MPP; registers at `0xfdbd0000`, identification word `0x50603312`) from the kernel and replays recorded H.264 and
H.265 encode jobs on it. VideoToolbox's hardware H.264 encoder reaches it through the `AppleAVE2` service below (`rk3588-venc-ave2-encode=1`, from recorded traces only: fixed quantizer and GOP, a few sizes); this is also the self-test of the block and of the way to drive it.
The device builds it with `build-rk3588-venc.py <path>` (the kernel's compile record, `-Werror`, Mach-O audit; the archive embeds `traces/*.vet`), force-loads it and calls `rk3588_venc_start()` from a probe; the board source defines
`rk3588_venc_config` (a DMA window below 4 GiB, 32 MiB on the Orange Pi 5; the block runs in IOMMU bypass, like the decoder, so buffers are addressed physically).

The method is the decoder's (see above): **the front end is not written here**. Rockchip's own MPP (the H.264 and H.265 encoders: parameter sets, rate control, reference management, and the `vepu580` register generators,
`mpp/hal/rkenc`) runs unmodified on an x86 host behind an `LD_PRELOAD` shim (`tools/trace-shim-enc.c`: a fake `/dev/mpp_service` and DMA heap) that answers every job "done" and records what MPP submitted: the 1444 (H.264) or
1644 (H.265) register words of the job, the registers that hold buffer addresses (which buffer, which offset) and the content the CPU had put into the buffers (the input picture, the parameter sets in front of the stream).
`tools/make-enc-trace.py` packs that into a `VET1` trace; `tools/make-enc-trace.sh` does the whole recording for a synthetic video (the decoder's `gen-yuv.py`). What differs from the decoder is the check: a lossy encoder has no
reference output to compare bit for bit, so the board prints the bit stream it produced and the host decodes it with libavc or libhevc (`tools/check-enc-output.py`: PSNR against the source pictures; a corrupted byte turns a picture to 17 dB and the check fails).

| File | Contents |
| --- | --- |
| `VencTrace.h` | The trace format (`VET1`: per job the register image, the address relocations, the CPU-written runs, where the stream goes), its parser (every length and index checked once) and the player, which does what the vendor driver's `rkvenc_run` does for a task: the VEPU580 forced clear (`0x14` = 2, 5 us, 0), the hardware counter clear (`0x5300` = 2), the register image in ascending order with the buffer addresses relocated into the window, the start register (`0x10`) last, then the interrupt status until done or an error bit (or the time limit), the stream length (`0x4000`), and the status cleared as `rkvenc_irq` does. Plain C++ over a register accessor and one DMA window; a `Player` and a `Trace` are big and are allocated on the heap (the kernel has no placement new: `Player::init`) |
| `Rk3588Venc.cpp`, `Rk3588Venc.h` | The kernel glue: the PMU domains (VCODEC, VENC0: power request, memory power, repair, the idle request), the clocks (aclk 500 MHz, core 800 MHz asked for; the firmware's PLLs give 500 and 750), the resets (`SOFTRST_CON(47)`), the DMA window mapped through IOKit as for the decoder, the self-test thread |
| `Rk3588VencAve2.cpp` | The `AppleAVE2Driver` service and `AppleAVE2UserClient` (nine selectors, the shared request ring, the asynchronous notification, the finished-frame fields; the template encoder behind `rk3588-venc-ave2-encode=1`, the encoder core behind `=2`) |
| `Rk3588VencCore.cpp` | The encoder core's kernel side: the sixteen self-test recipes of `rk3588-venc=6` (nine H.264, seven H.265), the core thread (one caller at a time, its own 16 KB stack) and the stream API (`rk3588_venc_core_open/encode/close`, `rk3588_venc_window_claim/release`) the `AppleAVE2` service encodes with |
| `core/` | The encoder core: MPP's H.264 and H.265 controllers, rate control, slice and parameter-set writers and `hal_h264e_vepu580` / `hal_h265e_vepu580` (63 files compiled on their own, listed in `core/mpp-files.txt`, and two more that MPP `#include`s into others, `core/mpp-included.txt`; at the commit in `core/mpp.lock`, fetched and checked by tree hash, never copied here and not modified) behind `EncShim.c` (MPP's OS layer: buffers in the DMA window, the device calls, the clock) and `VencCore.c` (MPP's controller driven by a loop of ours instead of its threads); `VencCore.h` is the C API; `kernel/` holds the C library stubs |
| `tools/` | `trace-shim-enc.c`, `make-enc-trace.sh`, `make-enc-trace.py`, `check-enc-output.py` (decode the replayed traces' streams), `check-core-output.py` (decode the core's self-test streams, PSNR with a floor that follows the quantiser), `check-vt-enc.py` (decode what VideoToolbox received, per session), `ave_emu.py` |
| `traces/` | Twelve recordings, six H.264 and six H.265: 320x240 with an I and a P frame, six P frames, constant bit rate, an IDR every three frames (the parameter sets in front of every IDR); 640x360 (crop to 360 lines) and 1280x720, three frames each. The H.265 ones were recorded with `VENC_TRACE_DELIVER=job` (see `tools/trace-shim-enc.c`: with the shim's default, which lets the read-backs lag behind the jobs, MPP's H.265 encoder coded IDR pictures where the interval asks for P; the mechanism was not traced) |
| `tests/` | `test-venc.cpp` (a model of the register interface that is strict about the vendor sequence: the clears first, every register of the image once, in order, with its value and the buffer addresses relocated, the CPU-written runs in the window before the start, the start last; the "block" writes a stream where the address register says; error status, a block that never answers, windows that are too small, every truncation of every trace) and `run.sh` (1176 checks, ASan and UBSan); `mutate.sh` breaks the player nine ways and requires the test to fail each time; `test-venc-core.cpp` (`run-core.sh`: the core against MPP's own recordings of all twelve traces, six H.264 and six H.265, then other sizes up to 1920x1080 as planar and NV12 for both codecs, rate control and settings changed in mid-stream, 1794 checks, ASan and UBSan; `mutate-core.sh`: a perturbed setting must fail on all twelve recordings) |

Boot argument `rk3588-venc=N`: 0 off (default); 1 look at the power domains, clocks and PLL rates (nothing is written); 2 also power the block, set the clocks, release the resets, read the identification and status registers and do the vendor's safe reset once;
3 also map the DMA window and replay every trace once, printing for each job the status, the time the block took and the bytes it wrote (32 per line: `rk3588-venc: stream NAME job N OFFSET HEX`); 4 replays them `rk3588-venc-passes=N` times (default 20),
comparing the stream of every pass with the first's (`rk3588-venc-dump=1` also prints the first pass's). The check on the host: `check-enc-output.py SERIAL.log TRACE.vet SOURCE.yuv W H --decoder avcdec|hevcdec [--codec h265]`.
Measured on the Orange Pi 5 (`UniXNU-orangepi5/evidence/device-venc-20261108.txt`): the first job of the first run came back done with a valid H.264 IDR slice (the stream starts `00 00 00 01 65`); all twelve traces, 20 passes each, return the same stream on every pass, and all twelve streams decode with the reference decoders
(PSNR of the luma 37.0 to 39.4 dB at the fixed quantizers 24 to 30 and 39.6 to 45.9 dB at the constant bit rate, the P frames included, so the reference pictures the block wrote are consistent). Time on the block: 160 us for a 320x240 frame, 443 us for 640x360, 1.72 ms for 1280x720 (H.264 and H.265 alike), the CPU polling.
The vendor's safe reset (`0x14` = 1, wait for bit 2 of the status) times out on this block; the vendor driver ignores that, and so does this one (it pulses the reset lines instead). Not understood.
Boot argument `rk3588-venc-ave2=N` (independent of `rk3588-venc`, no hardware touched): registers `AppleAVE2Driver` and its user client (`Rk3588VencAve2.cpp`), logs every call of VideoToolbox's hardware encoder plug-in with the structs dumped, maps the plug-in's request ring and answers the notifications; with `evb10-exec=2` the embedded test program (`Rk3588Vdec/usertest/vt_enc.h`) opens an H.264 `VTCompressionSession`. The Start answer's IOSurfaces are made in the kernel (the kext's own `createSurface`) and the frame-done fields the plug-in's handler reads are written (`tools/ave_emu.py` runs that handler on the host, unicorn, against a board dump: that is how the fields were found). With `rk3588-venc-ave2-encode=1` the application's NV12 pixel buffer is copied into the source picture of the recorded trace of the same size, RKVENC0 plays the next job and its slice, the parameter sets and the frame type go back; on the board the test's five 640x360 pictures come out as five samples that libavc decodes at 42-44 dB (`tools/check-vt-enc.py` takes the bytes the test program prints; `UniXNU-orangepi5/evidence/device-ave2-abi-20261108.txt`, `docs/media.md`). The H.265 plug-in is served the same way from the H.265 trace (the codec is word +0x10 of the Open call's structure); `tools/check-vt-enc.py --codec h265` decodes its samples with libhevc.
**The encoder core (`rk3588-venc=6`, `rk3588-venc-ave2-encode=2`).** `core/` is the replacement for the recordings: MPP's own H.264 and H.265 controllers, rate control, slice and parameter-set writers and `hal_h264e_vepu580` / `hal_h265e_vepu580` compiled unmodified into the kernel (`build-rk3588-venc.py`, second archive next to the decoder's, every symbol renamed so that the two cores can sit in one image), driven by a small loop and a shim for MPP's OS layer, so nothing is recorded: the picture, the size, the quantiser or the bit rate and the key-frame interval are inputs. Boot argument `rk3588-venc=6` runs a self-test (sixteen recipes: for H.264 six planar 4:2:0 (176x144, 320x240, 640x360, 1280x720, 1366x768, 1920x1080) and three NV12 (640x360, 1366x768, 1920x1080), for H.265 four planar (176x144, 320x240, 640x360, 1366x768) and three NV12 (640x360, 1366x768, 1920x1080), fixed quantiser or constant bit rate, key frames every N pictures) and prints the streams (`rk3588-venc-core-dump=1`; `check-core-output.py` decodes them with libavc or libhevc (`--hevc-decoder`) and checks the PSNR against the source, with a floor that falls 0.8 dB per quantiser step above 28; MPP reports an H.265 quantiser averaged over whole 64x64 units, so the checker scales it up for pictures with partial units before applying the floor). With `rk3588-venc-ave2-encode=2` the `AppleAVE2` service does what `=1` does, but a hardware H.264 or H.265 `VTCompressionSession` (the codec is word `+0x10` of the Open call's structure) gets its samples from the core: the pixel buffer's own size (NV12 with 16-aligned rows; the padding of the coded size is filled by repeating the edge), the average bit rate (`+0x1b0` of the Prepare/Start descriptor, found by diffing the descriptors of two sessions), the maximum key-frame interval (`+0x1c4`; 30 when the application sets none) and the frame rate (`+0x1dc`; the H.265 plug-in holds 0 there when the application sets none, and the service then gives the core 30/1), and the parameter sets (VPS, SPS, PPS for H.265) from MPP's writer; a session of the test program that asks for 1 Mbit/s and an IDR every 7 pictures gets IDRs at pictures 0, 7 and 14, in either codec. The core runs on a kernel thread of its own (the 16 KB kernel stack is painted and measured: 3.7 KB at the deepest, in either codec; one caller at a time), shares the block and the DMA window with the trace player (`rk3588_venc_window_claim/release`) and is created at the first picture of a stream. The H.265 plug-in (`H9.videoencoder`) is served by the same code; the recorded traces remain the path of `rk3588-venc-ave2-encode=1`.
Measured on the Orange Pi 5 (`UniXNU-orangepi5/evidence/device-venc-core-20261109.txt`, run `vt4`, 1500 s without a panic): 9 of 9 recipes decode, 41 pictures, luma PSNR 24.7 to 43.4 dB and always above the floor (the constant-bit-rate recipes lose quality as the rate control raises the quantiser: 33 to 45 over the six pictures of the NV12 640x360 recipe at 800 kbit/s, and back to 38 at its next key frame); the VideoToolbox sessions deliver every sample in both passes (640x360 at the default, at 1 Mbit/s with a key frame every 7 pictures and at 3 Mbit/s, 1920x1080 at 6 Mbit/s, 480x270) and libavc decodes them at 41 to 56 dB luma; 0.8 ms a picture at 640x360 (0.72 ms of it on the block), 4.2 ms at 1920x1080. Host: `tests/run-core.sh` (887 checks before H.265 was added, 1794 now: the register images of MPP's recordings, other sizes, NV12, rate control and settings changed mid-stream) and `tests/mutate-core.sh`.
**H.265 in the core (2026-11-09, `UniXNU-orangepi5/evidence/{host,device}-venc-core-h265-20261109.txt`, run `h2`, 1500 s without a panic):** the self-test decodes 16 of 16 recipes (the seven H.265 ones with libhevc, luma PSNR 28.9 to 43.1 dB above the floor, 0.36 ms for 176x144 to 4.1 ms for 1080p on the block); the five H.265 VideoToolbox sessions of the test program (the H.264 sessions' sizes and settings) deliver all their samples in both passes with the same byte counts, the key-frame session has its three sync samples, libhevc decodes everything at 47 to 56 dB, 0.84 ms a 640x360 picture (0.75 ms on the block), 4.2 ms at 1080p, 3.3 KB of the kernel stack; the stream is 4 to 25 % smaller than H.264's at the same settings and pictures. Host: the core's register images equal MPP's own on all twelve recordings and 1794 checks pass with the sanitizers. Finding on the way: the H.265 recordings made earlier were wrong (every second picture an IDR) because the recording shim let MPP's two read-backs per job lag behind the jobs; they were re-recorded with `VENC_TRACE_DELIVER=job` (the H.264 recordings keep the default delivery). MPP's H.265 HAL does not read the `hw:qbias_arr` table the core sets from the test tool (it uses `qbias_i/p`), and `mpp_get_mpp_service_cmd_cap()` answers all zero (no kernel service: no tile-parallel mode).
**Real-time use (2026-11-09, `UniXNU-orangepi5/evidence/{host,device}-venc-realtime-20261109.txt`, runs sp7 and sp9):** the stub no longer sleeps 300 ms in each call (that sleep was a diagnostic of the first runs: it capped every session at three pictures a second, 306 to 316 ms a sample, measured again with `rk3588-venc-ave2=3`, which keeps it). Taking it out showed a flaw behind it: the plug-in reads the buffer set (slot) of a frame from word `+0x3a8` of its request record (the original driver assigns it before it answers; an earlier reading had `+0x46c`, which nothing reads), the stub left it at 0, and once frames came faster than the plug-in's receiver copied them out of slot 0 they overwrote each other. The answer now writes the slot, round robin over eight sets (twice the four frames in flight that Start announces; `tools/ave_slot.py` shows on the host which record words the plug-in's frame-done code reads). With `rk3588-venc-ave2=1` (quiet on the frame path: the log lines of the first calls only; `=2` logs every call and costs about 3 ms a 640x360 picture, `=3` also dumps and sleeps) the test program's speed sessions (`usertest/vt_enc.h`: four pictures made in advance and used in turn, 60 pictures at 640x360, 30 at 1920x1080, 4 Mbit/s) deliver one sample every 1.4 to 1.8 ms at 640x360 (557 to 729 pictures a second, H.264 and H.265 alike, worst gap 1.7 to 8.5 ms) and every 7.1 to 9.2 ms at 1080p (109 to 140 a second, worst gap 9 to 27 ms), the first sample 178 to 424 ms after the first call (four passes in two runs; sp9 also ran 1500 s without a panic); libavc and libhevc decode the samples of the five sessions of each codec that dump them at 41 to 56 dB luma, among them a 16-sample session that wraps the eight slots twice (the speed sessions' samples are counted, not decoded). The block takes 0.7 ms (640x360) and 4.2 ms (1080p) of that; where the rest goes (the copy out of the application's pixel buffer, the plug-in's receiver and the application's callback) was not split. The pictures are synthetic and compress well (no session reaches its bit rate): hard content costs the block more and was not measured, and nothing here captures or displays the pictures.
Not done: the application's `ForceKeyFrame` and bit-rate changes during a stream (the core has `venc_core_set_s32`; the plug-in's request that carries them is not read), rate control tested on real, complex content (the test picture compresses so well that the quantiser never gets near the ceiling), NV12 with strides that are not multiples of 16, H.265 Main10 / several slices / tiles and sizes above 1080p (the window is 32 MiB), the second encoder (RKVENC1), the encoder's IOMMU, interrupts, other formats (NV12 and 4:2:2 input, FBC), multiple slices and tiles, ROI and the other `mpi_enc_test` features, rate control that depends on what the block wrote (the replay is a fixed register sequence: MPP's rate control saw the shim's invented stream length),
frame sizes above 720p (the traces are recordings of 1280x720 at most; a 1080p trace would add 6 MB per frame to the repository and the kernel).

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
