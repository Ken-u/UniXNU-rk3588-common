// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// Rk3588Dvfs: frequency and voltage management shared by the RK3588 boards (Rk3588Dvfs.h). The board's probe calls rk3588_dvfs_start() once
// the kernel is up.
//
// Boot-args: rk3588-dvfs=N   1: read-only report of the firmware's clocks (SCMI), at start and every rk3588-dvfs-report seconds
//                            2: mode 1, then a sweep: every CPU cluster is set to each rate the firmware lists, ascending, and the clock the
//                               cores really run at is measured on a bound thread; the clusters end at their top rate again
//                            3: mode 2's report, then a read-only look at the regulators: the RK806 on SPI2 and the RK860x on I2C0 and I2C1
//                            4: mode 3's report, then a sweep with the voltage: each cluster walks down its operating points (rate and voltage of the
//                               tables below, the voltage ordered against the rate as a governor would) and back up, the clock the cores really run
//                               at is measured at each; the clusters end at their top point
//                            5: mode 4's tables, then the DSU (the cluster's shared cache and its clock): the latency of a load from the L3 and from
//                               memory on a Cortex-A76 core while the Cortex-A55 clock, which the DSU follows, goes down its points, and whether
//                               the DSU takes a rate of its own
//                            6: the governor: every 16 ms each cluster's busy share (from the CPUs' idle time) picks its operating point, rate and
//                               voltage together (Rk3588Governor.cpp)
//            rk3588-dvfs-min=MHz        the governor's lowest point for the Cortex-A76 and Cortex-A55 clusters (default 0: the lowest the tables have)
//            rk3588-dvfs-boost-ms=N     how long a touch holds the Cortex-A76 clusters at their top point (default 1000, 0 = no boost)
//            rk3588-dvfs-report=N  seconds between reports of mode 1 (default 10, 1..600), three reports in all
#include <IOKit/IOLib.h>
#include <kern/clock.h>
#include <kern/cpu_number.h>
#include <machine/machine_routines.h>
#include <pexpert/pexpert.h>
#include <stdint.h>
#include <string.h>
#include "Rk3588Domain.h"
#include "Rk3588Dvfs.h"
#include "Rk3588Governor.h"
#include "Rk3588Pmic.h"
#include "Rk3588Scmi.h"

// (kern/thread.h is not included: it changes how IOLib.h's typed allocators compile under -Werror)
extern "C" kern_return_t kernel_thread_start(thread_continue_t continuation, void *parameter, thread_t *new_thread);
extern "C" void thread_set_thread_name(thread_t thread, const char *name);
extern "C" void thread_deallocate(thread_t thread);
extern "C" kern_return_t thread_terminate(thread_t thread);
extern "C" thread_t current_thread(void);
extern "C" struct processor *cpu_to_processor(int cpu);
extern "C" struct processor *thread_bind(struct processor *processor);
extern "C" wait_result_t thread_block(thread_continue_t continuation);

namespace {

using rk3588::Domain;
using rk3588::domainSetUp;
using rk3588::levelSet;
using rk3588::mhz;
using rk3588::voltageGet;

constexpr unsigned kClocks = 7;  // CPUL, DSU, CPUB01, CPUB23, DDR, GPU, NPU

unsigned gMode, gReportSeconds;
Domain gDomain[3];
unsigned gDomains;


void reportIdentity() {
  char vendor[17] = {}, sub[17] = {};
  uint32_t implementation = 0, version = 0;
  unsigned clocks = 0;
  int result = rk3588_scmi_base(vendor, sub, &implementation);
  if (result) {
    printf("rk3588-dvfs: scmi base: error %d\n", result);
    return;
  }
  printf("rk3588-dvfs: scmi base: vendor '%s' sub-vendor '%s' implementation 0x%x\n", vendor, sub, implementation);
  result = rk3588_scmi_clock_version(&version);
  const int counted = rk3588_scmi_clock_count(&clocks);
  printf("rk3588-dvfs: scmi clock protocol: version 0x%x (%d) clocks %u (%d)\n", version, result, clocks, counted);
}

// One line per clock: name, state, rate.
void reportClocks(const char *when) {
  for (uint32_t id = 0; id < kClocks; ++id) {
    char name[17] = {};
    uint32_t attributes = 0;
    uint64_t hz = 0;
    const int a = rk3588_scmi_clock_attributes(id, name, &attributes);
    const int r = rk3588_scmi_clock_rate_get(id, &hz);
    printf("rk3588-dvfs: %s clk%u '%s' %u MHz (attributes 0x%x: %d, rate: %d)\n", when, id, a ? "?" : name, mhz(hz), attributes, a, r);
  }
}

// The rates the firmware offers per clock.
void reportRates() {
  for (uint32_t id = 0; id < kClocks; ++id) {
    uint64_t rates[40] = {};
    unsigned count = 0;
    int ranged = 0;
    const int result = rk3588_scmi_clock_rates(id, rates, 40, &count, &ranged);
    char line[400];
    unsigned used = (unsigned)snprintf(line, sizeof line, "rk3588-dvfs: clk%u rates (%d, %s, %u):", id, result, ranged ? "range" : "list", count);
    for (unsigned i = 0; i < count && used + 8 < sizeof line; ++i) used += (unsigned)snprintf(line + used, sizeof line - used, " %u", mhz(rates[i]));
    printf("%s\n", line);
  }
}

// ---- the regulators (mode 3, read only) ---------------------------------------------------------------------------------------------

void reportPmics() {
  uint32_t clkdiv0 = 0, clkdiv1 = 0, spi[4] = {};
  rk3588_pmic_state(0, &clkdiv0, spi);
  printf("rk3588-dvfs: pmic: i2c0 CLKDIV 0x%08x, spi2 CTRLR0 0x%x BAUDR %u SER 0x%x SR 0x%x\n", clkdiv0, spi[0], spi[1], spi[2], spi[3]);
  rk3588_pmic_state(1, &clkdiv1, spi);
  printf("rk3588-dvfs: pmic: i2c1 CLKDIV 0x%08x\n", clkdiv1);
  uint8_t name = 0, version = 0;
  const int a = rk3588_rk806_read(0x5a, &name), b = rk3588_rk806_read(0x5b, &version);
  printf("rk3588-dvfs: pmic: rk806 chip name 0x%02x version 0x%02x (%d, %d)\n", name, version, a, b);
  for (unsigned n = 1; n <= 5; ++n) {
    uint32_t uv = 0;
    const int r = rk3588_rk806_buck_get_uv(n, &uv);
    printf("rk3588-dvfs: pmic: rk806 buck%u %u mV (%d)\n", n, uv / 1000, r);
  }
  static const struct { unsigned bus; uint8_t address; const char *what; } kBucks[] = {
      {0, 0x42, "i2c0 0x42"}, {0, 0x43, "i2c0 0x43"}, {1, 0x42, "i2c1 0x42"}};
  for (const auto &buck : kBucks) {
    uint32_t uv = 0;
    unsigned id = 0;
    uint8_t enable = 0;
    const int r = rk3588_rk860x_get_uv(buck.bus, buck.address, &uv, &id);
    const int e = rk3588_i2c_read_reg(buck.bus, buck.address, 0x00, &enable);
    printf("rk3588-dvfs: pmic: rk860x %s die id %u: %u uV (%d), reg0 0x%02x (%d)\n", buck.what, id, uv, r, enable, e);
  }
}

// ---- the sweep (mode 2) ------------------------------------------------------------------------------------------------------------

// A chain of dependent adds: 8 per iteration, one cycle each on the Cortex-A55 and the Cortex-A76, the loop control runs beside it.
void spin(uint64_t iterations) {
  uint64_t x = 0;
  __asm__ volatile(
      "1:\n\t"
      "add %0, %0, #1\n\t"
      "add %0, %0, #1\n\t"
      "add %0, %0, #1\n\t"
      "add %0, %0, #1\n\t"
      "add %0, %0, #1\n\t"
      "add %0, %0, #1\n\t"
      "add %0, %0, #1\n\t"
      "add %0, %0, #1\n\t"
      "subs %1, %1, #1\n\t"
      "b.ne 1b\n\t"
      : "+r"(x), "+r"(iterations)
      :
      : "cc");
}

// The clock `cpu` really runs at, in MHz: the best of five runs on a thread bound to it (a preempted run only looks slower).
unsigned measureMhz(int cpu) {
  if (cpu < 0 || cpu > int(ml_get_max_cpu_number())) return 0;   // a core the boot arguments (cpumask) leave off
  struct processor *target = cpu_to_processor(cpu);
  if (!target) return 0;
  thread_bind(target);
  thread_block(THREAD_CONTINUE_NULL);
  unsigned best = 0;
  if (int(cpu_number()) == cpu) {
    constexpr uint64_t kIterations = 1500000;
    uint64_t shortest = ~0ULL;
    for (unsigned run = 0; run < 5; ++run) {
      const uint64_t t0 = mach_absolute_time();
      spin(kIterations);
      const uint64_t dt = mach_absolute_time() - t0;
      if (dt < shortest) shortest = dt;
    }
    uint64_t ns = 0;
    absolutetime_to_nanoseconds(shortest, &ns);
    if (ns) best = unsigned(kIterations * 8 * 1000ULL / ns);
  }
  thread_bind(nullptr);
  thread_block(THREAD_CONTINUE_NULL);
  return best;
}

struct Cluster {
  uint32_t clock;
  const char *name;
  int cpu, sibling;  // logical CPUs: the DTS lists the Cortex-A76 cores (0..3) before the Cortex-A55 cores (4..7)
  unsigned cpus;     // how many the cluster has
};
constexpr Cluster kClusters[] = {
    {kRk3588ScmiClkCpuB01, "cpub01", 0, 1, 2},
    {kRk3588ScmiClkCpuB23, "cpub23", 2, 3, 2},
    {kRk3588ScmiClkCpuL, "cpul", 4, 5, 4},   // cpumask=31 starts the boot core of this cluster only
};

void sweepCluster(const Cluster &cluster) {
  uint64_t rates[40] = {};
  unsigned count = 0;
  int ranged = 0;
  if (rk3588_scmi_clock_rates(cluster.clock, rates, 40, &count, &ranged) || ranged || !count) {
    printf("rk3588-dvfs: sweep %s: no rate list\n", cluster.name);
    return;
  }
  for (unsigned i = 0; i < count; ++i) {
    const uint64_t t0 = mach_absolute_time();
    const int set = rk3588_scmi_clock_rate_set(cluster.clock, rates[i]);
    const uint64_t dt = mach_absolute_time() - t0;
    uint64_t ns = 0, got = 0, dsu = 0;
    absolutetime_to_nanoseconds(dt, &ns);
    const int get = rk3588_scmi_clock_rate_get(cluster.clock, &got);
    rk3588_scmi_clock_rate_get(kRk3588ScmiClkDsu, &dsu);
    const unsigned first = measureMhz(cluster.cpu), second = measureMhz(cluster.sibling);
    printf("rk3588-dvfs: sweep %s set %u -> %u MHz (%d, %d) in %u us; cpu%d %u cpu%d %u MHz measured; dsu %u\n", cluster.name, mhz(rates[i]), mhz(got),
           set, get, unsigned(ns / 1000), cluster.cpu, first, cluster.sibling, second, mhz(dsu));
  }
  // the cluster ends at its top rate, the state the firmware left it in
  rk3588_scmi_clock_rate_set(cluster.clock, rates[count - 1]);
}

void sweep() {
  printf("rk3588-dvfs: sweep: baseline\n");
  for (int cpu = 0; cpu <= int(ml_get_max_cpu_number()); ++cpu) printf("rk3588-dvfs: sweep: cpu%d runs at %u MHz\n", cpu, measureMhz(cpu));
  for (const Cluster &cluster : kClusters) sweepCluster(cluster);
  printf("rk3588-dvfs: sweep: after\n");
  for (int cpu = 0; cpu <= int(ml_get_max_cpu_number()); ++cpu) printf("rk3588-dvfs: sweep: cpu%d runs at %u MHz\n", cpu, measureMhz(cpu));
}

void voltageSweepStep(Domain &d, const char *way, unsigned to) {
  const uint64_t t0 = mach_absolute_time();
  const int r = levelSet(d, to);
  uint64_t ns = 0;
  absolutetime_to_nanoseconds(mach_absolute_time() - t0, &ns);
  uint32_t rail = 0;
  const int g = voltageGet(*d.rail, &rail);
  IODelay(300);
  uint64_t hz = 0;
  rk3588_scmi_clock_rate_get(d.rail->clock, &hz);
  printf("rk3588-dvfs: vsweep %s %s %u MHz at %u mV (set %d in %u us; rail reads %u mV (%d), clock reads %u MHz): cpu%d runs at %u MHz\n", d.name, way,
         d.level[to].mhz, d.level[to].uv / 1000, r, unsigned(ns / 1000), rail / 1000, g, mhz(hz), d.cpu, measureMhz(d.cpu));
}

void voltageSweep() {
  for (unsigned i = 0; i < gDomains; ++i) {
    Domain &d = gDomain[i];
    printf("rk3588-dvfs: vsweep %s: %u points, now %u MHz at %u mV\n", d.name, d.count, d.level[d.current].mhz, d.uv / 1000);
    // start from the top point the table has, with its voltage
    if (d.current != d.count - 1 || d.uv < d.level[d.count - 1].uv) {
      if (levelSet(d, d.count - 1)) printf("rk3588-dvfs: vsweep %s: no top point\n", d.name);
    }
    for (unsigned to = d.count - 1; to-- > 0;) voltageSweepStep(d, "down", to);
    for (unsigned to = 1; to < d.count; ++to) voltageSweepStep(d, "up", to);
    printf("rk3588-dvfs: vsweep %s: done at %u MHz, %u mV\n", d.name, d.level[d.current].mhz, d.uv / 1000);
  }
}

// ---- the DSU (mode 5) -----------------------------------------------------------------------------------------------------------------

// A random cycle through one cache line per 64 bytes of a buffer: a dependent load chain that misses the caches smaller than the buffer.
uint64_t *chainMake(size_t bytes) {
  uint64_t *buffer = static_cast<uint64_t *>(IOMalloc(bytes));
  if (!buffer) return nullptr;
  const size_t lines = bytes / 64;
  for (size_t i = 0; i < lines; ++i) buffer[i * 8] = i;
  uint64_t state = 0x9e3779b97f4a7c15ull;
  for (size_t i = lines - 1; i > 0; --i) {   // Sattolo's shuffle: a single cycle
    state ^= state << 13;
    state ^= state >> 7;
    state ^= state << 17;
    const size_t j = state % i;
    const uint64_t swap = buffer[i * 8];
    buffer[i * 8] = buffer[j * 8];
    buffer[j * 8] = swap;
  }
  for (size_t i = 0; i < lines; ++i) buffer[i * 8] *= 8;   // a link is the index of a word
  return buffer;
}

uint64_t gChainSink;

// The time of one load of the chain on `cpu`, in tenths of a nanosecond (the best of three runs).
unsigned chainTenthNs(int cpu, const uint64_t *buffer, uint64_t loads) {
  static unsigned traced;
  if (!buffer || cpu < 0 || cpu > int(ml_get_max_cpu_number())) {
    if (traced++ < 3) printf("rk3588-dvfs: chain: refused (buffer %p cpu %d max %u)\n", static_cast<const void *>(buffer), cpu, unsigned(ml_get_max_cpu_number()));
    return 0;
  }
  struct processor *target = cpu_to_processor(cpu);
  if (!target) return 0;
  thread_bind(target);
  thread_block(THREAD_CONTINUE_NULL);
  unsigned best = 0;
  if (traced++ < 3) printf("rk3588-dvfs: chain: bound to cpu%d, running on cpu%d, loads %llu\n", cpu, int(cpu_number()), loads);
  if (int(cpu_number()) == cpu) {
    uint64_t shortest = ~0ULL, at = 0;
    for (unsigned run = 0; run < 3; ++run) {
      const uint64_t t0 = mach_absolute_time();
      for (uint64_t i = 0; i < loads; ++i) at = buffer[at];
      const uint64_t dt = mach_absolute_time() - t0;
      if (dt < shortest) shortest = dt;
    }
    gChainSink = at;
    uint64_t ns = 0;
    absolutetime_to_nanoseconds(shortest, &ns);
    best = unsigned(ns * 10 / loads);
    if (traced < 6) {
      ++traced;
      printf("rk3588-dvfs: chain: %llu ticks, %llu ns, at %llu\n", shortest, ns, at);
    }
  }
  thread_bind(nullptr);
  thread_block(THREAD_CONTINUE_NULL);
  return best;
}

void dsuProbe() {
  Domain *little = nullptr;
  for (unsigned i = 0; i < gDomains; ++i)
    if (gDomain[i].rail->clock == kRk3588ScmiClkCpuL) little = &gDomain[i];
  uint64_t *l3 = chainMake(1u << 21), *memory = chainMake(24u << 20);   // 2 MiB: past a core's L2 (512 KiB), in the L3; 24 MiB: memory
  if (!little || !l3 || !memory) {
    printf("rk3588-dvfs: dsu: no little domain (%p) or no buffer (%p, %p)\n", static_cast<void *>(little), static_cast<void *>(l3), static_cast<void *>(memory));
    return;
  }
  const unsigned top = little->count - 1;
  for (unsigned i = top + 1; i-- > 0;) {
    if (levelSet(*little, i)) printf("rk3588-dvfs: dsu: could not set the cortex-a55 point %u\n", i);
    uint64_t dsu = 0;
    rk3588_scmi_clock_rate_get(kRk3588ScmiClkDsu, &dsu);
    IODelay(300);
    printf("rk3588-dvfs: dsu: cortex-a55 %u MHz, dsu %u MHz: cpu0 load from the l3 %u.%u ns, from memory %u.%u ns\n", little->level[i].mhz, mhz(dsu),
           chainTenthNs(0, l3, 1u << 20) / 10, chainTenthNs(0, l3, 1u << 20) % 10, chainTenthNs(0, memory, 1u << 19) / 10,
           chainTenthNs(0, memory, 1u << 19) % 10);
  }
  // does the DSU take a rate of its own? with the cortex-a55 cores at their lowest point
  levelSet(*little, 0);
  const int set = rk3588_scmi_clock_rate_set(kRk3588ScmiClkDsu, 1800000000ull);
  uint64_t dsu = 0;
  const int get = rk3588_scmi_clock_rate_get(kRk3588ScmiClkDsu, &dsu);
  IODelay(300);
  const unsigned l3Tenths = chainTenthNs(0, l3, 1u << 20);
  printf("rk3588-dvfs: dsu: set 1800 MHz with the cortex-a55 cores at %u MHz: %d, reads %u MHz (%d): cpu0 load from the l3 %u.%u ns\n", little->level[0].mhz,
         set, mhz(dsu), get, l3Tenths / 10, l3Tenths % 10);
  rk3588_scmi_clock_rate_set(kRk3588ScmiClkDsu, uint64_t(little->level[0].mhz) * 1000000);
  if (levelSet(*little, top)) printf("rk3588-dvfs: dsu: the cortex-a55 cores did not return to their top point\n");
  IOFree(l3, 1u << 21);
  IOFree(memory, 24u << 20);
}

void workerMain(void *, wait_result_t) {
  IOSleep(3000);
  if (rk3588_scmi_init()) {
    printf("rk3588-dvfs: no shared memory\n");
    thread_terminate(current_thread());
    return;
  }
  reportIdentity();
  reportRates();
  const unsigned reports = gMode >= 2 ? 1 : 3;
  for (unsigned i = 0; i < reports; ++i) {
    reportClocks(i ? "again" : "start");
    if (i + 1 < reports) IOSleep(gReportSeconds * 1000);
  }
  if (gMode >= 3) reportPmics();
  if (gMode == 2 || gMode == 3) sweep();
  if (gMode >= 4) {
    for (unsigned i = 0; i < rk3588_dvfs_rail_count && gDomains < 3; ++i) {
      for (const Cluster &cluster : kClusters) {
        if (cluster.clock == rk3588_dvfs_rails[i].clock && domainSetUp(gDomain[gDomains], rk3588_dvfs_rails[i], cluster.cpu, cluster.cpus, cluster.name)) ++gDomains;
      }
    }
    if (gMode == 4) voltageSweep();
    if (gMode == 5) dsuProbe();
    if (gMode == 6) rk3588::governorRun(gDomain, gDomains);   // does not return
  }
  thread_terminate(current_thread());
}

}  // namespace

// Called by the board's probe once the kernel is up.
extern "C" void rk3588_dvfs_start(void) {
  static bool started;
  unsigned mode = 0, report = 10;
  if (!PE_parse_boot_argn("rk3588-dvfs", &mode, sizeof mode) || mode < 1 || started) return;
  PE_parse_boot_argn("rk3588-dvfs-report", &report, sizeof report);
  gReportSeconds = report < 1 ? 1 : report > 600 ? 600 : report;
  gMode = mode;
  started = true;
  thread_t t = nullptr;
  printf("rk3588-dvfs: starting (mode %u)\n", gMode);
  if (kernel_thread_start(workerMain, nullptr, &t) == KERN_SUCCESS) {
    thread_set_thread_name(t, "rk3588-dvfs");
    thread_deallocate(t);
  } else {
    printf("rk3588-dvfs: no worker thread\n");
  }
}
