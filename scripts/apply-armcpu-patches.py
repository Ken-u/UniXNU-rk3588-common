#!/usr/bin/env python3
"""Apply the RK3588 fixes to the ArmCpu driver checkout of the build tree (repeatable).

ArmCpu has no patch channel of its own (the XNU series patches XNU only), so the fixes live in
scripts/data/ArmCpu-rk3588-*.patch and this archive step applies them before `component:ArmCpu` builds the driver.

  ArmCpu-rk3588-boot-cpu-slot.patch
                                the boot core need not be logical core 0 (the device tree may list other cores
                                first: dts/rk3588.dtsi lists the Cortex-A76 cores first so the scheduler, which
                                takes the lowest-numbered idle core, prefers them): the secondaries that queue
                                before the boot core registers are finished from slot 0 on, not slot 1
  ArmCpu-rk3588-mmio-sgi.patch  boot argument armcpu-mmio-sgi=1: deliver an IPI by setting SGI 0 pending in the
                                target core's redistributor frame (fallback, off by default)
  ArmCpu-rk3588-sep-asm.patch   separates the instructions of inline assembly with "\\n\\t": the target is
                                Mach-O and the Darwin assembler treats ';' as a comment, so
                                "dsb ishst; msr icc_sgi1r_el1, %0; isb" assembled to a lone dsb
  ArmCpu-rk3588-startcpu-clean.patch
                                a secondary starts with its MMU and caches off and reads cpu_ttep and
                                sysreg_restore (TTBR1, TCR) straight from DRAM: startCPU cleans them to memory
                                first (boot argument armcpu-clean-reset-data=0 turns that off for diagnosis); a
                                watchdog thread per started core reports a core that is not running after 5 s
                                together with its reset-path breadcrumbs (rk3588-smp-el2.patch); every core
                                reports its clock (armcpu-mhz=0 turns that off)

The step restores drivers/ArmCpu of the tree (a link to the pinned checkout) to its pinned state first and then
applies the patches, so a repeated run gives the same result. Run `git -C <checkout> checkout -- .` to get an
unpatched ArmCpu back.
"""
import subprocess
import sys
from pathlib import Path

tree = Path(__file__).resolve().parent.parent
driver = tree / 'drivers' / 'ArmCpu'
# Name order: sep-asm is written against the file as mmio-sgi leaves it.
patches = sorted((Path(__file__).resolve().parent / 'data').glob('ArmCpu-rk3588-*.patch'))
if not (driver / 'ArmCpu.cpp').is_file():
    sys.exit(f'apply-armcpu-patches: {driver}/ArmCpu.cpp not found')


def git_apply(*args):
    return subprocess.run(['git', '-C', str(driver), 'apply', *args], capture_output=True, text=True)


# Start from the pinned sources, so that a repeated run does not depend on what an earlier one left behind.
restore = subprocess.run(['git', '-C', str(driver), 'checkout', '--', '.'], capture_output=True, text=True)
if restore.returncode:
    sys.exit(f'apply-armcpu-patches: cannot restore {driver}: {restore.stderr}')
for patch in patches:
    check = git_apply('--check', str(patch))
    if check.returncode:
        sys.exit(f'apply-armcpu-patches: {patch.name} does not apply to the ArmCpu checkout:\n{check.stderr}')
    result = git_apply(str(patch))
    if result.returncode:
        sys.exit(f'apply-armcpu-patches: {patch.name}: {result.stderr}')
    print(f'apply-armcpu-patches: applied {patch.name}')
