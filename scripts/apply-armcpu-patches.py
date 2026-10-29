#!/usr/bin/env python3
"""Apply the RK3588 fixes to the ArmCpu driver checkout of the build tree (idempotent).

ArmCpu has no patch channel of its own (the XNU series patches XNU only), so the fixes live in
scripts/data/ArmCpu-rk3588-*.patch and this archive step applies them before `component:ArmCpu` builds the driver.

  ArmCpu-rk3588-sep-asm.patch   separates the instructions of inline assembly with "\\n\\t": the target is
                                Mach-O and the Darwin assembler treats ';' as a comment, so
                                "dsb ishst; msr icc_sgi1r_el1, %0; isb" assembled to a lone dsb
  ArmCpu-rk3588-mmio-sgi.patch  boot argument armcpu-mmio-sgi=1: deliver an IPI by setting SGI 0 pending in the
                                target core's redistributor frame (fallback, off by default)

The step changes files in drivers/ArmCpu of the tree, which is a link to the pinned checkout: restore it with
`git -C <checkout> checkout -- .` when switching to an unpatched kernel.
"""
import subprocess
import sys
from pathlib import Path

tree = Path(__file__).resolve().parent.parent
driver = tree / 'drivers' / 'ArmCpu'
patches = sorted((Path(__file__).resolve().parent / 'data').glob('ArmCpu-rk3588-*.patch'), key=lambda p: (p.name != 'ArmCpu-rk3588-sep-asm.patch', p.name))
if not (driver / 'ArmCpu.cpp').is_file():
    sys.exit(f'apply-armcpu-patches: {driver}/ArmCpu.cpp not found')


def git_apply(*args):
    return subprocess.run(['git', '-C', str(driver), 'apply', *args], capture_output=True, text=True)


for patch in patches:
    if git_apply('--reverse', '--check', str(patch)).returncode == 0:
        print(f'apply-armcpu-patches: {patch.name} already applied')
        continue
    check = git_apply('--check', str(patch))
    if check.returncode:
        sys.exit(f'apply-armcpu-patches: {patch.name} does not apply to the ArmCpu checkout:\n{check.stderr}')
    result = git_apply(str(patch))
    if result.returncode:
        sys.exit(f'apply-armcpu-patches: {patch.name}: {result.stderr}')
    print(f'apply-armcpu-patches: applied {patch.name}')
