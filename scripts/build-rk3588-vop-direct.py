#!/usr/bin/env python3
"""Compile the shared RK3588 VOP2 zero-copy scanout (scripts/data/Rk3588VopDirect).

  build/rk3588-vop-direct/libRk3588VopDirect.a

A device lists `build-rk3588-vop-direct.py` in `[kernel] archives`, force-loads the archive in its
display defconfig, and NativeLegacyDisplay calls `evb10_vop_direct_init()` when `rk3588-vop-direct=1`
(or the EVB10 alias `evb10-vop-direct=1`).

An XNU driver has to be compiled with the same flags as the kernel object it lives next to. The
kernel build records the command in a `.cpo.json` file; this script reads that record. Run it from
the build tree (the SoC's scripts are copied there), after the first kernel pass.
"""
import json
import shutil
import struct
import subprocess
import sys
from pathlib import Path

SCRIPT = Path(__file__).resolve()
TREE = SCRIPT.parents[1]
SOURCES = sorted((SCRIPT.parent / 'data' / 'Rk3588VopDirect').glob('*.cpp'))
OUT = TREE / 'build' / 'rk3588-vop-direct'
RECORD = TREE / 'build/xnu/ios-obj/DEVELOPMENT_ARM64_BCM2837/iokit/DEVELOPMENT/IOPlatformExpert.cpo.json'

# MH_MAGIC_64, CPU_TYPE_ARM64, CPU_SUBTYPE_ARM64_ALL, MH_OBJECT.
MACHO = (0xfeedfacf, 0x100000c, 0, 1)
LC_BUILD_VERSION = 0x32
DEPLOYMENT = (2, 0x110000)   # PLATFORM_IOS, iOS 17.0


def compiler_command(record, source, destination):
    command, index, arguments = [], 0, record['arguments']
    while index < len(arguments):
        value = arguments[index]
        if value == record['file']:
            command.append(str(source))
        elif value == '-o':
            command.extend(['-o', str(destination)])
            index += 1
        elif value == '-Xclang' and index + 1 < len(arguments) and arguments[index + 1].startswith('-W'):
            index += 1
        elif not value.startswith('-W'):
            command.append(value)
        index += 1
    command.extend([
        '-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter',
        '-Wno-c99-designator', '-Wno-reorder-init-list',
        '-Wno-deprecated-enum-enum-conversion', '-Wno-unnecessary-virtual-specifier',
        '-I', str(TREE / '.cache/xnu-work/osfmk'),
        '-I', str(SCRIPT.parent / 'data' / 'Rk3588VopDirect'),
        '-Wno-error=unknown-warning-option',
    ])
    return command


def check_object(path):
    data = path.read_bytes()
    magic, cpu, subtype, kind, count, _ = struct.unpack_from('<6I', data, 0)
    if (magic, cpu, subtype, kind) != MACHO:
        sys.exit('%s is not an ARM64 Mach-O object (magic %#x, cputype %#x, filetype %d)' % (path, magic, cpu, kind))
    offset, deployment = 32, None
    for _ in range(count):
        command, length = struct.unpack_from('<II', data, offset)
        if command == LC_BUILD_VERSION:
            deployment = struct.unpack_from('<II', data, offset + 8)
        offset += length
    if deployment != DEPLOYMENT:
        sys.exit('%s is built for platform/deployment %s; the kernel is iOS 17.0 (%s)' % (path, deployment, DEPLOYMENT))


def ar_tool():
    for name in ('llvm-ar', 'ar'):
        found = shutil.which(name)
        if found:
            return found
    for candidate in ('/opt/homebrew/opt/llvm/bin/llvm-ar', '/usr/local/opt/llvm/bin/llvm-ar', '/usr/bin/llvm-ar'):
        if Path(candidate).is_file():
            return candidate
    sys.exit('no llvm-ar on PATH; a Mach-O archive needs one')


def main():
    if not SOURCES:
        sys.exit('build-rk3588-vop-direct: no sources next to %s (scripts/data/Rk3588VopDirect)' % SCRIPT)
    if not RECORD.is_file():
        sys.exit('%s is missing: the kernel step writes it on its first (bootstrap) pass, so run "unixnu build kernel" '
                 'before this builder' % RECORD)
    # The kernel build writes the record with a trailing comma, as a fragment.
    record = json.loads(RECORD.read_text().strip().rstrip(','))
    OUT.mkdir(parents=True, exist_ok=True)
    objects, logs = [], []
    for source in SOURCES:
        if not source.is_file():
            sys.exit('build-rk3588-vop-direct: %s is missing' % source)
        destination = OUT / (source.stem + '.o')
        destination.unlink(missing_ok=True)
        result = subprocess.run(compiler_command(record, source, destination), cwd=record['directory'],
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        logs.append(result.stdout)
        if result.returncode:
            sys.stderr.write(result.stdout[-4000:] + '\n')
            sys.exit('build-rk3588-vop-direct: the compiler rejected %s' % source.name)
        if not destination.is_file():
            sys.exit('build-rk3588-vop-direct: the compiler produced no object for %s' % source.name)
        check_object(destination)
        objects.append(destination)
    (OUT / 'compile.log').write_text(''.join(logs))
    archive = OUT / 'libRk3588VopDirect.a'
    archive.unlink(missing_ok=True)
    subprocess.run([ar_tool(), 'rcs', str(archive)] + [str(path) for path in objects], check=True)
    print('build-rk3588-vop-direct: -Werror build and ARM64/iOS 17 object audit passed -> %s' % archive)


if __name__ == '__main__':
    main()
