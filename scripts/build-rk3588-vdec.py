#!/usr/bin/env python3
"""Compile the shared RK3588 rkvdec2 video decoder (scripts/data/Rk3588Vdec) against the actual source-kernel ABI.

  build/rk3588-vdec/libRk3588Vdec.a

A device lists `build-rk3588-vdec.py <board source>...` in `[kernel] archives` (the board's sources are paths relative to the build tree, for
example `kits/orangepi5/Rk3588VdecBoard.cpp`: they define the DMA window Rk3588Vdec.h declares), force-loads the archive in its defconfig
(`-Wl,-force_load,{tree}/build/rk3588-vdec/libRk3588Vdec.a`) and calls `rk3588_vdec_start()` from its probe.

The decode-job traces the self-test replays (scripts/data/Rk3588Vdec/traces/*.vdt, binary) are turned into C arrays (VdecTrace.inc) in the output
directory, which the compiler gets as an include path; so are the H.264 streams and the CRC-32 of every picture a correct decoder produces
(streams/*.h264, streams/*.crc: VdecStreams.inc), which the second self-test feeds to the in-kernel decoder core.

The decoder core (scripts/data/Rk3588Vdec/core) is plain C: Rockchip's MPP H.264 parser and rkvdec2 register generator, fetched at a pinned commit
(core/mpp.lock, a tree hash of the files used) and compiled unmodified with the kernel's own flags, plus a shim that replaces MPP's operating system layer
(MppShim.c), the decode loop (VdecCore.c) and the few C library functions the kernel does not have (core/kernel/KernelLibc.c).

An XNU driver has to be compiled with the same flags, the same SDK and the same deployment target as the kernel object it lives next
to. The kernel build records the exact command of every object it compiles in a `.cpo.json` file; this script reads that record, points
it at the sources and runs the compiler. Run it from the build tree (the SoC's scripts are copied there), after the first kernel pass.
"""
import importlib.util
import json
import shutil
import struct
import subprocess
import sys
from pathlib import Path

SCRIPT = Path(__file__).resolve()
TREE = SCRIPT.parents[1]
DATA = SCRIPT.parent / 'data' / 'Rk3588Vdec'
TRACES = sorted((DATA / 'traces').glob('*.vdt'))
STREAMS = sorted((DATA / 'streams').glob('*.h264'))
CORE = DATA / 'core'
SOURCES = sorted(DATA.glob('*.cpp')) + [TREE / argument for argument in sys.argv[1:]]
OUT = TREE / 'build' / 'rk3588-vdec'
RECORD = TREE / 'build/xnu/ios-obj/DEVELOPMENT_ARM64_BCM2837/iokit/DEVELOPMENT/IOPlatformExpert.cpo.json'

# MH_MAGIC_64, CPU_TYPE_ARM64, CPU_SUBTYPE_ARM64_ALL, MH_OBJECT.
MACHO = (0xfeedfacf, 0x100000c, 0, 1)
LC_BUILD_VERSION = 0x32
DEPLOYMENT = (2, 0x110000)   # PLATFORM_IOS, iOS 17.0


# Flags of the C++ record that make no sense for a C file.
CXX_ONLY = ('-fsized-deallocation', '-fapple-kext', '--driver-mode=g++', '-fc++-static-destructors=none')
MPP_DEFINES = ('-DHAVE_H264D', '-DHW_VDPU34X', '-DHW_VDPU381')


def compiler_command(record, source, destination, c_flags=None):
    """The kernel's own compile command for `source`. c_flags is None for C++ (the driver), else (arguments in front of the record's, arguments after) for a
    C file of the decoder core."""
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
        elif c_flags is not None and value == '-xc++':
            command.append('-xc')
        elif c_flags is not None and value.startswith('-std=gnu++'):
            command.append('-std=gnu11')
        elif c_flags is not None and value in CXX_ONLY:
            pass
        elif not value.startswith('-W'):
            command.append(value)
        index += 1
    command.extend([
        '-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter',
        '-Wno-c99-designator', '-Wno-reorder-init-list',
        '-Wno-deprecated-enum-enum-conversion', '-Wno-unnecessary-virtual-specifier',
        '-I', str(TREE / '.cache/xnu-work/osfmk'),
        '-I', str(DATA),
        '-I', str(OUT),
        '-Wno-error=unknown-warning-option',
    ])
    if c_flags is not None:
        front, back = c_flags
        command[1:1] = front
        command.extend(['-Wno-unused-command-line-argument'] + back)
    return command


def core_sources(record):
    """(source, extra compile arguments) of the decoder core: MPP's files (third-party: no warnings, the shim headers of core/kernel/include in front of the
    kernel's), the cut of mpp_sys_cfg.c, the shim and loop of core/, and the C library stubs (compiled against the kernel's headers only)."""
    spec = importlib.util.spec_from_file_location('fetch_mpp', CORE / 'fetch_mpp.py')
    fetch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fetch)
    mpp = fetch.find(TREE / '.cache' / 'mpp')
    derived = OUT / 'mpp'
    derived.mkdir(parents=True, exist_ok=True)
    subprocess.run([sys.executable, str(CORE / 'derive-mpp.py'), str(mpp), str(derived)], check=True, stdout=subprocess.DEVNULL)
    shim = ['-I', str(CORE / 'kernel' / 'include')]   # first of all: the kernel's own bsd/sys/stat.h and friends are on the path of the record
    includes = [argument for directory in fetch.read_lines('mpp-includes.txt') for argument in ('-I', str(mpp / directory))]
    third_party = includes + ['-I', str(CORE), '-w'] + list(MPP_DEFINES)
    ours = includes + ['-I', str(CORE)] + list(MPP_DEFINES)
    sources = [(mpp / name, (shim, third_party)) for name in fetch.read_lines('mpp-files.txt')]
    sources.append((derived / 'mpp_sys_cfg_proc.c', (shim, third_party)))
    sources += [(CORE / 'MppShim.c', (shim, ours)), (CORE / 'VdecCore.c', (shim, ours)), (CORE / 'kernel' / 'KernelLibc.c', ([], []))]
    return sources


def write_trace_include(traces, destination):
    """kVdecTraces[]: {name, bytes, size} of every trace, each as a C array."""
    lines = ['// generated by build-rk3588-vdec.py from scripts/data/Rk3588Vdec/traces/*.vdt']
    for index, trace in enumerate(traces):
        data = trace.read_bytes()
        if data[:4] != b'VDT1':
            sys.exit('%s is not a VDT1 trace' % trace)
        lines.append('static const unsigned char kVdecTrace%d[] = {  // %s, %d bytes' % (index, trace.name, len(data)))
        for offset in range(0, len(data), 24):
            lines.append(','.join('0x%02x' % byte for byte in data[offset:offset + 24]) + ',')
        lines.append('};')
    lines.append('static const struct { const char *name; const unsigned char *data; unsigned long size; } kVdecTraces[] = {')
    for index, trace in enumerate(traces):
        lines.append('  {"%s", kVdecTrace%d, sizeof kVdecTrace%d},' % (trace.stem, index, index))
    lines.append('};')
    destination.write_text('\n'.join(lines) + '\n')


def write_stream_include(streams, destination):
    """kVdecStreams[]: {name, bytes, size, crcs, crc count} of every stream; the CRC list has one hex word per line (display order) and # comments."""
    lines = ['// generated by build-rk3588-vdec.py from scripts/data/Rk3588Vdec/streams/*.h264 and *.crc']
    for index, stream in enumerate(streams):
        data = stream.read_bytes()
        crcs = [int(line, 16) for line in stream.with_suffix('.crc').read_text().splitlines() if line.strip() and not line.startswith('#')]
        if not crcs:
            sys.exit('%s has no CRC list' % stream.with_suffix('.crc'))
        lines.append('static const unsigned char kVdecStream%d[] = {  // %s, %d bytes' % (index, stream.name, len(data)))
        for offset in range(0, len(data), 24):
            lines.append(','.join('0x%02x' % byte for byte in data[offset:offset + 24]) + ',')
        lines.append('};')
        lines.append('static const unsigned kVdecStreamCrc%d[] = {%s};' % (index, ','.join('0x%08x' % crc for crc in crcs)))
    lines.append('static const struct { const char *name; const unsigned char *data; unsigned long size; const unsigned *crcs; unsigned crcCount; } '
                 'kVdecStreams[] = {')
    for index, stream in enumerate(streams):
        lines.append('  {"%s", kVdecStream%d, sizeof kVdecStream%d, kVdecStreamCrc%d, sizeof kVdecStreamCrc%d / sizeof(unsigned)},' % ((stream.stem,) + (index,) * 4))
    lines.append('};')
    destination.write_text('\n'.join(lines) + '\n')


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
        sys.exit('build-rk3588-vdec: no sources next to %s (scripts/data/Rk3588Vdec)' % SCRIPT)
    if not RECORD.is_file():
        sys.exit('%s is missing: the kernel step writes it on its first (bootstrap) pass, so run "unixnu build kernel" '
                 'before this builder' % RECORD)
    # The kernel build writes the record with a trailing comma, as a fragment.
    record = json.loads(RECORD.read_text().strip().rstrip(','))
    OUT.mkdir(parents=True, exist_ok=True)
    if not TRACES:
        sys.exit('build-rk3588-vdec: no traces next to %s (scripts/data/Rk3588Vdec/traces)' % SCRIPT)
    if not STREAMS:
        sys.exit('build-rk3588-vdec: no streams next to %s (scripts/data/Rk3588Vdec/streams)' % SCRIPT)
    write_trace_include(TRACES, OUT / 'VdecTrace.inc')
    write_stream_include(STREAMS, OUT / 'VdecStreams.inc')
    objects, logs = [], []
    jobs = [(source, None) for source in SOURCES] + core_sources(record)
    for source, c_flags in jobs:
        if not source.is_file():
            sys.exit('build-rk3588-vdec: %s is missing' % source)
        destination = OUT / (source.stem + '.o')
        destination.unlink(missing_ok=True)
        result = subprocess.run(compiler_command(record, source, destination, c_flags), cwd=record['directory'],
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        logs.append(result.stdout)
        if result.returncode:
            sys.stderr.write(result.stdout[-4000:] + '\n')
            sys.exit('build-rk3588-vdec: the compiler rejected %s' % source.name)
        if not destination.is_file():
            sys.exit('build-rk3588-vdec: the compiler produced no object for %s' % source.name)
        check_object(destination)
        objects.append(destination)
    (OUT / 'compile.log').write_text(''.join(logs))
    archive = OUT / 'libRk3588Vdec.a'
    archive.unlink(missing_ok=True)
    subprocess.run([ar_tool(), 'rcs', str(archive)] + [str(path) for path in objects], check=True)
    print('build-rk3588-vdec: -Werror build and ARM64/iOS 17 object audit passed -> %s' % archive)


if __name__ == '__main__':
    main()
