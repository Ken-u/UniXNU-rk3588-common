#!/usr/bin/env python3
"""Compile the shared RK3588 RKVENC0 video encoder driver (scripts/data/Rk3588Venc) against the actual source-kernel ABI.

  build/rk3588-venc/libRk3588Venc.a

A device lists `build-rk3588-venc.py <board source>...` in `[kernel] archives` (the board's sources are paths relative to the build tree, for
example `kits/orangepi5/Rk3588VencBoard.cpp`: they define the DMA window Rk3588Venc.h declares), force-loads the archive in its defconfig
(`-Wl,-force_load,{tree}/build/rk3588-venc/libRk3588Venc.a`) and calls `rk3588_venc_start()` from its probe.

An XNU driver has to be compiled with the same flags, the same SDK and the same deployment target as the kernel object it lives next
to. The kernel build records the exact command of every object it compiles in a `.cpo.json` file; this script reads that record, points
it at the sources and runs the compiler. Run it from the build tree (the SoC's scripts are copied there), after the first kernel pass.

The encode-job traces the self-test replays (scripts/data/Rk3588Venc/traces/*.vet, binary, packed by tools/make-enc-trace.py) are turned into C arrays
(VencTrace.inc) in the output directory, which the compiler gets as an include path.

The encoder core (scripts/data/Rk3588Venc/core) is plain C: Rockchip's MPP H.264 encoder (controller, rate control, reference management, the vepu580
register generator), fetched at a pinned commit (core/mpp.lock, a tree hash of the files used) and compiled unmodified with the kernel's own flags, plus
a shim that replaces MPP's operating system layer (EncShim.c), the encode loop (VencCore.c) and the few C library functions the kernel does not have
(core/kernel/KernelLibc.c).

The decoder archive (build-rk3588-vdec.py) carries its own copy of many of the same MPP objects and of a shim that defines the same names. Both archives
end up in one kernel image, so every symbol the core defines is renamed with a `mpp_`-keeping prefix: the core's files are compiled twice, the first time
only to list their external symbols (llvm-nm), the second with a force-included header (rename.h in the output directory) that turns every such name into
venc_<name> (the C API of VencCore.h, which already starts with venc_, keeps its names). MPP's module constructors are not left to the kernel's constructor
run either: core/kernel/SglnOverride.h turns them into plain functions, and venc_core_register_singletons() (generated: SglnTable.c) calls them all.
"""
import json
import os
import re
import shutil
import struct
import subprocess
import sys
import importlib.util
from pathlib import Path

SCRIPT = Path(__file__).resolve()
TREE = SCRIPT.parents[1]
DATA = SCRIPT.parent / 'data' / 'Rk3588Venc'
TRACES = sorted((DATA / 'traces').glob('*.vet'))
SOURCES = sorted(DATA.glob('*.cpp')) + [TREE / argument for argument in sys.argv[1:]]
CORE = DATA / 'core'
OUT = TREE / 'build' / 'rk3588-venc'
RECORD = TREE / 'build/xnu/ios-obj/DEVELOPMENT_ARM64_BCM2837/iokit/DEVELOPMENT/IOPlatformExpert.cpo.json'

# MH_MAGIC_64, CPU_TYPE_ARM64, CPU_SUBTYPE_ARM64_ALL, MH_OBJECT.
MACHO = (0xfeedfacf, 0x100000c, 0, 1)
LC_BUILD_VERSION = 0x32
DEPLOYMENT = (2, 0x110000)   # PLATFORM_IOS, iOS 17.0

# Flags of the C++ record that make no sense for a C file.
CXX_ONLY = ('-fsized-deallocation', '-fapple-kext', '--driver-mode=g++', '-fc++-static-destructors=none')
MPP_DEFINES = ('-DHAVE_H264E=1', '-DHAVE_H265E=1', '-DHW_VEPU580')
# Symbols that keep their names although the core defines them: the C API of VencCore.h and the shim's (prefix venc_) and the platform hooks of
# VencPlatform.h (vec_).
KEEP = re.compile(r'^(venc_|vec_)')
CONSTRUCTOR = re.compile(r'^venc_mpp_singleton_add_')


def compiler_command(record, source, destination, c_flags=None):
    """The kernel's own compile command for `source`. c_flags is None for C++ (the driver), else (arguments in front of the record's, arguments after) for a
    C file of the encoder core."""
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


def load_fetch():
    spec = importlib.util.spec_from_file_location('fetch_mpp', CORE / 'fetch_mpp.py')
    fetch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fetch)
    return fetch


def core_sources(fetch):
    """(source, extra compile arguments) of the encoder core: MPP's files (third-party: no warnings, the shim headers of core/kernel/include and the
    constructor override in front of the kernel's), the shim and loop of core/ (the loop includes MPP's controller), and the C library stubs (compiled
    against the kernel's headers only). Also returns MPP's checkout."""
    mpp = fetch.find(TREE / '.cache' / 'mpp')
    shim = ['-I', str(CORE / 'kernel' / 'include')]   # first of all: the kernel's own bsd/sys/stat.h and friends are on the path of the record
    override = ['-include', str(CORE / 'kernel' / 'SglnOverride.h')]
    includes = [argument for directory in fetch.read_lines('mpp-includes.txt') for argument in ('-I', str(mpp / directory))]
    defines = list(MPP_DEFINES) + ['-DMPP_ENC_IMPL_C="%s"' % (mpp / 'mpp/codec/mpp_enc_impl.c')]
    third_party = includes + ['-I', str(CORE), '-w'] + defines
    # VencCore.c includes MPP's controller: that code is third-party too, the file is checked by review and by the host test
    ours = includes + ['-I', str(CORE), '-w'] + defines
    sources = []
    for name in fetch.read_lines('mpp-files.txt'):
        sources.append((mpp / name, (shim + override, third_party)))
    sources += [(CORE / 'EncShim.c', (shim + override, ours)), (CORE / 'VencCore.c', (shim + override, ours))]
    return sources, [(CORE / 'kernel' / 'KernelLibc.c', ([], []))]


def write_trace_include(traces, destination):
    """kVencTraces[]: {name, bytes, size} of every trace, each as a C array."""
    lines = ['// generated by build-rk3588-venc.py from scripts/data/Rk3588Venc/traces/*.vet']
    for index, trace in enumerate(traces):
        data = trace.read_bytes()
        if data[:4] != b'VET1':
            sys.exit('%s is not a VET1 trace' % trace)
        lines.append('static const unsigned char kVencTrace%d[] = {  // %s, %d bytes' % (index, trace.name, len(data)))
        for offset in range(0, len(data), 24):
            lines.append(','.join('0x%02x' % byte for byte in data[offset:offset + 24]) + ',')
        lines.append('};')
    lines.append('static const struct { const char *name; const unsigned char *data; unsigned long size; } kVencTraces[] = {')
    for index, trace in enumerate(traces):
        lines.append('  {"%s", kVencTrace%d, sizeof kVencTrace%d},' % (trace.stem, index, index))
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


def find_tool(names, extra=()):
    for name in names:
        found = shutil.which(name)
        if found:
            return found
    for candidate in extra:
        if Path(candidate).is_file():
            return candidate
    sys.exit('none of %s on PATH; a Mach-O archive needs them' % ', '.join(names))


def ar_tool():
    return find_tool(('llvm-ar', 'ar'), ('/opt/homebrew/opt/llvm/bin/llvm-ar', '/usr/local/opt/llvm/bin/llvm-ar', '/usr/bin/llvm-ar'))


def nm_tool():
    return find_tool(('llvm-nm', 'nm'), ('/opt/homebrew/opt/llvm/bin/llvm-nm', '/usr/local/opt/llvm/bin/llvm-nm', '/usr/bin/llvm-nm'))


def compile_all(record, jobs, directory, logs):
    objects, failed = [], []
    directory.mkdir(parents=True, exist_ok=True)
    for source, c_flags in jobs:
        if not source.is_file():
            sys.exit('build-rk3588-venc: %s is missing' % source)
        destination = directory / (source.stem + '.o')
        destination.unlink(missing_ok=True)
        result = subprocess.run(compiler_command(record, source, destination, c_flags), cwd=record['directory'],
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        logs.append(result.stdout)
        if result.returncode and os.environ.get('VENC_KEEP_GOING'):   # a porting aid: show every file's errors, not only the first file's
            failed.append(source.name)
            sys.stderr.write(''.join(line + '\n' for line in result.stdout.splitlines() if ' error: ' in line))
            continue
        if result.returncode:
            sys.stderr.write(result.stdout[-4000:] + '\n')
            sys.exit('build-rk3588-venc: the compiler rejected %s' % source.name)
        if not destination.is_file():
            sys.exit('build-rk3588-venc: the compiler produced no object for %s' % source.name)
        check_object(destination)
        objects.append(destination)
    if failed:
        sys.exit('build-rk3588-venc: the compiler rejected %s' % ', '.join(failed))
    return objects


def defined_symbols(objects):
    """External symbols the objects define (without the leading underscore Mach-O adds)."""
    names = set()
    for path in objects:
        result = subprocess.run([nm_tool(), '-g', '--defined-only', '-j', str(path)], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True)
        names.update(line[1:] for line in result.stdout.split() if line.startswith('_'))
    return names


def write_rename_header(names, destination):
    renamed = sorted(name for name in names if not KEEP.match(name) and re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', name))
    lines = ['// generated by build-rk3588-venc.py: the external symbols of the encoder core, made private to it (see that script)']
    lines += ['#define %s venc_%s' % (name, name) for name in renamed]
    destination.write_text('\n'.join(lines) + '\n')
    return renamed


def write_singleton_table(constructors, destination):
    """venc_core_register_singletons(): calls every module constructor (MPP_SINGLETON / MPP_MODULE_ADD) of the core once, in a fixed order."""
    lines = ['// generated by build-rk3588-venc.py: the module constructors of the objects of the encoder core', '#include "VencCore.h"']
    lines += ['void %s(void);' % name for name in constructors]
    lines += ['', 'void venc_core_register_singletons(void) {']
    lines += ['  %s();' % name for name in constructors]
    lines += ['}']
    destination.write_text('\n'.join(lines) + '\n')


def main():
    if not SOURCES:
        sys.exit('build-rk3588-venc: no sources next to %s (scripts/data/Rk3588Venc)' % SCRIPT)
    if not RECORD.is_file():
        sys.exit('%s is missing: the kernel step writes it on its first (bootstrap) pass, so run "unixnu build kernel" '
                 'before this builder' % RECORD)
    # The kernel build writes the record with a trailing comma, as a fragment.
    record = json.loads(RECORD.read_text().strip().rstrip(','))
    if not TRACES:
        sys.exit('build-rk3588-venc: no traces next to %s (scripts/data/Rk3588Venc/traces)' % SCRIPT)
    OUT.mkdir(parents=True, exist_ok=True)
    write_trace_include(TRACES, OUT / 'VencTrace.inc')
    logs = []

    # the core, pass one: only to learn which external symbols it defines
    fetch = load_fetch()
    mpp_sources, stubs = core_sources(fetch)
    first = OUT / 'pass1'
    names = defined_symbols(compile_all(record, mpp_sources + stubs, first, logs))
    renamed = write_rename_header(names, OUT / 'rename.h')
    shutil.rmtree(first)

    # pass two: the same files with every one of those symbols renamed, then the driver
    force = ['-include', str(OUT / 'rename.h')]
    objects = compile_all(record, [(source, None) for source in SOURCES], OUT, logs)
    core_objects = compile_all(record, [(source, (force + list(front), back)) for source, (front, back) in mpp_sources + stubs], OUT / 'core', logs)
    # The module constructors are functions named by pasting MPP_SINGLETON's arguments after macro expansion, which the renaming has changed in ways that depend
    # on whether the module's name is also the name of a symbol: the table is made from the names that came out (it needs no renaming itself).
    constructors = sorted(name for name in defined_symbols(core_objects) if CONSTRUCTOR.match(name))
    if not constructors:
        sys.exit('build-rk3588-venc: no module constructor found in the core (MPP_SINGLETON was not overridden?)')
    write_singleton_table(constructors, OUT / 'SglnTable.c')
    objects += core_objects + compile_all(record, [(OUT / 'SglnTable.c', ([], ['-I', str(CORE)]))], OUT / 'core', logs)
    (OUT / 'compile.log').write_text(''.join(logs))
    archive = OUT / 'libRk3588Venc.a'
    archive.unlink(missing_ok=True)
    subprocess.run([ar_tool(), 'rcs', str(archive)] + [str(path) for path in objects], check=True)
    print('build-rk3588-venc: -Werror build and ARM64/iOS 17 object audit passed -> %s (%d core symbols renamed, %d module constructors)' %
          (archive, len(renamed), len(constructors)))


if __name__ == '__main__':
    main()
