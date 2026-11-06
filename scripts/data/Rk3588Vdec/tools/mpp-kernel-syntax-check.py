#!/usr/bin/env python3
"""Syntax-check Rockchip MPP sources with the flags of the XNU kernel build (a feasibility probe for running MPP's decoders in the kernel).

  mpp-kernel-syntax-check.py MPP_CHECKOUT KERNEL_TREE SOURCE...

MPP_CHECKOUT is a checkout of rockchip-linux/mpp (the traces were recorded with 9b56348), KERNEL_TREE the build tree of a device whose kernel has
been built once (it has build/xnu/ios-obj/.../IOPlatformExpert.cpo.json, the compile record the other archive builders use), the SOURCEs are
files under the checkout. Each is compiled as C with -fsyntax-only and the kernel's -mkernel / -nostdlibinc flags against a directory of EMPTY
libc headers (stdio.h, stdlib.h, ... except those clang itself provides), so every error left is something the kernel does not have: this prints
the error lines, grouped, per file. Nothing is linked or run.

Result with 9b56348 (2026-11-07): the ten files of codec/dec/h264 (the parser) fail only on libc declarations: abort (35 uses), FILE and the stdio calls
of its debug dumps, qsort, abs, sprintf, and the pthread / clockid types that mpp_thread.h pulls in; no kernel-incompatible construct showed up (see
UniXNU-orangepi5/evidence/host-mpp-kernel-port-20261107.txt).
"""
import json
import subprocess
import sys
import tempfile
from collections import Counter
from pathlib import Path

LIBC_HEADERS = ('stdio', 'stdlib', 'string', 'errno', 'unistd', 'pthread', 'semaphore', 'time', 'assert', 'math', 'ctype', 'fcntl', 'signal',
                'sys/types', 'sys/time', 'sys/stat', 'sys/ioctl', 'sys/mman')
MPP_INCLUDES = ('mpp/base/inc', 'mpp/inc', 'inc', 'osal/inc', 'utils', 'mpp/codec/inc', 'mpp/hal/inc', 'mpp/common', 'mpp/codec/dec/common',
                'mpp/codec/dec/h264', 'mpp/hal/rkdec/h264d', 'mpp/hal/rkdec/inc', 'mpp/hal/common', 'mpp/hal/vpu', 'osal', 'mpp/hal/rkdec/common')


def kernel_command(record, source, shim, mpp, tree):
    command, arguments, index = [], record['arguments'], 0
    while index < len(arguments):
        value = arguments[index]
        if value == record['file']:
            command.append(str(source))
        elif value == '-o':
            command.extend(['-o', '/dev/null'])
            index += 1
        elif value == '-xc++':
            command.append('-xc')
        elif value.startswith('-std=gnu++'):
            command.append('-std=gnu11')
        elif value in ('-fsized-deallocation', '-fapple-kext', '--driver-mode=g++', '-fc++-static-destructors=none'):
            pass
        elif value == '-Xclang' and arguments[index + 1].startswith('-W'):
            index += 1
        elif not value.startswith('-W'):
            command.append(value)
        index += 1
    for directory in (shim,) + tuple(mpp / name for name in MPP_INCLUDES):
        command.extend(['-I', str(directory)])
    command.extend(['-I', str(tree / '.cache/xnu-work/osfmk'), '-w', '-fsyntax-only'])
    return command


def main():
    if len(sys.argv) < 4:
        sys.exit(__doc__)
    mpp, tree, sources = Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve(), [Path(name).resolve() for name in sys.argv[3:]]
    record_path = tree / 'build/xnu/ios-obj/DEVELOPMENT_ARM64_BCM2837/iokit/DEVELOPMENT/IOPlatformExpert.cpo.json'
    record = json.loads(record_path.read_text().strip().rstrip(','))
    with tempfile.TemporaryDirectory() as shim_name:
        shim = Path(shim_name)
        for name in LIBC_HEADERS:
            (shim / (name + '.h')).parent.mkdir(parents=True, exist_ok=True)
            (shim / (name + '.h')).write_text('/* empty on purpose: the kernel has no such header */\n')
        # Constants MPP's inline helpers use; nothing else is provided.
        (shim / 'fcntl.h').write_text('#define F_DUPFD 0\n#define F_SETFD 2\n#define FD_CLOEXEC 1\nint fcntl(int, int, ...);\n')
        total = Counter()
        for source in sources:
            result = subprocess.run(kernel_command(record, source, shim, mpp, tree), cwd=record['directory'],
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            errors = [line for line in result.stdout.splitlines() if ' error: ' in line]
            print('%s: %d error(s)' % (source.name, len(errors)))
            for line in errors:
                message = line.split(' error: ', 1)[1]
                total[message.split('[')[0].strip()[:110]] += 1
        print('\nerror classes over all files:')
        for message, count in total.most_common():
            print('%4d  %s' % (count, message))


if __name__ == '__main__':
    main()
