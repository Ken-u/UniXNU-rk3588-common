The C library headers MPP's sources include, for the kernel build only (scripts/build-rk3588-vdec.py puts this directory first on the include path of the
core's C files and of nothing else). The kernel is compiled with -nostdlibinc and has no <stdio.h>: these declare exactly what the compiled MPP files use.
What the kernel has (mem*/str*, qsort, abs, snprintf, sprintf, vsnprintf) is declared as the kernel does; the rest is either an inline no-op (pthread: the
core runs in one thread under its caller's lock) or a stub in ../KernelLibc.c that fails (files, directories) on paths that are debug options of MPP and
never run.
