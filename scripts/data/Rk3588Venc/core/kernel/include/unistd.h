#pragma once
#include <stddef.h>
#include <sys/types.h>
typedef long ssize_t;
static inline int getpid(void) { return 0; }
#define SEEK_SET 0
#define SEEK_END 2
// the file calls are only reached from MPP's configuration-file loading, which nothing asks for (KernelLibc.c: they fail the way a missing file does)
// (the kernel has its own open, close, read and lseek: system call handlers with other signatures; these go to the stubs venc_open ... of KernelLibc.c)
int open(const char *, int, ...) __asm__("_venc_open");
int close(int) __asm__("_venc_close");
ssize_t read(int, void *, size_t) __asm__("_venc_read");
off_t lseek(int, off_t, int) __asm__("_venc_lseek");
