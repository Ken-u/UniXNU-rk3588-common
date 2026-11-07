#pragma once
#include <sys/types.h>
#define S_IRWXU 0700
#define S_IRGRP 040
#define S_IXGRP 010
#define S_IROTH 04
#define S_IXOTH 01
// the kernel defines mkdir() as the system call (another signature): calls to this one go to the symbol vdc_mkdir (KernelLibc.c)
int mkdir(const char *, mode_t) __asm__("_vdc_mkdir");
