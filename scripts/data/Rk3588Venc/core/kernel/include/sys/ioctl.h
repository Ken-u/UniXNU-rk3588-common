#pragma once
// The Linux request number encoding (direction, type, number, size), which kmpp's headers use to name the requests of its kernel module; the call itself
// fails (the kernel's own ioctl is a system call handler: this one is bound to the stub venc_ioctl of KernelLibc.c).
#define _IOC(dir, type, nr, size) (((unsigned long)(dir) << 30) | ((unsigned long)(size) << 16) | ((unsigned long)(type) << 8) | (unsigned long)(nr))
#define _IO(type, nr) _IOC(0, type, nr, 0)
#define _IOR(type, nr, size) _IOC(2, type, nr, sizeof(size))
#define _IOW(type, nr, size) _IOC(1, type, nr, sizeof(size))
#define _IOWR(type, nr, size) _IOC(3, type, nr, sizeof(size))
int ioctl(int, unsigned long, ...) __asm__("_venc_ioctl");
