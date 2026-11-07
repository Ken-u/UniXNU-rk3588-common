#pragma once
extern int vdc_errno;   // one for the core (KernelLibc.c)
#define errno vdc_errno
#define EPERM 1
#define ENOENT 2
#define EINTR 4
#define EIO 5
#define ENOMEM 12
#define EACCES 13
#define EEXIST 17
#define EINVAL 22
#define ENAMETOOLONG 63
