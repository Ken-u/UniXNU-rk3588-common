// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// The few C library functions MPP's sources call that the kernel does not have (core/kernel/include/ declares them). Compiled without that include
// directory: this file talks to the kernel's own headers.
//
// None of the file and directory calls is on a path that runs: they are MPP's debug dumps (the H.264 parser writes the stream it gets to a file when an
// environment variable asks for it, which the shim's mpp_env_get_u32 never answers) and its directory creation. They fail the way a missing file does.
// abort() is what a broken invariant inside MPP's buffer slots leads to (slot_assert); the kernel has its own, which panics. The kernel also has a mkdir (the
// system call, another signature), so MPP's calls to it are bound to the symbol vdc_mkdir by include/sys/stat.h.
#include <stdarg.h>
#include <stddef.h>

typedef struct vdc_file FILE;
typedef struct vdc_dir DIR;
struct dirent;

int vdc_errno;

int vsnprintf(char *, size_t, const char *, va_list);

// The kernel declares sprintf but does not define it any more; MPP's debug code names files with it.
int sprintf(char *buffer, const char *format, ...) {
  va_list ap;
  va_start(ap, format);
  int n = vsnprintf(buffer, 0x7fffffff, format, ap);
  va_end(ap);
  return n;
}

int abs(int x) {
  return x < 0 ? -x : x;
}

FILE *fopen(const char *path, const char *mode) {
  (void)path;
  (void)mode;
  return NULL;
}
int fclose(FILE *f) {
  (void)f;
  return -1;
}
int fflush(FILE *f) {
  (void)f;
  return -1;
}
size_t fwrite(const void *p, size_t size, size_t n, FILE *f) {
  (void)p;
  (void)size;
  (void)n;
  (void)f;
  return 0;
}
size_t fread(void *p, size_t size, size_t n, FILE *f) {
  (void)p;
  (void)size;
  (void)n;
  (void)f;
  return 0;
}
int vdc_mkdir(const char *path, unsigned int mode) {
  (void)path;
  (void)mode;
  return -1;
}
DIR *opendir(const char *path) {
  (void)path;
  return NULL;
}
struct dirent *readdir(DIR *d) {
  (void)d;
  return NULL;
}
int closedir(DIR *d) {
  (void)d;
  return -1;
}
