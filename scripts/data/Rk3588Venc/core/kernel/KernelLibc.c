// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// The few C library functions MPP's sources call that the kernel does not have (core/kernel/include/ declares them). Compiled without that include
// directory: this file talks to the kernel's own headers.
//
// None of the file and directory calls is on a path that runs: they are MPP's debug dumps (the H.264 parser writes the stream it gets to a file when an
// environment variable asks for it, which the shim's mpp_env_get_u32 never answers) and its directory creation. They fail the way a missing file does.
// abort() is what a broken invariant inside MPP's buffer slots leads to (slot_assert); the kernel has its own, which panics. The kernel also has a mkdir (the
// system call, another signature), so MPP's calls to it are bound to the symbol venc_mkdir by include/sys/stat.h.
#include <stdarg.h>
#include <stddef.h>

typedef struct venc_file FILE;
typedef struct venc_dir DIR;
struct dirent;

int venc_errno;

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
int venc_mkdir(const char *path, unsigned int mode) {
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

// The file descriptor calls (kmpp's /dev/kmpp_ioc, configuration files) are bound by include/unistd.h to these names: the kernel's own open() and friends are
// system call handlers.
typedef long venc_ssize_t;
int venc_open(const char *path, int flags, ...) {
  (void)path;
  (void)flags;
  return -1;
}
int venc_close(int fd) {
  (void)fd;
  return -1;
}
venc_ssize_t venc_read(int fd, void *p, size_t n) {
  (void)fd;
  (void)p;
  (void)n;
  return -1;
}
long venc_lseek(int fd, long offset, int whence) {
  (void)fd;
  (void)offset;
  (void)whence;
  return -1;
}
int venc_ioctl(int fd, unsigned long request, ...) {
  (void)fd;
  (void)request;
  return -1;
}
char *strerror(int error) {
  (void)error;
  return (char *)"error";
}
long double strtold(const char *text, char **end) {
  if (end) *end = (char *)text;
  return 0;
}

// natural logarithm, for the one place MPP's rate control takes it (the starting quantiser of a re-encoded intra picture from the mean absolute difference of
// the picture): x = m * 2^e with m in [sqrt(1/2), sqrt(2)), log m = 2 (s + s^3/3 + s^5/5 + ...) with s = (m - 1) / (m + 1); the error is below 1e-9.
double log(double x) {
  if (!(x > 0)) return -1e300;
  union {
    double d;
    unsigned long long u;
  } bits = {x};
  int e = (int)((bits.u >> 52) & 0x7ff) - 1023;
  if (e == -1023) return -1e300;   // subnormal: not a value the rate control passes
  bits.u = (bits.u & 0x000fffffffffffffull) | 0x3ff0000000000000ull;   // m in [1, 2)
  double m = bits.d;
  if (m > 1.4142135623730951) {
    m *= 0.5;
    e++;
  }
  double s = (m - 1) / (m + 1), s2 = s * s, term = s, sum = 0;
  for (int k = 1; k < 24; k += 2) {
    sum += term / k;
    term *= s2;
  }
  return 2 * sum + e * 0.6931471805599453;
}

// log2 and ceil, for the one place the H.265 register generator takes them: log2_ctu_num = ceil(log2(number of 64x64 units)). The number is whole, and for a power of
// two the answer must be exact (2^e is e + log(1) / ln 2 = e: the exponent comes out of the double first), or the picture would be told it has twice the units.
double log2(double x) {
  if (!(x > 0)) return -1e300;
  union {
    double d;
    unsigned long long u;
  } bits = {x};
  int e = (int)((bits.u >> 52) & 0x7ff) - 1023;
  if (e == -1023) return -1e300;
  bits.u = (bits.u & 0x000fffffffffffffull) | 0x3ff0000000000000ull;   // m in [1, 2)
  return e + log(bits.d) * 1.4426950408889634;
}
double ceil(double x) {
  const double truncated = (double)(long long)x;   // (values far from the range of long long are not what the register generator passes)
  return truncated < x ? truncated + 1 : truncated;
}

// The kernel has no strcpy or strstr (it has strlcpy and strnstr); MPP's string handling in its configuration code calls these.
char *strcpy(char *to, const char *from) {
  char *start = to;
  while ((*to++ = *from++) != 0) {
  }
  return start;
}
char *strstr(const char *haystack, const char *needle) {
  if (!*needle) return (char *)haystack;
  for (; *haystack; haystack++) {
    const char *h = haystack, *n = needle;
    while (*h && *n && *h == *n) {
      h++;
      n++;
    }
    if (!*n) return (char *)haystack;
  }
  return 0;
}
