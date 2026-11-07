#pragma once
#include <stdarg.h>
#include <stddef.h>
typedef struct vdc_file FILE;   // never has a value: fopen below fails
int snprintf(char *, size_t, const char *, ...) __attribute__((format(printf, 3, 4)));
int sprintf(char *, const char *, ...) __attribute__((format(printf, 2, 3)));
int vsnprintf(char *, size_t, const char *, va_list) __attribute__((format(printf, 3, 0)));
int printf(const char *, ...) __attribute__((format(printf, 1, 2)));
FILE *fopen(const char *, const char *);
int fclose(FILE *);
int fflush(FILE *);
size_t fwrite(const void *, size_t, size_t, FILE *);
size_t fread(void *, size_t, size_t, FILE *);
int fprintf(FILE *, const char *, ...);
