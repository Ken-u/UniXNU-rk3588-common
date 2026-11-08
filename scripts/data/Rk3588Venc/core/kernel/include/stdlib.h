#pragma once
#include <stddef.h>
void abort(void) __attribute__((noreturn));
void qsort(void *, size_t, size_t, int (*)(const void *, const void *));
int abs(int);
long double strtold(const char *, char **);
#define alloca(n) __builtin_alloca(n)
