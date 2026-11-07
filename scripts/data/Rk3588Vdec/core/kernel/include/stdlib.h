#pragma once
#include <stddef.h>
void abort(void) __attribute__((noreturn));
void qsort(void *, size_t, size_t, int (*)(const void *, const void *));
int abs(int);
