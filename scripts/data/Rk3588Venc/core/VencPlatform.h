// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// VencPlatform.h -- what the encoder core (core/VencCore.c, core/EncShim.c and the unmodified MPP sources they run) needs from the program it is linked
// into. The host test supplies malloc/printf; the kernel build supplies IOMalloc/printf and the real block. Plain C so it can be included from the C of
// MPP's tree as well as the C++ of the kernel driver.
//
// (The names differ from the decoder core's VdecPlatform.h on purpose: both cores end up in one kernel image and each carries its own copy of MPP's
// objects; see core/rename.py.)
#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Memory for MPP's objects (contexts, configuration tables, ...). Zero-filled; the size is passed back on free because the kernel allocator wants it.
void *vec_calloc(size_t size);
void vec_free(void *p, size_t size);

// One line of text (no newline). level: 1 fatal, 2 error, 3 warning, 4 info, 5 debug, 6 verbose (MPP's numbering).
void vec_log(int level, const char *line);

// How loud MPP's own messages are (messages above this level are dropped before formatting). The embedder may change it at any time.
extern int vec_log_level;

// Monotonic microseconds (MPP's mpp_time(): rate control and stop watches look at it).
int64_t vec_time_us(void);

#ifdef __cplusplus
}
#endif
