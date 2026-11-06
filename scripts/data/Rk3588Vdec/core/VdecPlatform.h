// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// VdecPlatform.h -- what the decoder core (core/VdecCore.c, core/MppShim.c and the unmodified MPP sources they run) needs from the program it is
// linked into. The host test supplies malloc/printf; the kernel build supplies IOMalloc/printf and the real block. Plain C so it can be included from
// the C of MPP's tree as well as the C++ of the kernel driver.
#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Memory for MPP's objects (parser contexts, slot tables, ...). Zero-filled; size is passed back on free because the kernel allocator wants it.
void *vdc_calloc(size_t size);
void vdc_free(void *p, size_t size);

// One line of text (no newline). level: 1 fatal, 2 error, 3 warning, 4 info, 5 debug, 6 verbose (MPP's numbering).
void vdc_log(int level, const char *line);

// How loud MPP's own messages are (messages above this level are dropped before formatting). The embedder may change it at any time.
extern int vdc_log_level;

#ifdef __cplusplus
}
#endif
