// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// EncShim.h -- the few things of core/EncShim.c that core/VencCore.c uses beyond MPP's own interfaces. Needs MPP's headers.
#pragma once
#include "VencCore.h"
#include "mpp_buffer.h"
#include "mpp_device.h"

#ifdef __cplusplus
extern "C" {
#endif

// Points the shim at the block and its window (the DMA window is global: MPP's buffer calls carry no context).
void venc_shim_bind(const VencHw *hw);
void venc_shim_set_observer(MppDev dev, VencJobObserver *observer);
void venc_shim_window_stats(uint32_t *buffers, uint32_t *bytes, uint32_t *peak);
uint64_t venc_shim_buffer_bus(MppBuffer buffer);

// MPP's one-time module setups (see mpp_singleton_add in EncShim.c); 0 when they ran (now or before), negative when no module registered itself.
int venc_shim_run_singletons(void);
void venc_shim_deinit_singletons(void);

#ifdef __cplusplus
}
#endif
