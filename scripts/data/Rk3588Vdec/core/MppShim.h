// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// MppShim.h -- the few things of core/MppShim.c that core/VdecCore.c uses beyond MPP's own interfaces. Needs MPP's headers.
#pragma once
#include "VdecCore.h"
#include "mpp_buffer.h"
#include "mpp_device.h"

#ifdef __cplusplus
extern "C" {
#endif

// Points the shim at the block and its window (the DMA window is global: MPP's buffer calls carry no context).
void vdec_shim_bind(const VdecHw *hw);
void vdec_shim_set_observer(MppDev dev, VdecJobObserver *observer);
void vdec_shim_window_stats(uint32_t *buffers, uint32_t *bytes, uint32_t *peak);
uint64_t vdec_shim_buffer_bus(MppBuffer buffer);

#ifdef __cplusplus
}
#endif
