// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// Rk3588VopDirect: zero-copy VOP2 scanout shared by RK3588 boards.
//
// The compositor's surfaces sit on scattered 4 KiB pages (some above 4 GiB). VOP2's window address
// register is 32-bit, so the VOP IOMMU (rockchip,iommu-v2, two banks) presents them as a linear IOVA
// range. With rk3588-vop-direct=1 (alias evb10-vop-direct=1) NativeLegacyDisplay flips that IOVA into
// the window instead of CPU-copying into the firmware GOP buffer.
//
// Plane / video port (boot-args, defaults favour Orange Pi 5 HDMI):
//   rk3588-vop-esmart=N   Esmart window 0..3 (OPi5 HDMI=2, EVB10 panel=3)
//   rk3588-vop-vp=N       Video port 0..3 for CFG_DONE (OPi5 HDMI VP2=2, EVB10 VP3=3)
//
// Symbol names keep the EVB10 `evb10_vop_direct_*` prefix so existing NativeLegacyDisplay forks link.
#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

// [0] 1=ok 2=failed; [1]/[2] MMU STATUS after init; [3] remaps; [4] zap wait timeouts.
extern volatile uint32_t evb10_vop_direct_stats[8];

bool evb10_vop_direct_init(void);
uint32_t evb10_vop_direct_map(unsigned slot, const uint64_t *pages, unsigned count);
void evb10_vop_direct_flip(uint32_t iova, uint32_t stride);
bool evb10_vop_direct_latched(void);
void evb10_vop_direct_boot(uint32_t *iova, uint32_t *stride);
void evb10_vop_direct_check(uint32_t iova);

#ifdef __cplusplus
}
#endif
