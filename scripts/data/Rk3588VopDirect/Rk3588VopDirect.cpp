// SPDX-License-Identifier: BSD-2-Clause-Patent
#include "Rk3588VopDirect.h"
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <pexpert/pexpert.h>
#include <mach/vm_param.h>
extern "C" void pmap_set_cache_attributes(unsigned int pn, unsigned int attributes);

// The CPU copy of a frame into the write-combined scanout buffer takes ~40–50 ms for 1080p: neither
// that nor a GPU blit into the same buffer can reach 60 fps. Nothing needs copying if the VOP window
// reads the compositor surface where it already sits. See Linux drivers/iommu/rockchip-iommu.c
// (rk_mk_dte_v2 / rk_mk_pte_v2) for the page-table encoding.
extern "C" vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern "C" unsigned int ml_phys_read_word(vm_offset_t paddr);
extern "C" void flush_dcache64(unsigned long long addr, unsigned count, int phys);
extern "C" bool mali_clean_caches(void);
extern "C" uint64_t evb10_fb_base;
extern "C" uint32_t evb10_fb_row;
extern "C" int (*evb10_direct_pixel_hook)(uint64_t byteOffset, uint32_t *value);

namespace {

constexpr uint64_t kVopPhys = 0xfdd90000ULL;
constexpr uint32_t kVopSize = 0x10000, kMmuBank[2] = {0x7e00, 0x7f00};
constexpr uint32_t kIovaSlotBase = 0xf0000000U, kIovaSlotBytes = 0x01000000U, kIovaSlots = 16;
constexpr uint32_t kIdentityTables = 960;
// Esmart0 REGION0 YRGB_MST / VIR; each Esmart is +0x200.
constexpr uint32_t kEsmart0Mst = 0x1814, kEsmart0Vir = 0x181c;
constexpr uint32_t kVopCfgDone = 0;

volatile uint8_t *vopRegs;
IOBufferMemoryDescriptor *vopTables;
uint32_t *vopDt, *vopPt;
uint64_t vopTablesPhys;
bool vopDirectReady, vopDirectTried;
uint32_t vopWinMst, vopWinVir, vopCommit;
uint32_t vopFlipIova, vopFlipStride, vopShownIova, vopShownStride;
uint64_t vopSlotPages[kIovaSlots][kIovaSlotBytes / 4096];
uint32_t vopSlotCount[kIovaSlots];

inline uint32_t vopRead(uint32_t off) { return *reinterpret_cast<volatile uint32_t *>(vopRegs + off); }
inline void vopWrite(uint32_t off, uint32_t value) { *reinterpret_cast<volatile uint32_t *>(vopRegs + off) = value; }
inline uint32_t mmuRead(unsigned bank, uint32_t off) { return vopRead(kMmuBank[bank] + off); }
inline void mmuWrite(unsigned bank, uint32_t off, uint32_t value) { vopWrite(kMmuBank[bank] + off, value); }
void mmuCommand(uint32_t command) { for (unsigned b = 0; b < 2; ++b) mmuWrite(b, 0x08, command); }

bool mmuWait(uint32_t mask, uint32_t value) {
	for (unsigned i = 0; i < 10000; ++i) {
		if ((mmuRead(0, 0x04) & mask) == value && (mmuRead(1, 0x04) & mask) == value) return true;
		IODelay(10);
	}
	return false;
}

inline uint32_t mmuEntry(uint64_t pa, uint32_t flags) {
	uint64_t v = (pa & 0xfffff000ULL) | ((pa & 0xf00000000ULL) >> 24) | ((pa & 0xf000000000ULL) >> 32);
	return uint32_t(v & 0xfffffff0ULL) | flags;
}
inline uint64_t tablePhys(uint32_t index) { return vopTablesPhys + 4096ULL * (1 + index); }

// CFG_DONE_EN | BIT(vp) | (BIT(vp) << 16)
inline uint32_t cfgDoneForVp(unsigned vp) {
	return 0x8000U | (1U << vp) | (1U << (vp + 16));
}

void selectWindow(void) {
	unsigned esmart = 2, vp = 2;
	PE_parse_boot_argn("rk3588-vop-esmart", &esmart, sizeof(esmart));
	PE_parse_boot_argn("rk3588-vop-vp", &vp, sizeof(vp));
	// EVB10 historical defaults when only evb10-vop-direct is set and no rk3588-* given: keep OPi5 defaults
	// unless the board asks for esmart 3 (panel).
	if (esmart > 3) esmart = 2;
	if (vp > 3) vp = 2;
	vopWinMst = kEsmart0Mst + esmart * 0x200;
	vopWinVir = kEsmart0Vir + esmart * 0x200;
	vopCommit = cfgDoneForVp(vp);
	printf("rk3588: vop-direct: Esmart%u MST %#x VIR %#x VP%u cfg_done %#x\n",
		esmart, vopWinMst, vopWinVir, vp, vopCommit);
}

static int vopDirectPixel(uint64_t byteOffset, uint32_t *value);

} // namespace

volatile uint32_t evb10_vop_direct_stats[8];
volatile uint32_t evb10_vop_check[8];
volatile uint64_t evb10_vop_check_at;

extern "C" bool evb10_vop_direct_init(void) {
	if (vopDirectReady) return true;
	if (vopDirectTried) return false;
	vopDirectTried = true;
	selectWindow();
	vopRegs = reinterpret_cast<volatile uint8_t *>(ml_io_map(kVopPhys, kVopSize));
	if (!vopRegs) {
		evb10_vop_direct_stats[0] = 2;
		printf("rk3588: vop-direct: no mapping of the VOP\n");
		return false;
	}
	const uint32_t before[2] = {mmuRead(0, 0x04), mmuRead(1, 0x04)};
	const uint64_t bytes = 4096ULL * (1 + 1024);
	// Page-table entries encode a 40-bit PA (rockchip iommu-v2). EVB10 asked for memory below
	// 4 GiB; Orange Pi 5's owned window is the upper 4 GiB bank, so that mask fails. Prefer any
	// contiguous pages the kernel can give (still within 40 bits), fall back to the low mask.
	vopTables = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
		static_cast<IOOptionBits>(kIODirectionInOut) | static_cast<IOOptionBits>(kIOMemoryMapperNone) |
			kIOMemoryPhysicallyContiguous | kIOMapInhibitCache,
		bytes, 0x000000ffffffffffULL & ~0xfffULL);
	if (!vopTables) {
		vopTables = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
			static_cast<IOOptionBits>(kIODirectionInOut) | static_cast<IOOptionBits>(kIOMemoryMapperNone) |
				kIOMemoryPhysicallyContiguous | kIOMapInhibitCache,
			bytes, 0x00000000fffff000ULL);
	}
	if (!vopTables) {
		evb10_vop_direct_stats[0] = 2;
		printf("rk3588: vop-direct: no %llu contiguous bytes for the page tables\n", bytes);
		return false;
	}
	IOByteCount length = 0;
	if (vopTables->prepare() != kIOReturnSuccess) {
		evb10_vop_direct_stats[0] = 2;
		printf("rk3588: vop-direct: the page tables could not be wired\n");
		return false;
	}
	vopTablesPhys = vopTables->getPhysicalSegment(0, &length, kIOMemoryMapperNone);
	vopDt = static_cast<uint32_t *>(vopTables->getBytesNoCopy());
	if (!vopDt || !vopTablesPhys || length < bytes || (vopTablesPhys & 0xfff) || vopTablesPhys + bytes > (1ULL << 40)) {
		evb10_vop_direct_stats[0] = 2;
		printf("rk3588: vop-direct: tables at %#llx, %llu contiguous bytes of %llu: not usable\n",
			vopTablesPhys, (uint64_t)length, bytes);
		return false;
	}
	vopPt = vopDt + 1024;
	for (uint64_t i = 0; i < bytes >> PAGE_SHIFT; ++i)
		pmap_set_cache_attributes((unsigned int)((vopTablesPhys >> PAGE_SHIFT) + i), 0x6U);
	for (uint32_t d = 0; d < 1024; ++d) {
		vopDt[d] = mmuEntry(tablePhys(d), 1);
		uint32_t *pt = vopPt + 1024ULL * d;
		for (uint32_t i = 0; i < 1024; ++i)
			pt[i] = d < kIdentityTables ? mmuEntry((uint64_t(d) << 22) | (uint64_t(i) << 12), 3) : 0;
	}
	__asm__ volatile("dsb sy\n\tisb" ::: "memory");
	if ((before[0] & 1) && (before[1] & 1)) {
		mmuCommand(2);
		if (!mmuWait(0x4, 0x4)) {
			printf("rk3588: vop-direct: stall timed out (status %08x %08x)\n", mmuRead(0, 0x04), mmuRead(1, 0x04));
			evb10_vop_direct_stats[0] = 2;
			return false;
		}
	}
	mmuCommand(6);
	IODelay(100);
	for (unsigned b = 0; b < 2; ++b) {
		mmuWrite(b, 0x00, mmuEntry(vopTablesPhys, 1));
		mmuWrite(b, 0x08, 4);
		mmuWrite(b, 0x1c, 0);
		mmuWrite(b, 0x24, mmuRead(b, 0x24) | 0x80000000U);
	}
	mmuCommand(0);
	const bool paging = mmuWait(0x1, 0x1);
	for (unsigned b = 0; b < 2; ++b)
		if (mmuRead(b, 0x04) & 0x4) mmuWrite(b, 0x08, 3);
	mmuWait(0x4, 0);
	evb10_vop_direct_stats[1] = mmuRead(0, 0x04);
	evb10_vop_direct_stats[2] = mmuRead(1, 0x04);
	printf("rk3588: vop-direct: IOMMU status %08x %08x before, %08x %08x after (paging %s), tables at %#llx, "
		"dte %08x %08x, window %08x vir %08x\n",
		before[0], before[1], mmuRead(0, 0x04), mmuRead(1, 0x04), paging ? "on" : "NOT ON", vopTablesPhys,
		mmuRead(0, 0x00), mmuRead(1, 0x00), vopRead(vopWinMst), vopRead(vopWinVir));
	if (!paging) {
		evb10_vop_direct_stats[0] = 2;
		return false;
	}
	vopDirectReady = true;
	evb10_vop_direct_stats[0] = 1;
	evb10_direct_pixel_hook = &vopDirectPixel;
	return true;
}

extern "C" void evb10_vop_direct_boot(uint32_t *iova, uint32_t *stride) {
	if (evb10_fb_base && evb10_fb_row) {
		*iova = uint32_t(evb10_fb_base);
		*stride = evb10_fb_row;
		return;
	}
	PE_Video video = {};
	if (!PE_current_console(&video) && video.v_baseAddr && video.v_rowBytes) {
		*iova = uint32_t(video.v_baseAddr);
		*stride = uint32_t(video.v_rowBytes);
		return;
	}
	*iova = 0;
	*stride = 0;
}

extern "C" uint32_t evb10_vop_direct_map(unsigned slot, const uint64_t *pages, unsigned count) {
	if (!vopDirectReady || slot >= kIovaSlots || !count || count > kIovaSlotBytes / 4096) return 0;
	const uint32_t iova = kIovaSlotBase + slot * kIovaSlotBytes;
	uint32_t *pt = vopPt + 1024ULL * (iova >> 22);
	for (unsigned i = 0; i < kIovaSlotBytes / 4096; ++i)
		pt[i] = i < count ? mmuEntry(pages[i], 3) : 0;
	for (unsigned i = 0; i < count; ++i) vopSlotPages[slot][i] = pages[i];
	vopSlotCount[slot] = count;
	__asm__ volatile("dsb sy" ::: "memory");
	mmuCommand(2);
	if (!mmuWait(0x4, 0x4)) evb10_vop_direct_stats[4] = evb10_vop_direct_stats[4] + 1;
	mmuCommand(4);
	mmuCommand(3);
	evb10_vop_direct_stats[3] = evb10_vop_direct_stats[3] + 1;
	return iova;
}

extern "C" void evb10_vop_direct_flip(uint32_t iova, uint32_t stride) {
	vopWrite(vopWinMst, iova);
	vopWrite(vopWinVir, stride / 4);
	__asm__ volatile("dsb sy" ::: "memory");
	vopFlipIova = iova;
	vopFlipStride = stride;
	if (iova < kIovaSlotBase) vopShownIova = iova;
	vopWrite(kVopCfgDone, vopCommit);
	__asm__ volatile("dsb sy" ::: "memory");
}

extern "C" bool evb10_vop_direct_latched(void) {
	const bool latched = vopRead(vopWinMst) == vopFlipIova;
	if (latched) {
		vopShownStride = vopFlipStride;
		vopShownIova = vopFlipIova;
	}
	return latched;
}

namespace {
static int vopDirectPixel(uint64_t byteOffset, uint32_t *value) {
	const uint32_t iova = vopShownIova, rowBytes = evb10_fb_row ? evb10_fb_row : vopShownStride;
	if (iova < kIovaSlotBase || !rowBytes || !vopShownStride) return 0;
	const uint32_t slot = (iova - kIovaSlotBase) / kIovaSlotBytes;
	if (slot >= kIovaSlots) return 0;
	const uint64_t at = uint64_t(iova - kIovaSlotBase - slot * kIovaSlotBytes) +
		(byteOffset / rowBytes) * vopShownStride + byteOffset % rowBytes;
	if ((at >> 12) >= vopSlotCount[slot]) return 0;
	*value = ml_phys_read_word(vm_offset_t(vopSlotPages[slot][at >> 12] + (at & 0xfff)));
	return 1;
}
} // namespace

extern "C" void evb10_vop_direct_check(uint32_t iova) {
	if (!vopDirectReady || iova < kIovaSlotBase) return;
	const uint32_t slot = (iova - kIovaSlotBase) / kIovaSlotBytes;
	if (slot >= kIovaSlots || !vopSlotCount[slot]) return;
	constexpr unsigned kSamples = 512;
	static uint32_t before[kSamples];
	static uint64_t where[kSamples];
	const uint64_t bytes = uint64_t(vopSlotCount[slot]) * 4096;
	const uint64_t step = (bytes / kSamples) & ~uint64_t(3);
	const uint64_t shift = (uint64_t(evb10_vop_check[0]) * 52) & 0x3f;
	for (unsigned i = 0; i < kSamples; ++i) {
		const uint64_t at = ((uint64_t(i) * step + shift) % bytes) & ~uint64_t(3);
		const uint64_t pa = vopSlotPages[slot][at >> 12] + (at & 0xfff);
		where[i] = pa;
		flush_dcache64(pa & ~63ULL, 64, 1);
		before[i] = ml_phys_read_word(vm_offset_t(pa));
	}
	const bool cleaned = mali_clean_caches();
	uint32_t changed = 0;
	for (unsigned i = 0; i < kSamples; ++i) {
		flush_dcache64(where[i] & ~63ULL, 64, 1);
		const uint32_t after = ml_phys_read_word(vm_offset_t(where[i]));
		if (after != before[i]) {
			if (!changed && !evb10_vop_check[2]) {
				evb10_vop_check[3] = before[i];
				evb10_vop_check[4] = after;
				evb10_vop_check_at = where[i];
			}
			++changed;
		}
	}
	evb10_vop_check[0] = evb10_vop_check[0] + 1;
	evb10_vop_check[1] = evb10_vop_check[1] + kSamples;
	evb10_vop_check[2] = evb10_vop_check[2] + changed;
	if (!cleaned) evb10_vop_check[5] = evb10_vop_check[5] + 1;
}
