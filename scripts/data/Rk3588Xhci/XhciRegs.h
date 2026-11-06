// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// XhciRegs.h -- register offsets, TRB types and context field layouts of an xHCI host controller (xHCI specification 1.2), and the DWC3 global
// registers the probe prints. No kernel include: XhciCore.h and tests/test-xhci.cpp use it.
#pragma once
#include <stdint.h>

namespace xhci {

// ---- capability registers (offsets from the base)
constexpr uint32_t kCapLength = 0x00;          // byte 0 CAPLENGTH, bytes 2..3 HCIVERSION
constexpr uint32_t kHcsParams1 = 0x04;         // MaxSlots 7:0, MaxIntrs 18:8, MaxPorts 31:24
constexpr uint32_t kHcsParams2 = 0x08;         // IST 3:0, ERST Max 7:4, Max Scratchpad Bufs Hi 25:21, SPR 26, Lo 31:27
constexpr uint32_t kHcsParams3 = 0x0c;
constexpr uint32_t kHccParams1 = 0x10;         // AC64 0, CSZ 2 (64 byte contexts), PPC 3, xECP 31:16 (32 bit words)
constexpr uint32_t kDbOff = 0x14;
constexpr uint32_t kRtsOff = 0x18;
constexpr uint32_t kHccParams2 = 0x1c;

constexpr uint32_t kHccAc64 = 1u << 0, kHccCsz = 1u << 2, kHccPpc = 1u << 3;

// ---- operational registers (offsets from base + CAPLENGTH)
constexpr uint32_t kUsbCmd = 0x00, kUsbSts = 0x04, kPageSize = 0x08, kDnCtrl = 0x14, kCrcr = 0x18, kDcbaap = 0x30, kConfig = 0x38;
constexpr uint32_t kPortBase = 0x400;          // port register set n (n from 0): kPortBase + 0x10 * n: PORTSC, PORTPMSC, PORTLI, PORTHLPMC

constexpr uint32_t kCmdRun = 1u << 0, kCmdHcReset = 1u << 1, kCmdIntEn = 1u << 2, kCmdHsee = 1u << 3;
constexpr uint32_t kStsHalted = 1u << 0, kStsHse = 1u << 2, kStsEint = 1u << 3, kStsPcd = 1u << 4, kStsCnr = 1u << 11, kStsHce = 1u << 12;

// CRCR (64 bit): RCS 0, CS 1, CA 2, CRR 3, pointer 63:6
constexpr uint64_t kCrcrRcs = 1u << 0, kCrcrCrr = 1u << 3;

// PORTSC: CCS 0, PED 1 (write 1 disables!), OCA 3, PR 4, PLS 8:5, PP 9, speed 13:10, PIC 15:14, LWS 16, CSC 17, PEC 18, WRC 19, OCC 20, PRC 21,
// PLC 22, CEC 23, CAS 24, WCE 25, WDE 26, WOE 27, DR 30, WPR 31. Bits 17..23 are write-1-to-clear, bit 1 is write-1-to-disable, bit 4 starts a reset.
constexpr uint32_t kPortCcs = 1u << 0, kPortPed = 1u << 1, kPortOca = 1u << 3, kPortPr = 1u << 4, kPortPp = 1u << 9;
constexpr uint32_t kPortPlsShift = 5, kPortPlsMask = 0xfu << 5, kPortSpeedShift = 10, kPortSpeedMask = 0xfu << 10;
constexpr uint32_t kPortLws = 1u << 16, kPortCsc = 1u << 17, kPortPec = 1u << 18, kPortWrc = 1u << 19, kPortOcc = 1u << 20, kPortPrc = 1u << 21,
                   kPortPlc = 1u << 22, kPortCec = 1u << 23, kPortChange = 0x7fu << 17, kPortWpr = 1u << 31;
// the bits of PORTSC that a read-modify-write has to keep out of the write: the change bits (a 1 clears them), PED (a 1 disables the port), PR, LWS
constexpr uint32_t kPortPreserve = kPortChange | kPortPed | kPortPr | kPortLws | kPortWpr;
constexpr uint32_t kPlsU0 = 0, kPlsU3 = 3, kPlsDisabled = 4, kPlsRxDetect = 5, kPlsInactive = 6, kPlsPolling = 7, kPlsRecovery = 8, kPlsResume = 15;

// ---- runtime registers (offsets from base + RTSOFF); interrupter n at 0x20 + 0x20 * n
constexpr uint32_t kIman = 0x00, kImod = 0x04, kErstSz = 0x08, kErstBa = 0x10, kErdp = 0x18;
constexpr uint32_t kImanIp = 1u << 0, kImanIe = 1u << 1;
constexpr uint64_t kErdpEhb = 1u << 3;

// ---- extended capabilities
constexpr uint32_t kXcapSupportedProtocol = 2;

// ---- TRBs (16 bytes: parameter 63:0, status 95:64, control 127:96)
constexpr uint32_t kTrbCycle = 1u << 0, kTrbToggleCycle = 1u << 1, kTrbIsp = 1u << 2, kTrbChain = 1u << 4, kTrbIoc = 1u << 5, kTrbIdt = 1u << 6;
constexpr uint32_t kTrbBsr = 1u << 9, kTrbDirIn = 1u << 16;
constexpr unsigned kTrbTypeShift = 10;

enum TrbType : uint32_t {
  kTrbNormal = 1, kTrbSetupStage = 2, kTrbDataStage = 3, kTrbStatusStage = 4, kTrbIsoch = 5, kTrbLink = 6, kTrbEventData = 7, kTrbNoOp = 8,
  kCmdEnableSlot = 9, kCmdDisableSlot = 10, kCmdAddressDevice = 11, kCmdConfigureEndpoint = 12, kCmdEvaluateContext = 13, kCmdResetEndpoint = 14,
  kCmdStopEndpoint = 15, kCmdSetTrDequeue = 16, kCmdResetDevice = 17, kCmdNoOp = 23,
  kEvTransfer = 32, kEvCommandCompletion = 33, kEvPortStatusChange = 34, kEvBandwidthRequest = 35, kEvDoorbell = 36, kEvHostController = 37,
  kEvDeviceNotification = 38, kEvMfindexWrap = 39,
};

enum CompletionCode : uint32_t {
  kCcInvalid = 0, kCcSuccess = 1, kCcDataBuffer = 2, kCcBabble = 3, kCcUsbTransaction = 4, kCcTrb = 5, kCcStall = 6, kCcResource = 7, kCcBandwidth = 8,
  kCcNoSlots = 9, kCcInvalidStreamType = 10, kCcSlotNotEnabled = 11, kCcEndpointNotEnabled = 12, kCcShortPacket = 13, kCcRingUnderrun = 14,
  kCcRingOverrun = 15, kCcVfEventRingFull = 16, kCcParameter = 17, kCcBandwidthOverrun = 18, kCcContextState = 19, kCcNoPingResponse = 20,
  kCcEventRingFull = 21, kCcIncompatibleDevice = 22, kCcMissedService = 23, kCcCommandRingStopped = 24, kCcCommandAborted = 25, kCcStopped = 26,
  kCcStoppedLength = 27, kCcMaxExitLatency = 29, kCcIsochBuffer = 31, kCcEventLost = 32, kCcUndefined = 33, kCcInvalidStreamId = 34, kCcSecondaryBandwidth = 35,
  kCcSplitTransaction = 36,
};

// ---- contexts (32 or 64 bytes each, HCCPARAMS1.CSZ). Device context: slot, then EP0 (device context index 1), then endpoints 1 OUT (2), 1 IN (3), ...
// Input context: input control context, then the device context's entries one place later.
// Slot context: DW0 route string 19:0, speed 23:20, MTT 25, hub 26, context entries 31:27; DW1 max exit latency 15:0, root hub port 23:16, number of ports 31:24;
// DW2 parent hub slot 7:0, parent port 15:8, TT think time 17:16, interrupter target 31:22; DW3 device address 7:0, slot state 31:27.
// Endpoint context: DW0 EP state 2:0, mult 9:8, max primary streams 14:10, interval 23:16; DW1 CErr 2:1, EP type 5:3, max burst 15:8, max packet size 31:16;
// DW2 DCS 0, TR dequeue pointer 63:4 (DW2 and DW3); DW4 average TRB length 15:0, max ESIT payload 31:16.
constexpr unsigned kSlotSpeedShift = 20, kSlotEntriesShift = 27, kSlotRootPortShift = 16, kSlotNumPortsShift = 24;
constexpr uint32_t kSlotMtt = 1u << 25, kSlotHub = 1u << 26;
enum EpType : uint32_t {
  kEpIsochOut = 1, kEpBulkOut = 2, kEpInterruptOut = 3, kEpControl = 4, kEpIsochIn = 5, kEpBulkIn = 6, kEpInterruptIn = 7,
};
enum SlotSpeed : uint32_t { kSpeedFull = 1, kSpeedLow = 2, kSpeedHigh = 3, kSpeedSuper = 4, kSpeedSuperPlus = 5 };
enum SlotStateCode : uint32_t { kSlotDisabled = 0, kSlotDefault = 1, kSlotAddressed = 2, kSlotConfigured = 3 };
enum EpState : uint32_t { kEpDisabled = 0, kEpRunning = 1, kEpHalted = 2, kEpStopped = 3, kEpError = 4 };

// ---- DWC3 global registers (offsets from the base of the block; Synopsys DWC_usb3 databook; the probe prints them)
constexpr uint32_t kDwc3Gctl = 0xc110, kDwc3Gsts = 0xc118, kDwc3Gsnpsid = 0xc120, kDwc3Ghwparams0 = 0xc140, kDwc3Gusb2PhyCfg = 0xc200,
                   kDwc3Gusb3PipeCtl = 0xc2c0, kDwc3Gfladj = 0xc630, kDwc3Dcfg = 0xc700, kDwc3Dsts = 0xc70c;

}  // namespace xhci
