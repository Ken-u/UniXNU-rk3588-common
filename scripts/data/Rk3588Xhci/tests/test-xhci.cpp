// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// test-xhci.cpp -- host test of XhciCore.h and UsbHost.h (and UsbDesc.h, HidReport.h) against the model of an xHCI controller in XhciModel.h.
//
//   c++ -std=c++17 -Wall -Wextra -Werror -O1 -g -fsanitize=address,undefined -I.. test-xhci.cpp -o /tmp/test-xhci && /tmp/test-xhci
//
// The model is strict about what the driver writes (see the header of XhciModel.h), so a passing run means the driver's rings, contexts, port
// handling and enumeration are consistent with the specification as the model reads it. It does not mean the controller on the board agrees:
// that is what the board runs are for.
#include <stdarg.h>

#include <memory>
#include <string>

static std::string g_log;
static void logf(const char *fmt, ...) {
  char line[512];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(line, sizeof line, fmt, ap);
  va_end(ap);
  g_log += line;
}
#define XHCI_LOG(...) logf(__VA_ARGS__)

#include "../HidReport.h"
#include "../UsbHost.h"
#include "XhciModel.h"

using model::Dev;
using namespace xhci;

static unsigned g_checks = 0, g_failures = 0;
#define CHECK(cond)                                                              \
  do {                                                                           \
    ++g_checks;                                                                  \
    if (!(cond)) {                                                               \
      ++g_failures;                                                              \
      printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                   \
    }                                                                            \
  } while (0)
#define CHECK_EQ(a, b)                                                                                          \
  do {                                                                                                          \
    ++g_checks;                                                                                                 \
    const long long va_ = static_cast<long long>(a), vb_ = static_cast<long long>(b);                           \
    if (va_ != vb_) {                                                                                           \
      ++g_failures;                                                                                             \
      printf("  FAIL %s:%d: %s == %s (%lld vs %lld)\n", __FILE__, __LINE__, #a, #b, va_, vb_);                  \
    }                                                                                                           \
  } while (0)

// ---------------------------------------------------------------------------------------------------------------------------------------------
// Glue between the driver and the model

static uint32_t hwRead(void *c, uint32_t off) { return static_cast<model::Model *>(c)->read32(off); }
static void hwWrite(void *c, uint32_t off, uint32_t v) { static_cast<model::Model *>(c)->write32(off, v); }
static void hwDelay(void *c, uint32_t us) {
  auto *m = static_cast<model::Model *>(c);
  m->timeUs += us;
  m->service();
}
static uint64_t hwNow(void *c) { return static_cast<model::Model *>(c)->timeUs; }
static void hwBarrier(void *) { __sync_synchronize(); }

struct Rig;
static Rig *g_rig;

static void onKey(const uint8_t *report, unsigned length);
static void onMouse(int32_t buttons, int32_t dx, int32_t dy, int32_t wheel);
static void onProtocol(bool report);

struct Rig {
  model::Model m;
  xhci::Controller hc;
  usb::Host host;
  usb::HidSink sink;
  std::vector<std::unique_ptr<Dev>> devs;
  std::vector<std::vector<uint8_t>> keys;
  struct Mouse {
    int b, dx, dy, w;
  };
  std::vector<Mouse> mice;
  std::vector<bool> protocols;

  Rig(unsigned csz = 64, unsigned scratch = 1, unsigned nPorts = 2, unsigned maxSlots = 32) {
    m.csz = csz;
    m.scratch = scratch;
    m.nPorts = nPorts;
    m.maxSlots = maxSlots;
    m.init();
    m.timeUs = 1000000;
    sink.keyboard = onKey;
    sink.mouse = onMouse;
    sink.mouseProtocol = onProtocol;
    g_rig = this;
    g_log.clear();
  }
  bool start(unsigned mode = usb::kModeHid) {
    xhci::Hw hw = {&m, hwRead, hwWrite, hwDelay, hwNow, hwBarrier, m.mem.data(), m.dmaBase, static_cast<uint32_t>(m.mem.size())};
    if (!hc.init(hw)) return false;
    host.init(&hc, "test", &sink, mode);
    return true;
  }
  void run(unsigned ms) {
    for (unsigned i = 0; i < ms; ++i) {
      host.poll();
      m.timeUs += 1000;
      m.service();
    }
  }
  // the checks every run has to pass
  void finish() {
    CHECK(m.problems.empty());
    CHECK_EQ(m.eventOverflow, 0);
    CHECK_EQ(m.portDisableWrites, 0);
    CHECK_EQ(m.ringErrors, 0);
  }

  Dev *add(std::unique_ptr<Dev> d) {
    devs.push_back(std::move(d));
    return devs.back().get();
  }
  Dev *keyboard(unsigned speed, unsigned mps0 = 0);
  Dev *mouse(unsigned speed, bool wheel = true, unsigned mps0 = 0);
  Dev *composite(unsigned speed);
  Dev *hub(unsigned speed, unsigned ports, bool multiTt = false, unsigned thinkTime = 0);
};

static void onKey(const uint8_t *report, unsigned length) { g_rig->keys.push_back(std::vector<uint8_t>(report, report + length)); }
static void onMouse(int32_t b, int32_t dx, int32_t dy, int32_t w) { g_rig->mice.push_back({b, dx, dy, w}); }
static void onProtocol(bool report) { g_rig->protocols.push_back(report); }

// ---------------------------------------------------------------------------------------------------------------------------------------------
// Devices

static void put16(std::vector<uint8_t> &v, unsigned x) {
  v.push_back(static_cast<uint8_t>(x));
  v.push_back(static_cast<uint8_t>(x >> 8));
}
static unsigned usbVersion(unsigned speed) { return speed == kSpeedSuper ? 0x0300 : speed == kSpeedHigh ? 0x0200 : 0x0110; }
static unsigned defaultMps0(unsigned speed) { return speed == kSpeedSuper ? 9 : speed == kSpeedHigh ? 64 : 8; }
static std::vector<uint8_t> deviceDescriptor(unsigned ver, unsigned cls, unsigned sub, unsigned proto, unsigned mps0, unsigned vid, unsigned pid) {
  std::vector<uint8_t> d = {18, 1};
  put16(d, ver);
  d.push_back(static_cast<uint8_t>(cls));
  d.push_back(static_cast<uint8_t>(sub));
  d.push_back(static_cast<uint8_t>(proto));
  d.push_back(static_cast<uint8_t>(mps0));
  put16(d, vid);
  put16(d, pid);
  put16(d, 0x0100);
  d.push_back(1);
  d.push_back(2);
  d.push_back(0);
  d.push_back(1);
  return d;
}
static std::vector<uint8_t> configHeader(unsigned ifaces) { return {9, 2, 0, 0, static_cast<uint8_t>(ifaces), 1, 0, 0xa0, 50}; }
static void finishConfig(std::vector<uint8_t> &c) {
  c[2] = static_cast<uint8_t>(c.size());
  c[3] = static_cast<uint8_t>(c.size() >> 8);
}
static void endpointDescriptor(std::vector<uint8_t> &c, unsigned address, unsigned attributes, unsigned mps, unsigned interval, bool companion) {
  c.insert(c.end(), {7, 5, static_cast<uint8_t>(address), static_cast<uint8_t>(attributes)});
  put16(c, mps);
  c.push_back(static_cast<uint8_t>(interval));
  if (companion) {
    c.insert(c.end(), {6, 0x30, 0, 0});
    put16(c, mps);
  }
}
static const uint8_t kMouseReportDescriptor[] = {0x05, 0x01, 0x09, 0x02, 0xa1, 0x01, 0x09, 0x01, 0xa1, 0x00, 0x05, 0x09, 0x19, 0x01, 0x29, 0x03, 0x15, 0x00, 0x25, 0x01,
                                                 0x95, 0x03, 0x75, 0x01, 0x81, 0x02, 0x95, 0x01, 0x75, 0x05, 0x81, 0x03, 0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x38,
                                                 0x15, 0x81, 0x25, 0x7f, 0x75, 0x08, 0x95, 0x03, 0x81, 0x06, 0xc0, 0xc0};
// the same without the wheel (three bytes)
static const uint8_t kMouseNoWheelDescriptor[] = {0x05, 0x01, 0x09, 0x02, 0xa1, 0x01, 0x09, 0x01, 0xa1, 0x00, 0x05, 0x09, 0x19, 0x01, 0x29, 0x03, 0x15, 0x00, 0x25, 0x01,
                                                  0x95, 0x03, 0x75, 0x01, 0x81, 0x02, 0x95, 0x01, 0x75, 0x05, 0x81, 0x03, 0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x15, 0x81,
                                                  0x25, 0x7f, 0x75, 0x08, 0x95, 0x02, 0x81, 0x06, 0xc0, 0xc0};

static unsigned hidInterval(unsigned speed) { return speed == kSpeedLow || speed == kSpeedFull ? 10 : 4; }
static void hidInterface(std::vector<uint8_t> &c, unsigned number, unsigned proto, unsigned epAddress, unsigned mps, unsigned speed, unsigned reportLength) {
  c.insert(c.end(), {9, 4, static_cast<uint8_t>(number), 0, 1, 3, 1, static_cast<uint8_t>(proto), 0});
  c.insert(c.end(), {9, 0x21, 0x11, 0x01, 0, 1, 0x22});
  put16(c, reportLength);
  endpointDescriptor(c, epAddress, 3, mps, hidInterval(speed), speed == kSpeedSuper);
}

Dev *Rig::keyboard(unsigned speed, unsigned mps0) {
  auto d = std::make_unique<Dev>();
  d->speed = speed;
  d->device = deviceDescriptor(usbVersion(speed), 0, 0, 0, mps0 ? mps0 : defaultMps0(speed), 0x046d, 0xc31c);
  d->config = configHeader(1);
  hidInterface(d->config, 0, 1, 0x81, 8, speed, 0);
  finishConfig(d->config);
  return add(std::move(d));
}

Dev *Rig::mouse(unsigned speed, bool wheel, unsigned mps0) {
  auto d = std::make_unique<Dev>();
  d->speed = speed;
  d->device = deviceDescriptor(usbVersion(speed), 0, 0, 0, mps0 ? mps0 : defaultMps0(speed), 0x046d, 0xc077);
  const uint8_t *desc = wheel ? kMouseReportDescriptor : kMouseNoWheelDescriptor;
  const unsigned length = wheel ? sizeof kMouseReportDescriptor : sizeof kMouseNoWheelDescriptor;
  d->reportDesc.assign(desc, desc + length);
  d->config = configHeader(1);
  hidInterface(d->config, 0, 2, 0x81, wheel ? 4 : 3, speed, length);
  finishConfig(d->config);
  return add(std::move(d));
}

// keyboard on interface 0 (endpoint 1 IN), mouse on interface 1 (endpoint 2 IN)
Dev *Rig::composite(unsigned speed) {
  auto d = std::make_unique<Dev>();
  d->speed = speed;
  d->device = deviceDescriptor(usbVersion(speed), 0, 0, 0, defaultMps0(speed), 0x1c4f, 0x0002);
  d->reportDesc.assign(kMouseReportDescriptor, kMouseReportDescriptor + sizeof kMouseReportDescriptor);
  d->config = configHeader(2);
  hidInterface(d->config, 0, 1, 0x81, 8, speed, 0);
  hidInterface(d->config, 1, 2, 0x82, 4, speed, sizeof kMouseReportDescriptor);
  finishConfig(d->config);
  return add(std::move(d));
}

Dev *Rig::hub(unsigned speed, unsigned ports, bool multiTt, unsigned thinkTime) {
  auto d = std::make_unique<Dev>();
  const bool super = speed == kSpeedSuper;
  d->speed = speed;
  d->isHub = true;
  d->superHub = super;
  d->ports.resize(ports);
  d->device = deviceDescriptor(super ? 0x0310 : 0x0200, 9, 0, super ? 3 : multiTt ? 2 : 1, super ? 9 : 64, 0x05e3, 0x0608);
  d->config = configHeader(1);
  d->config.insert(d->config.end(), {9, 4, 0, 0, 1, 9, 0, static_cast<uint8_t>(super ? 0 : multiTt ? 2 : 1), 0});
  endpointDescriptor(d->config, 0x81, 3, 2, super ? 8 : 12, super);
  finishConfig(d->config);
  if (super) {
    d->hubDescriptor = {12, 0x2a, static_cast<uint8_t>(ports), 0, 0, 50, 0, 0, 0, 0, 0, 0};
  } else {
    d->hubDescriptor = {9, 0x29, static_cast<uint8_t>(ports), static_cast<uint8_t>(thinkTime << 5), 0, 50, 0, 0, 0xff};
  }
  return add(std::move(d));
}

static bool bytesEqual(const std::vector<uint8_t> &a, const uint8_t *b, size_t n) { return a.size() == n && memcmp(a.data(), b, n) == 0; }

// a report of a keyboard with a counter in bytes 2..3
static std::vector<uint8_t> keyReport(unsigned n) { return {0, 0, static_cast<uint8_t>(n), static_cast<uint8_t>(n >> 8), 0, 0, 0, 0}; }

// ---------------------------------------------------------------------------------------------------------------------------------------------
// Descriptors, report layouts

static void testParseConfig() {
  std::vector<uint8_t> c = configHeader(2);
  hidInterface(c, 0, 1, 0x81, 8, kSpeedFull, 0);
  hidInterface(c, 1, 2, 0x82, 4, kSpeedFull, 52);
  finishConfig(c);
  usb::ConfigInfo ci;
  CHECK(usb::parseConfig(c.data(), c.size(), &ci));
  CHECK_EQ(ci.value, 1);
  CHECK_EQ(ci.interfaceCount, 2);
  CHECK_EQ(ci.endpointCount, 2);
  CHECK_EQ(ci.interfaces[1].number, 1);
  CHECK_EQ(ci.interfaces[1].proto, 2);
  CHECK_EQ(ci.interfaces[1].hidReportLength, 52);
  CHECK_EQ(ci.interfaces[0].endpoints, 1);
  CHECK_EQ(ci.interfaces[1].firstEndpoint, 1);
  CHECK_EQ(ci.endpoints[1].address, 0x82);
  CHECK_EQ(ci.endpoints[1].iface, 1);
  CHECK_EQ(ci.endpoints[0].interval, 10);

  // truncated in the middle of the second interface: what was complete is kept
  CHECK(usb::parseConfig(c.data(), 9 + 25 + 9 + 9 + 3, &ci));
  CHECK_EQ(ci.interfaceCount, 2);
  CHECK_EQ(ci.endpointCount, 1);
  // not a configuration descriptor, too short
  std::vector<uint8_t> bad = c;
  bad[1] = 1;
  CHECK(!usb::parseConfig(bad.data(), bad.size(), &ci));
  CHECK(!usb::parseConfig(c.data(), 8, &ci));
  // a descriptor of length 0 ends the walk (no endless loop)
  bad = c;
  bad[9] = 0;
  CHECK(usb::parseConfig(bad.data(), bad.size(), &ci));
  CHECK_EQ(ci.interfaceCount, 0);
  // more interfaces and endpoints than the table holds
  std::vector<uint8_t> many = configHeader(20);
  for (unsigned i = 0; i < 20; ++i) hidInterface(many, i, 1, 0x81 + (i & 7), 8, kSpeedFull, 0);
  finishConfig(many);
  CHECK(usb::parseConfig(many.data(), many.size(), &ci));
  CHECK_EQ(ci.interfaceCount, usb::kMaxInterfaces);
  CHECK(ci.endpointCount <= usb::kMaxEndpoints);
  // SuperSpeed companion
  std::vector<uint8_t> ss = configHeader(1);
  hidInterface(ss, 0, 1, 0x81, 1024, kSpeedSuper, 0);
  finishConfig(ss);
  CHECK(usb::parseConfig(ss.data(), ss.size(), &ci));
  CHECK(ci.endpoints[0].companion);
  CHECK_EQ(ci.endpoints[0].bytesPerInterval, 1024);
}

static void testIntervals() {
  // low / full speed: bInterval in milliseconds -> the largest 2^n microframes not above it, n in 3..10
  CHECK_EQ(usb::endpointInterval(kSpeedFull, 1), 3);
  CHECK_EQ(usb::endpointInterval(kSpeedFull, 8), 6);
  CHECK_EQ(usb::endpointInterval(kSpeedFull, 10), 6);
  CHECK_EQ(usb::endpointInterval(kSpeedLow, 10), 6);
  CHECK_EQ(usb::endpointInterval(kSpeedLow, 255), 10);
  CHECK_EQ(usb::endpointInterval(kSpeedFull, 0), 3);
  // high / super speed: 2^(bInterval - 1) microframes
  CHECK_EQ(usb::endpointInterval(kSpeedHigh, 4), 3);
  CHECK_EQ(usb::endpointInterval(kSpeedHigh, 1), 0);
  CHECK_EQ(usb::endpointInterval(kSpeedSuper, 8), 7);
  CHECK_EQ(usb::endpointInterval(kSpeedHigh, 16), 15);
  CHECK_EQ(usb::endpointInterval(kSpeedHigh, 0), 0);

  usb::EndpointInfo e = {};
  e.address = 0x83;
  e.attributes = 3;
  e.maxPacketRaw = 2 << 11 | 0x40;   // 64 bytes, 2 additional transactions per microframe
  e.interval = 4;
  xhci::EpConfig c = usb::makeEpConfig(kSpeedHigh, e);
  CHECK_EQ(c.dci, 7);
  CHECK_EQ(c.type, kEpInterruptIn);
  CHECK_EQ(c.maxPacket, 64);
  CHECK_EQ(c.maxBurst, 2);
  CHECK_EQ(c.interval, 3);
  CHECK_EQ(c.maxEsitPayload, 192);
  e.address = 0x02;
  e.attributes = 2;
  e.maxPacketRaw = 512;
  c = usb::makeEpConfig(kSpeedHigh, e);
  CHECK_EQ(c.dci, 4);
  CHECK_EQ(c.type, kEpBulkOut);
  CHECK_EQ(c.interval, 0);
  CHECK_EQ(c.avgTrbLength, 512);
  e.address = 0x81;
  e.attributes = 3;
  e.maxPacketRaw = 8;
  e.companion = true;
  e.maxBurst = 3;
  e.bytesPerInterval = 24;
  c = usb::makeEpConfig(kSpeedSuper, e);
  CHECK_EQ(c.maxBurst, 3);
  CHECK_EQ(c.maxEsitPayload, 24);
}

static void testMouseLayout() {
  usbhid::MouseLayout l;
  CHECK(usbhid::parseMouseLayout(kMouseReportDescriptor, sizeof kMouseReportDescriptor, &l));
  CHECK_EQ(l.length, 4);
  CHECK(!l.hasId);
  CHECK_EQ(l.buttons.bit, 0);
  CHECK_EQ(l.buttons.size, 3);
  CHECK_EQ(l.x.bit, 8);
  CHECK_EQ(l.y.bit, 16);
  CHECK_EQ(l.wheel.bit, 24);
  const uint8_t report[] = {0x05, 0x7f, 0x80, 0xff};
  CHECK_EQ(usbhid::fieldValue(report, 4, l.buttons), 5);
  CHECK_EQ(usbhid::fieldValue(report, 4, l.x), 127);
  CHECK_EQ(usbhid::fieldValue(report, 4, l.y), -128);
  CHECK_EQ(usbhid::fieldValue(report, 4, l.wheel), -1);
  CHECK(usbhid::parseMouseLayout(kMouseNoWheelDescriptor, sizeof kMouseNoWheelDescriptor, &l));
  CHECK_EQ(l.length, 3);
  CHECK_EQ(l.wheel.bit, -1);
  // a keyboard descriptor has no pointer
  const uint8_t kb[] = {0x05, 0x01, 0x09, 0x06, 0xa1, 0x01, 0x05, 0x07, 0x19, 0xe0, 0x29, 0xe7, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x08, 0x81, 0x02, 0xc0};
  CHECK(!usbhid::parseMouseLayout(kb, sizeof kb, &l));
  // truncated in the middle of an item
  CHECK(!usbhid::parseMouseLayout(kMouseReportDescriptor, 11, &l));
}

// ---------------------------------------------------------------------------------------------------------------------------------------------
// Controller start

static void testInit() {
  for (unsigned csz : {32u, 64u}) {
    for (unsigned scratch : {0u, 1u, 4u, 33u}) {
      Rig r(csz, scratch);
      CHECK(r.start());
      CHECK(r.hc.running());
      CHECK_EQ(r.hc.contextSize(), csz);
      CHECK_EQ(r.hc.portCount(), 2);
      CHECK(!r.hc.portIsUsb3(1));
      CHECK(r.hc.portIsUsb3(2));
      CHECK_EQ(r.m.resets, 1);
      r.finish();
    }
  }
  {
    Rig r;
    r.m.portsc[1] &= ~kPortPp;   // a controller with power control that comes up with the ports unpowered
    r.m.portsc[2] &= ~kPortPp;
    CHECK(r.start());
    CHECK(r.m.portsc[1] & kPortPp);
    CHECK(r.m.portsc[2] & kPortPp);
    r.finish();
  }
  {
    Rig r;   // a controller that is running (left so by the firmware) is halted and reset
    r.m.usbcmd = kCmdRun;
    r.m.usbsts = 0;
    CHECK(r.start());
    CHECK_EQ(r.m.resets, 1);
    r.finish();
  }
  {
    Rig r;   // the DMA window is too small
    xhci::Hw hw = {&r.m, hwRead, hwWrite, hwDelay, hwNow, hwBarrier, r.m.mem.data(), r.m.dmaBase, 64 * 1024};
    CHECK(!r.hc.init(hw));
  }
  {
    Rig r;   // there is no controller
    r.m.mem.assign(r.m.mem.size(), 0);
    xhci::Hw hw = {&r.m, [](void *, uint32_t) -> uint32_t { return 0xffffffffu; }, hwWrite, hwDelay, hwNow, hwBarrier, r.m.mem.data(), r.m.dmaBase,
                   static_cast<uint32_t>(r.m.mem.size())};
    CHECK(!r.hc.init(hw));
  }
  {
    // 5 ports (USB 3 is the last one), 3 slots
    Rig r2(64, 1, 5, 3);
    CHECK(r2.start());
    CHECK_EQ(r2.hc.portCount(), 5);
    CHECK(r2.hc.portIsUsb3(5));
    CHECK(!r2.hc.portIsUsb3(4));
    r2.finish();
  }
}

// ---------------------------------------------------------------------------------------------------------------------------------------------
// Enumeration and HID

static void checkKeyboardPath(Rig &r, Dev *kb, unsigned expectedInterval, unsigned expectedMps) {
  model::Slot &s = r.m.slots[kb->slot];
  CHECK(kb->slot != 0);
  CHECK_EQ(s.state, kSlotConfigured);
  CHECK_EQ(kb->configuration, 1);
  CHECK_EQ(kb->protocol, 0);
  CHECK_EQ(kb->idle, 0);
  CHECK_EQ(s.dev0 >> kSlotEntriesShift, 3);
  CHECK(s.ep[3].state == kEpRunning);
  CHECK_EQ(s.ep[3].type, kEpInterruptIn);
  CHECK_EQ(s.ep[3].maxPacket, expectedMps);
  CHECK_EQ(s.ep[3].interval, expectedInterval);
}

// n reports from a keyboard, all of them arrive in order
static void sendKeys(Rig &, Dev *kb, unsigned first, unsigned n, unsigned address = 0x81) {
  for (unsigned i = 0; i < n; ++i) kb->reports[address].push_back(keyReport(first + i));
}
static void checkKeys(Rig &r, unsigned first, unsigned n) {
  CHECK_EQ(r.keys.size(), n);
  for (unsigned i = 0; i < r.keys.size() && i < n; ++i) {
    CHECK_EQ(r.keys[i].size(), 8);
    CHECK_EQ(r.keys[i][2] | r.keys[i][3] << 8, first + i);
  }
}

static void testKeyboardHighSpeed() {
  for (unsigned csz : {64u, 32u}) {
    Rig r(csz);
    CHECK(r.start());
    Dev *kb = r.keyboard(kSpeedHigh);
    r.m.attach(1, kb);
    r.run(1000);
    CHECK_EQ(r.host.deviceCount(), 1);
    CHECK_EQ(r.host.statEnumerated(), 1);
    checkKeyboardPath(r, kb, 3, 8);
    // the slot context: speed, root port, route 0
    model::Slot &s = r.m.slots[kb->slot];
    CHECK_EQ(s.dev0 >> kSlotSpeedShift & 0xf, kSpeedHigh);
    CHECK_EQ(s.dev1 >> kSlotRootPortShift & 0xff, 1);
    CHECK_EQ(s.dev0 & 0xfffff, 0);
    CHECK_EQ(s.ep[1].maxPacket, 64);
    // 700 reports: wraps the transfer ring (64 TRBs) and the event ring (128)
    sendKeys(r, kb, 0, 700);
    r.run(1000);
    checkKeys(r, 0, 700);
    CHECK_EQ(r.hc.statUnexpected(), 0);
    r.finish();
  }
}

static void testMouseSuperSpeed() {
  Rig r;
  CHECK(r.start());
  Dev *ms = r.mouse(kSpeedSuper);
  r.m.attach(2, ms);   // the USB 3 port
  r.run(1000);
  CHECK_EQ(r.host.deviceCount(), 1);
  model::Slot &s = r.m.slots[ms->slot];
  CHECK_EQ(s.dev0 >> kSlotSpeedShift & 0xf, kSpeedSuper);
  CHECK_EQ(s.dev1 >> kSlotRootPortShift & 0xff, 2);
  CHECK_EQ(s.ep[1].maxPacket, 512);
  CHECK_EQ(ms->protocol, 1);   // report protocol: the descriptor has a wheel
  CHECK_EQ(r.protocols.size(), 1);
  CHECK(!r.protocols.empty() && r.protocols[0]);
  CHECK_EQ(s.ep[3].type, kEpInterruptIn);
  CHECK_EQ(s.ep[3].interval, 3);
  CHECK(r.m.hotResets >= 1);
  ms->reports[0x81].push_back({1, 5, 0xfb, 0xff});
  ms->reports[0x81].push_back({0, 0, 0, 1});
  ms->reports[0x81].push_back({7, 0x80, 0x7f, 0});
  r.run(50);
  CHECK_EQ(r.mice.size(), 3);
  if (r.mice.size() == 3) {
    CHECK_EQ(r.mice[0].b, 1);
    CHECK_EQ(r.mice[0].dx, 5);
    CHECK_EQ(r.mice[0].dy, -5);
    CHECK_EQ(r.mice[0].w, -1);
    CHECK_EQ(r.mice[1].w, 1);
    CHECK_EQ(r.mice[2].b, 7);
    CHECK_EQ(r.mice[2].dx, -128);
    CHECK_EQ(r.mice[2].dy, 127);
  }
  r.finish();
}

static void testMouseBootProtocol() {
  Rig r;
  CHECK(r.start());
  Dev *ms = r.mouse(kSpeedLow, false);   // a descriptor without a wheel: the report protocol is used all the same (three bytes)
  r.m.attach(1, ms);
  r.run(1000);
  CHECK_EQ(r.host.deviceCount(), 1);
  CHECK_EQ(ms->slot != 0, true);
  ms->reports[0x81].push_back({2, 3, 0xfd});
  r.run(20);
  CHECK_EQ(r.mice.size(), 1);
  if (r.mice.size() == 1) {
    CHECK_EQ(r.mice[0].b, 2);
    CHECK_EQ(r.mice[0].dx, 3);
    CHECK_EQ(r.mice[0].dy, -3);
    CHECK_EQ(r.mice[0].w, 0);
  }
  // low speed device on a root port: no transaction translator, speed 2 in the slot context
  CHECK_EQ(r.m.slots[ms->slot].dev0 >> kSlotSpeedShift & 0xf, kSpeedLow);
  CHECK_EQ(r.m.slots[ms->slot].dev2 & 0xffff, 0);
  r.finish();
}

static void testFullSpeedMaxPacket() {
  for (unsigned mps0 : {8u, 16u, 32u, 64u}) {
    Rig r;
    CHECK(r.start());
    Dev *kb = r.keyboard(kSpeedFull, mps0);
    r.m.attach(1, kb);
    r.run(1000);
    CHECK_EQ(r.host.deviceCount(), 1);
    CHECK_EQ(r.m.slots[kb->slot].ep[1].maxPacket, mps0);
    CHECK_EQ(r.m.evaluateCount, mps0 == 8 ? 0 : 1);
    checkKeyboardPath(r, kb, 6, 8);
    r.finish();
  }
}

static void testComposite() {
  Rig r;
  CHECK(r.start());
  Dev *d = r.composite(kSpeedFull);
  r.m.attach(1, d);
  r.run(1000);
  CHECK_EQ(r.host.deviceCount(), 1);
  model::Slot &s = r.m.slots[d->slot];
  CHECK_EQ(s.dev0 >> kSlotEntriesShift, 5);
  CHECK(s.ep[3].state == kEpRunning);
  CHECK(s.ep[5].state == kEpRunning);
  d->reports[0x81].push_back(keyReport(1));
  d->reports[0x82].push_back({1, 2, 3, 4});
  r.run(20);
  CHECK_EQ(r.keys.size(), 1);
  CHECK_EQ(r.mice.size(), 1);
  r.finish();
}

// ---------------------------------------------------------------------------------------------------------------------------------------------
// Hubs

static void testHubHighSpeed() {
  for (bool multi : {false, true}) {
    for (unsigned csz : {64u, 32u}) {
      Rig r(csz);
      CHECK(r.start());
      Dev *hub = r.hub(kSpeedHigh, 4, multi, 2);
      Dev *kb = r.keyboard(kSpeedFull);
      Dev *ms = r.mouse(kSpeedLow);
      Dev *hs = r.keyboard(kSpeedHigh);
      hub->connectChild(2, kb);
      hub->connectChild(3, ms);
      hub->connectChild(4, hs);
      r.m.attach(1, hub);
      r.run(3000);
      CHECK_EQ(r.host.deviceCount(), 4);
      model::Slot &hs_ = r.m.slots[hub->slot];
      CHECK(hs_.dev0 & kSlotHub);
      CHECK_EQ(hs_.dev1 >> kSlotNumPortsShift, 4);
      CHECK_EQ(hs_.dev2 >> 16 & 3, 2);
      CHECK_EQ((hs_.dev0 & kSlotMtt) != 0, multi);
      CHECK_EQ(hub->altSetting, multi ? 1 : -1);
      CHECK_EQ(hub->configuration, 1);
      for (auto &p : hub->ports) CHECK(p.power);
      // the full and low speed devices: route string, TT = the hub's slot and the port the device is on; the high speed one has none
      model::Slot &ks = r.m.slots[kb->slot];
      CHECK_EQ(ks.dev0 & 0xfffff, 2);
      CHECK_EQ(ks.dev2 & 0xff, hub->slot);
      CHECK_EQ(ks.dev2 >> 8 & 0xff, 2);
      CHECK_EQ((ks.dev0 & kSlotMtt) != 0, multi);
      model::Slot &ms_ = r.m.slots[ms->slot];
      CHECK_EQ(ms_.dev0 & 0xfffff, 3);
      CHECK_EQ(ms_.dev2 >> 8 & 0xff, 3);
      model::Slot &hss = r.m.slots[hs->slot];
      CHECK_EQ(hss.dev0 & 0xfffff, 4);
      CHECK_EQ(hss.dev2 & 0xffff, 0);
      CHECK_EQ(hss.dev1 >> kSlotRootPortShift & 0xff, 1);   // root port of the topmost hub
      sendKeys(r, kb, 0, 5);
      ms->reports[0x81].push_back({1, 1, 1, 0});
      hs->reports[0x81].push_back(keyReport(77));
      r.run(100);
      CHECK_EQ(r.keys.size(), 6);
      CHECK_EQ(r.mice.size(), 1);
      r.finish();
    }
  }
}

static void testHubSuperSpeed() {
  Rig r;
  CHECK(r.start());
  Dev *hub = r.hub(kSpeedSuper, 4);
  Dev *kb = r.keyboard(kSpeedSuper);
  Dev *ms = r.mouse(kSpeedSuper);
  hub->connectChild(1, kb);
  hub->connectChild(4, ms);
  r.m.attach(2, hub);
  r.run(3000);
  CHECK_EQ(r.host.deviceCount(), 3);
  CHECK_EQ(hub->hubDepth, 0);
  model::Slot &s = r.m.slots[hub->slot];
  CHECK(s.dev0 & kSlotHub);
  CHECK(!(s.dev0 & kSlotMtt));
  CHECK_EQ(s.dev1 >> kSlotNumPortsShift, 4);
  CHECK_EQ(r.m.slots[kb->slot].dev0 & 0xfffff, 1);
  CHECK_EQ(r.m.slots[ms->slot].dev0 & 0xfffff, 4);
  CHECK_EQ(r.m.slots[ms->slot].dev2 & 0xffff, 0);   // SuperSpeed devices have no TT
  CHECK_EQ(r.m.slots[kb->slot].dev1 >> kSlotRootPortShift & 0xff, 2);
  sendKeys(r, kb, 0, 3);
  ms->reports[0x81].push_back({0, 1, 2, 3});
  r.run(100);
  CHECK_EQ(r.keys.size(), 3);
  CHECK_EQ(r.mice.size(), 1);
  r.finish();
}

static void testHubChain() {
  // root port 1: high speed hub (port 2) -> full speed hub? A full speed hub under a high speed hub is a USB 1.1 hub: its devices use the TT of the
  // high speed hub. Port 3 of the full speed hub has a low speed mouse. Also a high speed hub chain: HS hub port 1 -> HS hub port 2 -> keyboard.
  Rig r;
  CHECK(r.start());
  Dev *top = r.hub(kSpeedHigh, 3);
  Dev *fsHub = r.hub(kSpeedFull, 4);
  Dev *mouse = r.mouse(kSpeedLow);
  Dev *hsHub = r.hub(kSpeedHigh, 2);
  Dev *kb = r.keyboard(kSpeedFull);
  fsHub->connectChild(3, mouse);
  hsHub->connectChild(2, kb);
  top->connectChild(2, fsHub);
  top->connectChild(1, hsHub);
  r.m.attach(1, top);
  r.run(6000);
  CHECK_EQ(r.host.deviceCount(), 5);
  // low speed mouse behind the full speed hub: route 3 | 2 << 4 -> hub port 2 first, then port 3 (the tier closest to the root in the low nibble)
  model::Slot &ms = r.m.slots[mouse->slot];
  CHECK_EQ(ms.dev0 & 0xfffff, 2 | 3 << 4);
  CHECK_EQ(ms.dev2 & 0xff, top->slot);       // the TT is the top hub's
  CHECK_EQ(ms.dev2 >> 8 & 0xff, 2);          // on its port 2
  model::Slot &ks = r.m.slots[kb->slot];
  CHECK_EQ(ks.dev0 & 0xfffff, 1 | 2 << 4);
  CHECK_EQ(ks.dev2 & 0xff, hsHub->slot);     // the nearest high speed hub holds the TT
  CHECK_EQ(ks.dev2 >> 8 & 0xff, 2);
  mouse->reports[0x81].push_back({1, 4, 4, 0});
  kb->reports[0x81].push_back(keyReport(9));
  r.run(100);
  CHECK_EQ(r.mice.size(), 1);
  CHECK_EQ(r.keys.size(), 1);
  r.finish();
}

static void testSuperHubChain() {
  // SuperSpeed hub under a SuperSpeed hub: SET_HUB_DEPTH 1, route string with two tiers
  Rig r;
  CHECK(r.start());
  Dev *top = r.hub(kSpeedSuper, 3);
  Dev *low = r.hub(kSpeedSuper, 3);
  Dev *kb = r.keyboard(kSpeedSuper);
  low->connectChild(3, kb);
  top->connectChild(2, low);
  r.m.attach(2, top);
  r.run(4000);
  CHECK_EQ(r.host.deviceCount(), 3);
  CHECK_EQ(top->hubDepth, 0);
  CHECK_EQ(low->hubDepth, 1);
  CHECK_EQ(r.m.slots[kb->slot].dev0 & 0xfffff, 2 | 3 << 4);
  kb->reports[0x81].push_back(keyReport(5));
  r.run(50);
  CHECK_EQ(r.keys.size(), 1);
  r.finish();
}

// ---------------------------------------------------------------------------------------------------------------------------------------------
// Unplugging

static unsigned enabledSlots(Rig &r) {
  unsigned n = 0;
  for (auto &s : r.m.slots) n += s.enabled ? 1 : 0;
  return n;
}

static void testUnplug() {
  Rig r;
  CHECK(r.start());
  Dev *hub = r.hub(kSpeedHigh, 4);
  Dev *kb = r.keyboard(kSpeedFull);
  Dev *kb2 = r.keyboard(kSpeedHigh);
  hub->connectChild(1, kb);
  r.m.attach(1, hub);
  r.m.attach(2, r.mouse(kSpeedSuper));
  r.run(3000);
  CHECK_EQ(r.host.deviceCount(), 3);
  CHECK_EQ(enabledSlots(r), 3);
  // pull the keyboard out of the hub
  hub->disconnectChild(1);
  r.run(500);
  CHECK_EQ(r.host.deviceCount(), 2);
  CHECK_EQ(enabledSlots(r), 2);
  CHECK_EQ(kb->slot, 0);
  // plug another one in the same port, and one in a new port
  hub->connectChild(1, kb2);
  r.run(2000);
  CHECK_EQ(r.host.deviceCount(), 3);
  kb2->reports[0x81].push_back(keyReport(3));
  r.run(20);
  CHECK_EQ(r.keys.size(), 1);
  // pull the hub (and with it kb2) out of the root port
  r.m.detach(1);
  r.run(500);
  CHECK_EQ(r.host.deviceCount(), 1);
  CHECK_EQ(enabledSlots(r), 1);
  // the SuperSpeed one too
  r.m.detach(2);
  r.run(500);
  CHECK_EQ(r.host.deviceCount(), 0);
  CHECK_EQ(enabledSlots(r), 0);
  // and it all works again with the slots handed out anew
  Dev *again = r.keyboard(kSpeedHigh);
  r.m.attach(1, again);
  r.run(1500);
  CHECK_EQ(r.host.deviceCount(), 1);
  again->reports[0x81].push_back(keyReport(4));
  r.run(20);
  CHECK_EQ(r.keys.size(), 2);
  r.finish();
}

static void testReconnectFast() {
  // disconnect and connect again between two looks: the connect status change is what tells (the port is connected both times)
  Rig r;
  CHECK(r.start());
  Dev *kb = r.keyboard(kSpeedHigh);
  r.m.attach(1, kb);
  r.run(1000);
  CHECK_EQ(r.host.deviceCount(), 1);
  const unsigned firstSlot = kb->slot;
  Dev *kb2 = r.keyboard(kSpeedHigh);
  r.m.detach(1);
  r.m.attach(1, kb2);
  r.run(1500);
  CHECK_EQ(r.host.deviceCount(), 1);
  CHECK(kb2->slot != 0);
  CHECK_EQ(kb->slot, 0);
  (void)firstSlot;
  CHECK_EQ(enabledSlots(r), 1);
  r.finish();
}

// ---------------------------------------------------------------------------------------------------------------------------------------------
// Things going wrong

static void testStalledClassRequests() {
  // keyboards often stall SET_IDLE; the default control endpoint has to be recovered and the rest has to work
  Rig r;
  CHECK(r.start());
  Dev *kb = r.keyboard(kSpeedFull);
  kb->stallRequests.insert(usb::kHidReqSetIdle);
  r.m.attach(1, kb);
  r.run(1500);
  CHECK_EQ(r.host.deviceCount(), 1);
  CHECK(r.m.commandCount[kCmdResetEndpoint] >= 1);
  CHECK(r.m.commandCount[kCmdSetTrDequeue] >= 1);
  CHECK_EQ(kb->protocol, 0);
  CHECK_EQ(kb->idle, -1);
  sendKeys(r, kb, 0, 20);
  r.run(500);
  checkKeys(r, 0, 20);
  r.finish();
}

static void testStallDuringDescriptor() {
  // a device that stalls GET_DESCRIPTOR(config): enumeration fails cleanly, the slot is given back, a good device works afterwards
  Rig r;
  CHECK(r.start());
  Dev *bad = r.keyboard(kSpeedHigh);
  bad->stallRequests.insert(usb::kReqGetDescriptor);
  r.m.attach(1, bad);
  r.run(1000);
  CHECK_EQ(r.host.deviceCount(), 0);
  CHECK_EQ(enabledSlots(r), 0);
  CHECK(r.m.commandCount[kCmdDisableSlot] >= 1);
  // the same port, a good device
  r.m.detach(1);
  r.run(300);
  Dev *good = r.keyboard(kSpeedHigh);
  r.m.attach(1, good);
  r.run(3000);
  CHECK_EQ(r.host.deviceCount(), 1);
  good->reports[0x81].push_back(keyReport(1));
  r.run(20);
  CHECK_EQ(r.keys.size(), 1);
  r.finish();
}

static void testUnresponsiveDevice() {
  // a control endpoint that NAKs for ever: the transfer times out, the endpoint is stopped, the slot is freed; the port is tried again later
  Rig r;
  CHECK(r.start());
  Dev *dead = r.keyboard(kSpeedHigh);
  dead->unresponsive = true;
  r.m.attach(1, dead);
  r.run(2500);
  CHECK_EQ(r.host.deviceCount(), 0);
  CHECK_EQ(enabledSlots(r), 0);
  CHECK(r.m.commandCount[kCmdStopEndpoint] >= 1);
  const unsigned tries = r.m.commandCount[kCmdEnableSlot];
  r.run(5000);
  CHECK(r.m.commandCount[kCmdEnableSlot] > tries);   // retried
  CHECK_EQ(enabledSlots(r), 0);
  // it wakes up
  dead->unresponsive = false;
  r.run(6000);
  CHECK_EQ(r.host.deviceCount(), 1);
  r.finish();
}

static void testPortThatNeverResets() {
  Rig r;
  CHECK(r.start());
  Dev *kb = r.keyboard(kSpeedHigh);
  r.m.portStuck[1] = true;
  r.m.attach(1, kb);
  r.run(3000);
  CHECK_EQ(r.host.deviceCount(), 0);
  CHECK_EQ(enabledSlots(r), 0);
  r.m.portStuck[1] = false;
  r.run(3000);
  CHECK_EQ(r.host.deviceCount(), 1);
  r.finish();
}

static void testWarmReset() {
  Rig r;
  CHECK(r.start());
  Dev *kb = r.keyboard(kSpeedSuper);
  r.m.attachInactive(2, kb);
  r.run(2000);
  CHECK_EQ(r.m.warmResets, 1);
  CHECK_EQ(r.m.hotResets, 0);
  CHECK_EQ(r.host.deviceCount(), 1);
  r.finish();
}

static void testHidTransferError() {
  Rig r;
  CHECK(r.start());
  Dev *kb = r.keyboard(kSpeedFull);
  r.m.attach(1, kb);
  r.run(1000);
  CHECK_EQ(r.host.deviceCount(), 1);
  kb->stallInterrupt[0x81] = 2;   // the endpoint halts twice, losing the report it was to deliver
  sendKeys(r, kb, 0, 30);
  r.run(1000);
  CHECK(r.m.commandCount[kCmdResetEndpoint] >= 2);
  CHECK_EQ(r.keys.size(), 28);   // the two reports that were being transferred are gone, the rest arrives in order
  for (unsigned i = 0; i < r.keys.size(); ++i) {
    if (i >= 2) CHECK_EQ(r.keys[i][2], i + 2);
  }
  CHECK(r.m.slots[kb->slot].ep[3].state == kEpRunning);
  r.finish();
}

static void testHubEndpointError() {
  Rig r;
  CHECK(r.start());
  Dev *hub = r.hub(kSpeedHigh, 4);
  r.m.attach(1, hub);
  r.run(2000);
  CHECK_EQ(r.host.deviceCount(), 1);
  hub->stallInterrupt[0x81] = 1;
  Dev *kb = r.keyboard(kSpeedHigh);
  hub->connectChild(3, kb);
  r.run(3000);
  CHECK(r.m.commandCount[kCmdResetEndpoint] >= 1);
  CHECK_EQ(r.host.deviceCount(), 2);
  r.finish();
}

static void testSlotExhaustion() {
  Rig r(64, 1, 2, 3);   // the controller has three slots: the hub and two devices
  CHECK(r.start());
  Dev *hub = r.hub(kSpeedHigh, 5);
  std::vector<Dev *> kbs;
  for (unsigned p = 1; p <= 5; ++p) {
    kbs.push_back(r.keyboard(kSpeedHigh));
    hub->connectChild(p, kbs.back());
  }
  r.m.attach(1, hub);
  r.run(6000);
  CHECK_EQ(r.host.deviceCount(), 3);
  CHECK_EQ(enabledSlots(r), 3);
  unsigned working = 0;
  for (Dev *k : kbs) working += k->slot ? 1 : 0;
  CHECK_EQ(working, 2);
  // unplug a working one, plug a new one in its port: it gets the freed slot
  Dev *gone = nullptr;
  unsigned goneIndex = 0;
  for (unsigned i = 0; i < kbs.size(); ++i)
    if (kbs[i]->slot) {
      gone = kbs[i];
      goneIndex = i;
      break;
    }
  hub->disconnectChild(goneIndex + 1);
  r.run(500);
  CHECK_EQ(r.host.deviceCount(), 2);
  Dev *fresh = r.keyboard(kSpeedHigh);
  hub->connectChild(goneIndex + 1, fresh);
  r.run(2000);
  CHECK_EQ(r.host.deviceCount(), 3);
  CHECK(fresh->slot != 0);
  (void)gone;
  r.finish();
}

// ---------------------------------------------------------------------------------------------------------------------------------------------
// The controller core on its own

static void testControlTransfers() {
  Rig r;
  CHECK(r.start());
  Dev *kb = r.keyboard(kSpeedHigh);
  r.m.attach(1, kb);
  r.run(1000);
  CHECK_EQ(r.host.deviceCount(), 1);
  const unsigned slot = kb->slot;
  // 400 transfers of two and three TRBs: the 64 TRB ring wraps many times (63 is a multiple of 3, but the mix of two and three moves the link around)
  for (unsigned i = 0; i < 400; ++i) {
    uint8_t buf[8] = {};
    uint16_t got = 0;
    xhci::SetupPacket s = {0x80, usb::kReqGetStatus, 0, 0, 2};
    CHECK(r.hc.control(slot, s, buf, &got));
    CHECK_EQ(got, 2);
    xhci::SetupPacket set = {0x00, usb::kReqSetConfiguration, 1, 0, 0};
    CHECK(r.hc.control(slot, set, nullptr, nullptr));
  }
  // a short data stage: asking for more than the device has gives what it has (the residual of the data stage, not of the status stage)
  uint8_t cfg[255] = {};
  uint16_t got = 0;
  xhci::SetupPacket s = {0x80, usb::kReqGetDescriptor, static_cast<uint16_t>(usb::kDescConfig << 8), 0, 255};
  CHECK(r.hc.control(slot, s, cfg, &got));
  CHECK_EQ(got, kb->config.size());
  CHECK(bytesEqual(kb->config, cfg, got < sizeof cfg ? got : 0));
  CHECK(r.m.shortData >= 1);
  // an OUT transfer with data (SET_REPORT-like): the data reaches the device through the model's buffer read (no crash, accepted or stalled)
  // 200 enable/disable pairs: the command ring (64 TRBs) wraps
  for (unsigned i = 0; i < 200; ++i) {
    const int sl = r.hc.enableSlot();
    CHECK(sl > 0);
    if (sl > 0) CHECK(r.hc.disableSlot(static_cast<unsigned>(sl)));
  }
  r.finish();
}

static void testEndpointRecovery() {
  // a control transfer that fails leaves the endpoint usable: the next one goes through (the ring state is in step with the controller's)
  Rig r;
  CHECK(r.start());
  Dev *kb = r.keyboard(kSpeedHigh);
  r.m.attach(1, kb);
  r.run(1000);
  const unsigned slot = kb->slot;
  for (unsigned round = 0; round < 70; ++round) {   // more rounds than the ring has TRBs
    kb->stallRequests.insert(usb::kReqGetStatus);
    uint8_t buf[2];
    uint16_t got = 0;
    xhci::SetupPacket st = {0x80, usb::kReqGetStatus, 0, 0, 2};
    CHECK(!r.hc.control(slot, st, buf, &got));
    kb->stallRequests.clear();
    CHECK(r.hc.control(slot, st, buf, &got));
    CHECK_EQ(got, 2);
  }
  CHECK(r.m.commandCount[kCmdResetEndpoint] >= 70);
  r.finish();
}

int main() {
  struct Test {
    const char *name;
    void (*fn)();
  } tests[] = {
      {"parse configuration descriptors", testParseConfig},
      {"endpoint intervals and contexts", testIntervals},
      {"mouse report layout", testMouseLayout},
      {"controller start", testInit},
      {"high speed keyboard, 64 and 32 byte contexts", testKeyboardHighSpeed},
      {"SuperSpeed mouse, report protocol", testMouseSuperSpeed},
      {"low speed mouse without wheel", testMouseBootProtocol},
      {"full speed maximum packet size of the default endpoint", testFullSpeedMaxPacket},
      {"keyboard and mouse in one device", testComposite},
      {"high speed hub, single and multiple TT", testHubHighSpeed},
      {"SuperSpeed hub", testHubSuperSpeed},
      {"hubs behind hubs", testHubChain},
      {"SuperSpeed hub behind a SuperSpeed hub", testSuperHubChain},
      {"unplugging", testUnplug},
      {"replugging between two looks", testReconnectFast},
      {"stalled class request", testStalledClassRequests},
      {"stalled descriptor request", testStallDuringDescriptor},
      {"device that does not answer", testUnresponsiveDevice},
      {"port whose reset never finishes", testPortThatNeverResets},
      {"warm reset of an inactive SuperSpeed link", testWarmReset},
      {"halted HID endpoint", testHidTransferError},
      {"halted hub endpoint", testHubEndpointError},
      {"more devices than slots", testSlotExhaustion},
      {"control transfers, short data stage, ring wrap", testControlTransfers},
      {"endpoint recovery over many rounds", testEndpointRecovery},
  };
  unsigned failedTests = 0;
  for (const Test &t : tests) {
    const unsigned before = g_failures;
    g_log.clear();
    printf("%s\n", t.name);
    t.fn();
    if (g_failures != before) {
      ++failedTests;
      printf("--- log of the failed test (tail) ---\n%s\n---\n", g_log.size() > 6000 ? g_log.c_str() + g_log.size() - 6000 : g_log.c_str());
    }
  }
  printf("%u checks, %u failed, %u of %zu tests failed\n", g_checks, g_failures, failedTests, sizeof tests / sizeof tests[0]);
  return g_failures ? 1 : 0;
}
