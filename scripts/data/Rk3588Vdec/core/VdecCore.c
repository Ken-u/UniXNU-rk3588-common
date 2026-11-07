// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// VdecCore.c -- MPP's decoder loop without MPP's threads: take stream bytes, let the codec's parser cut them into frames and describe each one, give it a
// place in the DMA window, let the register generator write the job, run it, and hand out the pictures in display order.
//
// This is mpp_dec_decode() of MPP's no-thread mode (mpp/codec/mpp_dec_no_thread.c) with its Linux-isms taken out: the same steps in the same order,
// because the parser, the buffer slots and the register generator depend on that order. What differs: a picture buffer that is not there yet is simply
// allocated (the DMA window is ours alone; MPP waits for the application to return buffers), a format change is acknowledged at once (MPP hands the
// new size to the application and waits for it to say it is ready), and the pictures are given to a callback instead of a queue.
#include <string.h>

#include "VdecPlatform.h"
#include "VdecCore.h"
#include "MppShim.h"

#include "mpp_buf_slot.h"
#include "mpp_common.h"
#include "mpp_dec_cb_param.h"
#include "mpp_log.h"
#include "mpp_mem.h"
#include "mpp_buffer.h"
#include "mpp_dec_cfg.h"
#include "mpp_frame.h"
#include "mpp_hal.h"
#include "mpp_packet.h"
#include "mpp_parser.h"
#include "mpp_soc.h"

// from MPP's tree: the parsers and the register generators of the codecs
extern const ParserApi mpp_h264d, mpp_h265d;
extern const MppHalApi hal_h264d_vdpu34x, hal_h265d_vdpu34x;

#define kHalTasks 3   // MPP: hal_task_count when the HAL supports fast parse

// H.265: the stream is cut into access units here, not by MPP's splitter (see MppShim.c, mpp_ring): what has been fed and is not yet known to be complete.
#define kMaxSegments 32
typedef struct HevcAu {
  uint8_t *buf;
  size_t len, cap;
  size_t scanned;          // bytes of buf the scanner has looked at
  uint64_t scan;           // the last eight bytes it saw
  int vclSeen;             // a first slice segment of the access unit being collected has been seen (MPP: nal_boundary_detected)
  size_t vclAtPlus1;       // 1 + the offset in buf of the NAL header of that first slice segment (0: none yet); the unit's pts is the one of the call that held it
  struct {
    size_t at;             // offset in buf where the bytes of one call to vdec_core_decode start
    int64_t pts;
  } seg[kMaxSegments];
  unsigned nSeg;
} HevcAu;

struct VdecCore {
  VdecHw hw;
  int codec;
  MppDecCfgSet cfg;
  MppBufSlots frameSlots, packetSlots;
  MppCbCtx decCb;
  MppHal hal;
  MppHalCfg halCfg;
  Parser parser;
  MppBufferGroup packetGroup, frameGroup;
  MppDev dev;
  const MppDecHwCap *hwInfo;
  HalTaskInfo task;
  int taskParsed;            // the parser produced a task that is not finished (a format change in the way)
  int infoUpdated;
  VdecStats stats;
  HevcAu au;
};

static VdecCore *gOpen;

static void task_init(HalTaskInfo *task) {
  HalDecTask *p = &task->dec;
  memset(p, 0, sizeof *p);
  p->output = -1;
  p->input = -1;
  memset(p->refer, -1, sizeof p->refer);
}

static MPP_RET callback_hal_to_parser(const char *caller, void *ctx, RK_S32 cmd, void *param) {
  (void)caller;
  (void)cmd;
  VdecCore *core = (VdecCore *)ctx;
  return core->parser ? mpp_parser_callback(core->parser, param) : MPP_OK;
}

static void check_data_layout(VdecCore *core) {
  MppFmtLayout layout = MPP_FMT_RASTER;   // out_fmt is plain NV12 (no FBC, no tile)
  mpp_slots_set_prop(core->packetSlots, SLOTS_DATA_LAYOUT, &layout);
  mpp_slots_set_prop(core->frameSlots, SLOTS_DATA_LAYOUT, &layout);
}

int vdec_core_create(VdecCore **out, const VdecHw *hw, int codec) {
  VdecCore *core;
  SlotHalFbcAdjCfg fbcAdj;
  MppCodingType coding;
  RK_U32 halTasks = 2;

  if (!out || !hw || !hw->run || (codec != kVdecH264 && codec != kVdecH265)) return -1;
  *out = NULL;
  if (gOpen) {
    mpp_err("only one decoder can use the block\n");
    return -1;
  }
  core = (VdecCore *)mpp_calloc(VdecCore, 1);
  if (!core) return -1;
  core->hw = *hw;
  core->codec = codec;
  coding = codec == kVdecH265 ? MPP_VIDEO_CodingHEVC : MPP_VIDEO_CodingAVC;
  vdec_shim_bind(&core->hw);

  // MPP's defaults (mpp_dec_cfg_set_default) and what mpp_init / the decoder test program set on top
  core->cfg.base.type = MPP_CTX_DEC;
  core->cfg.base.coding = coding;
  core->cfg.base.hw_type = -1;
  core->cfg.base.fast_parse = 1;
  core->cfg.base.enable_fast_play = MPP_ENABLE_FAST_PLAY;
  core->cfg.base.meta_chk_len = MPP_DEC_META_CHK_LEN_DEFAULT;
  core->cfg.base.split_parse = codec == kVdecH264;   // H.265: access units are cut here (hevc_feed)
  core->cfg.base.disable_thread = 1;

  core->decCb.callBack = callback_hal_to_parser;
  core->decCb.ctx = core;
  core->decCb.cmd = DEC_PARSER_CALLBACK;

  mpp_buffer_group_get_internal(&core->packetGroup, MPP_BUFFER_TYPE_ION);
  mpp_buffer_group_get_internal(&core->frameGroup, MPP_BUFFER_TYPE_ION);

  do {
    if (mpp_buf_slot_init(&core->frameSlots) || mpp_buf_slot_init(&core->packetSlots)) break;

    if (codec == kVdecH265) {
      mpp_hal_api_register(&hal_h265d_vdpu34x);
      mpp_parser_api_register(&mpp_h265d);
    } else {
      mpp_hal_api_register(&hal_h264d_vdpu34x);
      mpp_parser_api_register(&mpp_h264d);
    }

    memset(&fbcAdj, 0, sizeof fbcAdj);
    {
      MppHalCfg halCfg = {
          .type = MPP_CTX_DEC,
          .coding = coding,
          .frame_slots = core->frameSlots,
          .packet_slots = core->packetSlots,
          .cfg = &core->cfg,
          .dec_cb = &core->decCb,
          .hw_info = NULL,
          .dev = NULL,
          .support_fast_mode = 0,
          .hal_fbc_adj_cfg = &fbcAdj,
          .buf_group = NULL,
      };
      if (mpp_hal_init(&core->hal, &halCfg)) break;
      core->halCfg = halCfg;
    }
    if (fbcAdj.func) mpp_slots_set_prop(core->frameSlots, SLOTS_HAL_FBC_ADJ, &fbcAdj);

    if (core->cfg.base.fast_parse && core->halCfg.support_fast_mode)
      halTasks = 3;
    else
      core->cfg.base.fast_parse = 0;
    core->cfg.status.hal_support_fast_mode = core->halCfg.support_fast_mode;
    core->cfg.status.hal_task_count = halTasks;

    mpp_buf_slot_setup(core->packetSlots, halTasks);
    mpp_slots_set_prop(core->packetSlots, SLOTS_CODING_TYPE, &coding);
    mpp_slots_set_prop(core->frameSlots, SLOTS_CODING_TYPE, &coding);

    core->hwInfo = core->halCfg.hw_info;
    core->dev = core->halCfg.dev;
    check_data_layout(core);

    {
      ParserCfg parserCfg = {coding, core->frameSlots, core->packetSlots, &core->cfg, core->hwInfo};
      if (mpp_parser_init(&core->parser, &parserCfg)) break;
    }
    task_init(&core->task);
    gOpen = core;
    *out = core;
    return 0;
  } while (0);
  vdec_core_destroy(core);
  return -1;
}

void vdec_core_destroy(VdecCore *core) {
  if (!core) return;
  // A decoder destroyed without an end of stream still holds pictures that wait for their display turn (a B-frame stream: the last reference) and the
  // slots of its reference frames; give them back first, as MPP's own teardown does, so that the slot table is clean when it is freed (otherwise
  // mpp_buf_slot asserts "!slot->status.on_used" in clear_slots_impl).
  if (core->parser && core->hal && core->frameSlots && core->packetSlots) vdec_core_reset(core);
  if (core->parser) mpp_parser_deinit(core->parser);
  if (core->hal) mpp_hal_deinit(core->hal);
  if (core->frameSlots) mpp_buf_slot_deinit(core->frameSlots);
  if (core->packetSlots) mpp_buf_slot_deinit(core->packetSlots);
  if (core->packetGroup) mpp_buffer_group_put(core->packetGroup);
  if (core->frameGroup) mpp_buffer_group_put(core->frameGroup);
  if (core->au.buf) mpp_free(core->au.buf);
  if (gOpen == core) gOpen = NULL;
  mpp_free(core);
}

void vdec_core_set_observer(VdecCore *core, VdecJobObserver *observer) { vdec_shim_set_observer(core->dev, observer); }

void vdec_core_stats(const VdecCore *core, VdecStats *stats) {
  *stats = core->stats;
  vdec_shim_window_stats(NULL, NULL, &stats->windowPeakBytes);
}

// What the sinks are told about a frame of the slot table (valid while the buffer is held).
static void fill_picture(VdecPicture *pic, MppFrame frame, MppBuffer buf) {
  memset(pic, 0, sizeof *pic);
  pic->data = (const uint8_t *)mpp_buffer_get_ptr(buf);
  pic->bytes = (uint32_t)mpp_buffer_get_size(buf);
  pic->width = mpp_frame_get_width(frame);
  pic->height = mpp_frame_get_height(frame);
  pic->hstride = mpp_frame_get_hor_stride(frame);
  pic->vstride = mpp_frame_get_ver_stride(frame);
  pic->cropLeft = mpp_frame_get_offset_x(frame);
  pic->cropTop = mpp_frame_get_offset_y(frame);
  pic->pts = mpp_frame_get_pts(frame);
  pic->poc = mpp_frame_get_poc(frame);
  if (mpp_frame_get_errinfo(frame)) pic->flags |= kVdecPicError;
  if (mpp_frame_get_discard(frame)) pic->flags |= kVdecPicDiscard;
}

// the pictures that are ready, in display order (mpp_dec_push_display + mpp_dec_put_frame)
static int push_display(VdecCore *core, HalDecTaskFlag flags, const VdecSink *sink) {
  MppBufSlots slots = core->frameSlots;
  RK_S32 index = -1;
  int n = 0;
  RK_U32 eos = flags.eos;
  RK_U32 error = flags.parse_err || flags.ref_err;
  RK_U32 refer = flags.used_for_ref;

  while (MPP_OK == mpp_buf_slot_dequeue(slots, &index, QUEUE_DISPLAY)) {
    MppFrame frame = NULL;
    MppBuffer buf;
    RK_U32 last = eos && mpp_slots_is_empty(slots, QUEUE_DISPLAY);

    mpp_buf_slot_get_prop(slots, index, SLOT_FRAME_PTR, &frame);
    buf = frame ? mpp_frame_get_buffer(frame) : NULL;
    if (frame && buf && sink && sink->picture) {
      VdecPicture pic;
      fill_picture(&pic, frame, buf);
      if (last) pic.flags |= kVdecPicEos;
      if (last && error) pic.flags |= refer ? kVdecPicError : kVdecPicDiscard;
      sink->picture(sink->ctx, &pic);
    }
    // MPP hands the application a copy of the frame (mpp_frame_copy shares the buffer without a reference of its own) and the application's
    // mpp_frame_deinit() puts the buffer: one reference per picture that goes out. The sink has had its look; this is that put.
    if (frame && buf) mpp_buffer_put(buf);
    mpp_buf_slot_clr_flag(slots, index, SLOT_QUEUE_USE);
    n++;
  }
  core->stats.pictures += (uint32_t)n;
  return n;
}

static void flush(VdecCore *core) {
  mpp_parser_flush(core->parser);
  mpp_hal_flush(core->hal);
}

// one task the parser has prepared (mpp_dec_decode, steps 4 to 10)
static int run_task(VdecCore *core, const VdecSink *sink) {
  HalTaskInfo *task = &core->task;
  HalDecTask *dec = &task->dec;
  MppBufSlots frameSlots = core->frameSlots, packetSlots = core->packetSlots;
  MppBuffer bufIn = NULL, bufOut = NULL;
  RK_S32 slotPkt = -1;
  int out = 0;

  // 4, 5. a packet slot and a hardware buffer for the stream
  mpp_buf_slot_get_unused(packetSlots, &slotPkt);
  if (slotPkt < 0) {
    mpp_err("no packet slot\n");
    return -1;
  }
  size_t streamSize = mpp_packet_get_size(dec->input_packet);
  mpp_buf_slot_get_prop(packetSlots, slotPkt, SLOT_BUFFER, &bufIn);
  if (!bufIn) {
    mpp_buffer_get(core->packetGroup, &bufIn, streamSize);
    if (!bufIn) return -1;
    mpp_buf_slot_set_prop(packetSlots, slotPkt, SLOT_BUFFER, bufIn);
    mpp_buffer_put(bufIn);
  }
  dec->input = slotPkt;

  // 6. the stream into the hardware buffer
  {
    void *src = mpp_packet_get_data(dec->input_packet);
    size_t length = mpp_packet_get_length(dec->input_packet);
    mpp_buffer_write(bufIn, 0, src, length);
    mpp_buffer_sync_partial_end(bufIn, 0, length);
  }
  mpp_buf_slot_set_flag(packetSlots, dec->input, SLOT_CODEC_READY);
  mpp_buf_slot_set_flag(packetSlots, dec->input, SLOT_HAL_INPUT);

  // 7. parse
  mpp_parser_parse(core->parser, dec);
  core->taskParsed = 1;

  // 8. nothing to decode
  if (dec->output < 0 || !dec->valid) {
    if (dec->flags.eos) {
      flush(core);
      out += push_display(core, dec->flags, sink);
    }
    mpp_buf_slot_clr_flag(packetSlots, dec->input, SLOT_HAL_INPUT);
    core->taskParsed = 0;
    task_init(task);
    return out;
  }

  // 9. a new picture format: the pictures of the old one go out first, then the slots are told the application is ready
  if (mpp_buf_slot_is_changed(frameSlots)) {
    RK_U32 eos = dec->flags.eos;
    dec->flags.info_change = 1;
    dec->flags.eos = 0;
    flush(core);
    out += push_display(core, dec->flags, sink);
    dec->flags.eos = eos;
    mpp_buf_slot_ready(frameSlots);
    dec->flags.info_change = 0;
  }

  // 10. the picture buffer
  mpp_buf_slot_get_prop(frameSlots, dec->output, SLOT_BUFFER, &bufOut);
  if (!bufOut) {
    size_t size = mpp_buf_slot_get_size(frameSlots);
    mpp_buffer_get(core->frameGroup, &bufOut, size);
    if (!bufOut) {
      mpp_err("no picture buffer of %u bytes\n", (unsigned)size);
      return -1;
    }
    mpp_buf_slot_set_prop(frameSlots, dec->output, SLOT_BUFFER, bufOut);
  }

  // 11. the job
  mpp_hal_reg_gen(core->hal, task);
  mpp_hal_hw_start(core->hal, task);
  mpp_hal_hw_wait(core->hal, task);
  core->stats.jobs++;

  // the picture of this job, decode order, before anything else can touch the window
  if (sink && sink->decoded && dec->output >= 0) {
    MppFrame frame = NULL;
    mpp_buf_slot_get_prop(frameSlots, dec->output, SLOT_FRAME_PTR, &frame);
    if (frame && bufOut) {
      VdecPicture pic;
      fill_picture(&pic, frame, bufOut);
      if (dec->flags.parse_err || dec->flags.ref_err) pic.flags |= kVdecPicError;
      sink->decoded(sink->ctx, &pic);
    }
  }

  // 12. the block is done with the stream, the picture and its references
  mpp_buf_slot_clr_flag(packetSlots, dec->input, SLOT_HAL_INPUT);
  if (dec->output >= 0) mpp_buf_slot_clr_flag(frameSlots, dec->output, SLOT_HAL_OUTPUT);
  for (unsigned i = 0; i < (unsigned)MPP_ARRAY_ELEMS(dec->refer); i++) {
    RK_S32 index = dec->refer[i];
    if (index >= 0) mpp_buf_slot_clr_flag(frameSlots, index, SLOT_HAL_INPUT);
  }
  if (dec->flags.eos) flush(core);
  out += push_display(core, dec->flags, sink);

  core->taskParsed = 0;
  task_init(task);
  return out;
}

static int feed(VdecCore *core, const uint8_t *data, size_t size, int64_t pts, int eos, const VdecSink *sink) {
  MppPacket pkt = NULL;
  int out = 0;

  if (mpp_packet_init(&pkt, (void *)data, size)) return -1;
  mpp_packet_set_pts(pkt, pts);
  mpp_packet_set_length(pkt, size);
  if (eos) mpp_packet_set_eos(pkt);

  for (;;) {
    HalDecTask *dec = &core->task.dec;
    task_init(&core->task);
    mpp_parser_prepare(core->parser, pkt, dec);
    if (!dec->valid) {
      // no task: the input is used up (or empty). An end of stream still flushes the pictures held back for reordering.
      if (dec->flags.eos) {
        flush(core);
        out += push_display(core, dec->flags, sink);
      }
      break;
    }
    int n = run_task(core, sink);
    if (n < 0) {
      out = -1;
      break;
    }
    out += n;
    if (!mpp_packet_get_length(pkt)) break;
  }
  mpp_packet_deinit(&pkt);
  return out;
}

// ---- H.265: access units
//
// The rule is MPP's (h265d_find_frame_end in h265d_spliter.c): an access unit ends where the next one starts, which is at a VPS, SPS, PPS, AUD, prefix SEI (or one of
// the reserved types 41..44 and 48..55) once a slice of the unit has been seen, or at the first slice segment of a picture once a first slice segment has been seen. A
// 3 byte start code is where the cut goes; the zero byte of a 4 byte start code stays at the end of the previous unit (trailing_zero_8bits).
static int hevc_emit(VdecCore *core, size_t cut, const VdecSink *sink) {
  HevcAu *au = &core->au;
  // the unit starts at offset 0 and the first segment always does too: its pts is the presentation time given with the first byte of the unit
  // The unit's pts is the one given with its first slice segment. (The first byte of the unit can be an access unit delimiter, a parameter set or an SEI that
  // came with the previous call: a caller that ends each of its calls with a delimiter would see every picture carry the pts of the call before.)
  unsigned from = 0;
  if (au->vclAtPlus1)
    for (unsigned i = 0; i < au->nSeg; i++)
      if (au->seg[i].at <= au->vclAtPlus1 - 1) from = i;
  int n = feed(core, au->buf, cut, au->nSeg ? au->seg[from].pts : 0, 0, sink);
  au->vclAtPlus1 = 0;
  // what is left moves to the front, and the segments with it
  memmove(au->buf, au->buf + cut, au->len - cut);
  au->len -= cut;
  au->scanned -= cut;
  if (!au->len) {
    au->nSeg = 0;
    return n;
  }
  unsigned out = 0, last = 0;
  for (unsigned i = 0; i < au->nSeg; i++)
    if (au->seg[i].at <= cut) last = i;
  for (unsigned i = last; i < au->nSeg; i++) {
    au->seg[out] = au->seg[i];
    au->seg[out].at = au->seg[i].at > cut ? au->seg[i].at - cut : 0;
    out++;
  }
  au->nSeg = out;
  return n;
}

// Takes the bytes in and emits every access unit that is complete; with `eos` the last one as well.
static int hevc_feed(VdecCore *core, const uint8_t *data, size_t size, int64_t pts, int eos, const VdecSink *sink) {
  HevcAu *au = &core->au;
  int out = 0;

  if (size) {
    if (au->len + size > au->cap) {
      size_t cap = au->cap ? au->cap : 65536;
      while (cap < au->len + size) cap *= 2;
      uint8_t *grown = (uint8_t *)mpp_malloc(uint8_t, cap);
      if (!grown) return -1;
      if (au->len) memcpy(grown, au->buf, au->len);
      if (au->buf) mpp_free(au->buf);
      au->buf = grown;
      au->cap = cap;
    }
    if (au->len == 0) au->nSeg = 0;
    if (au->nSeg == 0 || au->seg[au->nSeg - 1].pts != pts) {
      if (au->nSeg == kMaxSegments) {   // more calls in one unit than we remember: the oldest are forgotten (the unit takes the pts of the oldest left)
        memmove(&au->seg[0], &au->seg[1], (kMaxSegments - 1) * sizeof au->seg[0]);
        au->nSeg--;
        au->seg[0].at = 0;
      }
      au->seg[au->nSeg].at = au->len;
      au->seg[au->nSeg].pts = pts;
      au->nSeg++;
    }
    memcpy(au->buf + au->len, data, size);
    au->len += size;
  }

  while (au->scanned < au->len) {
    size_t i = au->scanned++;
    au->scan = (au->scan << 8) | au->buf[i];
    if (((au->scan >> 24) & 0xFFFFFF) != 1) continue;
    // the start code is buf[i-5 .. i-3], the NAL header buf[i-2], buf[i-1], the first byte of the slice segment header buf[i]
    int nut = (int)((au->scan >> 17) & 0x3F);
    int layer = (int)((((au->scan >> 16) & 0x01) << 5) + (((au->scan >> 8) & 0xF8) >> 3));
    int cut = 0;
    if ((nut >= 32 && nut <= 35) || nut == 39 || (nut >= 41 && nut <= 44) || (nut >= 48 && nut <= 55)) {   // VPS .. AUD, prefix SEI
      if (au->vclSeen && !layer) {
        au->vclSeen = 0;
        cut = 1;
      }
    } else if (nut <= 9 || (nut >= 16 && nut <= 21)) {   // a slice segment
      if ((au->buf[i] >> 7) && !layer) {
        if (!au->vclSeen) {
          au->vclSeen = 1;
          au->vclAtPlus1 = i - 2 + 1;
        } else {
          cut = 1;
        }
      }
    }
    if (cut) {
      int n = hevc_emit(core, i - 5, sink);
      if (n < 0) return -1;
      out += n;
      au->vclSeen = (nut <= 9 || (nut >= 16 && nut <= 21)) ? 1 : 0;   // the unit that starts here has had its first slice segment if the cut was one
      if (au->vclSeen) au->vclAtPlus1 = 3 + 1;   // the cut was at the start code (i - 5): the NAL header is the 4th byte of the new unit
    }
  }

  if (eos && au->len) {
    int n = hevc_emit(core, au->len, sink);
    if (n < 0) return -1;
    out += n;
    au->scan = 0;
    au->vclSeen = 0;
    au->vclAtPlus1 = 0;
  }
  return out;
}

int vdec_core_decode(VdecCore *core, const uint8_t *data, size_t size, int64_t pts, int eos, const VdecSink *sink) {
  if (!core || (size && !data)) return -1;
  if (core->codec == kVdecH265) {
    int n = hevc_feed(core, data, size, pts, eos, sink);
    if (n < 0) return n;
    if (eos) {
      int m = feed(core, NULL, 0, pts, 1, sink);
      if (m < 0) return m;
      n += m;
    }
    return n;
  }
  // The H.264 parser throws away the data of an end-of-stream packet shorter than four bytes (it takes it for the empty marker MPP's own applications send,
  // h264d_prepare), so the end of the stream always goes in as a packet of its own, empty.
  int out = 0;
  if (size) {
    out = feed(core, data, size, pts, 0, sink);
    if (out < 0) return out;
  }
  if (eos) {
    int n = feed(core, NULL, 0, pts, 1, sink);
    if (n < 0) return n;
    out += n;
  }
  return out;
}

void vdec_core_reset(VdecCore *core) {
  HalDecTask *dec = &core->task.dec;
  RK_S32 index;

  core->au.len = core->au.scanned = 0;
  core->au.scan = 0;
  core->au.vclSeen = 0;
  core->au.vclAtPlus1 = 0;
  core->au.nSeg = 0;

  mpp_parser_reset(core->parser);
  mpp_hal_reset(core->hal);
  if (core->taskParsed) {
    if (dec->output >= 0) mpp_buf_slot_clr_flag(core->frameSlots, dec->output, SLOT_HAL_OUTPUT);
    for (unsigned i = 0; i < (unsigned)MPP_ARRAY_ELEMS(dec->refer); i++)
      if (dec->refer[i] >= 0) mpp_buf_slot_clr_flag(core->frameSlots, dec->refer[i], SLOT_HAL_INPUT);
    if (dec->input >= 0) {
      mpp_buf_slot_clr_flag(core->packetSlots, dec->input, SLOT_HAL_INPUT);
      mpp_buf_slot_clr_flag(core->packetSlots, dec->input, SLOT_CODEC_READY);
    }
    core->taskParsed = 0;
  }
  while (MPP_OK == mpp_buf_slot_dequeue(core->frameSlots, &index, QUEUE_DISPLAY)) {
    MppBuffer buffer = NULL;
    mpp_buf_slot_get_prop(core->frameSlots, index, SLOT_BUFFER, &buffer);
    if (buffer) mpp_buffer_put(buffer);
    mpp_buf_slot_clr_flag(core->frameSlots, index, SLOT_QUEUE_USE);
  }
  task_init(&core->task);
}
