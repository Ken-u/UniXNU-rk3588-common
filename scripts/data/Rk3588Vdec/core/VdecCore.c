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

// from MPP's tree: the parser and the register generator of the codec
extern const ParserApi mpp_h264d;
extern const MppHalApi hal_h264d_vdpu34x;

#define kHalTasks 3   // MPP: hal_task_count when the HAL supports fast parse

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

  if (!out || !hw || !hw->run || codec != kVdecH264) return -1;
  *out = NULL;
  if (gOpen) {
    mpp_err("only one decoder can use the block\n");
    return -1;
  }
  core = (VdecCore *)mpp_calloc(VdecCore, 1);
  if (!core) return -1;
  core->hw = *hw;
  core->codec = codec;
  coding = MPP_VIDEO_CodingAVC;
  vdec_shim_bind(&core->hw);

  // MPP's defaults (mpp_dec_cfg_set_default) and what mpp_init / the decoder test program set on top
  core->cfg.base.type = MPP_CTX_DEC;
  core->cfg.base.coding = coding;
  core->cfg.base.hw_type = -1;
  core->cfg.base.fast_parse = 1;
  core->cfg.base.enable_fast_play = MPP_ENABLE_FAST_PLAY;
  core->cfg.base.meta_chk_len = MPP_DEC_META_CHK_LEN_DEFAULT;
  core->cfg.base.split_parse = 1;
  core->cfg.base.disable_thread = 1;

  core->decCb.callBack = callback_hal_to_parser;
  core->decCb.ctx = core;
  core->decCb.cmd = DEC_PARSER_CALLBACK;

  mpp_buffer_group_get_internal(&core->packetGroup, MPP_BUFFER_TYPE_ION);
  mpp_buffer_group_get_internal(&core->frameGroup, MPP_BUFFER_TYPE_ION);

  do {
    if (mpp_buf_slot_init(&core->frameSlots) || mpp_buf_slot_init(&core->packetSlots)) break;

    mpp_hal_api_register(&hal_h264d_vdpu34x);
    mpp_parser_api_register(&mpp_h264d);

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
  if (core->parser) mpp_parser_deinit(core->parser);
  if (core->hal) mpp_hal_deinit(core->hal);
  if (core->frameSlots) mpp_buf_slot_deinit(core->frameSlots);
  if (core->packetSlots) mpp_buf_slot_deinit(core->packetSlots);
  if (core->packetGroup) mpp_buffer_group_put(core->packetGroup);
  if (core->frameGroup) mpp_buffer_group_put(core->frameGroup);
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

int vdec_core_decode(VdecCore *core, const uint8_t *data, size_t size, int64_t pts, int eos, const VdecSink *sink) {
  if (!core || (size && !data)) return -1;
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
