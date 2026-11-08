// SPDX-License-Identifier: BSD-2-Clause-Patent
//
// VencCore.c -- MPP's encoder loop without MPP's threads: take one picture, let the controller (rate control, reference management, parameter sets) describe
// the frame, let the register generator write the job, run it on the block, and hand out the access unit.
//
// MPP's encoder is a thread (mpp_enc_thread() in mpp/codec/mpp_enc_impl.c) that waits for a task on an input port, does the steps of one picture, and puts the
// result on an output port. This file includes that source file as it is (so that its static step functions are reachable) and runs the thread's body
// itself, once per picture: the same steps in the same order, because the rate control and the reference management depend on that order. What differs: the
// control commands are processed directly (the thread does that between pictures), a picture is put on the input port and taken from the output port by
// the same call, and the buffers are pieces of the DMA window.
#include <string.h>

#include "VencPlatform.h"
#include "VencCore.h"
#include "EncShim.h"

// MPP's controller: the file is the pinned checkout's (core/build-host.sh and scripts/build-rk3588-venc.py pass its path)
#include MPP_ENC_IMPL_C

#include "mpp_enc_hal.h"
#include "mpp_task_impl.h"
#include "mpp_frame.h"
#include "mpp_packet.h"
#include "mpp_meta.h"
#include "mpp_buffer.h"
#include "mpp_list.h"
#include "hal_h264e_vepu580.h"
#include "rk_venc_cmd.h"
#include "rk_venc_cfg.h"

struct VencCore {
  VencHw hw;
  VencParams params;
  Mpp mpp;
  MppEnc enc;
  MppEncCfg cfg;
  MppBufferGroup frameGroup, packetGroup;
  MppBuffer frameBuf, packetBuf, mdBuf;
  MppTask inputTask;          // the input task the application holds between two pictures (mpp_put_frame() keeps it in Mpp::mInputTask)
  uint32_t hstride, vstride, sourceBytes;
  EncAsyncTaskInfo task;      // MPP's thread keeps this on its stack: about 2 KiB, too much for a kernel thread's, so it lives here
  EncAsyncWait wait;
  VencStats stats;
  int needIdr;
};

static VencCore *gOpen;

// the tables of MPP's test tool (utils/mpi_enc_utils.c), copied; the hardware's adaptive quantisation (thresholds and QP steps by motion class) and the quantiser bias
static RK_S32 kQbiasAvc[18] = {3, 6, 13, 683, 683, 683, 683, 3, 6, 13, 683, 683, 683, 683, 341, 341, 341, 341};
static RK_S32 kAqRange[10] = {5, 5, 10, 12, 12, 5, 5, 10, 12, 12};
static RK_S32 kAqThd[16] = {0, 0, 0, 0, 3, 3, 5, 5, 8, 8, 8, 15, 15, 20, 25, 25};
static RK_S32 kAqStepI[16] = {-8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 5, 7, 7, 8};
static RK_S32 kAqStepP[16] = {-8, -7, -6, -5, -4, -2, -1, -1, 0, 2, 3, 4, 6, 8, 9, 10};

static uint32_t align_up(uint32_t v, uint32_t a) { return (v + a - 1u) & ~(a - 1u); }
static uint32_t align16(uint32_t v) { return align_up(v, 16u); }

// mpp_enc_thread()'s handling of a control command, as the thread does it between two pictures
static int core_control(VencCore *core, MpiCmd cmd, void *param) {
  MppEncImpl *enc = (MppEncImpl *)core->enc;
  MPP_RET ret = mpp_enc_proc_cfg(enc, cmd, param);
  mpp_enc_hal_prepare(enc->enc_hal);
  /* NOTE: here will clear change flag of rc and prep cfg */
  mpp_enc_proc_rc_update(enc);
  return ret ? -1 : 0;
}

static int rc_mode_of(int mode) {
  switch (mode) {
    case kVencRcFixQp: return MPP_ENC_RC_MODE_FIXQP;
    case kVencRcVbr: return MPP_ENC_RC_MODE_VBR;
    default: return MPP_ENC_RC_MODE_CBR;
  }
}

// mpi_enc_cfg_setup() of MPP's test tool (utils/mpi_enc_utils.c) for H.264, with its command line values at their defaults
// every setting must be taken: a name MPP does not know is a mistake of ours (or a different MPP), not something to go on from
#define CFG_S32(name, value) do { if (mpp_enc_cfg_set_s32(cfg, name, value)) { vec_log(2, "venc: MPP does not take " name); return -1; } } while (0)
#define CFG_U32(name, value) do { if (mpp_enc_cfg_set_u32(cfg, name, value)) { vec_log(2, "venc: MPP does not take " name); return -1; } } while (0)
#define CFG_ST(name, value) do { if (mpp_enc_cfg_set_st(cfg, name, value)) { vec_log(2, "venc: MPP does not take " name); return -1; } } while (0)

static int core_configure(VencCore *core) {
  const VencParams *p = &core->params;
  MppEncCfg cfg = core->cfg;
  int rcMode = rc_mode_of(p->rcMode);
  int fpsNum = p->fpsNum ? (int)p->fpsNum : 30, fpsDen = p->fpsDen ? (int)p->fpsDen : 1;
  int bps = p->bitrate ? (int)p->bitrate : (int)(p->width * p->height / 8 * (uint32_t)(fpsNum / fpsDen));
  int gop = p->gop ? (int)p->gop : fpsNum * 2;

  CFG_S32("codec:type", MPP_VIDEO_CodingAVC);
  CFG_S32("prep:width", (RK_S32)p->width);
  CFG_S32("prep:height", (RK_S32)p->height);
  CFG_S32("prep:hor_stride", (RK_S32)core->hstride);
  CFG_S32("prep:ver_stride", (RK_S32)core->vstride);
  CFG_S32("prep:format", p->format == kVencI420 ? MPP_FMT_YUV420P : MPP_FMT_YUV420SP);
  CFG_S32("prep:range", MPP_FRAME_RANGE_JPEG);

  CFG_S32("rc:mode", rcMode);
  CFG_U32("rc:max_reenc_times", 0);
  CFG_U32("rc:super_mode", 0);
  CFG_U32("rc:drop_mode", MPP_ENC_RC_DROP_FRM_DISABLED);
  CFG_U32("rc:drop_thd", 20);
  CFG_U32("rc:drop_gap", 1);

  CFG_S32("tune:atf_str", 0);
  CFG_S32("tune:atr_str_i", 0);
  CFG_S32("tune:atr_str_p", 0);
  CFG_S32("tune:atl_str", 0);
  CFG_S32("tune:deblur_en", 0);
  CFG_S32("tune:deblur_str", 0);
  CFG_S32("tune:sao_str_i", 0);
  CFG_S32("tune:sao_str_p", 0);
  CFG_S32("tune:lambda_idx_p", 0);
  CFG_S32("tune:lambda_idx_i", 0);
  CFG_S32("tune:rc_container", 0);
  CFG_S32("tune:scene_mode", 0);
  CFG_S32("tune:speed", 0);
  CFG_S32("tune:vmaf_opt", 0);

  CFG_S32("hw:qbias_en", 1);
  CFG_S32("hw:qbias_i", 0);
  CFG_S32("hw:qbias_p", 0);
  CFG_S32("hw:skip_bias_en", 0);
  CFG_S32("hw:skip_bias", 4);
  CFG_S32("hw:skip_sad", 8);

  CFG_S32("prep:mirroring", 0);
  CFG_S32("prep:rotation", 0);
  CFG_S32("prep:flip", 0);

  CFG_S32("rc:fps_in_flex", 0);
  CFG_S32("rc:fps_in_num", fpsNum);
  CFG_S32("rc:fps_in_denom", fpsDen);
  CFG_S32("rc:fps_out_flex", 0);
  CFG_S32("rc:fps_out_num", fpsNum);
  CFG_S32("rc:fps_out_denom", fpsDen);

  CFG_S32("rc:bps_target", bps);
  switch (rcMode) {
    case MPP_ENC_RC_MODE_FIXQP:
      break;
    case MPP_ENC_RC_MODE_VBR:
      CFG_S32("rc:bps_max", p->bitrateMax ? (int)p->bitrateMax : bps * 17 / 16);
      CFG_S32("rc:bps_min", p->bitrateMin ? (int)p->bitrateMin : bps * 1 / 16);
      break;
    default:
      CFG_S32("rc:bps_max", p->bitrateMax ? (int)p->bitrateMax : bps * 17 / 16);
      CFG_S32("rc:bps_min", p->bitrateMin ? (int)p->bitrateMin : bps * 15 / 16);
      break;
  }

  if (rcMode == MPP_ENC_RC_MODE_FIXQP) {
    int fix = p->qp ? p->qp : 28;
    CFG_S32("rc:qp_init", fix);
    CFG_S32("rc:qp_max", fix);
    CFG_S32("rc:qp_min", fix);
    CFG_S32("rc:qp_max_i", fix);
    CFG_S32("rc:qp_min_i", fix);
    CFG_S32("rc:qp_ip", 0);
    CFG_S32("rc:fqp_min_i", fix);
    CFG_S32("rc:fqp_max_i", fix);
    CFG_S32("rc:fqp_min_p", fix);
    CFG_S32("rc:fqp_max_p", fix);
  } else {
    CFG_S32("rc:qp_init", p->qp ? p->qp : -1);
    CFG_S32("rc:qp_max", p->qpMax ? p->qpMax : 51);
    CFG_S32("rc:qp_min", p->qpMin ? p->qpMin : 10);
    CFG_S32("rc:qp_max_i", p->qpMax ? p->qpMax : 51);
    CFG_S32("rc:qp_min_i", p->qpMin ? p->qpMin : 10);
    CFG_S32("rc:qp_ip", 2);
    CFG_S32("rc:fqp_min_i", 10);
    CFG_S32("rc:fqp_max_i", 45);
    CFG_S32("rc:fqp_min_p", 10);
    CFG_S32("rc:fqp_max_p", 45);
  }

  CFG_S32("h264:profile", p->profile ? p->profile : 100);
  CFG_S32("h264:level", p->level ? p->level : 40);
  CFG_S32("h264:cabac_en", p->cabac < 0 ? 0 : 1);
  CFG_S32("h264:cabac_idc", 0);
  CFG_S32("h264:trans8x8", 1);

  CFG_S32("rc:gop", gop);

  // the hardware's adaptive quantisation and quantiser bias tables (mpi_enc_utils.c: the "ipc" set, H.264)
  CFG_ST("hw:aq_thrd_i", kAqThd);
  CFG_ST("hw:aq_thrd_p", kAqThd);
  CFG_ST("hw:aq_step_i", kAqStepI);
  CFG_ST("hw:aq_step_p", kAqStepP);
  CFG_ST("hw:aq_rnge_arr", kAqRange);
  CFG_ST("hw:qbias_arr", kQbiasAvc);

  if (core_control(core, MPP_ENC_SET_CFG, cfg)) return -1;

  {
    RcApiBrief brief;
    brief.type = MPP_VIDEO_CodingAVC;
    brief.name = "default";
    if (core_control(core, MPP_ENC_SET_RC_API_CURRENT, &brief)) return -1;
  }
  {
    MppEncSeiMode sei = MPP_ENC_SEI_MODE_DISABLE;
    if (core_control(core, MPP_ENC_SET_SEI_CFG, &sei)) return -1;
  }
  {
    MppEncHeaderMode mode = MPP_ENC_HEADER_MODE_EACH_IDR;
    if (core_control(core, MPP_ENC_SET_HEADER_MODE, &mode)) return -1;
  }
  return 0;
}

int venc_core_create(VencCore **out, const VencHw *hw, const VencParams *params) {
  VencCore *core;
  Mpp *mpp;
  MppEncInitCfg initCfg;
  uint32_t packetBytes;

  *out = NULL;
  if (gOpen) return -1;
  if (params->codec != kVencH264) return -1;
  if (!params->width || !params->height || params->width > 4096 || params->height > 4096) return -1;

  venc_shim_bind(hw);
  if (venc_shim_run_singletons()) return -2;
  mpp_enc_hal_api_register(&hal_h264e_vepu580);

  core = (VencCore *)vec_calloc(sizeof *core);
  if (!core) return -3;
  core->hw = *hw;
  core->params = *params;
  // MPP's test tool: the rows of a planar picture are padded to 16 pixels (so the chroma rows get 8), semi-planar to 8; the plane is as tall as the picture, rounded up to 2
  core->hstride = params->hstride ? params->hstride : align_up(params->width, params->format == kVencI420 ? 16u : 8u);
  core->vstride = params->vstride ? params->vstride : align_up(params->height, 2u);
  // ... and the buffer it reads from is sized as that tool sizes it
  core->sourceBytes = align_up(core->hstride, 64u) * align_up(core->vstride, 64u) * 3 / 2;
  mpp = &core->mpp;
  mpp->mType = MPP_CTX_ENC;
  mpp->mCoding = MPP_VIDEO_CodingAVC;
  mpp->mInputTaskCount = 1;
  mpp->mOutputTaskCount = 8;
  mpp->mInputTimeout = MPP_POLL_BLOCK;
  mpp->mOutputTimeout = MPP_POLL_NON_BLOCK;
  mpp->mPktOut = mpp_list_create(NULL);
  mpp->mFrmIn = mpp_list_create(NULL);
  mpp_buffer_group_get_internal(&mpp->mPacketGroup, MPP_BUFFER_TYPE_ION);
  mpp_buffer_group_get_internal(&mpp->mFrameGroup, MPP_BUFFER_TYPE_ION);
  mpp_task_queue_init(&mpp->mInputTaskQueue, mpp, "input");
  mpp_task_queue_init(&mpp->mOutputTaskQueue, mpp, "output");
  mpp_task_queue_setup(mpp->mInputTaskQueue, mpp->mInputTaskCount);
  mpp_task_queue_setup(mpp->mOutputTaskQueue, mpp->mOutputTaskCount);
  mpp->mUsrInPort = mpp_task_queue_get_port(mpp->mInputTaskQueue, MPP_PORT_INPUT);
  mpp->mUsrOutPort = mpp_task_queue_get_port(mpp->mOutputTaskQueue, MPP_PORT_OUTPUT);
  mpp->mMppInPort = mpp_task_queue_get_port(mpp->mInputTaskQueue, MPP_PORT_OUTPUT);
  mpp->mMppOutPort = mpp_task_queue_get_port(mpp->mOutputTaskQueue, MPP_PORT_INPUT);
  core->frameGroup = mpp->mFrameGroup;
  core->packetGroup = mpp->mPacketGroup;

  initCfg.coding = MPP_VIDEO_CodingAVC;
  initCfg.task_cnt = mpp->mInputTaskCount;
  initCfg.mpp = mpp;
  if (mpp_enc_init_v2(&mpp->mEnc, &initCfg)) {
    venc_core_destroy(core);
    return -4;
  }
  core->enc = mpp->mEnc;
  gOpen = core;

  // the buffers: the picture the caller writes, and the access unit
  packetBytes = core->sourceBytes;   // (as MPP's test tool sizes it: as big as the source picture)
  if (mpp_buffer_get(core->frameGroup, &core->frameBuf, core->sourceBytes) || mpp_buffer_get(core->packetGroup, &core->packetBuf, packetBytes)) {
    venc_core_destroy(core);
    return -5;
  }

  if (params->motionInfo &&
      mpp_buffer_get(core->frameGroup, &core->mdBuf, (align_up(core->hstride, 64u) >> 6) * (align_up(core->vstride, 64u) >> 6) * 32)) {   // get_mdinfo_size() of the test tool, RK3588
    venc_core_destroy(core);
    return -5;
  }

  // the settings start from the encoder's own defaults (a setting that equals the value in the object is not a change, and is not applied)
  if (mpp_enc_cfg_init(&core->cfg) || mpp_enc_control_v2(core->enc, MPP_ENC_GET_CFG, core->cfg) || core_configure(core)) {
    venc_core_destroy(core);
    return -6;
  }
  *out = core;
  return 0;
}

void venc_core_destroy(VencCore *core) {
  Mpp *mpp;
  if (!core) return;
  mpp = &core->mpp;
  if (mpp->mEnc) {
    mpp_enc_deinit_v2(mpp->mEnc);
    mpp->mEnc = NULL;
  }
  if (core->cfg) mpp_enc_cfg_deinit(core->cfg);
  if (core->frameBuf) mpp_buffer_put(core->frameBuf);
  if (core->packetBuf) mpp_buffer_put(core->packetBuf);
  if (core->mdBuf) mpp_buffer_put(core->mdBuf);
  if (mpp->mInputTaskQueue) mpp_task_queue_deinit(mpp->mInputTaskQueue);
  if (mpp->mOutputTaskQueue) mpp_task_queue_deinit(mpp->mOutputTaskQueue);
  if (mpp->mPacketGroup) mpp_buffer_group_put(mpp->mPacketGroup);
  if (mpp->mFrameGroup) mpp_buffer_group_put(mpp->mFrameGroup);
  if (mpp->mPktOut) mpp_list_destroy(mpp->mPktOut);
  if (mpp->mFrmIn) mpp_list_destroy(mpp->mFrmIn);
  if (gOpen == core) gOpen = NULL;
  vec_free(core, sizeof *core);
}

void venc_core_set_observer(VencCore *core, VencJobObserver *observer) {
  MppEncImpl *enc = (MppEncImpl *)core->enc;
  venc_shim_set_observer(enc->dev, observer);
}

int venc_core_set_s32(VencCore *core, const char *name, int32_t value) {
  if (mpp_enc_cfg_set_s32(core->cfg, name, value)) return -1;
  return core_control(core, MPP_ENC_SET_CFG, core->cfg);
}

uint8_t *venc_core_source(VencCore *core, uint32_t *bytes) {
  if (bytes) *bytes = core->sourceBytes;
  return (uint8_t *)mpp_buffer_get_ptr(core->frameBuf);
}

void venc_core_stats(const VencCore *core, VencStats *stats) {
  *stats = core->stats;
  venc_shim_window_stats(NULL, NULL, &stats->windowPeakBytes);
}

int venc_core_header(VencCore *core, uint8_t *out, uint32_t capacity) {
  uint8_t store[512];
  MppPacket packet = NULL;
  int length;
  if (mpp_packet_init(&packet, store, sizeof store)) return -1;
  mpp_packet_set_length(packet, 0);
  if (core_control(core, MPP_ENC_GET_HDR_SYNC, packet)) {
    mpp_packet_deinit(&packet);
    return -1;
  }
  length = (int)mpp_packet_get_length(packet);
  if (length < 0 || (uint32_t)length > capacity) length = -1;
  else memcpy(out, mpp_packet_get_pos(packet), (size_t)length);
  mpp_packet_deinit(&packet);
  return length;
}

int venc_core_encode(VencCore *core, int64_t pts, int forceIdr, VencPacket *out) {
  Mpp *mpp = &core->mpp;
  MppEncImpl *enc = (MppEncImpl *)core->enc;
  MppFrame frame = NULL;
  MppPacket packet = NULL;
  MppTask task = NULL;
  MppMeta meta;
  int rc = -1;

  memset(out, 0, sizeof *out);
  if (forceIdr || core->needIdr) {
    core_control(core, MPP_ENC_SET_IDR_FRAME, NULL);
    core->needIdr = 0;
  }

  // the frame, with the output packet in its meta data (what MPP's test tool does)
  mpp_frame_init(&frame);
  mpp_frame_set_width(frame, core->params.width);
  mpp_frame_set_height(frame, core->params.height);
  mpp_frame_set_hor_stride(frame, core->hstride);
  mpp_frame_set_ver_stride(frame, core->vstride);
  mpp_frame_set_fmt(frame, core->params.format == kVencI420 ? MPP_FMT_YUV420P : MPP_FMT_YUV420SP);
  mpp_frame_set_pts(frame, pts);
  mpp_frame_set_buffer(frame, core->frameBuf);
  mpp_packet_init_with_buffer(&packet, core->packetBuf);
  mpp_packet_set_length(packet, 0);
  meta = mpp_frame_get_meta(frame);
  mpp_meta_set_packet(meta, KEY_OUTPUT_PACKET, packet);
  if (core->mdBuf) mpp_meta_set_buffer(meta, KEY_MOTION_INFO, core->mdBuf);

  // mpp_put_frame(): a task from the input port with the frame (and the packet) in it, back to the port
  if (!core->inputTask) {
    if (mpp_port_poll(mpp->mUsrInPort, MPP_POLL_NON_BLOCK) < 0 || mpp_port_dequeue(mpp->mUsrInPort, &core->inputTask) || !core->inputTask) goto done;
  }
  task = core->inputTask;
  mpp_task_meta_set_frame(task, KEY_INPUT_FRAME, frame);
  mpp_task_meta_set_packet(task, KEY_OUTPUT_PACKET, packet);
  if (core->mdBuf) mpp_task_meta_set_buffer(task, KEY_MOTION_INFO, core->mdBuf);
  if (mpp_port_enqueue(mpp->mUsrInPort, task)) goto done;
  core->inputTask = NULL;

  // the thread's body, once
  memset(&core->wait, 0, sizeof core->wait);
  if (try_get_enc_task(enc, &core->task, &core->wait)) {
    core->task.status.val = 0;
    core->needIdr = 1;
    core->stats.jobErrors++;
    goto done;
  }
  if (!core->task.status.enc_start) goto done;
  if (try_proc_normal_task(enc, &core->task)) {
    core->needIdr = 1;
    core->stats.jobErrors++;
    goto done;
  }
  core->stats.jobs++;

  // mpp_get_packet(): the result from the output port
  task = NULL;
  if (mpp_port_poll(mpp->mUsrOutPort, MPP_POLL_NON_BLOCK) < 0 || mpp_port_dequeue(mpp->mUsrOutPort, &task) || !task) goto done;
  {
    MppPacket result = NULL;
    MppMeta pm;
    RK_S32 intra = 0, qp = 0, idr = 0;
    mpp_task_meta_get_packet(task, KEY_OUTPUT_PACKET, &result);
    if (result) {
      out->data = (const uint8_t *)mpp_packet_get_pos(result);
      out->length = (uint32_t)mpp_packet_get_length(result);
      out->pts = pts;
      pm = mpp_packet_get_meta(result);
      if (pm) {
        mpp_meta_get_s32(pm, KEY_OUTPUT_INTRA, &intra);
        mpp_meta_get_s32(pm, KEY_ENC_AVERAGE_QP, &qp);
      }
      idr = core->task.rc.frm.is_idr;
      out->isIdr = idr != 0;
      out->isIntra = intra != 0 || idr != 0;
      out->qp = qp;
      out->hwOffset = core->task.task.header_length;
      core->stats.frames++;
      rc = 0;
    }
  }
  mpp_port_enqueue(mpp->mUsrOutPort, task);

done:
  // the input task comes back to the application (mpp_put_frame() polls and dequeues it again), then the frame is released
  if (!core->inputTask && mpp_port_poll(mpp->mUsrInPort, MPP_POLL_NON_BLOCK) >= 0) mpp_port_dequeue(mpp->mUsrInPort, &core->inputTask);
  if (core->inputTask) {   // the controller gave the frame back in the task: it is released below, and the task must not name it any more
    MppFrame back = NULL;
    mpp_task_meta_get_frame(core->inputTask, KEY_INPUT_FRAME, &back);
  }
  if (packet) mpp_packet_deinit(&packet);
  if (frame) mpp_frame_deinit(&frame);
  return rc;
}
