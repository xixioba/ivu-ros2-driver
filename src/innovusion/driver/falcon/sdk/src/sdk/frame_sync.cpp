/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include <algorithm>
#include <sstream>

#include "sdk/frame_sync.h"
// #include "frame_sync.h"
#include "utils/inno_lidar_log.h"
#include "utils/utils.h"
#include "sdk/lidar.h"
#include "sdk/reg.h"
#include "sdk/lidar_communication.h"

namespace innovusion {

FrameSync::FrameSync(double time_to_sync, InnoLidar *lidar)
  : lidar_(lidar) {
  target_sync_time_ns_ = (uint64_t)(time_to_sync * InnoConsts::kNsInSecond);
  sync_state_ = STATE_INIT;
  time_sync_cost_ = 0;
  time_start_to_sync_ = 0;
  it_sync_ = new InnoThread("frame-sync", 80, 1, sync_s_,
                            this, 0, NULL);
  inno_log_verify(it_sync_, "it_sync_");
  double frame_rate;
  lidar_->get_attribute("frame_rate", &frame_rate);
  frame_period_ns_ = (uint64_t)((1.0 / frame_rate) * InnoConsts::kNsInSecond);
  inno_log_verify(frame_period_ns_ != 0, "frame_period_ns != 0");

  ptp_clock_id_ = CLOCK_REALTIME;
  // todo we should only use /dev/ptp0 when ptp4l use hardware clock
#if 0
  if (lidar_->is_live_direct_memory_lidar_()) {
    ptp_dev_fd_ = open(PTP_DEV, O_RDWR);
    inno_log_verify(ptp_dev_fd_ >= 0, "opening %s\n", PTP_DEV);
    if (ptp_dev_fd_ >= 0) {
      ptp_clock_id_ = get_clockid_(ptp_dev_fd_);
    } else {
      inno_log_info("open " PTP_DEV " failed, use CLOCK_REALTIME as ptp clock");
    }
  } else {
    inno_log_info("not inner PCS, use CLOCK_REALTIME as ptp clock");
  }
#endif

  // init stats
  stats_frame_sync_diff_.reset();
  stats_fpga_fsync_ts_interval_.reset();
  stats_time_sync_drift_.reset();
  histogram_diff_ms_sz_ = frame_period_ns_ / kHistogramStepNs;
  histogram_diff_ms_ = reinterpret_cast<uint32_t *>
      (malloc(histogram_diff_ms_sz_ * sizeof(uint32_t)));
  inno_log_verify(histogram_diff_ms_, "histogram_diff_ms_");
  memset(histogram_diff_ms_, 0, histogram_diff_ms_sz_ * sizeof(uint32_t));
  last_big_offset_frame_ts_us_ = 0;
}

FrameSync::~FrameSync() {
  if (it_sync_) {
    delete it_sync_;
    it_sync_ = NULL;
  }
  if (histogram_diff_ms_) {
    free(histogram_diff_ms_);
    histogram_diff_ms_ = NULL;
  }
  if (ptp_dev_fd_ >= 0) {
    close(ptp_dev_fd_);
  }
}

void *FrameSync::sync_s_(void *ctx) {
  auto frame_sync = reinterpret_cast<FrameSync *>(ctx);
  frame_sync->sync_loop_();
  return NULL;
}

/**
 * before first time sync
 *
 * For example, user set next frame sync time is 111.900s, now is 110.123s
 * and FPGA's clock is 100000
 * We should tell FPGA at clock (111.900 - 110.123) = 0.777s / 8ns + 100000, it
 * should send a sync signal to DSP.
 *
 * We set register 2E4 2E8 to tell FPGA to do this.
 * Main steps:
 *   1. Calculate the <diff> of user set and current real system time.
 *   2. Get <FPGA's current clock> from register 1E4 and 1E8
 *   3. The clock should be set is <FPGA's current clock> + <diff>
 *   4. Set register 2E4 and 2E8
 *
 * There are several constraints:
 *   1. There must enough margin before we set register value to avoid set a
 *     clock after FPGA's real clock
 *   2. We should do this periodically (every 1 second).
 *
 * For do sync periodically:
 *   1. Base on time_set_ and frame_rate, we can calculate start time of each
 *     frame.
*/
void *FrameSync::sync_loop_() {
  inno_log_info("time sync thread start, user set ts_ns=%" PRI_SIZEU,
                target_sync_time_ns_);
  while (!it_sync_->has_shutdown()) {
    enum InnoTimeSyncType state = lidar_->get_clock().
        get_sync_state_and_diff(&last_effective_diff_sys_fpga_);
    if (state == INNO_TIME_SYNC_TYPE_GPS_LOCKED ||
    state == INNO_TIME_SYNC_TYPE_PTP_LOCKED ||
    state == INNO_TIME_SYNC_TYPE_NTP_LOCKED) {
      break;
    } else {
      it_sync_->timed_wait(InnoConsts::kUsInSecond);
    }
  }

  // here we should get LiDAR's time instead of the local system time.
  // uint64_t cur_sys_time = InnoUtils::get_time_ns(ptp_clock_id_);
  uint64_t cur_sys_time = get_lidar_ptp_clock_till_success();
  time_start_to_sync_ = InnoUtils::get_time_ns(ptp_clock_id_);
  if (target_sync_time_ns_ < cur_sys_time) {
    // target_sync_time_ns_ too small
    target_sync_time_ns_ +=
        ((cur_sys_time - target_sync_time_ns_) / frame_period_ns_
            + 1) * frame_period_ns_;
    inno_log_info("current ptp clock is %" PRI_SIZEU ","
                  " ts_ns set by user is too small,"
                  " adjusted to %" PRI_SIZEU "",
                  cur_sys_time, target_sync_time_ns_);
  }

  uint64_t loop_t0 = InnoUtils::get_time_ns(CLOCK_MONOTONIC_RAW);
  uint64_t N = 0;
  // sync every 10 frames
  int64_t sync_period_ns = kSyncPeriodNs;
  // min margin = 1 frame period
  uint64_t kMinMarginToSyncInNs = frame_period_ns_;
  // max margin = 2 frame periods
  uint64_t kMaxMarginToSyncInNs = 2 * frame_period_ns_;
  while (!it_sync_->has_shutdown()) {
    uint64_t time_loop_start = InnoUtils::get_time_ns(CLOCK_MONOTONIC_RAW);
    int64_t sync_time_err =
        static_cast<int64_t>(time_loop_start) - loop_t0 - N * sync_period_ns;
    N++;
    uint64_t target_sleep_ns =
        static_cast<uint64_t>(sync_period_ns - sync_time_err);

    // update next time_to_sync
    target_sync_time_ns_ += sync_period_ns;

    // calculate fpga_fsync_ts_ns_
    // If time sync drift is too big, say greater than 1ms, we need re-calculate
    // fpga_fsync_ts, or we just accumulate fpga_fsync_ts with one sync_period.
    // If time sync is not locked, we will not re-calculate fpga_fsync_ts
    double diff_sys_fpga;
    enum InnoTimeSyncType state =
        lidar_->get_clock().get_sync_state_and_diff(&diff_sys_fpga);
    // currently, we re-calculate every sync period if time sync was locked
    bool time_sync_locked =
        state == INNO_TIME_SYNC_TYPE_GPS_LOCKED ||
        state == INNO_TIME_SYNC_TYPE_PTP_LOCKED ||
        state == INNO_TIME_SYNC_TYPE_NTP_LOCKED;
    // time_sync_locked &= sync_count_ == 0 ||
    //   sys_time_sync_drift_too_big(diff_sys_fpga);
    if (time_sync_locked) {
      // re-calculate fpga_fsnyc_ts = cur_sys_ts - diff_sys_fpga
      // check if the diff drift too much, and update last effective diff
      check_sys_time_sync_drift_too_big_(diff_sys_fpga);
    } else {
      inno_log_warning("use last effective sys fpga ts diff %lf to calculate "
                       "fpga fsync_ts since time sync is not locked.",
                       last_effective_diff_sys_fpga_);
      diff_sys_fpga = last_effective_diff_sys_fpga_;
    }
    fpga_fsync_ts_ns_ = target_sync_time_ns_ -
        (uint64_t)(diff_sys_fpga * InnoConsts::kNsInSecond);
    inno_log_trace("will set fpga_fsync_ts_ns_=%" PRI_SIZEU ","
                   " diff_sys_fpga=%lf",
                  fpga_fsync_ts_ns_, diff_sys_fpga);

    // if fpga_fsync_ts_ns is not in (min_margin, max_margin), adjust it
    uint64_t fpga_ts_ns;
    if (lidar_->get_fpga_ts(&fpga_ts_ns)) {
      inno_log_warning("get fpga ts failed, but will still try to set"
                       " fsync ts to %" PRI_SIZEU, fpga_fsync_ts_ns_);
    } else {
      uint64_t time_diff = 0;
      if (fpga_fsync_ts_ns_ <= fpga_ts_ns + kMinMarginToSyncInNs) {
        time_diff =
            ((fpga_ts_ns + kMinMarginToSyncInNs - fpga_fsync_ts_ns_)
            / frame_period_ns_ + 1) * frame_period_ns_;
        if (sync_count_ > 0) {
          // if it is not the first time to sync, it is expected that we needn't
          // adjust fpga_fsync_ts_ns_, because the sync_period should be very
          // stable.
          inno_log_warning("[frame sync phase drift] fpga_fsync_ts_ns_"
                           " too small, fpga_fsync_ts_ns_=%" PRI_SIZED ","
                           " fpga_ts_ns=%" PRI_SIZED ","
                           " will drift %" PRI_SIZED " ns to the right by",
                           fpga_fsync_ts_ns_, fpga_ts_ns, time_diff);
        }
        fpga_fsync_ts_ns_ += time_diff;
        target_sync_time_ns_ += time_diff;
      } else if (fpga_fsync_ts_ns_ >= fpga_ts_ns + kMaxMarginToSyncInNs) {
        time_diff =
            ((fpga_fsync_ts_ns_ - fpga_ts_ns - kMaxMarginToSyncInNs)
            / frame_period_ns_ + 1) * frame_period_ns_;
        if (sync_count_ > 0) {
          inno_log_warning("[frame sync phase drift] fpga_fsync_ts_ns_"
                           " too big, fpga_fsync_ts_ns_=%" PRI_SIZED ","
                           " fpga_ts_ns=%" PRI_SIZED ","
                           " will drift %" PRI_SIZED " ns to the left",
                           fpga_fsync_ts_ns_, fpga_ts_ns, time_diff);
        }
        fpga_fsync_ts_ns_ -= time_diff;
        target_sync_time_ns_ -= time_diff;
      }
    }

    // set fsync_ts register
    uint64_t final_fsync_value = fpga_fsync_ts_ns_ - kDspTuneErrNs;
    // todo add a compensation ?
    if (lidar_->set_fpga_frame_sync_ts(final_fsync_value)) {
      inno_log_error("set fpga fsync_ts failed.");
    } else {
      inno_log_trace("set fsync_ts to %" PRI_SIZEU ","
                     " last_diff_sys_fpga=%" PRI_SIZEU ","
                     " corresponding_sys_time=%" PRI_SIZEU "",
                     fpga_fsync_ts_ns_,
                     final_fsync_value,
                     fpga_fsync_ts_ns_
                     + (uint64_t)(last_effective_diff_sys_fpga_
                     * InnoConsts::kNsInSecond));
      sync_count_++;
    }

    uint64_t time_loop_end = InnoUtils::get_time_ns(CLOCK_MONOTONIC_RAW);
    uint64_t elapsed_ns = time_loop_end - time_loop_start;
    if (target_sleep_ns > elapsed_ns) {
      it_sync_->timed_wait((target_sleep_ns - elapsed_ns) / 1000);
    }
  }
  return NULL;
}

void FrameSync::start() {
  inno_log_verify(it_sync_, "it_sync_");
  it_sync_->start();
}

// caller should avoid race condition with start()
void FrameSync::stop() {
  if (it_sync_ && !it_sync_->has_shutdown()) {
    it_sync_->shutdown();
  }
}

bool FrameSync::check_sys_time_sync_drift_too_big_(double diff_sys_fpga) {
  double time_sync_drift = diff_sys_fpga - last_effective_diff_sys_fpga_;
  if (time_sync_drift < 0) {
    time_sync_drift = -time_sync_drift;
  }
  last_effective_diff_sys_fpga_ = diff_sys_fpga;
  stats_time_sync_drift_.add(time_sync_drift);
  if (time_sync_drift >= kTimeSyncDriftThreshold) {
    inno_log_warning("diff_sys_fpga drift = %lf", time_sync_drift);
    return true;
  } else {
    return false;
  }
}

/**
 * DSP will send the status to FPAG, we can read it from PS reg B4[22]
 * @return
 */
void FrameSync::check_sync_status_() {
  // start to check at 1 second after frame sync start
  uint64_t cur_time = InnoUtils::get_time_ns(ptp_clock_id_);
  if (time_start_to_sync_ == 0
      || cur_time - time_start_to_sync_ <= InnoConsts::kNsInSecond) {
    return;
  }
  uint32_t value = 0;
  int ret = lidar_->get_dsp_packet_reg(&value);
  if (ret) {
    inno_log_error("read frame sync status from ps register failed: ret=%d",
                   ret);
    return;
  }
  bool synced = value & PS_DSP_PACKET_FRAME_SYNC_STATUS;
  switch (sync_state_) {
    case STATE_INIT:
      if (synced) {
        sync_state_ = STATE_SYNCED;
        // init to sync state, record sync time cost
        time_sync_cost_ = cur_time - time_start_to_sync_;
        inno_log_info("frame sync state changed: init->sync");
      }
      break;
    case STATE_LOST:
      if (synced) {
        sync_state_ = STATE_SYNCED;
        inno_log_info("frame sync state changed: lost->sync");
      }
    break;
    case STATE_SYNCED:
      if (!synced) {
        sync_state_ = STATE_LOST;
        inno_log_error("frame sync state changed: sync->lost");
      }
    break;
    default:
      break;
  }
}

// accurate to ms
bool FrameSync::in_step_with(double ts_s) const {
  uint64_t new_step =
      ((uint64_t)(ts_s * InnoConsts::kNsInSecond) % frame_period_ns_)
      / kNsInMs;
  uint64_t current_step = (target_sync_time_ns_ % frame_period_ns_) / kNsInMs;
  return new_step == current_step;
}

void FrameSync::frame_start_time_statistic(InnoDataPacket *packet) {
  check_sync_status_();
  if (sync_state_ <= STATE_INIT) {
    return;
  }

  uint64_t real_frame_start =
      static_cast<uint64_t>(packet->common.ts_start_us) * 1000;
  inno_log_trace("last_sync_time_ns=%" PRI_SIZEU ","
                 " real_frame_start_ns=%" PRI_SIZEU,
                 target_sync_time_ns_, real_frame_start);
  uint64_t abs_diff_ns;
  if (real_frame_start > target_sync_time_ns_) {
    abs_diff_ns = (real_frame_start - target_sync_time_ns_) % frame_period_ns_;
  } else {
    abs_diff_ns = (target_sync_time_ns_ - real_frame_start) % frame_period_ns_;
  }
  auto diff = static_cast<double>(abs_diff_ns);
  int32_t idx = 500;
  // abs diff 50 ms is -50 ms diff
  if (abs_diff_ns >= (frame_period_ns_ >> 1)) {
    diff = -static_cast<double>(frame_period_ns_ - abs_diff_ns);
    abs_diff_ns = frame_period_ns_ - abs_diff_ns;
  }

  stats_frame_sync_diff_.add(diff);
  // abs_diff_ns always less than kNsInMs in normal case
  int idx_off = static_cast<int64_t >(abs_diff_ns) / kHistogramStepNs;
  idx += diff > 0 ? idx_off : -idx_off;
  if (abs_diff_ns >= kNsInMs) {
    last_big_offset_frame_ts_us_ = packet->common.ts_start_us;
  }
  if (idx >= 0 && static_cast<uint64_t>(idx) < histogram_diff_ms_sz_) {
    histogram_diff_ms_[idx]++;
  } else {
    inno_log_warning("histogram out of boundary, abs_diff_ns=%" PRI_SIZEU ","
                     " histogram_diff_ms_sz_=%u",
                     abs_diff_ns, histogram_diff_ms_sz_);
  }
  // only publish locked or not locked
  packet->frame_sync_locked = (sync_state_ == STATE_SYNCED);
}

void FrameSync::print_stats() const {
  char buf[8192] = {};
  get_stats_string(buf, sizeof(buf));
  inno_log_info("%s", buf);
}

void FrameSync::get_stats_string(char *buf, size_t buf_size) const {
  std::ostringstream hist_ss;
  uint32_t i = 0;
  double cur_diff_fpga_sys;
  enum InnoTimeSyncType state = lidar_->get_clock().
      get_sync_state_and_diff(&cur_diff_fpga_sys);
  for (; i < histogram_diff_ms_sz_ - 1; i++) {
    hist_ss << histogram_diff_ms_[i] << ", ";
  }
  hist_ss << histogram_diff_ms_[i];
  int ret = snprintf(buf, buf_size, "{"
              "\"frame_sync_stats\":{"
                "\"sync_status\":{"
                  "\"value\":%d,"
                  "\"time_sync_start_us\":%" PRI_SIZEU ","
                  "\"cost_time_ms\":%" PRI_SIZEU ","
                  "\"desc\":\"value: 0-init; 1-synced; 2-lost."
                  "cost_time_ms is time from init to synced\""
                "},"
                "\"frame_start_diff_ns\":{"
                  "\"mean\":%.0lf,"
                  "\"max\":%.0lf,"
                  "\"std_dev\":%.0lf,"
                  "\"count\":%" PRI_SIZEU ","
                  "\"histogram_diff\":[%s],"
                  "\"last_big_offset_frame_ts_us\":%.3lf,"
                  "\"desc\":\"Diff between frame start time and the"
                  " use set expected start time. The domain of"
                  " histogram is [-500, 499] in 100 us."
                  " Start to statistic after synced."
                  " last_big_offset_frame_ts_us is last frame whose offset"
                  " to expected start time is more than 1 ms.\""
                "}"
              "},"
              "\"time_sync_stats\":{"
                "\"time_sync_diff_drift_s\":{"
                  "\"mean\":%.6lf,"
                  "\"max\":%.6lf,"
                  "\"std_dev\":%.6lf,"
                  "\"count\":%" PRI_SIZEU ","
                  "\"desc\":\"After frame sync started, the diff"
                  " between raw clock and ptp clock will be checked"
                  " every 1 second. time_sync_diff_drift means the"
                  " diff between this time of raw-ptp clock diff and last"
                  " time. This indicates whether the ptp clock is stable.\""
                "},"
                "\"time_sync_status\":%d"
              "}"
            "}",
            sync_state_,
            time_start_to_sync_ / 1000,
            time_sync_cost_ / 1000000,
            stats_frame_sync_diff_.mean(),
            stats_frame_sync_diff_.max(),
            stats_frame_sync_diff_.std_dev(),
            stats_frame_sync_diff_.count(),
            hist_ss.str().c_str(),
            last_big_offset_frame_ts_us_,
            stats_time_sync_drift_.mean(),
            stats_time_sync_drift_.max(),
            stats_time_sync_drift_.std_dev(),
            stats_time_sync_drift_.count(),
            state);
  if (ret >= ssize_t(buf_size)) {
    buf[buf_size - 1] = 0;
    inno_log_warning("buffer size too small: ret=%d", ret);
  }
}

int FrameSync::config_dsp_() {
  int ret = 0;
  double polygon_rpm = lidar_->get_motor_speed_config();
  if (polygon_rpm < 0) {
    inno_log_error("Start to do frame sync failed: get motor speed failed");
    return 1;
  }
  int polygon_rpf = static_cast<int>
      (polygon_rpm * kSyncPeriodNs
      / InnoConsts::kNsInSecond / InnoConsts::kSecondInMinute);
  // set polygon rpf and enable frame sync in dsp
  ret = lidar_->comm_->send_command_and_free_reply("scanh_tran STW01800001ND");
//  std::string cmd_set_dsp_polygon_rpf = "scanh_tran STW019000";
//  cmd_set_dsp_polygon_rpf +=  std::to_string(polygon_rpf);
//  cmd_set_dsp_polygon_rpf += "ND";
  char cmd_set_dsp_polygon_rpf[50] = {};
  uint32_t sn = snprintf(cmd_set_dsp_polygon_rpf,
                         sizeof(cmd_set_dsp_polygon_rpf),
                         "scanh_tran STW019%05dND", polygon_rpf);
  inno_log_verify(sn < sizeof(cmd_set_dsp_polygon_rpf),
                  "cmd_set_dsp_polygon_rpf size too small: sn=%d", sn);
  cmd_set_dsp_polygon_rpf[sn] = 0;
  ret |= lidar_->comm_
      ->send_command_and_free_reply("%s", cmd_set_dsp_polygon_rpf);
  // galvo sync enable
  ret |= lidar_->comm_
      ->send_command_and_free_reply("scanh_tran STW01200001ND");
  // make config effective
  ret |= lidar_->comm_
      ->send_command_and_free_reply("scanh_tran STW02000001ND");
  if (ret) {
    inno_log_error("Start frame sync failed: set polygon rpf failed.");
    return 3;
  } else {
    inno_log_info("set FRAME_SYNC_POLYGON_RPF=%05d, set command:%s",
                  polygon_rpf, cmd_set_dsp_polygon_rpf);
  }
  return 0;
}

clockid_t FrameSync::get_clockid_(int fd) {
#define CLOCKFD 3
  return static_cast<clockid_t>((((unsigned int) ~fd) << 3) | CLOCKFD);
}

uint64_t FrameSync::get_lidar_ptp_clock_till_success() {
  // if internal PCS we just get the time from ptp_clockid_
  if (lidar_->is_live_direct_memory_lidar_()) {
    return InnoUtils::get_time_ns(ptp_clock_id_);
  } else {
    // if external PCS we get the diff and the current FPGA clock
    bool time_sync_locked = false;
    uint64_t fpga_clk = 0;
    double diff_sys_fpga = 0;
    while (!time_sync_locked) {
      enum InnoTimeSyncType state =
          lidar_->get_clock().get_sync_state_and_diff(&diff_sys_fpga);
      // currently, we re-calculate every sync period if time sync was locked
      time_sync_locked = state == INNO_TIME_SYNC_TYPE_GPS_LOCKED ||
                         state == INNO_TIME_SYNC_TYPE_PTP_LOCKED ||
                         state == INNO_TIME_SYNC_TYPE_NTP_LOCKED;
      if (!time_sync_locked) {
        inno_log_info("time sync is not locked, retry 1 second later");
        it_sync_->timed_wait(InnoConsts::kUsInSecond);
        continue;
      }
      int ret = 1;
      while (ret) {
        ret = lidar_->get_fpga_ts(&fpga_clk);
        if (ret) {
          inno_log_info("get fpga ts failed, will retry 1 second later");
          it_sync_->timed_wait(InnoConsts::kUsInSecond);
        }
      }
    }
    return fpga_clk
           + static_cast<uint64_t>(diff_sys_fpga * InnoConsts::kNsInSecond);
  }
}

}  // namespace innovusion
