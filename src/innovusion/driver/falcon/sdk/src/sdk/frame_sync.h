/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */
#ifndef SDK_FRAME_SYNC_H_
#define SDK_FRAME_SYNC_H_

#include "utils/inno_thread.h"
#include "utils/utils.h"
#include "sdk_common/inno_lidar_packet.h"

#define CLOCK_INVALID (-1)
#define PTP_DEV "/dev/ptp0"

namespace innovusion {
class InnoLidar;

class FrameSync {
 public:
  static FrameSync* create(double time_to_sync, InnoLidar *lidar) {
    auto *inst = new FrameSync(time_to_sync, lidar);
    if (!inst) {
      return NULL;
    }
    if (inst->config_dsp_()) {
      delete inst;
      return NULL;
    }
    return inst;
  }

 public:
  /**
   * enter INIT after started
   * enter SYNCED when sync_status_bit = 1, from INIT or LOST
   * enter LOST when sync_status_bit = 0, only from INIT
   */
  enum State {
    STATE_INIT = 0,
    STATE_SYNCED = 1,
    STATE_LOST,
  };

 public:
  // 2.5 ms?
  static const uint64_t kDspTuneErrNs = 2423000L;
  // sync period in frame
  static const uint64_t kSyncPeriodInFrame = 10;
  // sync period in ns
  static const uint64_t kSyncPeriodNs = InnoConsts::kNsInSecond;
  static constexpr double kTimeSyncDriftThreshold = 0.001;
  static const uint64_t kNsInMs = 1000000L;
  static const uint64_t kHistogramStepNs = kNsInMs / 10;

 public:
  explicit FrameSync(double time_to_sync, InnoLidar *lidar);
  ~FrameSync();
  void start();
  void stop();

  bool in_step_with(double ts_s) const;
  void frame_start_time_statistic(InnoDataPacket *packet);
  void print_stats() const;
  void get_stats_string(char *buf, size_t buf_size) const;

 private:
  static void *sync_s_(void *ctx);
  void *sync_loop_();
  bool check_sys_time_sync_drift_too_big_(double diff_sys_fpga);
  void check_sync_status_();
  int config_dsp_();
  static clockid_t get_clockid_(int fd);
  uint64_t get_lidar_ptp_clock_till_success();

 private:
  uint64_t target_sync_time_ns_{};
  InnoThread *it_sync_{NULL};
  InnoLidar *lidar_{NULL};
  // check sync state from PS reg B4[22]
  State sync_state_;
  uint64_t time_sync_cost_{0};
  uint64_t time_start_to_sync_{0};
  uint64_t frame_period_ns_{};
  uint64_t sync_count_{0};
  // timestamp to be set to fpga fsync_ts
  uint64_t fpga_fsync_ts_ns_{0};
  double last_effective_diff_sys_fpga_{0};
  int ptp_dev_fd_{-1};
  clockid_t ptp_clock_id_{static_cast<clockid_t>(CLOCK_INVALID)};
  // stats
  InnoMean stats_frame_sync_diff_;
  InnoMean stats_fpga_fsync_ts_interval_;
  InnoMean stats_time_sync_drift_;

  // histogram
  uint32_t histogram_diff_ms_sz_;
  uint32_t *histogram_diff_ms_;
  double last_big_offset_frame_ts_us_{0};
};

}  // namespace innovusion

#endif  // SDK_FRAME_SYNC_H_
