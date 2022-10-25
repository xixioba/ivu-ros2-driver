/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_STAGE_NOISE_FILTER_H_
#define SDK_STAGE_NOISE_FILTER_H_

#include <mutex>  // NOLINT
#include <string>
#include <deque>

#include "sdk_common/inno_lidar_packet.h"
#include "sdk/stage_angle_job.h"
#include "utils/config.h"

namespace innovusion {
class InnoLidar;
class StageNoiseFilterConfig: public Config {
 public:
  StageNoiseFilterConfig() : Config() {
    roi_pattern = 2;
    firing_cycle = 2;
    noise_filtering = 2;
    filter_threshold = 0.5 * kInnoDistanceUnitPerMeter;  // 0.5m
    noise_filtering_2 = 2;
    filter_threshold_2 =  0.5 * kInnoDistanceUnitPerMeter;  // 0.5m
    road_point_noise_filtering = 2;
    road_point_filter_threshold = 3 * kInnoDistanceUnitPerMeter;  // 3m
    noise_2d_mode = 1;
    noise_2d_mode_sparse = 1;
    noise_2d_mode_sparse_2 = 0;
    noise_filtering_s = 2;
    filter_threshold_s = 2 * kInnoDistanceUnitPerMeter;  // 2m
    noise_filtering_2_s = 2;
    filter_threshold_2_s = 0.5 * kInnoDistanceUnitPerMeter;  // 0.5m
    rate_difference = 24;
    near_field_low_angle = 0;  // sinf(FROM_DEGREES(-15.0));
    near_field_high_angle = 0;  // sinf(FROM_DEGREES(15.0));
    near_field_distance =  80 * kInnoDistanceUnitPerMeter;  // 80m
    near_field_intensity = 100;
    near_field_noise_filtering = 2;
    near_field_filter_threshold = 0.5 * kInnoDistanceUnitPerMeter;  // 0.5m
    raw_output = 0;
    noise_filter_is_dynamic = 1;
    noise_filter_2d_to_1d = 10000;
    noise_filter_1d_to_2d = 10000;
    force_filter_multi_return = 1;
  }

  const char* get_type() const override {
    return "Lidar_StageNoiseFilter";
  }

  int set_key_value_(const std::string &key,
                             double value) override {
    if (strcmp(key.c_str(), "filter_threshold") == 0 ||
        strcmp(key.c_str(), "filter_threshold_2") == 0 ||
        strcmp(key.c_str(), "road_point_filter_threshold") == 0 ||
        strcmp(key.c_str(), "filter_threshold_s") == 0 ||
        strcmp(key.c_str(), "filter_threshold_2_s") == 0 ||
        strcmp(key.c_str(), "near_field_distance") == 0 ||
        strcmp(key.c_str(), "near_field_filter_threshold") == 0) {
      value *= kInnoDistanceUnitPerMeter;
    }
    SET_CFG(roi_pattern);
    SET_CFG(firing_cycle);
    SET_CFG(noise_filtering);
    SET_CFG(filter_threshold);
    SET_CFG(noise_filtering_2);
    SET_CFG(filter_threshold_2);
    SET_CFG(road_point_noise_filtering);
    SET_CFG(road_point_filter_threshold);
    SET_CFG(noise_2d_mode);
    SET_CFG(noise_2d_mode_sparse);
    SET_CFG(noise_2d_mode_sparse_2);
    SET_CFG(noise_filtering_s);
    SET_CFG(filter_threshold_s);
    SET_CFG(noise_filtering_2_s);
    SET_CFG(filter_threshold_2_s);
    SET_CFG(rate_difference);
    SET_CFG(near_field_low_angle);
    SET_CFG(near_field_high_angle);
    SET_CFG(near_field_distance);
    SET_CFG(near_field_intensity);
    SET_CFG(near_field_noise_filtering);
    SET_CFG(near_field_filter_threshold);
    SET_CFG(raw_output);
    SET_CFG(noise_filter_is_dynamic);
    SET_CFG(noise_filter_2d_to_1d);
    SET_CFG(noise_filter_1d_to_2d);
    SET_CFG(force_filter_multi_return);
    return -1;
  }

  int set_key_value_(const std::string &key,
                     const std::string value) override {
    // no string attribute
    return -1;
  }

  BEGIN_CFG_MEMBER()
  int32_t roi_pattern;
  int32_t firing_cycle;
  int32_t noise_filtering;
  int32_t filter_threshold;
  int32_t noise_filtering_2;
  int32_t filter_threshold_2;
  int32_t road_point_noise_filtering;
  int32_t road_point_filter_threshold;
  int32_t noise_2d_mode;
  int32_t noise_2d_mode_sparse;
  int32_t noise_2d_mode_sparse_2;
  int32_t noise_filtering_s;
  int32_t filter_threshold_s;
  int32_t noise_filtering_2_s;
  int32_t filter_threshold_2_s;
  int32_t rate_difference;
  int32_t near_field_low_angle;
  int32_t near_field_high_angle;
  int32_t near_field_distance;
  int32_t near_field_intensity;
  int32_t near_field_noise_filtering;
  int32_t near_field_filter_threshold;
  int32_t raw_output;
  int32_t noise_filter_is_dynamic;
  int32_t noise_filter_2d_to_1d;
  int32_t noise_filter_1d_to_2d;
  int32_t force_filter_multi_return;
  END_CFG_MEMBER()
};

class WorkerTraversalPattern {
 public:
  RawBlock *const *line_start;
  const int *line_length;
  int32_t first_line[2];  // for 2 firing cycles
  int32_t last_line[2];
  uint32_t filter_return_times;
  int32_t firing_cycle;
};

class StageNoiseFilter {
  friend InnoLidar;

 public:
  static int process(void *job, void *ctx, bool prefer);

 public:
  explicit StageNoiseFilter(InnoLidar *l, int phase);
  ~StageNoiseFilter(void);

 public:
  void print_stats(void) const;
  void get_stats_string(char *buf, size_t buf_size) const;
  bool noise_level_high() {
    std::unique_lock<std::mutex> lk(mutex_);
    return noise_level_high_;
  }

 private:
  const char *get_name_(void) const;
  void set_neighbor_range_(RawBlock *block,
                           int firing_cycle,
                           int *p_start,
                           int *p_end,
                           int *p_step);
  void set_filter_params_(RawBlock *block,
                          RawChannelPoint *point,
                          int distance,
                          int firing_cycle,
                          int *noise_filtering,
                          int *filter_threshold);
  int process_job_(StageAngleJob *job, bool prefer);

 private:
  static RawBlock paddingline_[ScanLine::kMaxBlocksBetweenP];
  static const uint32_t kExcessiveNoiseSetThreshold = 20000;
  static const uint32_t kExcessiveNoiseHealThreshold = 5000;
  static const uint32_t kTotalPointsThresholdCheckNoise = 45000;

 private:
  int phase_;
  InnoLidar *lidar_;
  std::mutex mutex_;
  std::deque<StageAngleJob *> sc_deque_;

  StageNoiseFilterConfig config_base_;
  StageNoiseFilterConfig config_;

  InnoMean mean_noise_points_;
  InnoMean mean_real_points_;
  InnoMeanLite mean_noise_intensity_[2][kInnoChannelNumber];
  InnoMeanLite mean_noise_distance_[2][kInnoChannelNumber];
  uint32_t stats_revive_points_[kInnoChannelNumber];
  uint32_t stats_real_points_[2][kInnoChannelNumber];
  uint32_t stats_revive_points_per_frame_[kInnoChannelNumber];
  uint32_t stats_noise_points_per_frame_[2][kInnoChannelNumber];
  uint32_t stats_real_points_per_frame_[2][kInnoChannelNumber];
  uint32_t stats_mean_noise_intensity_per_frame_[2][kInnoChannelNumber];
  uint32_t stats_std_dev_noise_intensity_per_frame_[2][kInnoChannelNumber];
  uint32_t stats_mean_noise_distance_per_frame_[2][kInnoChannelNumber];
  uint32_t stats_std_dev_noise_distance_per_frame_[2][kInnoChannelNumber];

  size_t stats_frames_;
  size_t stats_job_count_;
  size_t stats_jobs_dropped_;
  size_t stats_jobs_skipped_;
  size_t stats_lost_frames_;
  size_t last_frame_idx_;
  size_t total_noisy_frame_;

  bool print_noise_frame_err_{true};
  bool noise_level_high_ = true;

  bool force_use_1d_;
  int noise_2d_mode_;
  int noise_2d_mode_sparse_;
  int noise_2d_mode_sparse_2_;
  void do_noise_filter_(int start_channel, int end_channel,
                        const WorkerTraversalPattern traversal_pattern);
};

}  // namespace innovusion

#endif  // SDK_STAGE_NOISE_FILTER_H_
