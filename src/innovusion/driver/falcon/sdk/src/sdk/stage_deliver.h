/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_STAGE_DELIVER_H_
#define SDK_STAGE_DELIVER_H_

#include <mutex>  // NOLINT
#include <string>

#include "sdk/stage_angle_job.h"
#include "utils/config.h"
#include "sdk/lidar.h"
#include "sdk/misc_tables.h"

namespace innovusion {
class InnoLidar;

class StageDeliverConfig: public Config {
 public:
  StageDeliverConfig() : Config() {
    published_frames = 0;
    published_halves = 15;
    firing_cycle = 2;
    max_intensity = 8192;
    max_packet_size = 8192;  // 1470;
    roi_pattern = 2;
    flyback_angle_threshold = 1.0 * kInnoAngleUnitPerDegree;
    hori_roi_size = 40.0 * kInnoAngleUnitPerDegree / 2;
    min_distance = 1.3 * kInnoDistanceUnitPerMeter;  // 1.3m
    max_distance = 510 * kInnoDistanceUnitPerMeter;  // 510m
    latency_low_threshold_ms = 30;                   // 30 ms
    latency_high_threshold_ms = 100;  // normal value 46ms short and cali 40ms
    channel_as_intensity = 0;
    blooming_up_distance_diff = 0.5 * kInnoDistanceUnitPerMeter;
    blooming_down_distance_diff = 2.5 * kInnoDistanceUnitPerMeter;
    blooming_min_vert_angle_diff = 0.5 * kInnoAngleUnitPerDegree;
    blooming_max_vert_angle_diff = 10.0 * kInnoAngleUnitPerDegree;
    retro_distance_diff = 1.0 * kInnoDistanceUnitPerMeter;
    retro_count_threshold = 20;
  }

  const char* get_type() const override {
    return "Lidar_StageDeliver";
  }

  int set_key_value_(const std::string &key,
                             double value) override {
    if (strcmp(key.c_str(), "hori_roi_size") == 0) {
      value *= kInnoAngleUnitPerDegree / 2;
    } else if (strcmp(key.c_str(), "flyback_angle_threshold") == 0 ||
               strcmp(key.c_str(), "blooming_min_vert_angle_diff") == 0 ||
               strcmp(key.c_str(), "blooming_max_vert_angle_diff") == 0) {
      value *= kInnoAngleUnitPerDegree;
    } else if (strcmp(key.c_str(), "min_distance") == 0 ||
               strcmp(key.c_str(), "max_distance") == 0 ||
               strcmp(key.c_str(), "blooming_up_distance_diff") == 0 ||
               strcmp(key.c_str(), "blooming_down_distance_diff") == 0 ||
               strcmp(key.c_str(), "retro_distance_diff") == 0) {
      value *= kInnoDistanceUnitPerMeter;
    }
    SET_CFG(published_frames);
    SET_CFG(published_halves);
    SET_CFG(firing_cycle);
    SET_CFG(max_intensity);
    SET_CFG(max_packet_size);
    SET_CFG(roi_pattern);
    SET_CFG(hori_roi_size);
    SET_CFG(min_distance);
    SET_CFG(max_distance);
    SET_CFG(channel_as_intensity);
    SET_CFG(latency_low_threshold_ms);
    SET_CFG(latency_high_threshold_ms);
    SET_CFG(blooming_up_distance_diff);
    SET_CFG(blooming_down_distance_diff);
    SET_CFG(blooming_min_vert_angle_diff);
    SET_CFG(blooming_max_vert_angle_diff);
    SET_CFG(retro_distance_diff);
    SET_CFG(retro_count_threshold);
    return -1;
  }

  int set_key_value_(const std::string &key,
                     const std::string value) override {
    // no string attribute
    return -1;
  }

  BEGIN_CFG_MEMBER()
  uint32_t published_frames;
  uint32_t published_halves;
  uint32_t firing_cycle;
  uint32_t max_intensity;
  uint32_t max_packet_size;
  uint32_t roi_pattern;
  uint32_t hori_roi_size;
  uint32_t flyback_angle_threshold;
  uint32_t min_distance;
  uint32_t max_distance;
  uint32_t channel_as_intensity;
  uint32_t latency_low_threshold_ms;
  uint32_t latency_high_threshold_ms;
  int32_t blooming_up_distance_diff;
  int32_t blooming_down_distance_diff;
  int32_t blooming_min_vert_angle_diff;
  int32_t blooming_max_vert_angle_diff;
  int32_t retro_distance_diff;
  int32_t retro_count_threshold;
  END_CFG_MEMBER()
};

class StageDeliver {
  friend InnoLidar;

 public:
  static int process(void *job, void *ctx, bool prefer);

 public:
  explicit StageDeliver(InnoLidar *l);
  ~StageDeliver(void);

 public:
  void get_stats(InnoStatusCounters *counters) const;
  void print_stats(void) const;
  void get_stats_string(char *buf, size_t buf_size) const;

 public:
  static const size_t kDeliverMaxPacketNumber = 1000;
  static const size_t kMaxXyzDataPacketBufSize = 1024 * 1024;
  // Max distance check params
  static const uint32_t kInnoMaxDistanceMinPointSize = 400;
  // Max distance threshold of check times
  static const uint32_t kInnoMaxDistanceThMaxTimes = 1800;
  static const uint32_t kInnoMaxDistanceThMinValidTimes = 1200;
  // Max distance threshold of mean intensity
  static constexpr double kInnoMaxDistanceThNormalRefl = 2000.0;
  static constexpr double kInnoMaxDistanceThFaultRefl = 1400.0;
  // Max distance threshold of speed
  static constexpr double kInnoMaxDistanceThMinSpeed = 60.0;
  static constexpr double kInnoMaxDistanceThStartSpeed = 40.0;
  // Max distance check enable area
  static const int32_t kInnoMaxDistanceThHMaxAngle = 3000;
  static const int32_t kInnoMaxDistanceThHMinAngle = -3000;
  static const int32_t kInnoMaxDistanceThVMaxAngle = 700;
  static const int32_t kInnoMaxDistanceThVMinAngle = 0;
  // Max distance check enable max intensity
  static const uint32_t kInnoMaxDistanceThMaxIntensity = 100;
  // Max distance check enable radius range
  static const uint32_t kInnoMaxDistanceThMaxRadius = 150;
  static const uint32_t kInnoMaxDistanceThMinRadius = 50;

 private:
  InnoBlock2 paddingblock_;
  static const int kRetroMinAngle_ =
    - (60 * kInnoAngleUnitPerPiRad / kInnoDegreePerPiRad);
  static const int kRetroMaxAngle_ =
    60 * kInnoAngleUnitPerPiRad / kInnoDegreePerPiRad;
  static const int kRetroTableShift_ = 6;  // resolution is 0.35 degree
  static const int kRetroTableSize_ =
    (kRetroMaxAngle_ - kRetroMinAngle_) >> kRetroTableShift_;
  static const int kRetroTableBins_ = 4;
  int blooming_distance_diff_;
  struct {
    int min_retro_distance;
    int max_retro_distance;
    int16_t max_retro_vert_angle;
    int16_t min_retro_vert_angle;
    int16_t retro_count;
  } retro_table_[kRetroTableSize_][kRetroTableBins_];

 private:
  static const uint32_t kMaxCaliDataPoint_ = 40000;
  static const uint32_t kMaxCaliDataBufferSize_ =
    sizeof(InnoCaliDataBuffer) + sizeof(InnoCaliData) *  kMaxCaliDataPoint_;
  static const uint32_t kCaliDataPoolSize_ = 16;
  static const uint32_t kFramePointsTooFew_ = 30;

 private:
  inline uint32_t get_scaled_intensity(uint32_t raw_intensity) {
    if  (raw_intensity > config_.max_intensity) {
      raw_intensity = config_.max_intensity;
    }
    return raw_intensity * 254 / config_.max_intensity;
  }

  inline uint32_t get_refl_(bool use_reflectance,
                           bool record_cali_data,
                           int channel,
                           uint32_t intensity_seq,
                           uint32_t raw_intensity,
                           uint32_t ref_intensity,
                           uint32_t radius,
                           int32_t h_angle,
                           float *reflectance) {
    float ret;
    if (use_reflectance || record_cali_data) {
      float r = misc_tables_->get_scaled_reflectance(intensity_seq,
                                                     ref_intensity,
                                                     radius,
                                                     h_angle);
      ret = refl_factor_[channel] * r;
      if (reflectance) {
        *reflectance = ret;
      }
    }
#if 0
    if ((signed)raw_intensity > params_->iv_params.retro_intensity_2) {
      return 128;
    } else {
      return 0;
    }
#endif
    if (use_reflectance) {
      if (ret > 254) {
        ret = 254;
      }
      return static_cast<uint32_t>(ret);
    } else if (config_.channel_as_intensity == 0) {
      return get_scaled_intensity(raw_intensity);
    } else {
      return channel * 80;
    }
  }

  inline void set_retro_table_(int h_angle, int v_angle, int radius) {
    int ri = (h_angle - kRetroMinAngle_) >> kRetroTableShift_;
    if (ri >= 0 && ri < kRetroTableSize_) {
      // for (int rj = ri - 1; rj <= ri + 1; rj++) {
      for (int rk = 0; rk < kRetroTableBins_; rk++) {
        int min_retro_distance = retro_table_[ri][rk].min_retro_distance;
        int max_retro_distance = retro_table_[ri][rk].max_retro_distance;
        int min_retro_vert_angle = retro_table_[ri][rk].min_retro_vert_angle;
        int max_retro_vert_angle = retro_table_[ri][rk].max_retro_vert_angle;
        if (min_retro_distance == 0) {
          retro_table_[ri][rk].min_retro_distance = radius;
          retro_table_[ri][rk].max_retro_distance = radius;
          retro_table_[ri][rk].min_retro_vert_angle = v_angle;
          retro_table_[ri][rk].max_retro_vert_angle = v_angle;
          retro_table_[ri][rk].retro_count++;
          break;
        } else if ((radius > min_retro_distance -
                    config_.retro_distance_diff &&
                    radius < max_retro_distance +
                    config_.retro_distance_diff) ||
                   rk == kRetroTableBins_ - 1) {
          if (radius < min_retro_distance) {
            retro_table_[ri][rk].min_retro_distance = radius;
          } else if (radius > max_retro_distance) {
            retro_table_[ri][rk].max_retro_distance = radius;
          }
          if (v_angle < min_retro_vert_angle) {
            retro_table_[ri][rk].min_retro_vert_angle = v_angle;
          } else if (v_angle > max_retro_vert_angle) {
            retro_table_[ri][rk].max_retro_vert_angle = v_angle;
          }
          retro_table_[ri][rk].retro_count++;
          break;
        }
      }
    }
  }

  inline bool kill_blooming_(int h_angle, int v_angle, int radius) {
    int ri = (h_angle - kRetroMinAngle_) >> kRetroTableShift_;
    for (int rj = ri - 2; rj <= ri + 2; rj++) {
      if (rj >= 0 && rj < kRetroTableSize_) {
        for (int rk = 0; rk < kRetroTableBins_; rk++) {
          int retro_count = retro_table_[rj][rk].retro_count;
          if (retro_count == 0) {
            break;
          } else if (retro_count >
              (signed)config_.retro_count_threshold &&
              static_cast<int>(radius) >
              retro_table_[rj][rk].min_retro_distance -
              blooming_distance_diff_ &&
              static_cast<int>(radius) <
              retro_table_[rj][rk].max_retro_distance +
              blooming_distance_diff_ &&
              v_angle > retro_table_[rj][rk].min_retro_vert_angle -
              config_.blooming_min_vert_angle_diff &&
              v_angle < retro_table_[rj][rk].max_retro_vert_angle +
              config_.blooming_max_vert_angle_diff) {
            return true;
          }
        }
      }
    }
    return false;
  }

  inline bool need_to_check_max_distance_(int16_t h_angle, int16_t v_angle) {
    if (h_angle < kInnoMaxDistanceThHMaxAngle
        && h_angle > kInnoMaxDistanceThHMinAngle
        && v_angle < kInnoMaxDistanceThVMaxAngle
        && v_angle > kInnoMaxDistanceThVMinAngle) {
      return true;
    }
    return false;
  }

  const char *get_name_() const;
  int un_prefer_process_job_(StageAngleJob *job);
  int process_job_(StageAngleJob *job,
                   bool prefer);
  bool can_skip_(RawBlock *raw_block);
  void stage_ts_(StageAngleJob *job);
  int detect_latency_();
  inline void check_stats_frame_points_();
  inline void init_packet_(const StageAngleJob *job,
                           bool use_reflectance,
                           int unit_size,
                           int32_t packet_count,
                           const InnoLidarMode &mode,
                           const InnoLidarStatus &status,
                           int i,
                           int multi_return_mode,
                           int confidence = INNO_FULL_CONFIDENCE);
  int do_max_distance_check_callback_(const uint64_t idx);
  bool should_publish_(const StageAngleJob *job,
                       const InnoGalvoMode &galvo_mode,
                       int published_frames,
                       const InnoDataPacket *packet,
                       uint32_t i) const;
  void trace_log_frame_start_ts_(InnoDataPacket *packet,
                                 bool new_frame_start);
  void get_angles_(const int *v_angle_offset,
                   const RawBlock *raw_block,
                   int32_t *v_angle,
                   int32_t *h_angle) const;

 private:
  InnoLidar *lidar_;
  std::mutex mutex_;
  int sub_frame_idx_;
  InnoMean callback_mean_ms_;
  int32_t hori_roi_angle_;
  int64_t last_frame_idx_;
  const LidarParams *params_;
  const MiscTables *misc_tables_;

  double refl_factor_[kInnoChannelNumber];
  uint32_t ref_intensity_[kInnoChannelNumber];

  InnoEpSecondDouble stats_stage_sum_ts_[ScanLines::STAGE_TIME_MAX+1];
  InnoEpSecondDouble stats_stage_sumq_ts_[ScanLines::STAGE_TIME_MAX+1];
  InnoEpSecondDouble stats_stage_max_ts_[ScanLines::STAGE_TIME_MAX+1];
  size_t stats_dropped_jobs_;
  size_t stats_delivered_jobs_;
  size_t stats_frames_;
  size_t stats_points_;
  size_t stats_frame_points_;
  size_t stats_skipped_blocks_;
  size_t stats_lost_frames_;
  size_t stats_last_frame_idx_;
  ssize_t remap_last_origin_frame_idx_;
  ssize_t remap_last_mapped_frame_idx_;
  uint32_t frame_points_low_counter_;
  uint16_t stats_last_scan_direction_;
  bool send_to_next_stage_;

  StageDeliverConfig config_base_;
  StageDeliverConfig config_;
  InnoMean sum_latency_;
  double init_ts_;

  MemPool *cali_data_pool_;

  InnoDataPacket *packets_[kDeliverMaxPacketNumber];
  union {
    char xyz_data_packet_buf_[kMaxXyzDataPacketBufSize];
    InnoDataPacket xyz_data_packet_;
  };

  double max_distance_total_refl_;
  uint32_t max_distance_point_size_;
  InnoMean max_distance_intensity_;

  bool should_send_empty_packet_{true};
  int frame_sync_locked{0};
};

}  // namespace innovusion

#endif  // SDK_STAGE_DELIVER_H_
