/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_STAGE_ANGLE_H_
#define SDK_STAGE_ANGLE_H_

#ifndef __MINGW64__
#include <pwd.h>
#endif
#include <limits>
#include <mutex>  // NOLINT
#include <string>

#include "sdk_common/inno_lidar_packet.h"
#include "sdk_common/inno_faults_common.h"
#include "sdk/stage_angle_job.h"
#include "sdk/stage_angle_lookup.h"
#include "utils/config.h"
#include "sdk/cross_talk.h"

#define FRAME_REF_REG_BASE 0x210
namespace innovusion {
class InnoLidar;

class StageAngleConfig: public Config {
 public:
  StageAngleConfig() : Config() {
    fov_top_left_angle = -60.0 * kInnoAngleUnitPerDegree;
    fov_top_right_angle = 60.0 * kInnoAngleUnitPerDegree;
    fov_top_low_angle = -20.0 * kInnoAngleUnitPerDegree;
    fov_top_high_angle = 10.8 * kInnoAngleUnitPerDegree;
    filt_intensity_galvo_angle = 60 * kInnoAngleUnitPerDegree;
    fov_left_cal = -60.0 * kInnoAngleUnitPerDegree;
    fov_right_cal = 60.0 * kInnoAngleUnitPerDegree;
    fov_bottom_cal = -20.0 * kInnoAngleUnitPerDegree;
    fov_up_cal = 12.0 * kInnoAngleUnitPerDegree;
    remove_cross_talk = 1;
    cross_talk_distance1 = 1.0 * kInnoDistanceUnitPerMeter;  // 1.0m
    cross_talk_distance2 = 1.0 * kInnoDistanceUnitPerMeter;  // 1.0m
    enable_galvo_tracking = 1;
    galvo_tracking_angle_threshold = 0.5 * kInnoAngleUnitPerDegree;
  }

  const char* get_type() const override {
    return "Lidar_StageAngle";
  }

  int set_key_value_(const std::string &key,
                             double value) override {
    if (strcmp(key.c_str(), "fov_top_left_angle") == 0 ||
        strcmp(key.c_str(), "fov_top_right_angle") == 0 ||
        strcmp(key.c_str(), "fov_top_low_angle") == 0 ||
        strcmp(key.c_str(), "fov_top_high_angle") == 0 ||
        strcmp(key.c_str(), "filt_intensity_galvo_angle") == 0 ||
        strcmp(key.c_str(), "fov_left_cal") == 0 ||
        strcmp(key.c_str(), "fov_right_cal") == 0 ||
        strcmp(key.c_str(), "fov_bottom_cal") == 0 ||
        strcmp(key.c_str(), "fov_up_cal") == 0 ||
        strcmp(key.c_str(), "galvo_tracking_angle_threshold") == 0) {
      value *= kInnoAngleUnitPerDegree;
    } else if (strcmp(key.c_str(), "cross_talk_distance1") == 0 ||
               strcmp(key.c_str(), "cross_talk_distance2") == 0) {
      value *= kInnoDistanceUnitPerMeter;
    }
    SET_CFG(fov_top_left_angle);
    SET_CFG(fov_top_right_angle);
    SET_CFG(fov_top_low_angle);
    SET_CFG(fov_top_high_angle);
    SET_CFG(filt_intensity_galvo_angle);
    SET_CFG(fov_left_cal);
    SET_CFG(fov_right_cal);
    SET_CFG(fov_bottom_cal);
    SET_CFG(fov_up_cal);
    SET_CFG(remove_cross_talk);
    SET_CFG(cross_talk_distance1);
    SET_CFG(cross_talk_distance2);
    SET_CFG(enable_galvo_tracking);
    SET_CFG(galvo_tracking_angle_threshold);
    return -1;
  }

  int set_key_value_(const std::string &key,
                     const std::string value) override {
    // no string attribute
    return -1;
  }

  BEGIN_CFG_MEMBER()
  int fov_top_left_angle;
  int fov_top_right_angle;
  int fov_top_low_angle;
  int fov_top_high_angle;
  int filt_intensity_galvo_angle;
  int fov_left_cal;
  int fov_right_cal;
  int fov_bottom_cal;
  int fov_up_cal;
  int remove_cross_talk;
  int cross_talk_distance1;
  int cross_talk_distance2;
  int enable_galvo_tracking;
  int galvo_tracking_angle_threshold;
  END_CFG_MEMBER()
};

class StageAngle {
  friend InnoLidar;

  class GalvoTracking {
   public:
    explicit GalvoTracking(InnoLidar *l);
    ~GalvoTracking() {
    }
    void add_scatter_signal(RawBlock *block,
                            int16_t h_angle[],
                            int16_t v_angle[],
                            int scan_id);
    void do_galvo_tracking(int64_t frame_idx, int max_scan_id,
                           uint32_t frame_ave_ref_intensity[],
                           uint32_t ref_sample_points[],
                           int galvo_tracking_angle_threshold);

   private:
    int check_scatter_cali_file_();
    void save_scatter_cali_file_();
    void do_galvo_tracking_cali_phase_1_(int64_t frame_idx);
    void do_galvo_tracking_cali_phase_2_(int64_t frame_idx);
    void do_galvo_tracking_cali_phase_3_(int64_t frame_idx,
                                         uint32_t frame_ave_ref_intensity[],
                                         uint32_t ref_sample_points[]);
    void do_galvo_tracking_cali_phase_4_(int64_t frame_idx,
                                         int max_id,
                                         uint32_t frame_ave_ref_intensity[],
                                         uint32_t ref_sample_points[]);
    void do_galvo_tracking_normal_phase_1_(int64_t frame_idx);
    void do_galvo_tracking_normal_phase_2_(int64_t frame_idx);
    void do_galvo_tracking_normal_phase_3_(int64_t frame_idx,
                                           int max_id,
                                           uint32_t frame_ave_ref_intensity[],
                                           uint32_t ref_sample_points[],
                                           int galvo_tracking_angle_threshold);

   private:
    static const int kScatterMinAngle_ =
      - (19 * kInnoAngleUnitPerPiRad / kInnoDegreePerPiRad);
    static const int kScatterMaxAngle_ =
      17 * kInnoAngleUnitPerPiRad / kInnoDegreePerPiRad;
    static const int kScatterTableShift_ = 3;  // resolution is 0.044 degree
    static const int kScatterTableSize_ =
      (kScatterMaxAngle_ - kScatterMinAngle_) >> kScatterTableShift_;

    static const int kGalvoBitsShift_ = 5;  // step size is 0.176 degree
    static const int kGalvoShiftMinAngle_ =
      - (8 * kInnoAngleUnitPerPiRad / kInnoDegreePerPiRad >>
         kGalvoBitsShift_ << kGalvoBitsShift_);
    static const int kGalvoShiftMaxAngle_ =
      8 * kInnoAngleUnitPerPiRad / kInnoDegreePerPiRad;
    static const int kGalvoShiftSteps_ =
      (kGalvoShiftMaxAngle_ - kGalvoShiftMinAngle_) >> kGalvoBitsShift_;

    static const int kMaxScanLines_ = 100;  // per channel
    // normal -> quiet synced, wait for apd cal
    static const int kScrollWaitFrames_ = 20;
    // quiet synced -> quiet scroll, wait for apd cal
    static const int kCaliWaitFrames_ = 110;
    static const int kAPdCalWaitFrames_ = 180;
    // calibration
    static const int kCaliFrames_ = 50;
    static const int kCaliTransitionFrames_ = 10;
    // quiet scroll -> normal
    static const int kQuietToNormalWaitFrames_ = 50;
    // wait for apd cal
    static const int kNormalWaitFrames_ = 160;
    static const int kScatterMoveAvgSize_ = 2;

    static const int kErrorRatioHistSize_ = 10;
    static const int kScatterCaliMinCount_ = 20;
    static const int kScatterNormalMinCount_ = 20;
    static const int kScatterMaxIntensity_ = 8000;
    static const int kScatterMaxIntensitySeq_ = 445;
    // static const int kScatterAngleRange_ = 3 * kInnoAngleUnitPerDegree;
    static const int kScatterAngleRange_ = 3 / 0.005493;
    // static const int kMinAngleDiffThreshold_ = 0.5 * kInnoAngleUnitPerDegree;
    static const int kMinAngleDiffThreshold_ = 0.5 / 0.005493;
    // sum of all 4 channels (26x4)
    static const int kCaliMinValidLines_ = 80;
    // sum of all 4 channels(40x4)
    static const int kNormalMinValidLines_ = 100;
    static constexpr double kErrorRatioThreshold_ = 0.7;

    // normal vaule is around 960
    static const uint32_t kCaliRefPointsThreshold_ = 750;
    // normal value is around 2200
    static const uint32_t kNormalRefPointsThreshold_ = 1500;
    static const uint32_t kRefIntensityThreshold_ = 400;
    static const uint32_t kMaxStrongScatterPerFrame_ = 10;
    // 1-min testing
    static const uint32_t kMaxTestFrames_ = 600;
    static constexpr double kTestErrorRatioThreshold_ = 0.5;
    static constexpr double kMaxInvalidTestFramesRatio_ = 0.1;

    /*
    normal mode vertical angle range:
    chanel 0: -12.64 to  6.78 degree
    chanel 1: -11.44 to  8.00 degree
    chanel 2: -10.24 to  9.20 degree
    chanel 2:  -9.03 to 10.40 degree

    quiet mode vertical angle range:
    chanel 0: -13.00 to  12.93 degree
    chanel 1: -11.85 to  14.11 degree
    chanel 2: -10.60 to  15.34 degree
    chanel 2:  -9.48 to  16.53 degree

    up frame: (has more strong scatters)
    scan id  :    0     1      2
    channen 0: -12.33 -11.40 -10.42
    channel 1: -11.12 -10.20  -9.22
    channel 2:  -9.93  -9.00  -8.03
    channel 3:  -8.72  -7.80  -6.82

    down frame:
    scan id  :  39     38      37
    channel 0: -12.64 -11.69 -10.66
    chaannl 1: -11.43 -10.48  -9.46
    channel 2: -10.24  -9.28  -8.27
    channel 3:  -9.03  -8.08  -7.06
    */

   private:
    InnoLidar *lidar_;
    const MiscTables *misc_tables_;

    enum galvo_tracking_phase {
      GALVO_TRACKING_PHASE_NONE,
      GALVO_TRACKING_CALI_PHASE_1,
      GALVO_TRACKING_CALI_PHASE_2,
      GALVO_TRACKING_CALI_PHASE_3,
      GALVO_TRACKING_CALI_PHASE_4,
      GALVO_TRACKING_NORMAL_PHASE_1,
      GALVO_TRACKING_NORMAL_PHASE_2,
      GALVO_TRACKING_NORMAL_PHASE_3,
      GALVO_TRACKING_MAX
    };
    typedef struct {
      int32_t intensity_or_power_sum;
      int32_t vert_angle_sum;
      int32_t avg_power;
      int32_t avg_intensity;
      int32_t avg_vert_angle;
      int32_t cnt;
    } scatter_entry;
    int64_t frame_idx_start_scroll_;
    int64_t frame_idx_start_cali_;
    int64_t frame_idx_wait_apd_cal_;
    int64_t frame_idx_set_galvo_start_low_;
    int64_t frame_idx_galvo_start_low_transition_;
    int64_t frame_idx_stop_cali_;
    int64_t frame_idx_start_normal_;
    size_t stats_total_strong_scatter_;
    size_t stats_strong_scatter_per_frame_;
    size_t stats_skip_bad_reference_frames_;
    size_t stats_unreliable_detection_frames_;
    size_t stats_big_error_ratio_frames_;
    size_t stats_strong_scatter_frames_;
    size_t stats_invalid_lines_frames_;
    size_t stats_total_detection_frames_;
    size_t stats_real_detection_frames_;
    size_t stats_total_test_frames_;
    size_t stats_invalid_test_frames_;
    size_t scatter_error_ratio_hist_[kErrorRatioHistSize_];
    size_t galvo_min_angle_shift_hist_[kGalvoShiftSteps_];
    int up_angle_limit_[kInnoChannelNumber];
    int low_angle_limit_0_[kInnoChannelNumber];
    int low_angle_limit_1_[kInnoChannelNumber];
    int strong_scatter_angle_limit_[kInnoChannelNumber];
    uint32_t target_ref_intensity_[kInnoChannelNumber];
    uint32_t target_ref_power_[kInnoChannelNumber];
    std::string scatter_file_path_;
    enum galvo_tracking_phase galvo_tracking_phase_;
    bool is_galvo_start_low_set_;
    bool skip_galvo_tracking_;
    bool save_scatter_cali_file_after_testing_;
    uint32_t scatter_cali_sum_[kInnoChannelNumber][kScatterTableSize_];
    uint32_t scatter_cali_cnt_[kInnoChannelNumber][kScatterTableSize_];
    uint32_t scatter_cali_table_[kInnoChannelNumber][kScatterTableSize_];
    scatter_entry scatter_normal_table_[kInnoChannelNumber][kMaxScanLines_];
  };

 public:
  static int process(void *job, void *ctx, bool prefer);

 private:
  static void prepare_galvo_whole_(const EncoderSignal *galvo_1st,
                                   const EncoderSignal *galvo_2nd,
                                   InnoFpgaSubNs *whole,
                                   int64_t *whole_value);

 public:
  explicit StageAngle(InnoLidar *l);
  ~StageAngle(void);

 private:
  static const int kInnoRoiNumber = 2;
  static const uint32_t kPtNumberMax = UINT32_MAX >> 1;

 public:
  void init_lookup_table();
  bool is_lookup_table_inited(void) {
    return angle_lookup_.is_table_inited();
  }
  void print_stats(void) const;
  void get_stats_string(char *buf, size_t buf_size) const;

 private:
  const char *get_name_(void) const;
  int process_job_(StageAngleJob *job,
                   bool prefer);
  int32_t process_job_copy_pre_(StageAngleJob *job);
  void add_new_line_(StageAngleJob *cur_lines,
                     uint32_t block_start,
                     uint32_t block_so_far,
                     bool end_of_frame,
                     int32_t galvo_angle_change);
  void prepare_polygon_whole_(const EncoderSignal *polygon_1st,
                              const EncoderSignal *polygon_2nd,
                              InnoFpgaSubNs *whole,
                              int64_t *whole_value);

  void reset_ref_cnt_and_intensity();
  void send_ref_time_to_fw(const RawBlock* block);

 private:
  InnoLidar *lidar_;
  const LidarParams &params_;
  const MiscTables *misc_tables_;
  std::mutex mutex_;

  ScanLines *pre_lines_;
  int32_t pre_left_start_;

  int32_t last_facet_;  // < 0 means all others last_ are invalid
  enum InnoFrameDirection last_direction_;
  int32_t last_angle_hori_;
  int32_t last_angle_vert_;
  int32_t last_galvo_value_;
  enum InnoGalvoMode auto_galvo_mode_;

  int32_t galvo_angle_threshold_[kInnoChannelNumber];

  int fov_up_;
  int fov_bottom_;
  int fov_left_;
  int fov_right_;
  int fov_top_half_p_angle_;
  int fov_bottom_half_p_angle_;
  double fov_top_half_p_angle_linear_coeff_;

  StageAngleLookup angle_lookup_;

  CrossTalk cross_talk_;

  GalvoTracking galvo_tracking_;

  size_t stats_frames_;
  size_t stats_facets_;
  size_t stats_drop_few_adc_;
  size_t stats_drop_few_polygon_;
  size_t stats_drop_few_galvo_;
  size_t stats_fov_filter_blocks_;
  size_t stats_polygon_too_long_;
  size_t stats_polygon_too_short_;
  size_t stats_polygon_ooo1_;
  size_t stats_polygon_ooo2_;
  size_t stats_no_copy_;
  ssize_t fw_ipc_seq_begin_;
  ssize_t fw_ipc_seq_end_;

  uint64_t frame_total_ref_intensity_[kInnoRoiNumber][kInnoChannelNumber];
  uint64_t frame_total_points_[INNO_FRAME_DIRECTION_MAX][kInnoChannelNumber];
  size_t frame_total_ref_cnt_[kInnoRoiNumber][kInnoChannelNumber];

  int64_t frame_idx_;
  int64_t line_idx_;
  int64_t scan_id_;
  int64_t last_frame_scan_id_;

  uint64_t apd_report_last_ms_ {0};

  StageAngleConfig config_base_;
  StageAngleConfig config_;
};

}  // namespace innovusion

#endif  // SDK_STAGE_ANGLE_H_
