/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */
#ifndef SDK_STAGE_SIGNAL_H_
#define SDK_STAGE_SIGNAL_H_

#include <condition_variable>  // NOLINT
#include <deque>
#include <mutex>  // NOLINT
#include <string>
#include <thread>  // NOLINT
#include <unordered_map>
#include <queue>
#include <algorithm>
#include <utility>

#include "sdk_common/inno_lidar_packet.h"
#include "sdk_common/inno_faults_common.h"
#include "sdk/rawdata_type.h"
#include "sdk/stage_signal_clock.h"
#include "sdk/misc_tables.h"
#include "utils/config.h"
#include "utils/types_consts.h"
#include "utils/utils.h"

#define SCATTER_FILTER_RATIO_CONTROLLER  0xCC
#define CHA_REFERENCE_SAMPLE_RATE_REG  0x400

namespace innovusion {
class InnoLidar;
class LidarParams;
class RawBlock;
class RawChannelPoint;
class ScanLines;
class StageSignalJob;

enum OpticalCheckState {
  INNO_OPTICAL_TO_BE_CONFIRMED = 0,
  INNO_OPTICAL_SCATTER_TOO_LOW  = 1,
  INNO_OPTICAL_OK_WINDOW_NOT_BLOCKED = 2,
  INNO_OPTICAL_OK_WINDOW_MAYBE_BLOCKED = 3,
  INNO_OPTICAL_OK_WINDOW_BLOCKED = 4,
  INNO_OPTICAL_CHECK_STATE_MAX = 5
};

typedef struct {
  InnoFpgaNs fpga_ns;
  int32_t angle_value;
} PolygonStamp;

class StageSignalConfig: public Config {
 public:
  StageSignalConfig() : Config() {
    ref_window_center = 20 * 32;
    ref_window_half_width_init = 16 * 32;
    ref_window_half_width_min = 1 * 32;
    ref_window_half_width_max = 16 * 32;
    ref_window_half_width_max_by_histo = 7 * 32;
    ref_window_left_limit = 10 * 32;  // is 10 * 32 reasonable?
    ref_window_right_limit = 40 * 32;  // is 40 * 32 reasonable?
    ref_sample_num = 1024;
    ref_match_threshold = 0.5;
    ref_new_weight = 1.0;
    ref_min_intensity = 50;
    ref_max_intensity = 4000;
    ref_scatter_threshold = 4000;
    g_encoder_delay = 130 * 1000;  // 130us
    exclude_distance = 0.5 * kInnoDistanceUnitPerMeter;  // 0.5m
    pulse_pick = 0;
    min_distance = 1.3 * kInnoDistanceUnitPerMeter;  // 1.3m
    max_distance = 510 * kInnoDistanceUnitPerMeter;  // 510m
    sw_road_mode = 1;
    sw_start = 50 * kInnoDistanceUnitPerMeter;  // 50m
    sw_end = 130 * kInnoDistanceUnitPerMeter;  // 130m
    pre_retro_min_intensity = 2000;
    kill_retro_noise = 1;
    min_intensity = 5;
    raw_output = 0;
    // normal value 12500000/
    polygon_lowlimit_ns = 12000000;
    polygon_highlimit_ns = 13000000;
    galvo_lowlimit_ns = 30000;      // tested min value 61504ns
    galvo_highlimit_ns = 300000;    // tested max value 133504ns
    trigger_lowlimit_ns = 1000;      // normal value 2200
    trigger_highlimit_ns = 5000;
    return_pulse_ratio_threshold = 0.05;
    scatter_ratio_threshold = 0.05;
    reference_ratio_threshold = 0.05;
    window_not_blocked_return_threshold = 0.10;
    window_blockage_scatter_threshold = 0.95;
    scatter_check_cpu_threshold = 180;
    ingore_trigger_interval_drop = true;
  }

  const char* get_type() const override {
    return "Lidar_StageSignal";
  }

  int set_key_value_(const std::string &key, double value) override {
    if (strcmp(key.c_str(), "exclude_distance") == 0 ||
        strcmp(key.c_str(), "min_distance") == 0 ||
        strcmp(key.c_str(), "max_distance") == 0 ||
        strcmp(key.c_str(), "sw_start") == 0 ||
        strcmp(key.c_str(), "sw_end") == 0) {
      value *= kInnoDistanceUnitPerMeter;
    } else if (strcmp(key.c_str(), "g_encoder_delay") == 0 ||
               strcmp(key.c_str(), "time_sync_packet_timeout") == 0 ||
               strcmp(key.c_str(), "time_sync_packet_timeout_ntp") == 0 ||
               strcmp(key.c_str(), "time_sync_ignore_jobs") == 0) {
      value *= 1000;
    } else if (strcmp(key.c_str(), "ref_window_center") == 0 ||
               strcmp(key.c_str(), "ref_window_half_width_init") == 0 ||
               strcmp(key.c_str(), "ref_window_half_width_init_yaml") == 0 ||
               strcmp(key.c_str(), "ref_window_half_width_min") == 0 ||
               strcmp(key.c_str(), "ref_window_half_width_max") == 0 ||
               strcmp(key.c_str(), "ref_window_left_limit") == 0 ||
               strcmp(key.c_str(), "ref_window_right_limit") == 0) {
      value *= 32;
    }
    SET_CFG(ref_window_center);
    SET_CFG(ref_window_half_width_init);
    SET_CFG(ref_window_half_width_min);
    SET_CFG(ref_window_half_width_max);
    SET_CFG(ref_window_half_width_max_by_histo);
    SET_CFG(ref_window_left_limit);
    SET_CFG(ref_window_right_limit);
    SET_CFG(ref_sample_num);
    SET_CFG(ref_match_threshold);
    SET_CFG(ref_new_weight);
    SET_CFG(ref_min_intensity);
    SET_CFG(ref_max_intensity);
    SET_CFG(ref_scatter_threshold);
    SET_CFG(g_encoder_delay);
    SET_CFG(exclude_distance);
    SET_CFG(pulse_pick);
    SET_CFG(min_distance);
    SET_CFG(max_distance);
    SET_CFG(sw_road_mode);
    SET_CFG(sw_start);
    SET_CFG(sw_end);
    SET_CFG(pre_retro_min_intensity);
    SET_CFG(kill_retro_noise);
    SET_CFG(min_intensity);
    SET_CFG(raw_output);
    SET_CFG(polygon_lowlimit_ns);
    SET_CFG(polygon_highlimit_ns);
    SET_CFG(galvo_lowlimit_ns);
    SET_CFG(galvo_highlimit_ns);
    SET_CFG(trigger_lowlimit_ns);
    SET_CFG(trigger_highlimit_ns);
    SET_CFG(return_pulse_ratio_threshold);
    SET_CFG(scatter_ratio_threshold);
    SET_CFG(reference_ratio_threshold);
    SET_CFG(window_not_blocked_return_threshold);
    SET_CFG(window_blockage_scatter_threshold);
    SET_CFG(scatter_check_cpu_threshold);
    SET_CFG(ingore_trigger_interval_drop);

    return -1;
  }

  int set_key_value_(const std::string &key,
                     const std::string value) override {
    // no string attribute
    return -1;
  }

  BEGIN_CFG_MEMBER()
  int64_t ref_window_center;
  int64_t ref_window_half_width_init;
  int64_t ref_window_half_width_min;
  int64_t ref_window_half_width_max;
  int64_t ref_window_half_width_max_by_histo;
  int64_t ref_window_left_limit;
  int64_t ref_window_right_limit;
  int64_t ref_sample_num;
  double ref_match_threshold;
  double ref_new_weight;
  int64_t ref_min_intensity;
  int64_t ref_max_intensity;
  int64_t ref_scatter_threshold;
  int64_t g_encoder_delay;
  int64_t exclude_distance;
  int64_t pulse_pick;
  int64_t min_distance;
  int64_t max_distance;
  int64_t sw_road_mode;
  int64_t sw_start;
  int64_t sw_end;
  int64_t pre_retro_min_intensity;
  int64_t kill_retro_noise;
  int64_t min_intensity;
  int64_t raw_output;
  int64_t polygon_lowlimit_ns;
  int64_t polygon_highlimit_ns;
  int64_t galvo_lowlimit_ns;
  int64_t galvo_highlimit_ns;
  int64_t trigger_lowlimit_ns;
  int64_t trigger_highlimit_ns;
  double return_pulse_ratio_threshold;
  double scatter_ratio_threshold;
  double reference_ratio_threshold;
  double window_not_blocked_return_threshold;
  double window_blockage_scatter_threshold;
  uint32_t scatter_check_cpu_threshold;
  bool ingore_trigger_interval_drop;
  END_CFG_MEMBER()
};


class StageSignal {
  friend InnoLidar;
  friend ScanLines;

  class RefIntCheckHelper {
   private:
    struct TempFactorPair {
      int temp;
      double factor;
      TempFactorPair(int t, double f) : temp(t), factor(f) {}
    };

   public:
     RefIntCheckHelper();
     ~RefIntCheckHelper();
     RefIntCheckHelper(const RefIntCheckHelper& rhs) = delete;
     RefIntCheckHelper& operator=(const RefIntCheckHelper& rhs) = delete;
     int get_diag_threshold(std::pair<int, double>* thresholds);

   private:
     static const int kCorrectBeginTemp = 60;  // 60 degC
     static const int kCorrectEndTemp = 115;  // 115 degC
     static const int kInitTableSize = 6;

   private:
     void build_temperature_correct_table_();

   private:
     double temp_correct_table_[kCorrectEndTemp - kCorrectBeginTemp];
  };

  class OpticalPathCheck {
   public:
    explicit OpticalPathCheck(InnoLidar* lidar,
                              StageSignal* stage_signal,
                              double return_pulse_ratio_threshold,
                              double scatter_ratio_threshold,
                              double reference_ratio_threshold,
                              double window_not_blocked_return_threshold,
                              double window_blockage_scatter_threshold,
                              uint32_t scatter_check_cpu_threshold);
    ~OpticalPathCheck();
    void do_optical_and_blockage_diagnosis(uint32_t block_number);
    void do_ref_intensity_check();
    void load_ref_intensity_from_yaml();
    void prepare_optical_and_blockage_check();
    void prepare_ref_intensity_check_for_next_round();

   public:
    inline void add_return_sample(uint32_t channel) {
      ++return_pulse_counter_[channel];
    }
    inline void add_scatter_sample(uint32_t channel) {
      ++scatter_counter_[channel];
    }
    inline void add_reference_sample(uint32_t channel) {
      ++reference_counter_[channel];
    }
    inline void add_reference_intensity(int channel, int intensity) {
      reference_intensity_[channel] += intensity;
      reference_sample_counts_[channel]++;
    }
    inline void inc_frame_count() {
      frame_counts_++;
    }
    inline uint64_t get_frame_count() {
      return frame_counts_;
    }

   private:
    void do_reference_optical_check_(uint32_t block_number);
    void do_main_optical_and_blockage_check_(uint32_t block_number);
    enum OpticalCheckState do_optical_check_via_return_(uint32_t block_number,
                                                        bool optical_break,
                                                        bool blocked);
    void do_optical_check_via_scatter_(uint32_t block_number,
                                       bool optical_break,
                                       bool blocked);
    int enable_scatter_filter_(uint8_t scale);
    void init_env_();
    int get_avg_detector_temp(int64_t *avg_temp,
                              int64_t *min_temp,
                              int64_t *max_temp);

   private:
    static const uint8_t kCountOneScatterPerPeriod = 16;
    static const uint32_t kMinDiagnoseBlockNumber = 3000;
    // 458kHz / 10 FPS
    static const uint32_t kMinRefIntensityCheckCount = 350;
    static const uint32_t kRefIntensityTempTableDimension = 4;
    static const int32_t kDetTempLowThreshold = -100;   // -10degC
    static const int32_t kDetTempMidThreshold = 850;    // 85degC
    static const int32_t kDetTempHighThreshold = 1150;  // 115degC
    static const int32_t kMaxDetTempDiffThreshold = 100;  // 10degC
    static const double hysteresis_factor_return;
    static const double hysteresis_factor_scatter;
    static const double hysteresis_factor_reference;
    static const double kMaxRatioDiffBetChannels;

   private:
    double return_pulse_ratio_threshold_;
    double scatter_ratio_threshold_;
    double reference_ratio_threshold_;
    double window_not_blocked_return_threshold_;
    double window_blockage_scatter_threshold_;
    uint32_t scatter_check_cpu_threshold_;
    uint32_t frame_counts_;
    InnoLidar *lidar_;
    StageSignal* stage_signal_;

    size_t return_pulse_counter_[kInnoChannelNumber];
    size_t return_pulse_counter_old_[kInnoChannelNumber];
    size_t scatter_counter_[kInnoChannelNumber];
    size_t scatter_counter_old_[kInnoChannelNumber];
    size_t reference_counter_[kInnoChannelNumber];
    size_t reference_counter_old_[kInnoChannelNumber];
    size_t reference_intensity_[kInnoChannelNumber];
    size_t reference_sample_counts_[kInnoChannelNumber];
    size_t ref_intensity_in_yaml_[kInnoChannelNumber];
    bool scatter_running_;
    bool ref_intensity_initialized_;
    RefIntCheckHelper ref_int_check_helper_;
  };

 private:
  class RefTrackingResult {
   public:
    RefTrackingResult();
    ~RefTrackingResult();
    void init(InnoFpgaSubNs window_center,
              uint32_t channel);
    void update(InnoFpgaSubNs virtual_ref_offset, int in_roi);
    inline InnoFpgaSubNs get_virtual_ref_offset() const {
      return virtual_ref_offset_sub_ns_;
    }
   private:
    InnoFpgaSubNs virtual_ref_offset_sub_ns_;
    uint32_t channel_;
    uint32_t in_roi_count_;
  };

  class RefTracking {
   public:
    RefTracking();
    ~RefTracking();
    void init(uint32_t id,
              InnoFpgaSubNs window_center,
              InnoFpgaSubNs half_window_width,
              InnoFpgaSubNs half_min,
              InnoFpgaSubNs half_max,
              InnoFpgaSubNs left_limit,
              InnoFpgaSubNs right_limit,
              size_t sample_num,
              double match_thresh,
              double new_weight,
              uint32_t channel,
              uint32_t roi_type);
    void set_bound();
    void set_bound(InnoFpgaSubNs half_window_sub_ns);
    void set_bound(InnoFpgaSubNs half_window_sub_ns,
                   size_t trigger_count);
    void prepare_for_next_round(size_t trigger_cnt);
    void add_sample(InnoFpgaSubNs diff_trigger,
                    size_t trigger_cnt,
                    RefTrackingResult *tracking_result);
    inline InnoFpgaSubNs set_ref_center(InnoFpgaSubNs ref_center) {
      InnoFpgaSubNs old_ref_delay = current_average_sub_ns_;
      current_average_sub_ns_ = ref_center;
      return old_ref_delay;
    }
    inline InnoFpgaSubNs get_current_virtual_ref_offset_sub_ns() const {
      return current_average_sub_ns_;
    }
    inline bool is_before_window(InnoFpgaSubNs s) const {
      return s < ref_l_w_sub_ns_;
    }
    inline bool is_after_window(InnoFpgaSubNs s) const {
      return s >= ref_r_w_sub_ns_;
    }
    inline void add_scatter() {
      ref_is_scatter_++;
    }
    inline bool need_add_sample(int channel, size_t sample_rate) {
      ++count_;
      return ((count_ & (sample_rate - 1)) == 0);
    }

    inline bool has_ref() {
      return has_ref_;
    }

    inline void set_ref(bool has_ref) {
      has_ref_ = has_ref;
    }

    inline uint32_t get_search_window_l() {
      return half_window_l_;
    }

    inline void set_window_l(uint32_t window) {
      half_window_l_ = window;
    }

    inline uint32_t get_search_window_r() {
      return half_window_r_;
    }

    inline void set_window_r(uint32_t window) {
      half_window_r_ = window;
    }

    inline void reset_stats(size_t trigger_count) {
      prepare_for_next_round(trigger_count);
    }

    inline bool need_expand_window_external() {
      return window_expand_flags_ < 0;
    }

    inline void dec_window_expand_flags() {
      window_expand_flags_--;
    }

    inline void reset_window_expand_flags() {
      window_expand_flags_ = kCheckTimesPerRefUpdate;
    }

    void expand_window(uint32_t muti_factor = 1) {
      // left window
      half_window_l_ += InnoConsts::kSubNsInNs * muti_factor;
      uint32_t half_window_max = current_average_sub_ns_ -
                                 center_left_limit_ + half_window_min_;
      uint32_t min_window = std::min(half_window_max_, half_window_max);
      if (half_window_l_ > min_window) {
        half_window_l_ = min_window;
      }
      // right window
      half_window_r_ += InnoConsts::kSubNsInNs * muti_factor;
      half_window_max = center_right_limit_ -
                        current_average_sub_ns_ + half_window_min_;
      min_window = std::min(half_window_max_, half_window_max);
      if (half_window_r_ > min_window) {
        half_window_r_ = min_window;
      }
    }

   public:
    static const int get_ref_check_times_per_update() {
      return kCheckTimesPerRefUpdate;
    }

   private:
    static const int kCheckTimesPerRefUpdate = 2;

   private:
    void bound_check_and_correct_();

   private:
    uint32_t id_;
    size_t count_;
    size_t trigger_count_old_;

    bool has_ref_;
    bool locked_;
    size_t locked_count_;
    size_t unlocked_count_;
    size_t unlocked_to_locked_count_;
    size_t locked_to_unlocked_count_;
    uint32_t half_window_l_;
    uint32_t half_window_r_;
    uint32_t half_window_min_;
    uint32_t half_window_max_;
    uint32_t center_left_limit_;
    uint32_t center_right_limit_;
    size_t sample_per_round_;
    double update_match_thresh_;
    double update_new_weight_;
    double thershold_hysteresis_;
    InnoFpgaSubNs ref_l_w_sub_ns_;
    InnoFpgaSubNs ref_r_w_sub_ns_;
    InnoFpgaSubNs current_average_sub_ns_;

    size_t sampled_;
    InnoFpgaSubNs sampled_sub_ns_;
    size_t ref_is_scatter_;

    size_t total_sampled_;
    InnoFpgaSubNs total_sampled_sub_ns_;
    size_t total_ref_is_scatter_;

    uint32_t channel_;
    uint32_t roi_type_;
    uint64_t ref_left_limit_count_;
    uint64_t ref_right_limit_count_;
    int window_expand_flags_;
  };

  class RefFinder {
   private:
    struct RefCandidate {
      int count;
      int64_t avg_time;
      RefCandidate(int count, int64_t avg_time) {
        this->count = count;
        this->avg_time = avg_time;
      }
      bool operator < (const RefCandidate& rhs) const {
        // descending order for count, ascending order for avg_time
        if (this->count == rhs.count) {
          return this->avg_time > rhs.avg_time;
        }
        return this->count < rhs.count;
      }
    };

   public:
    RefFinder() {
      sample_count_ = 0;
      sample_interval_index_ = 0;
      memset(buckets_, 0, sizeof(buckets_));
    }
    ~RefFinder() {}

    void init(int channel) {
      channel_ = channel;
    }

    int find_ref_center(int trigger_count,
                        int64_t* ref_time,
                        std::string* info) {
      *info += "trigger count: " + std::to_string(trigger_count) +
               ", sample count: " + std::to_string(sample_count_) + ", ";
      if (sample_count_ < trigger_count * sample_ratio_thresh) {
        *info += "[ref sample ratio too low] ";
        return -1;
      }
      int left = 1;
      int right = 1;
      std::priority_queue<struct RefCandidate> pq;
      int counts = 0;
      int counts_old = counts;
      int64_t sum_time = 0;
      *info += "collect: [count, avg_time]:{ ";
      while (right < kMaxBucketSize) {
        counts += buckets_[right];
        sum_time += right * buckets_[right];
        right++;
        while (right - left >= kMaxSearchWindow) {
          if (counts > trigger_count * ref_ratio_thresh) {
            // to avoid enque too many candicates of the same counts
            if (counts > counts_old) {
              int64_t avg_time = sum_time * (1UL << InnoConsts::kSubNsBits) /
                                (counts * kSampleCountInOneNs);
              pq.emplace(counts, avg_time);
              *info += "[" + std::to_string(counts) + ", " +
                      std::to_string(avg_time) + "] ";
            }
            counts_old = counts;
          } else {
            counts_old = 0;
          }
          counts -= buckets_[left];
          sum_time -= left * buckets_[left];
          left++;
        }
      }
      *info += "}; pick: {";
      // now we finished collecting candicates from the histogram
      // and will select the reference
      int ret = 0;
      if (!pq.empty()) {
        int ref_center = pq.top().avg_time;
        pq.pop();
        *info += "1st: " + std::to_string(ref_center);
        *info += ", 2nd: ";
        int pick_times = kPeakNumSelectRef - 1;
        while (!pq.empty() && pick_times > 0) {
          int ref_second_center = pq.top().avg_time;
          pq.pop();
          *info += std::to_string(ref_second_center) + ", ";
          if (abs(ref_center - ref_second_center) <= kMinTimeToUpdate) {
            continue;
          }
          pick_times--;
          if (ref_second_center < ref_center) {
            ref_center = ref_second_center;
          }
        }
        *ref_time = ref_center;
      } else {
        *info += "[queue is empty, no possible reference cluster]";
        ret = -1;
      }
      *info += "}";
      return ret;
    }

    inline void add_sample(InnoFpgaSubNs time_to_trigger) {
      int64_t index = time_to_trigger * kSampleCountInOneNs;
      index /= (1UL << InnoConsts::kSubNsBits);
      // xxx todo(WYY): if we limit the search range to certain bound,
      // we can remove this if branch
      if (index < kMaxBucketSize) {
        buckets_[index]++;
        sample_count_++;
      } else {
        static uint32_t bound_access_count = 0;
        bound_access_count++;
        if (bound_access_count < 30 ||
            bound_access_count % 1024 == 0) {
          inno_log_warning("time_to_trigger(%" PRI_SIZED
                            ") occurs bound access %" PRI_SIZED "/%d, "
                           "please check bucket size for channel: %d",
                            time_to_trigger,
                            index, kMaxBucketSize, channel_);
        }
      }
    }
    void add_trigger_count(size_t trigger_count) {
      trigger_counts_ += trigger_count;
    }
    size_t get_trigger_count() {
      return trigger_counts_;
    }

    void reset_stats(int count = 1) {
      sample_count_ = 0;
      trigger_counts_ = 0;
      memset(buckets_, 0, sizeof(buckets_));
    }

    inline InnoSlidingMean& get_stastic_info() {
      return stastic_info_;
    }

    inline void pop() {
      stastic_info_.pop();
    }

    inline int q_size() {
      return stastic_info_.size();
    }

    inline bool q_is_full() {
      return stastic_info_.is_full();
    }

    inline bool q_is_empty() {
      return stastic_info_.is_empty();
    }

    inline void clear() {
      stastic_info_.clear();
    }

    void inc_sample_interval(int min_index) {
      sample_interval_index_++;
      if (sample_interval_index_ > kVariableSampleNum - 1) {
        sample_interval_index_ = kVariableSampleNum - 1;
      }
    }

    void dec_sample_interval(int min_index, int offset = 1) {
      sample_interval_index_ -= offset;
      if (sample_interval_index_ < min_index) {
        sample_interval_index_ = min_index;
      }
    }

    inline int get_sample_interval() {
      return (1U << sample_interval_index_);
    }

    inline int get_min_sample_index() {
      return kMinSampleIndex;
    }

    inline bool inited() {
      return inited_;
    }

    inline void set_inited() {
      inited_ = true;
    }

   private:
    static const int kSampleCountInOneNs = 4;  // 4 represents 1ns
    static const int kPeakNumSelectRef = 2;
    static const int kMinSampleIndex = 4;
    static const int kVariableSampleNum = 8;
    static const int kMaxSearchWindow = 3 * kSampleCountInOneNs;  // 3ns
    static const int kMinTimeToUpdate =
                     (5 * (1UL << InnoConsts::kSubNsBits));  // 5ns
    static const int kMaxBucketSize = 36 * kSampleCountInOneNs;  // 0~36ns
    static const int kMaxSampleInterval = 40;
    static constexpr double sample_ratio_thresh = 0.60;
    static constexpr double ref_ratio_thresh = 0.50;

   private:
    uint32_t buckets_[kMaxBucketSize];
    InnoSlidingMean stastic_info_;
    size_t trigger_counts_;
    int sample_count_;
    int channel_ = -1;
    int sample_interval_index_;
    bool inited_ = false;  // result is updated for at least once
  };

 public:
  static int process(void *job, void *ctx, bool prefer);

#define SUB_INNO_DIST_BITS 12
  static inline int32_t sub_ns_to_inno_distance_unit_(InnoSubNsOffset t) {
    return (int32_t)(kLightTraveInnoDistanceUnitPerSubNs * t);
  }

 public:
  explicit StageSignal(InnoLidar *l, double play_rate_x);
  ~StageSignal(void);

 public:
  void print_stats(void) const;
  void get_stats_string(char *buf, size_t buf_size) const;

  void set_save_raw_data_flag(std::string cause);
  size_t get_ref_sample_rate(uint32_t channel) {
    return ref_sample_rate_[channel];
  }

  inline InnoFpgaSubNs get_ref_center(int channel, int roi) {
    return ref_tracking_[roi][channel].\
           get_current_virtual_ref_offset_sub_ns();
  }

  inline InnoFpgaSubNs get_ref_center_result(int channel) {
    return ref_tracking_result_[channel].get_virtual_ref_offset();
  }

  inline uint32_t get_search_window_l(int channel, int roi) {
    return ref_tracking_[roi][channel].get_search_window_l();
  }

  inline uint32_t get_search_window_r(int channel, int roi) {
    return ref_tracking_[roi][channel].get_search_window_r();
  }

 private:
  const char *get_name_(void) const;
  int process_job_(StageSignalJob *job, bool prefer);
  uint64_t process_job_buffer_(const char *start, const char *end);
  const char *process_job_magic_(const char *start, const char *end);

  //
  std::mutex fault_done_mutex_;
  bool fault_done_{true};

  std::thread *fault_thread_{nullptr};
  void report_fault_();
  void update_time_status_(uint64_t time_now_ms);

  //
  int detect_signal_fault_(enum InnoLidarMode current_mode,
                           enum InnoLidarStatus status);
  void update_polygon_limit_(InnoLidarMode mode);
  void update_galvo_limit_(InnoLidarMode mode);
  void update_trigger_limit_(InnoLidarMode mode);
  int inc_job_count_();
  void reset_state_();

  int add_to_raw_deque_(StageSignalJob *job);
  void clear_deque_wo_lock_(std::deque<StageSignalJob *> *q);

  int raw2_recorder_(StageSignalJob *job);
  int raw2_send_();

  void raw3_raw4_recorder_();

  bool raw3_is_enable_();
  int raw3_recorder_();

  bool raw4_is_enable_();
  int raw4_recorder_();
  int raw4_send_();
  void do_optical_check_(size_t trigger_count);
  void update_ref_info_(enum InnoLidarMode current_mode,
                        enum InnoLidarMode pre_mode);

  InnoFpgaNs process_adc_data_clock_(const AdcComputedDataExt &adc);
  InnoFpgaNs process_trigger_data_clock_(const TriggerData &trigger);
  InnoFpgaSubNs process_encoder_data_clock_(const EncoderData &encoder);

  void process_pulse_to_point_(const AdcComputedDataExt &adc,
                               int32_t distance_0,
                               int32_t distance_1,
                               int32_t intensity_seq,
                               int32_t intensity,
                               int32_t point_type,
                               RawChannelPoint *p);
  void process_adc_data_pulse_(const AdcComputedDataExt &adc,
                               const InnoFpgaSubNs pulse_sub_ns,
                               int32_t channel,
                               int32_t intensity_seq,
                               int32_t intensity,
                               int32_t point_type,
                               RawBlock *block);
  void process_adc_data_(const AdcComputedDataExt &);
  void process_trigger_data_(const TriggerData &);
  void process_encoder_galvo_(const EncoderData &encoder);
  void check_and_insert_fake_polygon(InnoEpNs calc_ns,
                                     InnoEpNs curr_ns);
  void process_encoder_polygon_(InnoFpgaNs fpga_sub_ns, uint16_t fake_index);
  void process_encoder_polygon_(const EncoderData &encoder);
  void process_encoder_data_(const EncoderData &);
  void process_real_encoder_polygon_(const EncoderData &encoder);
  void process_padding_data_(const PaddingData &);
  int32_t galvo_find_angle_unit_(int32_t galvo_value);
  inline InnoFpgaSubNs galvo_adjust_sub_ns_(InnoFpgaSubNs in) {
    return in - InnoConverts::ns_to_sub_ns(config_.g_encoder_delay);
  }

  inline InnoFrameDirection get_galvo_direction_(uint32_t channel) const {
    return up_scan_;
  }

  void init_job_conf_seq_(ScanLines* job);
  void rate_control_(InnoTimestampUs last_data_us);
  void config_chA_ref_sample_rate_();
  void find_reference_(size_t trigger_count);
  void expand_ref_window_();

 public:
  static const double kLightTraveInnoDistanceUnitPerSubNs;

 public:
  static const size_t kWriteBlockSize = 64 * 1024;
  static const size_t kMaxLeftoverSize = 64;
  static const uint32_t kMaxPolygonEncodes = 10;

 private:
  static const uint32_t kRoiTypeCount = 2;
  static const size_t kDataTypeSize_[16];
  static const InnoFpgaSubNs kMaxToTriggerSubNs =
      (InnoConsts::kNsInSecond / InnoConsts::kMinTriggerPerSecond)
      << (InnoConsts::kSubNsBits);
  // xxx todo: enforce power of 2
  static const size_t kRefRealSampleRate = 16;  // must be power of 2
  static const size_t kRefFpgaSampleRate = 16;  // must be power of 2
  static const size_t kRefSampleRate[kInnoChannelNumber];
  // 50 times of max trigger interval as tolerance
  static const size_t kMaxTimeBetwenTriggers =
         (InnoConsts::kNsInSecond / InnoConsts::kMinTriggerPerSecond) * 50;
  static const uint32_t kIgnoreBeginFrameBlockage = 500;
  static const uint32_t kIgnoreBeginFrameRefIntensity = 250;
  static const uint32_t kMaxFreqUpdateFrames = 16;

 private:
  static const uint64_t kSignalFaultStartDetect = 1000;
  static const uint64_t kFastAdjustIntervalThresh = 5 * 32;  // 5ns
  static const double kMinStableTimeAfterStreamStart;
  static const double kMinStableTimeAfterRawDrop;
  InnoLidar *lidar_;
  double play_rate_x_;
  InnoTimestampUs start_time_us_;
  InnoTimestampUs first_data_us_;

  const LidarParams *params_;
  const MiscTables *misc_tables_;

  std::mutex mutex_;
  StageSignalClock fpga_clock_;
  StageSignalConfig config_base_;
  StageSignalConfig config_;

  char leftover_signal_[kMaxLeftoverSize];
  size_t leftover_signal_len_;

  // reference windows
  int in_roi_;
  size_t trigger_count_[kRoiTypeCount];
  size_t trigger_count_old_[kRoiTypeCount];

  // trigger and blocks
  RefTracking ref_tracking_[kRoiTypeCount][kInnoChannelNumber];
  RefTrackingResult ref_tracking_result_[kInnoChannelNumber];
  RefFinder ref_finder_[kInnoChannelNumber];
  InnoFpgaSubNs previous_trigger_sub_ns_;
  InnoFpgaSubNs current_trigger_sub_ns_;
  bool trigger_timeout_;
  bool galvo_timeout_;
  bool polygon_timeout_;
  int64_t motor_trigger_drop_;
  int64_t diff_polygon_ns_TO_;
  int64_t diff_galvo_ns_TO_;
  uint32_t trigger_period_ns_TO_;
  bool rawdata_fault_;
  uint32_t polygon_range_;
  bool update_limit_;
  RawBlock *previous_raw_block_;
  RawBlock *current_raw_block_;

  // galvo
  double galvo_slope_;
  double galvo_min_angle_;

  double dis_corr_slope_[kInnoChannelNumber];

  size_t job_count_;

  bool set_first_firing_cycle_real_;
  bool live_lidar_;

  size_t this_round_g_encoder_count_;

  // stats
  size_t stats_g_encoder_count_;
  size_t last_g_encoder_count_;
  size_t stats_pr_encoder_count_;
  size_t last_pr_encoder_count_;
  size_t stats_pf_encoder_count_;
  size_t stats_padding_count_;
  size_t stats_trigger_count_;
  size_t last_trigger_count_;
  size_t stats_trigger_count_old_;
  size_t stats_adc_count_;
  size_t stats_pulse_count_;
  size_t stats_ref_count_;
  size_t stats_ref_track_count_;
  size_t stats_previous_pulse_0_;
  size_t stats_previous_pulse_1_;
  size_t stats_discarded_multi_return_;
  size_t stats_scatter_;
  size_t stats_road_points_;
  size_t stats_retro_;
  size_t stats_discard_pre_retro_;
  size_t stats_discard_post_retro_;
  size_t stats_below_min_intensity_;
  ssize_t stats_frame_counts_;
  ssize_t stats_ref_unfit_count_[kInnoChannelNumber];
  ssize_t stats_ref_fit_count_[kInnoChannelNumber];
  ssize_t stats_total_ref_check_count_[kInnoChannelNumber];

  size_t stats_force_drop_0_;
  size_t stats_not_enough_g_0_;
  size_t stats_other_encoder_count_;
  size_t stats_bad_ts_0_;
  size_t stats_bad_ts_1_;
  size_t stats_discard_pulse_;
  size_t stats_discard_trigger_;
  size_t stats_discard_encoder_0_;
  size_t stats_discard_encoder_1_;
  size_t stats_discard_encoder_2_;
  size_t stats_galvo_delay_;
  size_t stats_ignore_polygon_no_active_;
  size_t stats_ignore_polygon_small_gap_;

  size_t stats_road_0_;
  size_t stats_road_1_;
  uint64_t stats_ref_intensity_sum_;

  uint64_t init_ts_;
  double raw_data_drop_ts_;
  uint64_t stats_trigger_interval_too_big_;
  PolygonStamp polygons_[kMaxPolygonEncodes];
  uint16_t fake_polygon_index_;
  bool is_calc_fake_polygon_;
  InnoFpgaNs next_fake_fpga_ns_;
  InnoFpgaNs last_polygon_ns_;
  InnoFpgaNs last_real_polygon_ns_;
  InnoMean polygon_mean_;
  InnoMean polygon_real_mean_;
  InnoFpgaNs curr_polygon_pre_ns_;
  InnoFpgaNs last_galvo_change_direction_ns_;
  InnoFrameDirection up_scan_;
  InnoFrameDirection pre_galvo_direction_;
  InnoMean galvo_mean_[INNO_FRAME_DIRECTION_MAX];

  static constexpr int stats_signals_size_ =
      sizeof(InnoStatusCounters::in_signals) /
      sizeof(InnoStatusCounters::in_signals[0]);

  // the same as InnoStatusCounters::signals
  static constexpr int kMaxSignalTypeBits = 4;
  uint32_t stats_signals_[0x1 << kMaxSignalTypeBits]{0};
  uint32_t ignore_begin_frames_blockage_;
  uint32_t ignore_begin_frames_ref_intensity_;
  uint64_t signal_fault_begin_detection_;

  // scanlines
  ScanLines *pre_lines_;
  ScanLines *cur_lines_;
  ScanLines *current_block_source_lines_;

  StageSignalJob *current_job_;

  //
  std::deque<StageSignalJob *> raw_deque_;

  //
  std::mutex raw_cause_mutex_;
  std::deque<std::string> raw_cause_;

  // raw2
  uint64_t raw2_call_count_;
  uint64_t frame_total_points_[kInnoChannelNumber];
  bool raw2_shutdown_;
  bool raw2_done_;
  std::deque<StageSignalJob *> raw2_send_deque_;

  std::mutex raw2_mutex_;
  std::condition_variable raw2_cond_;
  std::thread *raw2_thread_;

  // raw3
  bool raw_data_record_enable_;

  // raw4
  char sn_[InnoStatusPacket::kSnSize]{0};

  std::thread *raw4_thread_;
  std::deque<StageSignalJob *> raw4_send_deque_;

  std::mutex raw4_done_mutex_;
  bool raw4_done_;
  bool has_ref_old_[kInnoChannelNumber];
  OpticalPathCheck* optical_path_check_;
  bool need_find_magic_;
  uint64_t find_magic_tried_;
  uint32_t conf_seq_num_[INNO_CONFIDENCE_LEVEL_MAX]{0};
  int64_t ref_max_intensity_;
  int64_t ref_min_intensity_;
  size_t ref_sample_rate_[kInnoChannelNumber];
  enum InnoLidarMode current_mode_old_;
};

}  // namespace innovusion

#endif  // SDK_STAGE_SIGNAL_H_
