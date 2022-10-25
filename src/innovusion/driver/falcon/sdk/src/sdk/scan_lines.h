/*
 *  Copyright (C) 2021 Innovusion Inc.
 *
 *  License: BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef SDK_SCAN_LINES_H_
#define SDK_SCAN_LINES_H_

#include <mutex>  // NOLINT

#include "sdk_common/inno_lidar_packet.h"
#include "sdk/stage_signal.h"
#include "utils/log.h"
#include "utils/types_consts.h"

namespace innovusion {
class InnoLidar;

class __attribute__((packed)) PackedFields {
 public:
  union {
    struct {
      uint32_t radius:18;  // distance in inno_distance_unit, 0 means no point
      uint32_t support_count_is_good:1;
      uint32_t support_required:1;
      uint32_t need_support:1;
      uint32_t is_real:1;
    };
    struct {
      uint32_t others:18;
      uint32_t noise_field:4;
    };
  };
};

class __attribute__((packed)) RawChannelPoint {
 public:
  inline uint32_t get_raw_intensity() const {
    return raw_intensity;
  }

  inline void set_raw_intensity(uint32_t intensity) {
    raw_intensity = intensity;
  }

  inline uint32_t get_intensity_seq() const {
    return intensity_seq;
  }

  inline void set_intensity_seq(uint32_t intensity_seq_to_set) {
    intensity_seq = intensity_seq_to_set;
  }

  inline uint8_t get_type() {
    return type;
  }

  inline void set_type(uint8_t type_to_set) {
    type = type_to_set;
  }

  inline uint8_t get_elongation() {
    return elongation;
  }

  inline void set_elongation(uint8_t elongation_to_set) {
    elongation = elongation_to_set;
  }

  inline uint32_t get_radius(int f) const {
    return packed_fileds[f].radius;
  }

  inline void set_radius(int f, uint32_t radius) {
    packed_fileds[f].radius = radius;
  }

  inline uint8_t support_count_is_good(int f) {
    return packed_fileds[f].support_count_is_good;
  }

  inline void set_support_count_is_good(int f, uint8_t is_good) {
    packed_fileds[f].support_count_is_good = is_good;
  }

  inline uint8_t is_support_required(int f) {
    return packed_fileds[f].support_required;
  }

  inline void set_support_required(int f, uint8_t is_required) {
    packed_fileds[f].support_required = is_required;
  }

  inline uint8_t is_need_support(int f) {
    return packed_fileds[f].need_support;
  }

  inline void set_need_support(int f, uint8_t need_support) {
    packed_fileds[f].need_support = need_support;
  }

  inline uint8_t is_real(int f) {
    return packed_fileds[f].is_real;
  }

  inline void set_is_real(int f, uint8_t is_real) {
    packed_fileds[f].is_real = is_real;
  }

  inline void clear_noise_field() {
    packed_fileds[0].noise_field = 0;
    packed_fileds[1].noise_field = 0;
  }

 public:
  /* 
   * radius[0]: distance if from this firing cirle
   * radius[1]: distance if from previous firing cirle
   * radius are populated before StageDistance,
   * StageDistance needs to pick one or delete the point (set radius_0 to 0);
   */
  uint16_t intensity_seq;
  uint8_t type;
  uint8_t elongation;
  uint32_t raw_intensity;
  PackedFields packed_fileds[2];
};

class RawBlock {
 public:
  // 8 bytes
  InnoEpNs trigger_ts_ns;             // ns since fpga powered up
  // offset to current trigger
  // 16 bytes
  InnoSubNsOffset ref_ts_sub_ns_off[kInnoChannelNumber];
  // 16 bytes
  uint16_t ref_intensity[kInnoChannelNumber];
  uint16_t scatter_intensity[kInnoChannelNumber];
  // 17 bytes
  InnoBlockHeader header;  // will be used to populate the InnoBlock
  // use 4 bits for 4 channels, channel 0 uses least significant bit
  uint8_t has_retro;
  // uint8_t points_number;  // number of non-zero points
  // 2 bytes
  uint16_t trigger_period;  // time interval between current/previous triggers
  uint16_t poly_angle;     // polygon angle
  uint16_t galvo_angle;    // galvo angle
  // make points start at 64 byte align
  RawChannelPoint points[kInnoChannelNumber][kInnoMaxMultiReturn];
};

class EncoderSignal {
 public:
  uint64_t idx;  // do we need this?
  InnoFpgaSubNs ts_sub_ns;
  int32_t angle;  // -PI to PI
  enum InnoFrameDirection up_scan;
  inline void set_polygon(uint64_t ix, InnoFpgaSubNs ns, int32_t value) {
    idx = ix;
    ts_sub_ns = ns;
    angle = value;
  }
  inline void set_galvo(uint64_t ix, InnoFpgaSubNs ns,
                        int32_t value,
                        enum InnoFrameDirection up) {
    idx = ix;
    ts_sub_ns = ns;
    angle = value;
    up_scan = up;
  }
};

class ScanLine {
 public:
  static const size_t kMaxScanLinePolygonSignalNum = 3;
  static const size_t kMaxScanLineGalvoSignalNum =
      InnoConsts::kGalvoEncoderHz * InnoConsts::kSecondInMinute /
      InnoConsts::kPolygonFacet / InnoConsts::kMinPolygonRPM + 1;
  static const size_t kMaxBlocksBetweenP =
      InnoConsts::kMaxTriggerPerSecond * InnoConsts::kSecondInMinute /
      InnoConsts::kMinPolygonRPM + 1;
  static const size_t kMaxBlocksInScanLine =
      InnoConsts::kMaxTriggerPerSecond * InnoConsts::kSecondInMinute /
      InnoConsts::kMinPolygonRPM / InnoConsts::kPolygonFacet + 1;

 public:
  uint64_t frame_idx;  // which frame this scanline belongs to
  // uint64_t pre_frame_idx;
  uint64_t idx;
  uint32_t facet_id;

  /* t[0] is the biggest encoder signal <= all ts in this scanline */
  /* contains one t[] who is bigger than all ts in this scanline */
  /*
  uint32_t p_signals_number;
  uint32_t g_signals_number;
  EncoderSignal p_signals[kMaxScanLinePolygonSignalNum];
  EncoderSignal g_signals[kMaxScanLineGalvoSignalNum];
  */
  uint32_t active_blocks_start;
  uint32_t active_blocks_end;  // start + number
  uint32_t active_blocks_number;
  int32_t galvo_angle_change;

  InnoFrameDirection direction;
  bool end_of_frame;

  bool has_problematic_blocks;
};


/*
 * This is the basically job structure generated by StageSignal and 
 * processed by StageAngle and StageDistance
 * StageSignal: process encoder/pulse/trigger signals to populate
 *              ScanLines. The Will send the ScanLines to StageAngle once it
 *              received 2 polygon signals. examples
 *              ScanLines 1: polygon-A, polygon-A+1
 *                           galvo-F....galvo-F+m
 *                           all pulses in [polygon-A and polygon-A+1)
 *                           and their ts all in [galvo-F, galvo-F+m)
 *                           galvo-
 *              ScanLines 2: polygon-A+1, polygon-A+2
 *                           galvo-F+m....galvo-F+m+n
 *                           all pulses in [polygon-A+1 and polygon-A+2)
 *                           and their ts all in [galvo-F+m, galvo-F+m+n)
 * 
 * StageAngle: process ScanLines and convert ts to galvo+polygan angles, also
 *             setup ScanLine structures. may need to copy partial Scanline in
 *             the previous ScanLine. The output ScanLines always contains the
 *             complete scanlines (they may be from different frames though)
 */
class ScanLines {
 public:
  enum StageTime {
    STAGE_TIME_R0 = 0,
    STAGE_TIME_R1 = 1,
    STAGE_TIME_S0 = 2,
    STAGE_TIME_S1 = 3,
    STAGE_TIME_A0 = 4,
    STAGE_TIME_A1 = 5,
    STAGE_TIME_NA0 = 6,
    STAGE_TIME_NA1 = 7,
    STAGE_TIME_NB0 = 8,
    STAGE_TIME_NB1 = 9,
    STAGE_TIME_D0 = 10,
    STAGE_TIME_D1 = 11,
    STAGE_TIME_MAX = 12
  };

 public:
  static const size_t kMaxPolygonSignalNum = 3;
  static const size_t kMaxGalvoSignalNum =
      InnoConsts::kGalvoEncoderHz * InnoConsts::kSecondInMinute /
      InnoConsts::kMinPolygonRPM * 2 + 1;

 public:
  static inline void reset_block(RawBlock *block,
                                 bool in_roi,
                                 InnoFpgaNs fpga_ns,
                                 uint16_t trigger_period,
                                 StageSignal::RefTrackingResult *trackings) {
    block->header.in_roi = in_roi;
    block->trigger_ts_ns = fpga_ns;
    block->trigger_period = trigger_period;
    block->has_retro = 0;
    for (uint32_t channel = 0; channel < kInnoChannelNumber; channel ++) {
      block->ref_ts_sub_ns_off[channel] =
        trackings[channel].get_virtual_ref_offset();
      block->ref_intensity[channel] = 0;
      block->scatter_intensity[channel] = 0;
      RawChannelPoint *pp = &block->points[channel][0];
      for (uint32_t i = 0; i < kInnoMaxMultiReturn; i++, pp++) {
        pp->set_raw_intensity(0);
        pp->set_radius(0, 0);
        pp->set_radius(1, 0);
        pp->clear_noise_field();
      }
    }
  }

  static inline void set_ref_intensity(RawBlock *block,
                                       int32_t channel,
                                       int32_t intensity) {
    block->ref_intensity[channel] = intensity;
  }

 public:
  explicit ScanLines(size_t allocated_block, uint32_t pre_polygons = 1) {
    lines_number = 0;
    lines_processed[0] = 0;
    lines_processed[1] = 0;
    frame_count = 1;
    allocated_blocks_number_ = allocated_block;
    // kMaxBlocksBetweenP = InnoConsts::kMaxTriggerPerSecond *
    // InnoConsts::kSecondInMinute/InnoConsts::kMinPolygonRPM/pre_polygons + 1,
    inno_log_assert(allocated_block >=
                    ScanLine::kMaxBlocksInScanLine +
    InnoConsts::kMaxTriggerPerSecond * InnoConsts::kSecondInMinute /
    InnoConsts::kMinPolygonRPM / pre_polygons + 1,
                    "too small: %lu, %lu",
                    allocated_block,
                    ScanLine::kMaxBlocksInScanLine +
    InnoConsts::kMaxTriggerPerSecond * InnoConsts::kSecondInMinute /
    InnoConsts::kMinPolygonRPM / pre_polygons + 1);
    active_blocks_start_ = ScanLine::kMaxBlocksInScanLine;
    active_blocks_number_ = 0;
    last_allocated_blocks_ = blocks + allocated_blocks_number_ - 1;
    in_calibration_mode_ = false;
    polygons_number_ = 0;
    galvos_number_ = 0;
    remove_invalid_galvo_num_ = 0;
    adc_closed = false;
    polygon_closed = false;
    galvo_closed = false;
    ref_count_ = 1;
    time_sync_state = INNO_TIME_SYNC_TYPE_NONE;
    host_lidar_time_diff_sec = 0;
    auto_galvo_mode_ = INNO_GALVO_MODE_NONE;

    memset(stage_ts, 0, sizeof(stage_ts));
    memset(frame_total_points_, 0, sizeof(frame_total_points_));
  }

  ~ScanLines() {
    inno_log_verify(ref_count_ == 0, "ref=%d", ref_count_);
  }

  inline size_t get_size() const {
    return sizeof(this) + sizeof(RawBlock) * allocated_blocks_number_;
  }

  inline void inc_ref(void) {
    std::unique_lock<std::mutex> lk(mutex_);
    ref_count_++;
  }

  inline bool dec_ref(void) {
    std::unique_lock<std::mutex> lk(mutex_);
    ref_count_--;
    bool reach_zero = ref_count_ == 0;
    return reach_zero;
  }
  inline bool in_calibration_mode() const {
    return in_calibration_mode_;
  }
  inline void set_calibration_mode() {
    in_calibration_mode_ = true;
  }
  inline int add_polygon(uint64_t ix, InnoFpgaSubNs ns, int32_t value) {
    if (polygons_number_ < kMaxPolygonSignalNum) {
      polygons[polygons_number_].set_polygon(
          ix, ns, value);
      polygons_number_++;
      return 0;
    } else {
      return -1;
    }
  }

  inline int get_diff_polygon_sub_ns(int64_t* t_diff) {
    if (polygons_number_ < 2) {
      return -1;
    }
    *t_diff = polygons[polygons_number_ - 1].ts_sub_ns -
              polygons[polygons_number_ - 2].ts_sub_ns;
    return 0;
  }

  inline void reini_scanlines(uint64_t ix_p, InnoFpgaSubNs ns,
                                     int32_t angle) {
    if (galvos_number_ > 0) {
      remove_invalid_galvo_num_ += galvos_number() - 1;
      EncoderSignal *last_galvo = get_last_galvo();
      inno_log_verify(last_galvo, "last_galvo");
      galvos_number_ = 0;
      // init galvos[0] with latest galvo
      add_galvo(last_galvo->idx, last_galvo->ts_sub_ns, last_galvo->angle,
                                                 last_galvo->up_scan);
      // clear remain galvos
      memset(&galvos[1], 0, sizeof(galvos) - sizeof(galvos[0]));
    }
    update_exist_polygon_info(ix_p, ns, angle);
  }

  inline uint32_t get_invalid_galvo_num() const {
    return remove_invalid_galvo_num_;
  }

  inline int add_galvo(uint64_t ix, InnoFpgaSubNs ns,
                       int32_t value, enum InnoFrameDirection up) {
    if (galvos_number_ < kMaxGalvoSignalNum) {
      galvos[galvos_number_].set_galvo(
          ix, ns, value, up);
      galvos_number_++;
      return 0;
    } else {
      return -1;
    }
  }

  inline int64_t get_diff_galvo_sub_ns() {
    inno_log_verify(galvos_number_ >= 2,
                    "galvos_number_ = %u", galvos_number_);
    return  galvos[galvos_number_ - 1].ts_sub_ns -
            galvos[galvos_number_ - 2].ts_sub_ns;
  }

  inline int copy_galvo(const EncoderSignal &src) {
    if (galvos_number_ < kMaxGalvoSignalNum) {
      galvos[galvos_number_] = src;
      galvos_number_++;
      return 0;
    } else {
      return -1;
    }
  }

  inline EncoderSignal *get_first_polygon(void) {
    if (polygons_number_ > 0) {
      return &polygons[0];
    } else {
      return NULL;
    }
  }

  inline EncoderSignal *get_last_polygon(void) {
    if (polygons_number_ > 0) {
      return &polygons[polygons_number_ - 1];
    } else {
      return NULL;
    }
  }

  inline EncoderSignal *get_first_galvo(void) {
    if (galvos_number_ > 0) {
      return &galvos[0];
    } else {
      return NULL;
    }
  }

  inline EncoderSignal *get_last_galvo(void) {
    if (galvos_number_ > 0) {
      return &galvos[galvos_number_ - 1];
    } else {
      return NULL;
    }
  }

  inline EncoderSignal *iterate_polygons(int32_t *idx) {
    EncoderSignal *ret;
    inno_log_assert(idx, "idx");
    if (*idx >=0 && *idx < (int32_t)polygons_number_) {
      ret = &polygons[*idx];
      (*idx)++;
      return ret;
    } else {
      return NULL;
    }
  }

  inline EncoderSignal *iterate_galvos(int32_t *idx) {
    EncoderSignal *ret;
    inno_log_assert(idx, "idx");
    if (*idx >=0 && *idx < (int32_t)galvos_number_) {
      ret = &galvos[*idx];
      (*idx)++;
      return ret;
    } else {
      return NULL;
    }
  }

  inline uint32_t polygons_number() const {
    return polygons_number_;
  }

  inline uint32_t galvos_number() const {
    return galvos_number_;
  }

  inline uint32_t active_blocks_number() const {
    return active_blocks_number_;
  }

  inline RawBlock *allocate_first_block(void) {
    inno_log_verify(active_blocks_number_ == 0,
                    "active = %u", active_blocks_number_);
    active_blocks_number_ = 1;
    return &blocks[active_blocks_start_];
  }

  inline int append_from_source(const ScanLines &src,
                                int32_t copy_start) {
    inno_log_verify(copy_start >= 0,
                    "%d cannot copy negative", copy_start);
    inno_log_verify(src.active_blocks_number_ > 0,
                    "no active to copy");
    inno_log_verify((int32_t)src.active_blocks_number_ >= copy_start,
                    "%d vs %u",
                    copy_start,
                    src.active_blocks_number_);
    size_t to_copy = src.active_blocks_number_ - copy_start;
    if (active_blocks_start_ > to_copy) {  // enough space?
      memcpy(&blocks[active_blocks_start_ - to_copy],
             &src.blocks[src.active_blocks_start_ + copy_start],
             sizeof(blocks[0]) * to_copy);
      active_blocks_start_ -= to_copy;
      active_blocks_number_ += to_copy;
      return to_copy;
    } else {
      inno_log_verify(active_blocks_start_ > to_copy,
                      "%u > %u - %d",
                      active_blocks_start_,
                      src.active_blocks_number_,
                      copy_start);
      return -1;
    }
  }

  inline uint32_t get_active_block_storage_idx(size_t idx) {
    inno_log_verify(active_blocks_number_ > idx,
                    "active = %u > %lu",
                    active_blocks_number_, idx);
    return active_blocks_start_ + idx;
  }

  inline RawBlock &get_active_block(size_t idx) {
    return blocks[get_active_block_storage_idx(idx)];
  }

  inline RawBlock &get_first_active_block() {
    return get_active_block(0);
  }

  inline const RawBlock &get_last_active_block_const(void) const {
    inno_log_verify(active_blocks_number_ > 0,
                    "active = %u", active_blocks_number_);
    return blocks[active_blocks_end_() - 1];
  }

  inline bool can_advance_block(const RawBlock *block) const {
    return block < last_allocated_blocks_;
  }

  inline void advance_block(RawBlock **block) {
    active_blocks_number_++;
    (*block)++;
    return;
  }

  inline ScanLine &allocate_line(void) {
    inno_log_verify(lines_number < kMaxScanLineNum,
                    "too many lines %u", lines_number);
    lines_number++;
    return lines[lines_number - 1];
  }

  inline void add_frame_points(uint32_t channel, InnoFrameDirection direction) {
    frame_total_points_[direction][channel]++;
  }

  inline int32_t get_frame_points(uint32_t channel,
                                  InnoFrameDirection direction) {
    return frame_total_points_[direction][channel];
  }

  inline void set_confidence_seq(enum ConfidenceLevel conf_level,
                                 uint32_t seq_number) {
    conf_seq_num_[conf_level] = seq_number;
  }

  inline uint32_t get_confidence_seq(enum ConfidenceLevel conf_level) {
    return conf_seq_num_[conf_level];
  }

  inline void set_auto_galvo_mode(enum InnoGalvoMode auto_galvo_mode) {
    auto_galvo_mode_ = auto_galvo_mode;
  }

  inline enum InnoGalvoMode get_auto_galvo_mode() {
    return auto_galvo_mode_;
  }

 private:
  uint32_t active_blocks_end_() const {
    return active_blocks_start_ + active_blocks_number_;
  }

  inline void update_exist_polygon_info(uint64_t ix, InnoFpgaSubNs ns,
                                                   int32_t angle) {
    inno_log_verify(polygons_number_ == 1,
                    "polygons_number_ invalid: %u", polygons_number_);
    inno_log_verify(active_blocks_number_ == 0,
                    "active_blocks_number_ invalid: %u",
                    active_blocks_number_);
    polygons[polygons_number_ - 1].set_polygon(ix, ns, angle);
  }

 public:
  static const size_t kMaxScanLineNum = InnoConsts::kPolygonFacet + 1 + 1;
  static const size_t kMaxStageNum = 10;

 private:
  /* open question: do we need padding around scanlines, scanline? */
  uint32_t allocated_blocks_number_;
  uint32_t polygons_number_;
  uint32_t galvos_number_;
  uint32_t remove_invalid_galvo_num_;

  uint32_t active_blocks_start_;
  uint32_t active_blocks_number_;
  RawBlock *last_allocated_blocks_;

  bool in_calibration_mode_;
  enum InnoGalvoMode auto_galvo_mode_;

  int32_t ref_count_;
  uint32_t frame_total_points_[INNO_FRAME_DIRECTION_MAX][kInnoChannelNumber];
  uint32_t conf_seq_num_[INNO_CONFIDENCE_LEVEL_MAX]{0};
  std::mutex mutex_;

 public:
  uint64_t idx;
  ScanLine lines[kMaxScanLineNum];
  uint32_t lines_number;  // number of scan lines
  uint32_t lines_processed[2];  // last line having gone through noise filtering
  uint32_t frame_count;

  /* not used in distance stage */
  EncoderSignal polygons[kMaxPolygonSignalNum];
  EncoderSignal galvos[kMaxGalvoSignalNum];
  bool adc_closed;
  bool polygon_closed;
  bool galvo_closed;

  enum InnoTimeSyncType time_sync_state;
  double host_lidar_time_diff_sec;

  InnoEpSecondDouble stage_ts[STAGE_TIME_MAX];

  char padding[56];
  RawBlock blocks[0];
};

}  // namespace innovusion
#endif  // SDK_SCAN_LINES_H_
