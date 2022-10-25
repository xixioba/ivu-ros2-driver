/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_CROSS_TALK_H_
#define SDK_CROSS_TALK_H_

#include <vector>

#include "sdk/stage_angle_job.h"
#include "sdk/misc_tables.h"

namespace innovusion {

class Pulse {
 private:
  uint32_t channel_;
  uint32_t signal_number_;
  uint32_t distance_;
  uint32_t intensity_;
  uint32_t intensity_seq_;

 public:
  Pulse(int channel,
        int signal_number,
        int distance,
        int intensity,
        int intensity_seq)
      : channel_(channel)
      , signal_number_(signal_number)
      , distance_(distance)
      , intensity_(intensity)
      , intensity_seq_(intensity_seq) {
  }
  ~Pulse() {}
  // getters
  inline uint32_t getChannel() const {
    return channel_;
  }
  inline uint32_t getSignalNumber() const {
    return signal_number_;
  }
  inline uint32_t getDistance() const {
    return distance_;
  }
  inline uint32_t getIntensity() const {
    return intensity_;
  }
  inline uint32_t getIntensitySeq() const {
    return intensity_seq_;
  }
  // setters
  inline void setChannel(uint32_t channel) {
    channel_ = channel;
  }
  inline void setSignalNumber(uint32_t signal_number) {
    signal_number_ = signal_number;
  }
  inline void setDistance(uint32_t distance) {
    distance_ = distance;
  }
  inline void setIntensity(uint32_t intensity) {
    intensity_ = intensity;
  }
  inline void setIntensitySeq(uint32_t intensity_seq) {
    intensity_seq_ = intensity_seq;
  }
};

class CrossTalk {
 private:
  uint32_t retro_intensity_;
  int32_t cross_talk_distance1_;
  int32_t cross_talk_distance2_;
  double ctr_[kInnoChannelNumber - 1];

  const MiscTables *misc_tables_;

 private:
  void find_all_pulses_within_distance_(RawBlock *block,
                                        const Pulse &target_pulse,
                              std::vector<Pulse> *pulses_within_distance);
  bool within_distance_threshold_(const uint32_t &target_distance,
                                  const uint32_t &distance2);

  void sequentially_calculate_inherent_intensity_(\
    std::vector<Pulse> *same_distance_pulses, const Pulse &retro_pulse);

  void subtract_crosstalk_from_neighboring_channel_(\
    std::vector<Pulse>::iterator neighbor_pulse,
    double retro_power_decimal, int retro_channel);

  double get_crosstalk_ratio_(int retro_channel,
                              int neighbor_channel);

  void overwrite_image_result_(RawBlock *block,
             std::vector<Pulse> *same_distance_pulses);

  void re_sort_im_result_ins_arrays_(RawBlock *block,
                                     int channel,
                                     int signal_num);

 public:
  explicit CrossTalk(const MiscTables *misc_tables)
      : misc_tables_(misc_tables) {
    for (uint32_t i = 0; i < kInnoChannelNumber; i++) {
      stats_cross_talk[i] = 0;
    }
  }

  ~CrossTalk() {
  }

  void set_params(int retro_intensity,
                  int32_t cross_talk_distance1,
                  int32_t cross_talk_distance2,
                  const double ctr[]) {
    retro_intensity_ = retro_intensity;
    cross_talk_distance1_ = cross_talk_distance1;
    cross_talk_distance2_ = cross_talk_distance2;
    for (uint32_t i = 0; i < kInnoChannelNumber - 1; i++) {
      ctr_[i] = ctr[i];
    }
  }

  void remove_cross_talk(RawBlock *block);

 public:
  size_t stats_cross_talk[kInnoChannelNumber];
};

}  // namespace innovusion

#endif  // SDK_CROSS_TALK_H_

