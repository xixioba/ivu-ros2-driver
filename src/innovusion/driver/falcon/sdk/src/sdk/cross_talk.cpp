/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "sdk/cross_talk.h"

namespace innovusion {
void CrossTalk::remove_cross_talk(RawBlock *block) {
  for (uint32_t channel = 0; channel < kInnoChannelNumber; channel++) {
    // only consider the first pulse,
    // since it is unlikely to have two retro pulses
    if (block->points[channel][0].get_raw_intensity() >= retro_intensity_) {
      Pulse retro_pulse(channel, 0,
                        block->points[channel][0].get_radius(0),
                        block->points[channel][0].get_raw_intensity(),
                        block->points[channel][0].get_intensity_seq());
      std::vector<Pulse> same_distance_pulses;
      find_all_pulses_within_distance_(block, retro_pulse,
                                       &same_distance_pulses);
      if (same_distance_pulses.size() == 0) {
        continue;
      }
      // for (auto pulse_it = same_distance_pulses.begin();
      // pulse_it != same_distance_pulses.end(); ++pulse_it) {
      //  inno_log_info("same distance pulses channel %d: "
      //                "intensity %d distance %f",
      //    pulse_it->getChannel(),
      //    pulse_it->getIntensity(),
      //    pulse_it->getDistance());
      // }

      sequentially_calculate_inherent_intensity_(&same_distance_pulses,
                                                 retro_pulse);
      // for (auto pulse_it = pulses_with_inherent_intensity.begin();
      // pulse_it != pulses_with_inherent_intensity.end(); ++pulse_it) {
      //  inno_log_info("pulses after subtraction channel %d: "
      //                "intensity %d distance %f",
      //    pulse_it->getChannel(),
      //    pulse_it->getIntensity(),
      //    pulse_it->getDistance());
      // }
      overwrite_image_result_(block, &same_distance_pulses);
    }
  }
}

inline void CrossTalk::find_all_pulses_within_distance_(RawBlock *block,
                                                    const Pulse &target_pulse,
                                   std::vector<Pulse> *pulses_within_distance) {
  // find 1 signal in each channel that is the closest to target_pulse?
  // currently, we break at the first "within distance" pulse found
  // in each channel
  uint32_t distance1 = target_pulse.getDistance();
  for (uint32_t channel2 = 0; channel2 < kInnoChannelNumber; channel2++) {
    if (channel2 == target_pulse.getChannel()) {
      continue;
    }
    for (uint32_t signal_num2 = 0;
         signal_num2 < kInnoMaxMultiReturn;
         signal_num2++) {
      uint32_t distance2 =
        block->points[channel2][signal_num2].get_radius(0);
      if (within_distance_threshold_(distance1, distance2) == true) {
        uint32_t intensity2 =
          block->points[channel2][signal_num2].get_raw_intensity();
        uint32_t intensity_seq2 =
          block->points[channel2][signal_num2].get_intensity_seq();
        // only if the second pulse is not retro itself
        if (intensity2 < retro_intensity_) {
          pulses_within_distance->emplace_back(channel2, signal_num2,
                                               distance2, intensity2,
                                               intensity_seq2);
        }
        break;
      }
    }
  }
  return;
}

inline bool CrossTalk::within_distance_threshold_(\
  const uint32_t &target_distance,
  const uint32_t &distance2) {
  int32_t distance_diff = target_distance - distance2;
  return (distance_diff > -cross_talk_distance1_) &&
         (distance_diff < cross_talk_distance2_);
}

inline void CrossTalk::sequentially_calculate_inherent_intensity_(\
  std::vector<Pulse> *same_distance_pulses, const Pulse &retro_pulse) {
  double retro_power_decimal =
    misc_tables_->convert_to_power(retro_pulse.getIntensitySeq());
  int retro_channel = retro_pulse.getChannel();
  for (auto pulse_it = same_distance_pulses->begin();
       pulse_it != same_distance_pulses->end();
       ++pulse_it) {
    subtract_crosstalk_from_neighboring_channel_(pulse_it,
                                                 retro_power_decimal,
                                                 retro_channel);
  }
  // xxx todo: why push retro_pulse???
  same_distance_pulses->push_back(retro_pulse);
  return;
}

inline void CrossTalk::subtract_crosstalk_from_neighboring_channel_(\
  std::vector<Pulse>::iterator neighbor_pulse,
  double retro_power_decimal, int retro_channel) {
  double crosstalk_ratio = get_crosstalk_ratio_(retro_channel,
                                                neighbor_pulse->getChannel());
  double neighbor_power_decimal =
    misc_tables_->convert_to_power(neighbor_pulse->getIntensitySeq());
  double new_neighbor_power_decimal =
    neighbor_power_decimal - (retro_power_decimal / crosstalk_ratio);

  // uncomment when we are ready to add a new yaml parameter
  // if (new_neighbor_power <
  //     (retro_power / params_.iv_params.min_recovered_ratio_threshold)) {
  //   neighbor_pulse->setIntensity(-1);
  // }
  // else {
  uint32_t intensity;
  uint32_t intensity_seq;
  if (new_neighbor_power_decimal <= 0) {
    intensity = 0;
    intensity_seq = 0;
  } else {
    uint32_t new_neighbor_power =
      static_cast<uint32_t>(new_neighbor_power_decimal);
    intensity = misc_tables_->convert_to_intensity(new_neighbor_power);
    intensity_seq = misc_tables_->convert_to_intensity_seq(intensity);
  }
#if 0
  inno_log_info("cross talk: crosstalk_ratio: %f, retro_pulse_intensity: %d, "
                "neighbor_intensity: %d, retro_power_db: %f, "
                "retro_power_decimal: %f, neighbor_power_db: %f, "
                "neighbor_power_decimal: %f, new_neighbor_power_decimal: %f, "
                "new_intensity: %d, new_intensity_seq: %d",
                 crosstalk_ratio, retro_pulse.getIntensity(),
                neighbor_pulse->getIntensity(), retro_power_db,
                retro_power_decimal, neighbor_power_db,
                neighbor_power_decimal, new_neighbor_power_decimal,
                intensity, intensity_seq);
#endif
  neighbor_pulse->setIntensity(intensity);
  neighbor_pulse->setIntensitySeq(intensity_seq);
  // }
  return;
}

inline double CrossTalk::get_crosstalk_ratio_(int retro_channel,
                                              int neighbor_channel) {
  // 12 ratio version
  // std::pair<int, int> p(retro_channel, neighbor_channel);
  // int ctr_array_index = pair_to_ctr_array_idx_map[p];
  // return params_.iv_params.ctr[ctr_array_index];

  // 3 ratio version
  int channel_distance = retro_channel - neighbor_channel;
  if (channel_distance < 0) {
    channel_distance = -1 * channel_distance;
  }
  /**
   * [CST Bugfinder Defect] Reviewed
   * PolySpace report a defect here:
   * Attempt to access an array outside its bounds.
   * Additional Info:Expected values: [0 .. 2].Actual values: [-1 .. 2^31-1]
   *
   * Value range of retro_channel and neighbor_channel are both
   * [0 .. kInnoChannelNumber-1], max value of channel_distance is
   * kInnoChannelNumber-1
   */

  return ctr_[channel_distance - 1];
}

inline void CrossTalk::overwrite_image_result_(RawBlock *block,
                      std::vector<Pulse> *same_distance_pulses) {
  // if intensity == -1, remove the pulse
  // else, reassign the intensity value
  for (auto pulse_it = same_distance_pulses->begin();
       pulse_it != same_distance_pulses->end();
       ++pulse_it) {
    int signal_num = pulse_it->getSignalNumber();
    int channel = pulse_it->getChannel();
    block->points[channel][signal_num].set_raw_intensity(
      pulse_it->getIntensity());
    if (pulse_it->getIntensity() == 0) {
      block->points[channel][signal_num].set_radius(0, 0);
      block->points[channel][signal_num].set_radius(1, 0);
    }
    re_sort_im_result_ins_arrays_(block, channel, signal_num);
    stats_cross_talk[channel]++;
  }
}

inline void CrossTalk::re_sort_im_result_ins_arrays_(RawBlock *block,
                                                     int channel,
                                                     int signal_num) {
  // we only have up to 2 multiple returns, so we only need to check
  // the case where I am the first return and the next return is stronger
  if (signal_num == 0\
      && block->points[channel][signal_num].get_raw_intensity() <
        block->points[channel][signal_num + 1].get_raw_intensity()) {
    RawChannelPoint temp = block->points[channel][signal_num];
    block->points[channel][signal_num] = block->points[channel][signal_num + 1];
    block->points[channel][signal_num + 1] = temp;
  }
}

}  // namespace innovusion
