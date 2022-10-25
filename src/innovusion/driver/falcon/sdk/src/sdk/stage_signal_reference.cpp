/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "sdk/stage_signal.h"

#include "sdk/lidar.h"
#include "sdk/rawdata_type.h"

namespace innovusion {

const size_t StageSignal::kRefSampleRate[kInnoChannelNumber] = {
  StageSignal::kRefRealSampleRate / StageSignal::kRefFpgaSampleRate,
  StageSignal::kRefRealSampleRate / StageSignal::kRefFpgaSampleRate,
  StageSignal::kRefRealSampleRate / StageSignal::kRefFpgaSampleRate,
  StageSignal::kRefRealSampleRate / StageSignal::kRefFpgaSampleRate,
};

StageSignal::RefTracking::RefTracking() {
}

StageSignal::RefTracking::~RefTracking() {
}

void StageSignal::RefTracking::init(uint32_t id,
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
                                    uint32_t roi_type) {
  memset(this, 0, sizeof(*this));
  id_ = id;
  current_average_sub_ns_ = window_center;
  half_window_l_ = half_window_width;
  half_window_r_ = half_window_width;
  half_window_min_ = half_min;
  half_window_max_ = half_max;
  int32_t center_left_limit = current_average_sub_ns_ - (half_window_l_ -
                              half_window_min_);
  center_left_limit_ = center_left_limit;
  inno_log_verify(center_left_limit >= 0,
                 "center_left_limit invalid: %d", center_left_limit);
  int32_t center_right_limit = current_average_sub_ns_ + half_window_r_ -
                               half_window_min_;
  inno_log_verify(center_right_limit > center_left_limit,
                 "center_right_limit invalid: %d/%d",
                  center_left_limit,
                  center_right_limit);
  center_right_limit_ = center_right_limit;
  sample_per_round_ = sample_num;
  update_match_thresh_ = match_thresh;
  update_new_weight_ = new_weight;
  channel_ = channel;
  roi_type_ = roi_type;
  locked_ = false;
  ref_left_limit_count_ = 0;
  ref_right_limit_count_ = 0;
  thershold_hysteresis_ = 1;
  window_expand_flags_ = kCheckTimesPerRefUpdate;
  set_bound();
}

void StageSignal::RefTracking::set_bound() {
  ref_r_w_sub_ns_ = current_average_sub_ns_ + half_window_r_;
  ref_l_w_sub_ns_ = current_average_sub_ns_ - half_window_l_;
}

void StageSignal::RefTracking::set_bound(InnoFpgaSubNs half_window_sub_ns) {
  half_window_l_ = half_window_sub_ns;
  half_window_r_ = half_window_sub_ns;
  half_window_max_ = half_window_sub_ns;
  center_left_limit_ = current_average_sub_ns_ - (half_window_max_ -
                       half_window_min_);
  center_right_limit_ = current_average_sub_ns_ + (half_window_max_ -
                        half_window_min_);
  set_bound();
}

void StageSignal::RefTracking::set_bound(InnoFpgaSubNs half_window_sub_ns,
                                         size_t trigger_count) {
  if (half_window_l_ > half_window_sub_ns ||
      half_window_r_ > half_window_sub_ns) {
    reset_stats(trigger_count);
    set_bound(half_window_sub_ns);
  } else {
    half_window_max_ = half_window_sub_ns;
    center_left_limit_ = current_average_sub_ns_ - (half_window_max_ -
                         half_window_min_);
    center_right_limit_ = current_average_sub_ns_ + (half_window_max_ -
                          half_window_min_);
  }
}

void StageSignal::RefTracking::prepare_for_next_round(size_t trigger_cnt) {
  total_sampled_ += sampled_;
  total_sampled_sub_ns_ += sampled_sub_ns_;
  total_ref_is_scatter_ += ref_is_scatter_;
  sampled_ = 0;
  sampled_sub_ns_ = 0;
  ref_is_scatter_ = 0;
  trigger_count_old_ = trigger_cnt;
  set_bound();
}

void StageSignal::RefTracking::add_sample(InnoFpgaSubNs diff_trigger,
                                          size_t current_trigger_count,
                                          RefTrackingResult *tracking_result) {
  sampled_++;
  sampled_sub_ns_ += diff_trigger;

  if (sampled_ > sample_per_round_) {
    int64_t trigger_cnt = current_trigger_count - trigger_count_old_;
    if (sampled_ >
        trigger_cnt / kRefRealSampleRate * update_match_thresh_\
                                         * thershold_hysteresis_) {
      thershold_hysteresis_ = 0.97;  // hysteresis once locked the window
      // locked
      InnoFpgaSubNs average = sampled_sub_ns_ / sampled_;
      if (locked_count_) {
        current_average_sub_ns_ =
            current_average_sub_ns_ *
            (1 - update_new_weight_) +
            average * update_new_weight_;
      } else {
        current_average_sub_ns_ = average;
      }

      // xxx todo: more complex window adjustment
      uint32_t half_window_save_l = half_window_l_;
      uint32_t half_window_save_r = half_window_r_;
      half_window_l_ /= 2;
      half_window_r_ /= 2;
      bound_check_and_correct_();
      tracking_result->update(current_average_sub_ns_, roi_type_);

      // xxxx todo: shall we check the center between channels?
      if (half_window_l_ < half_window_min_) {
        half_window_l_ = half_window_min_;
      }
      if (half_window_r_ < half_window_min_) {
        half_window_r_ = half_window_min_;
      }
      locked_count_++;
      if (!locked_) {
        unlocked_to_locked_count_++;
        if (unlocked_to_locked_count_ < 30 ||
            unlocked_to_locked_count_ % 128 == 1) {
          inno_log_info("ref channel: %u, roi_type: %u, "
                        "unlock->locked %" PRI_SIZELU " (%"
                        PRI_SIZELU "/%" PRI_SIZELU ") half_w=(%u/%u) "
                        "center=%" PRI_SIZED
                        ", bound limited: %" PRI_SIZEU "/%" PRI_SIZEU,
                         channel_, roi_type_,
                         unlocked_to_locked_count_,
                         locked_count_,
                         unlocked_count_,
                         half_window_l_,
                         half_window_r_,
                         current_average_sub_ns_,
                         ref_left_limit_count_,
                         ref_right_limit_count_);
        }
      }

      if ((half_window_save_l != half_window_l_ ||
         half_window_save_r != half_window_r_) &&
         (locked_count_ <  30 || locked_count_ % 128 == 1)) {
        inno_log_info("ref channel: %u, roi_type: %u, "
                      "shrink half_w=(%u/%u)->(%u/%u) "
                      "percentage: %.2f center=%" PRI_SIZED,
                       channel_, roi_type_,
                       half_window_save_l,
                       half_window_save_r,
                       half_window_l_,
                       half_window_r_,
                       static_cast<double>(sampled_) /
                       (trigger_cnt / kRefRealSampleRate),
                       current_average_sub_ns_);
      }
      locked_ = true;

      if (locked_count_ % 512 == 1) {
        inno_log_info("ref channel: %u, roi_type: %u, "
                      "locked (%" PRI_SIZELU "/%" PRI_SIZELU ") half_w=(%u/%u) "
                      "center=%" PRI_SIZED
                      ", bound limited: %" PRI_SIZEU "/%" PRI_SIZEU,
                       channel_,  roi_type_,
                       locked_count_,
                       unlocked_count_,
                       half_window_l_,
                       half_window_r_,
                       current_average_sub_ns_,
                       ref_left_limit_count_,
                       ref_right_limit_count_);
      }
      reset_window_expand_flags();
    } else {
      // expand window
      unlocked_count_++;
      thershold_hysteresis_ = 1.0;
      uint32_t half_window_save_l = half_window_l_;
      uint32_t half_window_save_r = half_window_r_;
      expand_window();
      if (unlocked_count_ < 30 || unlocked_count_ % 128 == 1) {
        inno_log_info("ref channel: %u, percentage: %.2f too small",
                      channel_, static_cast<double>(sampled_) /
                      (trigger_cnt / kRefRealSampleRate));
      }
      if (locked_) {
        locked_to_unlocked_count_++;
        if (locked_to_unlocked_count_ < 30 ||
            locked_to_unlocked_count_ % 128 == 1) {
          inno_log_info("ref channel: %u, roi_type: %u, "
                        "lock->unlocked %" PRI_SIZELU " (%"
                        PRI_SIZELU "/%" PRI_SIZELU ") half_w=(%u/%u) "
                        "center=%" PRI_SIZED ", boundary limit: %"
                        PRI_SIZEU "/%" PRI_SIZEU,
                         channel_, roi_type_,
                         locked_to_unlocked_count_, locked_count_,
                         unlocked_count_,
                         half_window_l_,
                         half_window_r_,
                         current_average_sub_ns_,
                         ref_left_limit_count_,
                         ref_right_limit_count_);
        }
      }
      if ((half_window_save_l != half_window_l_ ||
         half_window_save_r != half_window_r_) &&
         (unlocked_count_ < 30 || unlocked_count_ % 128 == 1)) {
        inno_log_info("ref channel: %u, roi_type: %u, "
                      "expand half_w=(%u/%u)->(%u/%u) center=%" PRI_SIZED,
                      channel_, roi_type_,
                      half_window_save_l,
                      half_window_save_r,
                      half_window_l_,
                      half_window_r_,
                      current_average_sub_ns_);
      }
      locked_ = false;
    }
    prepare_for_next_round(current_trigger_count);
  }
}

void StageSignal::RefTracking::bound_check_and_correct_() {
  InnoFpgaSubNs current_average_sub_ns_save = current_average_sub_ns_;
  if (current_average_sub_ns_ > center_right_limit_) {
    current_average_sub_ns_ = center_right_limit_;
    half_window_r_ = half_window_min_;
    ref_right_limit_count_++;
    inno_log_warning("channel: %u, roi_type: %u, "
                     "exceed left limit(%" PRI_SIZEU "), center %"
                     PRI_SIZED "->%" PRI_SIZED,
                      channel_, roi_type_,
                      ref_right_limit_count_,
                      current_average_sub_ns_save,
                      current_average_sub_ns_);
  }
  if (current_average_sub_ns_ < center_left_limit_) {
    current_average_sub_ns_ = center_left_limit_;
    half_window_l_ = half_window_min_;
    ref_left_limit_count_++;
    inno_log_warning("channel: %u, roi_type: %u, "
                     "exceed left limit(%" PRI_SIZEU "), center %"
                     PRI_SIZED "->%" PRI_SIZED,
                      channel_, roi_type_,
                      ref_left_limit_count_,
                      current_average_sub_ns_save,
                      current_average_sub_ns_);
  }
}

StageSignal::RefTrackingResult::RefTrackingResult() {
}

StageSignal::RefTrackingResult::~RefTrackingResult() {
}

void StageSignal::RefTrackingResult::init(InnoFpgaSubNs window_center,
                                          uint32_t channel) {
  in_roi_count_ = 0;
  channel_ = channel;
  virtual_ref_offset_sub_ns_ = window_center;
}

void StageSignal::RefTrackingResult::update(InnoFpgaSubNs virtual_ref_offset,
                                            int in_roi) {
  if (in_roi) {
    // roi reference tracking has priority
    in_roi_count_ = 5;
    virtual_ref_offset_sub_ns_ = virtual_ref_offset;
  } else if (in_roi_count_ == 0) {
    // no roi reference tracking or has not seen
    // roi reference tracking update last 5 times
    virtual_ref_offset_sub_ns_ = virtual_ref_offset;
  } else {
    // don't update yet, decrease the count
    in_roi_count_--;
  }
}
}  // namespace innovusion
