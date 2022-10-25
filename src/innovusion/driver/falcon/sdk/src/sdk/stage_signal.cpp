/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "sdk/stage_signal.h"

#include <sstream>

#include "utils/utils.h"
#include "sdk/lidar.h"
#include "sdk/lidar_clock.h"
#include "sdk/rawdata_type.h"
#include "sdk/reg.h"

namespace innovusion {

constexpr size_t kRawDequeSize =
    InnoLidar::kSignalJobPoolSize / 2 - InnoLidar::kSignalJobPoolSizeReserve;

const double StageSignal::kLightTraveInnoDistanceUnitPerSubNs =
    InnoConsts::kLightTraveMeterPerSubNs * kInnoDistanceUnitPerMeter;


const size_t StageSignal::kDataTypeSize_[16] = {
  sizeof(PtpData),                // 0x0 kRawDataTypeOther ptp or gps
  sizeof(AdcComputedDataExt),     // 0x1 kRawDataTypeAdc
  0,                              // 0x2 RawDataTypeAdcRaw
  sizeof(AdcComputedDataExt),     // 0x3 kRawDataTypeAdc
  sizeof(PaddingData),            // 0x4 kRawDataTypePadding
  sizeof(AdcComputedDataExt),     // 0x5 kRawDataTypeAdc
  sizeof(EncoderData),            // 0x6 kRawDataTypeEncoder
  sizeof(AdcComputedDataExt),     // 0x7 kRawDataTypeAdc
  sizeof(TriggerData),            // 0x8 kRawDataTypeTrigger
  sizeof(AdcComputedDataExt),     // 0x9 kRawDataTypeAdc
  0,                              // 0xa invalid
  sizeof(AdcComputedDataExt),     // 0xb kRawDataTypeAdc
  0,                              // 0xc invalid
  sizeof(AdcComputedDataExt),     // 0xd kRawDataTypeAdc
  0,                              // 0xe invalid
  sizeof(AdcComputedDataExt),     // 0xf kRawDataTypeAdc
};

const double StageSignal::kMinStableTimeAfterStreamStart = 15 * 1000.0;  // 15s
const double StageSignal::kMinStableTimeAfterRawDrop = 5 * 1000.0;  // 5s
const double StageSignal::OpticalPathCheck::hysteresis_factor_return = 3.0;
const double StageSignal::OpticalPathCheck::hysteresis_factor_scatter = 1.3;
const double StageSignal::OpticalPathCheck::hysteresis_factor_reference = 3.5;
const double StageSignal::OpticalPathCheck::kMaxRatioDiffBetChannels = 0.65;

StageSignal::OpticalPathCheck::\
             OpticalPathCheck(InnoLidar* lidar,
                              StageSignal* stage_signal,
                              double return_pulse_ratio_threshold,
                              double scatter_ratio_threshold,
                              double reference_ratio_threshold,
                              double window_not_blocked_return_threshold,
                              double window_blockage_scatter_threshold,
                              uint32_t scatter_check_cpu_threshold) {
  lidar_ = lidar;
  stage_signal_ = stage_signal;
  inno_log_verify(lidar_ && stage_signal_,
                 "lidar_(%p) or stage_signal_(%p) is NULL",
                  lidar_, stage_signal_);
  return_pulse_ratio_threshold_ = return_pulse_ratio_threshold;
  scatter_ratio_threshold_ = scatter_ratio_threshold;
  reference_ratio_threshold_ = reference_ratio_threshold;
  window_not_blocked_return_threshold_ = window_not_blocked_return_threshold;
  window_blockage_scatter_threshold_ = window_blockage_scatter_threshold;
  scatter_check_cpu_threshold_ = scatter_check_cpu_threshold;
  scatter_running_ = false;
  frame_counts_ = 0;
  for (uint32_t i = 0; i < kInnoChannelNumber; i++) {
    return_pulse_counter_[i] = 0;
    return_pulse_counter_old_[i] = 0;
    scatter_counter_[i] = 0;
    scatter_counter_old_[i] = 0;
    reference_counter_[i] = 0;
    reference_counter_old_[i] = 0;
    reference_intensity_[i] =  0;
    reference_sample_counts_[i] = 0;
  }
  init_env_();
  ref_intensity_initialized_ = false;
}

StageSignal::OpticalPathCheck::~OpticalPathCheck() {
  lidar_ = NULL;
}

int StageSignal::OpticalPathCheck::\
                 enable_scatter_filter_(uint8_t scale) {
  int ret = 0;
  uint32_t reg_value = 0;
  ret = lidar_->read_pl_reg(SCATTER_FILTER_RATIO_CONTROLLER, &reg_value);
  if (ret != 0) {
    return -1;
  }
  if (scale  > 1) {
    reg_value |= ((uint32_t)scale << 8U);
    ret = lidar_->write_pl_reg(SCATTER_FILTER_RATIO_CONTROLLER, reg_value);
  } else {
    // do nothing
  }
  return ret;
}

void StageSignal::OpticalPathCheck::\
                  do_optical_check_via_scatter_(uint32_t block_number,
                                                bool optical_break,
                                                bool blocked) {
  uint32_t break_flag = 0;
  uint32_t block_counter = 0;
  double window_blockage_scatter_threshold =
                                 window_blockage_scatter_threshold_;
  double scatter_ratio_threshold = scatter_ratio_threshold_;
  if (optical_break) {
    scatter_ratio_threshold *= hysteresis_factor_scatter;
  }
  if (blocked) {
    window_blockage_scatter_threshold /= hysteresis_factor_scatter;
  }
  for (uint32_t i = 0; i < kInnoChannelNumber; i++) {
    double sampled_ratio = (scatter_counter_[i] -
                            scatter_counter_old_[i]) /
                            static_cast<double>(block_number);
    sampled_ratio *= kCountOneScatterPerPeriod;
    if (sampled_ratio < scatter_ratio_threshold) {
      break_flag |= (1U << i);
    } else {
      if (sampled_ratio > window_blockage_scatter_threshold) {
        block_counter++;
      } else {
        // do nothing
      }
    }
  }
  if ((break_flag & 0xf) == 0xf) {
    lidar_->set_raw_fault(INNO_LIDAR_IN_FAULT_OPTIC2_F, true,
                         "all channels break");
  } else {
    lidar_->heal_raw_fault(INNO_LIDAR_IN_FAULT_OPTIC2_F, true,
                          "not all channels break");
  }
  for (uint32_t i = 0; i < kInnoChannelNumber; i++) {
    InnoSubInFaults sub_fault_id =
                    InnoSubInFaults(INNO_SUB_FAULT_OPTIC2_0 + i);
    if (((break_flag >> i) & 0x1) != 0) {
      lidar_->set_raw_fault(sub_fault_id, true, true,
                            "INNO_SUB_FAULT_OPTIC2_%d, flags: %u, "
                            "trigger count: %u, "
                            "return pulse: %" PRI_SIZELU "/%"
                            PRI_SIZELU "/%" PRI_SIZELU "/%" PRI_SIZELU "/, "
                            "return threshold: %0.5f, "
                            "scatter: %" PRI_SIZELU "/%" PRI_SIZELU
                            "/%" PRI_SIZELU "/%" PRI_SIZELU "/, "
                            "scatter threshold: %0.5f",
                            i, break_flag, block_number,
                            return_pulse_counter_[0] -
                            return_pulse_counter_old_[0],
                            return_pulse_counter_[1] -
                            return_pulse_counter_old_[1],
                            return_pulse_counter_[2] -
                            return_pulse_counter_old_[2],
                            return_pulse_counter_[3] -
                            return_pulse_counter_old_[3],
                            return_pulse_ratio_threshold_,
                            (scatter_counter_[0] - scatter_counter_old_[0]) *
                            kCountOneScatterPerPeriod,
                            (scatter_counter_[1] - scatter_counter_old_[1]) *
                            kCountOneScatterPerPeriod,
                            (scatter_counter_[2] - scatter_counter_old_[2]) *
                            kCountOneScatterPerPeriod,
                            (scatter_counter_[3] - scatter_counter_old_[3]) *
                            kCountOneScatterPerPeriod,
                            scatter_ratio_threshold_);
    } else {
      lidar_->heal_raw_fault(sub_fault_id, true, true,
                            "INNO_SUB_FAULT_OPTIC2_%d", i);
    }
  }
  if (break_flag > 0) {
    return;
  }
  if (block_counter >= kInnoChannelNumber) {
    lidar_->set_raw_fault(INNO_LIDAR_IN_FAULT_WINDOW_BLOCKAGE1, true,
                         "trigger count: %u, return pulse: %"
                         PRI_SIZELU "/%" PRI_SIZELU "/%"
                         PRI_SIZELU "/%" PRI_SIZELU "/, "
                         "return threshold: %0.5f, "
                         "scatter: %" PRI_SIZELU "/%" PRI_SIZELU
                         "/%" PRI_SIZELU "/%" PRI_SIZELU "/, "
                         "scatter threshold: %0.5f",
                          block_number,
                          return_pulse_counter_[0] -
                          return_pulse_counter_old_[0],
                          return_pulse_counter_[1] -
                          return_pulse_counter_old_[1],
                          return_pulse_counter_[2] -
                          return_pulse_counter_old_[2],
                          return_pulse_counter_[3] -
                          return_pulse_counter_old_[3],
                          window_not_blocked_return_threshold_,
                          (scatter_counter_[0] - scatter_counter_old_[0]) *
                          kCountOneScatterPerPeriod,
                          (scatter_counter_[1] - scatter_counter_old_[1]) *
                          kCountOneScatterPerPeriod,
                          (scatter_counter_[2] - scatter_counter_old_[2]) *
                          kCountOneScatterPerPeriod,
                          (scatter_counter_[3] - scatter_counter_old_[3]) *
                          kCountOneScatterPerPeriod,
                          window_blockage_scatter_threshold_);
  } else {
    lidar_->heal_raw_fault(INNO_LIDAR_IN_FAULT_WINDOW_BLOCKAGE1, true,
                          "INNO_LIDAR_IN_FAULT_WINDOW_BLOCKAGE1 heals");
  }
}

void StageSignal::OpticalPathCheck::\
                  do_main_optical_and_blockage_check_(uint32_t block_number) {
  bool blocked =
       lidar_->get_current_fault_status(INNO_LIDAR_IN_FAULT_WINDOW_BLOCKAGE1);
  bool optical_break =
       lidar_->get_current_fault_status(INNO_LIDAR_IN_FAULT_OPTIC2);
  enum OpticalCheckState ret = do_optical_check_via_return_(block_number,
                                                            optical_break,
                                                            blocked);
  if (ret == INNO_OPTICAL_TO_BE_CONFIRMED ||
      ret == INNO_OPTICAL_OK_WINDOW_MAYBE_BLOCKED) {
    do_optical_check_via_scatter_(block_number, optical_break, blocked);
  } else {
    lidar_->heal_raw_fault(INNO_LIDAR_IN_FAULT_OPTIC2_F, true,
                          "not all channels break for returns");
    for (uint32_t i = 0; i < kInnoChannelNumber; i++) {
      InnoSubInFaults sub_fault_id =
                      InnoSubInFaults(INNO_SUB_FAULT_OPTIC2_0 + i);
      lidar_->heal_raw_fault(sub_fault_id, true, true,
                            "INNO_SUB_FAULT_OPTIC2_%d", i);
    }
    lidar_->heal_raw_fault(INNO_LIDAR_IN_FAULT_WINDOW_BLOCKAGE1, true,
                          "INNO_LIDAR_IN_FAULT_WINDOW_BLOCKAGE1 heals");
  }
}

int StageSignal::OpticalPathCheck::get_avg_detector_temp(int64_t *avg_temp,
                                                         int64_t *min_temp,
                                                         int64_t *max_temp) {
  if (!lidar_->is_live_lidar()) {
    static int kNormalTemperature = 450;  // 45 degC
    *avg_temp = kNormalTemperature;
    *min_temp = kNormalTemperature;
    *max_temp = kNormalTemperature;
    return 0;
  }
  int64_t invalid_temp = lidar_->get_invalid_temp();
  int64_t det_temp[kInnoChannelNumber] = {invalid_temp,
                                          invalid_temp,
                                          invalid_temp,
                                          invalid_temp};
  *avg_temp = 0;
  *min_temp = INT_MAX;
  *max_temp = INT_MIN;
  if (lidar_->get_detector_temps(det_temp) != 0) {
    return -1;
  }
  for (uint32_t i = 0; i < kInnoChannelNumber; i++) {
    if (det_temp[i] == invalid_temp) {
      return -1;
    }
    *avg_temp += det_temp[i];
    if (det_temp[i] < *min_temp) {
      *min_temp = det_temp[i];
    }
    if (det_temp[i] > *max_temp) {
      *max_temp = det_temp[i];
    }
  }
  if (*max_temp - *min_temp > kMaxDetTempDiffThreshold) {
    return -1;
  }
  *avg_temp /= kInnoChannelNumber;
  return 0;
}

void StageSignal::OpticalPathCheck::do_ref_intensity_check() {
  bool inhibit_fault_check = lidar_->\
               get_current_fault_status(INNO_LIDAR_IN_FAULT_OPTIC1);
  if (inhibit_fault_check) {
    return;
  }
  int64_t avg_temp = 0;
  int64_t min_temp = INT_MAX;
  int64_t max_temp = INT_MIN;
  if (get_avg_detector_temp(&avg_temp, &min_temp, &max_temp) != 0 ||
      max_temp - min_temp > kMaxDetTempDiffThreshold) {
    static uint32_t print_counter =  0;
    if (print_counter++ % 256 == 0) {
      inno_log_error("abort ref_intensity check due to "
                      "temperature: %" PRI_SIZED
                      "/%" PRI_SIZED "/%" PRI_SIZED,
                      avg_temp, min_temp, max_temp);
    }
    return;
  }
  for (uint32_t i = 0; i < kInnoChannelNumber; i++) {
    if (reference_sample_counts_[i] <
        kMinRefIntensityCheckCount *
        stage_signal_->get_ref_sample_rate(i)) {
      return;
    }
  }
  static const int kDetTempMidIntensityThreshold = 300;
  static const int ref_intensity_hysteresis = 100;
  static const int ref_intensity_thres_low_limit = 250;
  static const double ratio_ref_intensity_hysteresis = 0.05;
  static const double kIntensityDiffCrossChannel = 0.60;
  bool has_fault = lidar_->\
       get_current_fault_status(INNO_LIDAR_IN_FAULT_REFINTENSITY);
  // calculate the hysteresis
  double factor_hysteresis = 0.0;
  int intensity_hysteresis = 0;
  if (has_fault) {
    factor_hysteresis = ratio_ref_intensity_hysteresis;
    intensity_hysteresis = ref_intensity_hysteresis;
  }
  // double ratio_threshold;
  std::pair<int, double> current_thres;
  current_thres.first = avg_temp / 10;  // resolution is 0.1
  if (ref_int_check_helper_.get_diag_threshold(&current_thres) != 0) {
    static uint32_t print_count = 0;
    print_count++;
    if (print_count < 32 || print_count % 256 == 0) {
      inno_log_error("abort ref_intensity check, avg_temp: %d",
                      current_thres.first);
    }
    return;
  }

  double consistency_threshold = kIntensityDiffCrossChannel -
                                 factor_hysteresis;
  int avg_intensity[kInnoChannelNumber] = {0, 0, 0, 0};
  uint32_t low_intensity_flags = 0;
  double min_ref_intensity_ratio = 10000.0;
  int min_ref_intensity_channel = -1;
  double max_ref_intensity_ratio = -1.0;
  int max_ref_intensity_channel = -1;
  bool consistency_check_valid = true;
  for (uint32_t i = 0; i < kInnoChannelNumber; i++) {
    avg_intensity[i] = reference_intensity_[i] /
                       reference_sample_counts_[i];
    int intensity_thres = current_thres.second * ref_intensity_in_yaml_[i];
    if (kDetTempMidIntensityThreshold < intensity_thres) {
      intensity_thres = kDetTempMidIntensityThreshold;
    }
    if (intensity_thres < ref_intensity_thres_low_limit) {
      intensity_thres = ref_intensity_thres_low_limit;
    }
    intensity_thres += intensity_hysteresis;  // intensity final threshold
    if (avg_intensity[i] < intensity_thres) {
      low_intensity_flags |= (1U << i);
    } else {
      // do nothing, just keep old value
    }
    if (ref_intensity_in_yaml_[i] == 0) {
      consistency_check_valid = false;
      continue;
    }
    double div = avg_intensity[i] /
                 static_cast<double>(ref_intensity_in_yaml_[i]);
    if (div > max_ref_intensity_ratio) {
      max_ref_intensity_ratio = div;
      if (max_ref_intensity_ratio > 1.0) {
        max_ref_intensity_ratio = 1.0;
      }
      max_ref_intensity_channel = i;
    }
    if (div < min_ref_intensity_ratio) {
      min_ref_intensity_ratio = div;
      if (min_ref_intensity_ratio > 1.0) {
        min_ref_intensity_ratio = 1.0;
      }
      min_ref_intensity_channel = i;
    }
  }
  char buffer[512];
  buffer[0] = '\0';
  bool consist_error = consistency_check_valid &&
                       max_ref_intensity_ratio - min_ref_intensity_ratio >
                       consistency_threshold;
  if (low_intensity_flags > 0 ||
      consist_error) {
    int ret = snprintf(buffer, sizeof(buffer),
                      "temperature: %" PRI_SIZED ", "
                      "low intensity channel mask: %u, "
                      "avg intensity: %u/%u/%u/%u/ "
                      "ref center: (%" PRI_SIZED "/%"
                      PRI_SIZED "/%" PRI_SIZED "), (%"
                      PRI_SIZED "/%" PRI_SIZED "/%" PRI_SIZED "), "
                      "(%" PRI_SIZED "/% " PRI_SIZED
                      "/%" PRI_SIZED "), (%" PRI_SIZED
                      "/%" PRI_SIZED "/% " PRI_SIZED"), "
                      "search window: (%u/%u/%u/%u) (%u/%u/%u/%u) "
                      "(%u/%u/%u/%u) (%u/%u/%u/%u), "
                      "yaml ref_intensity: %" PRI_SIZELU
                      "/%" PRI_SIZELU "/%" PRI_SIZELU
                      "/%" PRI_SIZELU ", "
                      "ratio_threshold: %0.5f "
                      "max/min intensity ratio: %0.5f/%0.5f, "
                      "channel: %d/%d, "
                      "consistency threshold: %0.5f",
                      avg_temp / 10,
                      low_intensity_flags,
                      avg_intensity[0],
                      avg_intensity[1],
                      avg_intensity[2],
                      avg_intensity[3],
                      stage_signal_->get_ref_center(0, 0),
                      stage_signal_->get_ref_center(0, 1),
                      stage_signal_->get_ref_center_result(0),
                      stage_signal_->get_ref_center(1, 0),
                      stage_signal_->get_ref_center(1, 1),
                      stage_signal_->get_ref_center_result(1),
                      stage_signal_->get_ref_center(2, 0),
                      stage_signal_->get_ref_center(2, 1),
                      stage_signal_->get_ref_center_result(2),
                      stage_signal_->get_ref_center(3, 0),
                      stage_signal_->get_ref_center(3, 1),
                      stage_signal_->get_ref_center_result(3),
                      stage_signal_->get_search_window_l(0, 0),
                      stage_signal_->get_search_window_r(0, 0),
                      stage_signal_->get_search_window_l(0, 1),
                      stage_signal_->get_search_window_r(0, 1),
                      stage_signal_->get_search_window_l(1, 0),
                      stage_signal_->get_search_window_r(1, 0),
                      stage_signal_->get_search_window_l(1, 1),
                      stage_signal_->get_search_window_r(1, 1),
                      stage_signal_->get_search_window_l(2, 0),
                      stage_signal_->get_search_window_r(2, 0),
                      stage_signal_->get_search_window_l(2, 1),
                      stage_signal_->get_search_window_r(2, 1),
                      stage_signal_->get_search_window_l(3, 0),
                      stage_signal_->get_search_window_r(3, 0),
                      stage_signal_->get_search_window_l(3, 1),
                      stage_signal_->get_search_window_r(3, 1),
                      ref_intensity_in_yaml_[0],
                      ref_intensity_in_yaml_[1],
                      ref_intensity_in_yaml_[2],
                      ref_intensity_in_yaml_[3],
                      current_thres.second,
                      max_ref_intensity_ratio,
                      min_ref_intensity_ratio,
                      max_ref_intensity_channel,
                      min_ref_intensity_channel,
                      consistency_threshold);
    inno_log_verify(ret < (ssize_t)sizeof(buffer),
                   "%d vs. %" PRI_SIZELU", buffer too small?",
                    ret, sizeof(buffer));
    if (!has_fault &&
        !lidar_->get_fault_inhibit_status(INNO_LIDAR_IN_FAULT_REFINTENSITY)) {
      inno_log_warning("%s", buffer);
    }
  }
  if ((low_intensity_flags & 0xf) == 0xf) {
    lidar_->set_raw_fault(INNO_SUB_FAULT_REFINTENSITY4, true, true,
                          "4 channel check failed in low temperature, %s",
                          buffer);
  } else {
    lidar_->heal_raw_fault(INNO_SUB_FAULT_REFINTENSITY4, true, true,
                          "INNO_SUB_FAULT_REFINTENSITY4 heals");
  }
  if (avg_temp < kDetTempLowThreshold) {
    // when T < -10 degC, only check heal condition per channel
    for (uint32_t i = 0; i < kInnoChannelNumber; i++) {
      InnoSubInFaults sub_fault_id =
                      InnoSubInFaults(INNO_SUB_FAULT_REFINTENSITY1_0 + i);
      if (((low_intensity_flags >> i) & 0x1) != 0) {
        continue;
      } else {
        lidar_->heal_raw_fault(sub_fault_id, true, true,
                              "sub-fault(%d) heals", sub_fault_id);
        if (lidar_->\
            get_current_fault_status(sub_fault_id)) {
          lidar_->heal_raw_fault(INNO_LIDAR_IN_FAULT_REFINTENSITY, true,
                                "INNO_LIDAR_IN_FAULT_REFINTENSITY heals");
        }
      }
    }
  } else if (avg_temp < kDetTempHighThreshold) {
    // consistency check
    if (consist_error) {
      lidar_->set_raw_fault(INNO_SUB_FAULT_REFINTENSITY0, true, true,
                           "consistency check failed, %s", buffer);
    } else {
      lidar_->heal_raw_fault(INNO_SUB_FAULT_REFINTENSITY0, true, true,
                            "INNO_SUB_FAULT_REFINTENSITY0 heals");
    }
    // do single channel check when T > -10 degC
    for (uint32_t i = 0; i < kInnoChannelNumber; i++) {
      InnoSubInFaults sub_fault_id =
                      InnoSubInFaults(INNO_SUB_FAULT_REFINTENSITY1_0 + i);
      if (((low_intensity_flags >> i) & 0x1) != 0) {
        lidar_->set_raw_fault(sub_fault_id, true, true,
                             "single channel(%d) check failed, %s",
                              i, buffer);
      } else {
        lidar_->heal_raw_fault(sub_fault_id, true, true,
                              "sub-fault(%d) heals", sub_fault_id);
      }
    }
  } else {
    static int hi_temp_counter = 0;
    hi_temp_counter++;
    if (hi_temp_counter < 32 || hi_temp_counter % 256 == 0) {
      inno_log_error("temperature is out of range, avg: %"
                      PRI_SIZED "/%" PRI_SIZED "/%" PRI_SIZED,
                      avg_temp, min_temp, max_temp);
    }
  }
}

void StageSignal::OpticalPathCheck::\
                  load_ref_intensity_from_yaml() {
  if (ref_intensity_initialized_) {
    return;
  }
  ref_intensity_initialized_ = true;
  const LidarParams& params = lidar_->get_params();
  for (uint32_t i = 0; i < kInnoChannelNumber; i++) {
    ref_intensity_in_yaml_[i] = params.iv_params.f_int_vbr0[i];
  }
  inno_log_info("loaded reference intensity: %"
                 PRI_SIZELU "/%" PRI_SIZELU
                 "/%" PRI_SIZELU "/%" PRI_SIZELU,
                 ref_intensity_in_yaml_[0], ref_intensity_in_yaml_[1],
                 ref_intensity_in_yaml_[2], ref_intensity_in_yaml_[3]);
}

void StageSignal::OpticalPathCheck::\
                  prepare_ref_intensity_check_for_next_round() {
  for (uint32_t i = 0; i < kInnoChannelNumber; i++) {
    reference_intensity_[i] = 0;;
    reference_sample_counts_[i] = 0;
  }
}

void StageSignal::OpticalPathCheck::\
                  do_reference_optical_check_(uint32_t block_number) {
  bool inhibit = lidar_->\
                 get_current_fault_status(INNO_LIDAR_IN_FAULT_OPTIC2);
  if (inhibit) {
    return;
  }
  bool optical_break =
       lidar_->get_current_fault_status(INNO_LIDAR_IN_FAULT_OPTIC1);
  double reference_ratio_threshold = reference_ratio_threshold_;
  if (optical_break) {
    // need to make sure reference_ratio_threshold <= 1
    reference_ratio_threshold *= hysteresis_factor_reference;
  }
  uint32_t mask = 0;
  uint64_t diff[kInnoChannelNumber] = {0, 0, 0, 0};
  bool need_print = false;
  for (uint32_t i = 0; i < kInnoChannelNumber; i++) {
    diff[i] = reference_counter_[i] - reference_counter_old_[i];
    diff[i] *= StageSignal::kRefRealSampleRate /
               stage_signal_->get_ref_sample_rate(i);
    double sampled_ratio = diff[i] / static_cast<double>(block_number);
    uint32_t condition = sampled_ratio < reference_ratio_threshold;
    mask |= (condition << i);
    if (condition) {
      // print log if fault is not set
      need_print = !optical_break;
    }
  }
  if (need_print &&
     !lidar_->get_fault_inhibit_status(INNO_LIDAR_IN_FAULT_REFINTENSITY)) {
    inno_log_warning("reference count is too low! "
                     "trigger count: %u, "
                     "ref center: (%" PRI_SIZED "/%"
                     PRI_SIZED "/%" PRI_SIZED "), (%"
                     PRI_SIZED "/%" PRI_SIZED "/%" PRI_SIZED "), "
                     "(%" PRI_SIZED "/%" PRI_SIZED "/%" PRI_SIZED
                     "), (%" PRI_SIZED "/%" PRI_SIZED "/%" PRI_SIZED "), "
                     "search window: (%u/%u/%u/%u) (%u/%u/%u/%u) "
                     "(%u/%u/%u/%u) (%u/%u/%u/%u), "
                     "reference count: %" PRI_SIZEU "/%" PRI_SIZEU
                     "/%" PRI_SIZEU "/%" PRI_SIZEU "/, "
                     "frame ref_avg intensity: %" PRI_SIZELU "/%" PRI_SIZELU
                     "/%" PRI_SIZELU "/%" PRI_SIZELU "/, "
                     "threshold: %0.5f, fault mask: %u",
                      block_number,
                      stage_signal_->get_ref_center(0, 0),
                      stage_signal_->get_ref_center(0, 1),
                      stage_signal_->get_ref_center_result(0),
                      stage_signal_->get_ref_center(1, 0),
                      stage_signal_->get_ref_center(1, 1),
                      stage_signal_->get_ref_center_result(1),
                      stage_signal_->get_ref_center(2, 0),
                      stage_signal_->get_ref_center(2, 1),
                      stage_signal_->get_ref_center_result(2),
                      stage_signal_->get_ref_center(3, 0),
                      stage_signal_->get_ref_center(3, 1),
                      stage_signal_->get_ref_center_result(3),
                      stage_signal_->get_search_window_l(0, 0),
                      stage_signal_->get_search_window_r(0, 0),
                      stage_signal_->get_search_window_l(0, 1),
                      stage_signal_->get_search_window_r(0, 1),
                      stage_signal_->get_search_window_l(1, 0),
                      stage_signal_->get_search_window_r(1, 0),
                      stage_signal_->get_search_window_l(1, 1),
                      stage_signal_->get_search_window_r(1, 1),
                      stage_signal_->get_search_window_l(2, 0),
                      stage_signal_->get_search_window_r(2, 0),
                      stage_signal_->get_search_window_l(2, 1),
                      stage_signal_->get_search_window_r(2, 1),
                      stage_signal_->get_search_window_l(3, 0),
                      stage_signal_->get_search_window_r(3, 0),
                      stage_signal_->get_search_window_l(3, 1),
                      stage_signal_->get_search_window_r(3, 1),
                      diff[0], diff[1],
                      diff[2], diff[3],
                      reference_intensity_[0] /
                      (reference_sample_counts_[0] + 1),
                      reference_intensity_[1] /
                      (reference_sample_counts_[1] + 1),
                      reference_intensity_[2] /
                      (reference_sample_counts_[2] + 1),
                      reference_intensity_[3] /
                      (reference_sample_counts_[3] + 1),
                      reference_ratio_threshold, mask);
  }
  if ((mask & 0xf) == 0xf) {
    lidar_->set_raw_fault(INNO_SUB_FAULT_OPTIC1_4, true, true,
                         "INNO_SUB_FAULT_OPTIC1_4");
    lidar_->set_raw_fault(INNO_LIDAR_IN_FAULT_OPTIC1_F, true,
                         "all reference channel breaks");
  } else {
    lidar_->heal_raw_fault(INNO_SUB_FAULT_OPTIC1_4, true, true,
                          "INNO_SUB_FAULT_OPTIC1_4 heals");
    lidar_->heal_raw_fault(INNO_LIDAR_IN_FAULT_OPTIC1_F, true,
                          "not all reference channel breaks: %u", mask);
    int64_t avg_temp = 0;
    int64_t min_temp = INT_MAX;
    int64_t max_temp = INT_MIN;
    bool use_four_channel = false;
    if (get_avg_detector_temp(&avg_temp, &min_temp, &max_temp) != 0 ||
        avg_temp < kDetTempLowThreshold ||
        max_temp - min_temp > kMaxDetTempDiffThreshold) {
      use_four_channel = true;
    }
    // check 1 channel only when:
    // 1.noise level is not high
    // 2.temperature is above -10 degC
    // 3.velocity is above 5km/h(not implemented yet)
    if (!lidar_->noise_level_high() && !use_four_channel) {
      if (mask & 0x1) {
        lidar_->set_raw_fault(INNO_SUB_FAULT_OPTIC1_0, true, true,
                             "INNO_SUB_FAULT_OPTIC1_0");
      } else {
        lidar_->heal_raw_fault(INNO_SUB_FAULT_OPTIC1_0, true, true,
                              "INNO_SUB_FAULT_OPTIC1_0");
      }
      if (mask & 0x2) {
        lidar_->set_raw_fault(INNO_SUB_FAULT_OPTIC1_1, true, true,
                             "INNO_SUB_FAULT_OPTIC1_1");
      } else {
        lidar_->heal_raw_fault(INNO_SUB_FAULT_OPTIC1_1, true, true,
                              "INNO_SUB_FAULT_OPTIC1_1 heals");
      }
      if (mask & 0x4) {
        lidar_->set_raw_fault(INNO_SUB_FAULT_OPTIC1_2, true, true,
                             "INNO_SUB_FAULT_OPTIC1_2");
      } else {
        lidar_->heal_raw_fault(INNO_SUB_FAULT_OPTIC1_2, true, true,
                              "INNO_SUB_FAULT_OPTIC1_2 heals");
      }
      if (mask & 0x8) {
        lidar_->set_raw_fault(INNO_SUB_FAULT_OPTIC1_3, true, true,
                             "INNO_SUB_FAULT_OPTIC1_3");
      } else {
        lidar_->heal_raw_fault(INNO_SUB_FAULT_OPTIC1_3, true, true,
                              "INNO_SUB_FAULT_OPTIC1_3");
      }
    } else {
      for (int i = INNO_SUB_FAULT_OPTIC1_0;
               i < INNO_SUB_FAULT_OPTIC1_3; i++) {
        lidar_->heal_raw_fault((InnoSubInFaults)i, true, true,
                               "sub-fault(%d) heals", i);
      }
    }
  }
}

enum OpticalCheckState StageSignal::OpticalPathCheck::\
                       do_optical_check_via_return_(uint32_t block_number,
                                                    bool optical_break,
                                                    bool blocked) {
  uint32_t counter_break = 0;
  uint32_t counter_block = 0;
  double window_not_blocked_return_threshold =
                                   window_not_blocked_return_threshold_;
  double return_pulse_ratio_threshold = return_pulse_ratio_threshold_;
  if (optical_break) {
    return_pulse_ratio_threshold *= hysteresis_factor_return;
  }
  if (blocked) {
    window_not_blocked_return_threshold *= hysteresis_factor_return;
  }
  double max_ratio = 0;
  double min_ratio = 1.0e10;
  for (uint32_t i = 0; i < kInnoChannelNumber; i++) {
    double sampled_ratio = (return_pulse_counter_[i] -
                            return_pulse_counter_old_[i]) /
                            static_cast<double>(block_number);
    if (sampled_ratio > max_ratio) {
      max_ratio = sampled_ratio;
    }
    if (sampled_ratio < min_ratio) {
      min_ratio = sampled_ratio;
    }
    if (sampled_ratio < return_pulse_ratio_threshold) {
      counter_break++;
      // return INNO_OPTICAL_TO_BE_CONFIRMED;
    } else if (sampled_ratio <= window_not_blocked_return_threshold) {
      counter_block++;
    } else {
      // xxx todo(WYY): improve detecting for different area
      // threshold and consider intensity impact
    }
  }
  if (max_ratio - min_ratio >= kMaxRatioDiffBetChannels) {
    lidar_->set_raw_fault(INNO_LIDAR_IN_FAULT_RESERVED17, true,
                         "Sanity check failed between channels. trigger: %u, "
                         "return pulse: %" PRI_SIZELU " /%"
                         PRI_SIZELU " /%" PRI_SIZELU " /%" PRI_SIZELU,
                          block_number,
                          return_pulse_counter_[0] -
                          return_pulse_counter_old_[0],
                          return_pulse_counter_[1] -
                          return_pulse_counter_old_[1],
                          return_pulse_counter_[2] -
                          return_pulse_counter_old_[2],
                          return_pulse_counter_[3] -
                          return_pulse_counter_old_[3]);
  } else {
    lidar_->heal_raw_fault(INNO_LIDAR_IN_FAULT_RESERVED17, true,
                          "Sanity check betweens recorvered");
  }
  if (counter_break > 0) {
    return INNO_OPTICAL_TO_BE_CONFIRMED;
  }
  if (counter_block >= kInnoChannelNumber) {
    return INNO_OPTICAL_OK_WINDOW_MAYBE_BLOCKED;
  } else {
    return INNO_OPTICAL_OK_WINDOW_NOT_BLOCKED;
  }
}

void StageSignal::OpticalPathCheck::prepare_optical_and_blockage_check() {
  for (uint32_t i = 0; i < kInnoChannelNumber; i++) {
    return_pulse_counter_old_[i] = return_pulse_counter_[i];
    scatter_counter_old_[i] = scatter_counter_[i];
    reference_counter_old_[i] = reference_counter_[i];
  }
}

void StageSignal::OpticalPathCheck::init_env_() {
  if (lidar_->is_live_lidar_()) {
    int ret = enable_scatter_filter_(kCountOneScatterPerPeriod);
    if (ret == 0) {
      scatter_running_ = true;
      inno_log_info("main optic and blockage check enabled");
    } else {
      inno_log_error("enable scatter optical check failed: %d, "
                     "please ignore when using file play mode", ret);
    }
  } else {
    // set true for file play mode
    scatter_running_ = true;
    inno_log_info("non-live lidar, enable window blockage check");
  }
}

void StageSignal::OpticalPathCheck::\
                  do_optical_and_blockage_diagnosis(uint32_t block_number) {
  if (block_number > kMinDiagnoseBlockNumber) {
    if (scatter_running_) {
      do_main_optical_and_blockage_check_(block_number);
    }
    do_reference_optical_check_(block_number);
  }
}

StageSignal::StageSignal(InnoLidar *l,
                         double play_rate_x) {
  inno_log_verify(l, "invalid lidar");
  lidar_ = l;
  fpga_clock_.\
  set_lidar_live_direct_mode(lidar_->is_live_direct_memory_lidar_());
  play_rate_x_ = play_rate_x;
  start_time_us_ = 0;
  first_data_us_ = 0;
  params_ = &lidar_->get_params();
  misc_tables_ = &lidar_->get_misc_tables();
  lidar_->add_config(&config_base_);
  config_.copy_from_src(&config_base_);

  leftover_signal_len_ = 0;
  in_roi_ = 0;
  trigger_count_[0] = 0;
  trigger_count_[1] = 0;
  trigger_count_old_[0] = 0;
  trigger_count_old_[1] = 0;
  for (uint32_t channel = 0; channel < kInnoChannelNumber; channel ++) {
    ref_tracking_result_[channel].init(config_.ref_window_center,
                                       channel);
    for (uint32_t i = 0; i < kRoiTypeCount; i++) {
      ref_tracking_[i][channel].init(channel * kRoiTypeCount + i,
                                     config_.ref_window_center,
                                     config_.ref_window_half_width_init,
                                     config_.ref_window_half_width_min,
                                     config_.ref_window_half_width_max,
                                     config_.ref_window_left_limit,
                                     config_.ref_window_right_limit,
                                     config_.ref_sample_num,
                                     config_.ref_match_threshold,
                                     config_.ref_new_weight,
                                     channel, i);
    }
  }
  previous_trigger_sub_ns_ = 0;
  current_trigger_sub_ns_ = 0;
  previous_raw_block_ = NULL;
  current_raw_block_ = NULL;

  job_count_ = 0;

  this_round_g_encoder_count_ = 0;  // can be reset in error handling
  stats_g_encoder_count_ = 0;
  last_g_encoder_count_ = 0;
  stats_pr_encoder_count_ = 0;
  last_pr_encoder_count_ = 0;
  stats_pf_encoder_count_ = 0;
  stats_padding_count_ = 0;
  stats_trigger_count_ = 0;
  last_trigger_count_ = 0;
  stats_trigger_count_old_ = 0;
  stats_adc_count_ = 0;
  stats_pulse_count_ = 0;
  stats_ref_count_ = 0;
  stats_ref_track_count_ = 0;
  stats_previous_pulse_0_ = 0;
  stats_previous_pulse_1_ = 0;
  stats_discarded_multi_return_ = 0;
  stats_scatter_ = 0;
  stats_road_points_ = 0;
  stats_retro_ = 0;
  stats_discard_pre_retro_ = 0;
  stats_discard_post_retro_ = 0;
  stats_below_min_intensity_ = 0;
  stats_frame_counts_ = -1;

  stats_force_drop_0_ = 0;
  stats_not_enough_g_0_ = 0;
  stats_other_encoder_count_ = 0;
  stats_bad_ts_0_ = 0;
  stats_bad_ts_1_ = 0;
  stats_discard_pulse_ = 0;
  stats_discard_trigger_ = 0;
  stats_discard_encoder_0_ = 0;
  stats_discard_encoder_1_ = 0;
  stats_discard_encoder_2_ = 0;
  stats_galvo_delay_ = 0;
  stats_ignore_polygon_no_active_ = 0;
  stats_ignore_polygon_small_gap_ = 0;
  stats_road_0_ = 0;
  stats_road_1_ = 0;

  init_ts_ = lidar_->get_monotonic_raw_time_ms();
  raw_data_drop_ts_ = init_ts_;
  diff_polygon_ns_TO_ = init_ts_;
  diff_galvo_ns_TO_ = init_ts_;
  trigger_period_ns_TO_ = init_ts_;
  motor_trigger_drop_ = -1;
  stats_trigger_interval_too_big_ = 0;
  trigger_timeout_ = false;
  galvo_timeout_ = false;
  polygon_timeout_ = false;
  rawdata_fault_ = false;
  polygon_range_ = 10;  // +-10%
  update_limit_ = false;
  last_polygon_ns_ = 0;
  last_real_polygon_ns_ = 0;
  fake_polygon_index_ = 1;
  next_fake_fpga_ns_ = 0;
  for (uint32_t i = 0; i < kMaxPolygonEncodes; i++) {
    polygons_[i] = {0, 0};
  }
  is_calc_fake_polygon_ = false;
  polygon_mean_.reset();
  polygon_real_mean_.reset();
  last_galvo_change_direction_ns_ = 0;
  pre_galvo_direction_ = INNO_FRAME_DIRECTION_MAX;
  for (int i = 0; i < INNO_FRAME_DIRECTION_MAX; i++) {
    galvo_mean_[i].reset();
  }

  pre_lines_ = NULL;
  cur_lines_ = l->alloc_angle_job();
  inno_log_verify(cur_lines_, "cur_line");
  init_job_conf_seq_(cur_lines_);
  if (lidar_->in_calibration_mode()) {
    cur_lines_->set_calibration_mode();
  }
  // cur_lines_->add_polygon(stats_pr_encoder_count_, 0, 0);
  current_block_source_lines_ = NULL;

  current_job_ = NULL;
  raw2_call_count_ = 0;
  raw2_shutdown_ = false;
  raw2_done_ = true;
  raw2_thread_ = NULL;
  raw4_thread_ = NULL;
  raw_data_record_enable_ = true;
  stats_ref_intensity_sum_ = 0;
  live_lidar_ = lidar_->is_live_lidar();
  memset(has_ref_old_, 0, sizeof(has_ref_old_));
  ref_max_intensity_ = config_.ref_max_intensity;
  ref_min_intensity_ = config_.ref_min_intensity;

  optical_path_check_ =
          new OpticalPathCheck(lidar_,
                               this,
                               config_.return_pulse_ratio_threshold,
                               config_.scatter_ratio_threshold,
                               config_.reference_ratio_threshold,
                               config_.window_not_blocked_return_threshold,
                               config_.window_blockage_scatter_threshold,
                               config_.scatter_check_cpu_threshold);
  inno_log_verify(optical_path_check_, "optical_path_check invalid");
  for (uint32_t i = 0; i < kInnoChannelNumber; i++) {
    ref_sample_rate_[i] = kRefSampleRate[i];
  }
  config_chA_ref_sample_rate_();

  // must init ref_finder_ after sample rate is configured
  for (uint32_t i = 0; i < kInnoChannelNumber; i++) {
    ref_finder_[i].init(i);
    stats_ref_fit_count_[i] = 0;
    stats_ref_unfit_count_[i] = 0;
    stats_total_ref_check_count_[i] = 0;
  }

  int ret = lidar_->get_sn(sn_, sizeof(sn_));
  if (ret != 0) {
    inno_log_error("get_sn %d", ret);
  }
  need_find_magic_ = false;
  find_magic_tried_ = 0;
  ignore_begin_frames_blockage_ = kIgnoreBeginFrameBlockage;
  ignore_begin_frames_ref_intensity_ = kIgnoreBeginFrameRefIntensity;
  signal_fault_begin_detection_ = kSignalFaultStartDetect;
  if (!lidar_->is_live_lidar()) {
    ignore_begin_frames_blockage_ = 2;
    ignore_begin_frames_ref_intensity_ = 2;
    signal_fault_begin_detection_ = 2;
  }
  current_mode_old_ = INNO_LIDAR_MODE_NONE;
}

StageSignal::~StageSignal(void) {
  // todo: for test
  print_stats();
  if (optical_path_check_) {
    delete optical_path_check_;
    optical_path_check_ = NULL;
  }

  if (pre_lines_) {
    if (pre_lines_->dec_ref()) {
      lidar_->free_angle_job(pre_lines_);
    }
    pre_lines_ = NULL;
  }

  if (cur_lines_) {
    if (cur_lines_->dec_ref()) {
      lidar_->free_angle_job(cur_lines_);
    }
    cur_lines_ = NULL;
  }

  {
    bool need_join = false;
    {
      std::unique_lock<std::mutex> lk(raw2_mutex_);
      raw2_shutdown_ = true;
      raw2_cond_.notify_all();
      if (raw2_thread_) {
        need_join = true;
      }
    }

    if (need_join) {
      raw2_thread_->join();
      delete raw2_thread_;
      raw2_thread_ = NULL;
    }
  }

  if (raw4_thread_) {
    raw4_thread_->join();
    delete raw4_thread_;
    raw4_thread_ = NULL;
  }

  {
    clear_deque_wo_lock_(&raw4_send_deque_);
    clear_deque_wo_lock_(&raw2_send_deque_);
    clear_deque_wo_lock_(&raw_deque_);
  }
  lidar_->remove_config(&config_base_);

  if (this->fault_thread_) {
    this->fault_thread_->join();
    delete this->fault_thread_;
    this->fault_thread_ = nullptr;
  }
}

int StageSignal::process(void *in_job, void *ctx,
                         bool prefer) {
  StageSignal *s = reinterpret_cast<StageSignal *>(ctx);
  StageSignalJob *job = reinterpret_cast<StageSignalJob *>(in_job);
#ifdef __APPLE__
  INNER_BEGIN_LOG(StageSignal_process, OS_LOG_CATEGORY_DYNAMIC_TRACING,
                  StageSignal_process);
#endif
  int ret = s->process_job_(job, prefer);
#ifdef __APPLE__
  INNER_END_LOG(StageSignal_process);
#endif
  return ret;
}

int StageSignal::process_job_(StageSignalJob *job,
                              bool prefer) {
  inno_log_verify(prefer, "stage_signal job not prefer");
  inno_log_assert(job, "job");

  for (uint32_t i = 0; i < INNO_CONFIDENCE_LEVEL_MAX; i++) {
    enum ConfidenceLevel conf_level = ConfidenceLevel(i);
    conf_seq_num_[i] = job->get_confidence_seq(conf_level);
  }

  config_.copy_from_src(&config_base_);

  inc_job_count_();
  optical_path_check_->load_ref_intensity_from_yaml();

  if (cur_lines_) {
    if (cur_lines_->stage_ts[ScanLines::STAGE_TIME_S0] == 0) {
      cur_lines_->stage_ts[ScanLines::STAGE_TIME_S0] =
          lidar_->get_monotonic_raw_time();
    }
    if (cur_lines_->stage_ts[ScanLines::STAGE_TIME_R1] == 0) {
      cur_lines_->stage_ts[ScanLines::STAGE_TIME_R1] = job->enque_ts;
    }
  }
  uint64_t current_duration_s = 0;
  enum InnoLidarMode current_mode = INNO_LIDAR_MODE_NONE;
  enum InnoLidarMode pre_mode = INNO_LIDAR_MODE_NONE;
  enum InnoLidarStatus status = INNO_LIDAR_STATUS_NONE;
  if (lidar_->get_mode_status(&current_mode, &pre_mode,
                              &status, &current_duration_s) == 0) {
    if (current_mode_old_ != INNO_LIDAR_MODE_NONE &&
        current_mode != current_mode_old_) {
      update_ref_info_(current_mode, current_mode_old_);
    }
    current_mode_old_ = current_mode;
    detect_signal_fault_(current_mode, status);
  } else {
    inno_log_error("fail to get mode status");
  }
  expand_ref_window_();
  if (job->get_is_first_chunck()) {
    fpga_clock_.set_is_first_chunck();
  } else if (leftover_signal_len_) {  // ignore leftover for first chunck
    // copy leftover
    job->set_leftover(leftover_signal_, leftover_signal_len_);
    leftover_signal_len_ = 0;
  }
  current_job_ = job;

  const char *start_init = job->get_buffer_with_leftover();
  const char *end_init = start_init + job->get_buffer_with_leftover_len();
  const char *start = start_init;
  const char *end = end_init;

  if (need_find_magic_) {
    const char *new_start = process_job_magic_(start, end);
    if (new_start) {
      inno_log_info("find new magic offset=%" PRI_SIZELU
                    " at job %" PRI_SIZELU ". round=%" PRI_SIZEU,
                    new_start - start, job_count_,
                    find_magic_tried_);
      start = new_start;
      need_find_magic_ = false;
    } else {
      inno_log_warning("cannot find magic, len=%" PRI_SIZELU
                      " at job %" PRI_SIZELU " . round=%" PRI_SIZEU,
                       end - start, job_count_,
                       find_magic_tried_);
    }
    find_magic_tried_++;
  }
  try {
    uint64_t left = process_job_buffer_(start, end);
    // save leftover
    leftover_signal_len_ = left;
    // xxx todo kLeftoverSize must > max(kDataTypeSize_[])
    memcpy(leftover_signal_, end - left, leftover_signal_len_);
    job->set_tail_size(leftover_signal_len_);
    need_find_magic_ = false;
  } catch (enum InnoBadFpgaData eid) {
    inno_log_error("caught exception %u in processing signal", eid);
    need_find_magic_ = true;
    reset_state_();
  }

  raw2_recorder_(job);
  uint64_t time_now_ms = lidar_->get_monotonic_raw_time_ms();
  add_to_raw_deque_(job);
  raw3_raw4_recorder_();
  current_job_ = NULL;
  lidar_->stats_update_packet_bytes(stats_ref_count_,
                                    stats_ref_intensity_sum_);

  //
  update_time_status_(time_now_ms);

  InnoTimestampUs last_data_us =
      InnoConverts::ns_to_us(fpga_clock_.lastest_ns());
  rate_control_(last_data_us);

  return 0;
}

void StageSignal::reset_state_() {
  fpga_clock_.set_bad_state_();
  if (pre_lines_) {
    if (pre_lines_->dec_ref()) {
      lidar_->free_angle_job(pre_lines_);
    }
    pre_lines_ = NULL;
  }
  if (cur_lines_) {
    if (cur_lines_->dec_ref()) {
      lidar_->free_angle_job(cur_lines_);
    }
    cur_lines_ = NULL;
  }
  cur_lines_ = lidar_->alloc_angle_job();
  inno_log_verify(cur_lines_, "cur_line");
  init_job_conf_seq_(cur_lines_);
  if (lidar_->in_calibration_mode()) {
    cur_lines_->set_calibration_mode();
  }
  current_block_source_lines_ = NULL;
  previous_trigger_sub_ns_ = 0;
  current_trigger_sub_ns_ = 0;
  previous_raw_block_ = NULL;
  current_raw_block_ = NULL;
  last_polygon_ns_ = 0;
  last_real_polygon_ns_ = 0;
  fake_polygon_index_ = 1;
  next_fake_fpga_ns_ = 0;
  is_calc_fake_polygon_ = false;
  this_round_g_encoder_count_ = 0;
}

const char *StageSignal::process_job_magic_(const char *start,
                                            const char *end) {
  const char *raw = start;
  uint8_t type;
  const char *kMagic = "RaDiLnOiSuVoNnI";
  inno_log_verify(kDataTypeSize_[kRawDataTypeOther] - 1 ==
                  strlen(kMagic), "invalid len %" PRI_SIZELU,
                  kDataTypeSize_[kRawDataTypeOther]);
  while (raw + kDataTypeSize_[kRawDataTypeOther] <= end) {
    type = (*raw) & 0xf;
    if (type == kRawDataTypeOther) {
      uint8_t sub_type;
      sub_type = (*raw & 0xf0) >> 4;
      if (sub_type == kRawDataSubTypeMagic) {
        // match the rest
        if (memcmp(raw + 1, kMagic,
                   kDataTypeSize_[kRawDataTypeOther] - 1) == 0) {
          return raw + kDataTypeSize_[kRawDataTypeOther];
        }
      }
    }
    raw++;
  }

  // cannot find
  return NULL;
}

uint64_t StageSignal::process_job_buffer_(const char *start,
                                          const char *end) {
  const char *raw = start;
  uint8_t type;

  while (raw < end) {
    type = (*raw) & ((0x1 << kMaxSignalTypeBits) - 1);
    size_t size = kDataTypeSize_[type];
    if (raw + size > end) {
#ifdef MUST_PAD_TO_BOUNDARY
      // xxx todo: handle it gracefully
      inno_log_verify(0, "type=%hd size=%" PRI_SIZELU " %p vs %p",
                      type, size, raw, end);
#else
      return end - raw;
#endif
    }

    ++stats_signals_[type];
    if (type & kRawDataTypeAdc) {
      const AdcComputedDataExt &r =
          *reinterpret_cast<const AdcComputedDataExt *>(raw);
      process_adc_data_(r);
    } else if (type == kRawDataTypeTrigger) {
      const TriggerData &r = *reinterpret_cast<const TriggerData *>(raw);
      process_trigger_data_(r);
    } else if (type == kRawDataTypeEncoder) {
      const EncoderData &r = *reinterpret_cast<const EncoderData *>(raw);
      process_encoder_data_(r);
    } else if (type == kRawDataTypeOther) {
      uint8_t sub_type;
      sub_type = (*raw & 0xf0) >> 4;
      if (sub_type == kRawDataSubTypeGps) {
        const GpsData &r = *reinterpret_cast<const GpsData *>(raw);
        lidar_->get_clock().process_gps_data(
            r, lidar_->get_monotonic_raw_time_ms());
      } else if (sub_type == kRawDataSubTypePtp) {
        const PtpData &r = *reinterpret_cast<const PtpData *>(raw);
        lidar_->get_clock().process_ptp_data(
            r, lidar_->get_monotonic_raw_time_ms());
      } else if (sub_type == kRawDataSubTypeNtp) {
        const NtpData &r = *reinterpret_cast<const NtpData *>(raw);
        lidar_->get_clock().process_ntp_data(
            r, lidar_->get_monotonic_raw_time_ms());
      } else if (sub_type == kRawDataSubTypeMagic) {
        // do nothing
      } else {
        // todo: how to handle it gracefully
        inno_log_error("invalid data sub_type %d", sub_type);
        throw INNO_BAD_FPGA_DATA_INVALID_SUB_TYPE;
      }
    } else if (type == kRawDataTypePadding) {
      const PaddingData &r = *reinterpret_cast<const PaddingData *>(raw);
      process_padding_data_(r);
    } else {
      // 10, 12, 14 are all invalid
      // xxx todo: handle it gracefully
      inno_log_error("invalid data type %d", type);
      throw INNO_BAD_FPGA_DATA_INVALID_TYPE;
    }
    raw += size;
  }

  inno_log_verify(raw == end, "%p, %p", raw, end);
  return 0;
}

void StageSignal::report_fault_() {
  inno_log_info("thread start: to report INNO_LIDAR_IN_FAULT_NETWORK1");

  size_t numread = 0;
  char ptp4l_pid[201] = {0};

  {
    // get ptp4l id
    const char *cmd = "pidof ptp4l";
    FILE *fp = popen(cmd, "r");
    if (fp == NULL) {
      inno_log_error("fail to exec popen(%s)", cmd);
      return;
    }
    numread = fread(ptp4l_pid, 1, 200, fp);
    pclose(fp);

    if (numread > 0) {
      ptp4l_pid[200] = '\0';
    }
  }

  char pmc[2001] = {0};
  {
    const char *cmd =
        "/app/firmware/ptp/pmc -u -b1 -f "
        "/app/firmware/ptp/automotive-slave.cfg 'get port_stats_np'";
    FILE *fp = popen(cmd, "r");
    if (fp == NULL) {
      inno_log_error("fail to exec popen(%s)", cmd);
      return;
    }
    numread = fread(pmc, 1, 2000, fp);
    pclose(fp);

    pmc[2000] = '\0';
  }

  this->lidar_->set_raw_fault(
      INNO_LIDAR_IN_FAULT_NETWORK1, true,
      "can't receive ptp message in 30 sec, ptp4l id: %s, pmc: %s", ptp4l_pid,
      pmc);

  {
    std::unique_lock<std::mutex> lk(raw4_done_mutex_);
    this->fault_done_ = true;
  }

  inno_log_info("thread finish: to report INNO_LIDAR_IN_FAULT_NETWORK1");
}

//
//
//
void StageSignal::update_time_status_(uint64_t time_now_ms) {
  static bool s_is_heal = true;

  LidarClock::LostType lost_type =
      lidar_->get_clock().check_sync_lost(time_now_ms, init_ts_);

  switch (lost_type) {
    case LidarClock::LostType::HEAL:
      s_is_heal = true;
      lidar_->heal_raw_fault(INNO_LIDAR_IN_FAULT_NETWORK1, true,
                             "receive ptp message in 30 sec");
      break;
    case LidarClock::LostType::LOST: {
      if (s_is_heal) {
        s_is_heal = false;

        {
          std::unique_lock<std::mutex> lk(raw4_done_mutex_);
          if (fault_done_) {
            if (this->fault_thread_) {
              this->fault_thread_->join();
              delete this->fault_thread_;
              this->fault_thread_ = nullptr;

              this->fault_done_ = false;
            }
          }
        }

        if (!this->fault_thread_) {
          this->fault_done_ = false;
          this->fault_thread_ = new std::thread([this]() { report_fault_(); });
        }
      }
    } break;

    case LidarClock::LostType::NOP:
    default:
      break;
  }
}

//
//
//
void StageSignal::update_polygon_limit_(InnoLidarMode mode) {
  int64_t motor_rps = 1;
  int64_t pgon_ns = 0;
  switch (mode) {
    case INNO_LIDAR_MODE_WORK_NORMAL:
      motor_rps = InnoConsts::KNormalRPM / InnoConsts::kSecondInMinute;
      break;
    case INNO_LIDAR_MODE_WORK_SHORT_RANGE:
      motor_rps = InnoConsts::KShortRangeRPM / InnoConsts::kSecondInMinute;
      break;
    case INNO_LIDAR_MODE_WORK_CALIBRATION:
      motor_rps = InnoConsts::KCalibrationRPM / InnoConsts::kSecondInMinute;
      break;
    default:
      break;
  }
  pgon_ns = InnoConsts::kNsInSecond / motor_rps;

  uint32_t encodes_per_plg = lidar_->get_encodes_per_polygon();
  inno_log_verify(encodes_per_plg != 0, "encodes_per_polygon_ == 0");
  config_.polygon_highlimit_ns = (pgon_ns + polygon_range_ * pgon_ns / 100)
                                 / encodes_per_plg;
  config_.polygon_lowlimit_ns = (pgon_ns - polygon_range_ * pgon_ns / 100)
                                 / encodes_per_plg;
}

void StageSignal::update_galvo_limit_(InnoLidarMode mode) {
  // galvo_fpm_1000th is independent with diff_galvo_ns
  config_.galvo_lowlimit_ns = 30000;
  config_.galvo_highlimit_ns = 300000;
}

void StageSignal::update_trigger_limit_(InnoLidarMode mode) {
  switch (mode) {
    case INNO_LIDAR_MODE_WORK_NORMAL:  // ~2200
      config_.trigger_lowlimit_ns = 700;
      config_.trigger_highlimit_ns = 10000;
      break;
    case INNO_LIDAR_MODE_WORK_SHORT_RANGE:  // 4000~9000
      config_.trigger_lowlimit_ns = 2000;
      config_.trigger_highlimit_ns = 30000;
      break;
    case INNO_LIDAR_MODE_WORK_CALIBRATION:  // ~1700
      config_.trigger_lowlimit_ns = 500;
      config_.trigger_highlimit_ns = 8000;
      break;
    default:
      break;
  }
}

int StageSignal::detect_signal_fault_(enum InnoLidarMode current_mode,
                                      enum InnoLidarStatus status) {
  int ret = -1;
  double now_ts = lidar_->get_monotonic_raw_time_ms();
  uint32_t value = 0;
  static int reg_read_fail_counter = 0;
  int reg_read_ret = lidar_->read_pl_reg(MOTOR_DROP_PACKET_REG, &value);
  if (reg_read_ret == 0) {
    reg_read_fail_counter = 0;
    if (value != motor_trigger_drop_) {
      raw_data_drop_ts_ = now_ts;
      motor_trigger_drop_ = value;
    }
  } else {
    if (++reg_read_fail_counter > 2) {
      raw_data_drop_ts_ = now_ts;
    }
  }
  bool stable = ((now_ts - lidar_->get_streaming_start_ts() >
                 kMinStableTimeAfterStreamStart) &&
                (now_ts - raw_data_drop_ts_ > kMinStableTimeAfterRawDrop)) ||
                (!lidar_->is_live_lidar());
  if (status == INNO_LIDAR_STATUS_TRANSITION) {
    update_limit_ = false;
  }
  if (current_mode == INNO_LIDAR_MODE_WORK_NORMAL ||
      current_mode == INNO_LIDAR_MODE_WORK_SHORT_RANGE ||
      current_mode == INNO_LIDAR_MODE_WORK_CALIBRATION) {
    if (!update_limit_) {
      update_polygon_limit_(current_mode);
      update_galvo_limit_(current_mode);
      update_trigger_limit_(current_mode);
      update_limit_ = true;
    } else {
      if (stable) {
        if (trigger_timeout_ ||
            last_trigger_count_ == stats_trigger_count_) {
          lidar_->set_raw_fault(INNO_LIDAR_IN_FAULT_TRIGGER_TO, true,
                                "trigger_period is out of range! "
                                "trigger_period = %u, "
                                "config_.trigger_highlimit_ns = %" PRI_SIZED
                                ", config_.trigger_lowlimit_ns = %" PRI_SIZED
                                " last_trigger_count_ = %" PRI_SIZELU
                                ", stats_trigger_count_ = %" PRI_SIZELU,
                                trigger_period_ns_TO_,
                                config_.trigger_highlimit_ns,
                                config_.trigger_lowlimit_ns,
                                last_trigger_count_, stats_trigger_count_);
        } else {
          lidar_->heal_raw_fault(INNO_LIDAR_IN_FAULT_TRIGGER_TO, true,
                                "INNO_LIDAR_IN_FAULT_TRIGGER_TO heals");
        }
        if (galvo_timeout_ ||
            last_g_encoder_count_ == stats_g_encoder_count_) {
          lidar_->set_raw_fault(INNO_LIDAR_IN_FAULT_GALVO_TO, true,
                                "diff_galvo_ns is out of range! "
                                "diff_galvo_ns = %" PRI_SIZED
                                ", config_.galvo_highlimit_ns = %" PRI_SIZED
                                ", config_.galvo_lowlimit_ns = %" PRI_SIZED
                                " last_trigger_count_ = %" PRI_SIZELU
                                ", stats_trigger_count_ = %" PRI_SIZELU,
                                diff_galvo_ns_TO_,
                                config_.galvo_highlimit_ns,
                                config_.galvo_lowlimit_ns,
                                last_trigger_count_, stats_trigger_count_);
        } else {
          lidar_->heal_raw_fault(INNO_LIDAR_IN_FAULT_GALVO_TO, true,
                                "INNO_LIDAR_IN_FAULT_GALVO_TO heals");
        }
        if (polygon_timeout_ || ((job_count_ % 10 == 0) &&
            (last_pr_encoder_count_ == stats_pr_encoder_count_))) {
          lidar_->set_raw_fault(INNO_LIDAR_IN_FAULT_POLYGON_TO, true,
                                "diff_polygon_ns is out of range! "
                                "diff_polygon_ns = %" PRI_SIZED
                                ", config_.polygon_highlimit_ns = %" PRI_SIZED
                                ", config_.polygon_lowlimit_ns = %" PRI_SIZED
                                " last_trigger_count_ = %" PRI_SIZELU
                                ", stats_trigger_count_ = %" PRI_SIZELU,
                                 diff_polygon_ns_TO_,
                                 config_.polygon_highlimit_ns,
                                 config_.polygon_lowlimit_ns,
                                 last_trigger_count_, stats_trigger_count_);
        } else {
          lidar_->heal_raw_fault(INNO_LIDAR_IN_FAULT_POLYGON_TO, true,
                                "INNO_LIDAR_IN_FAULT_POLYGON_TO heals");
        }
        if (rawdata_fault_) {
          lidar_->set_raw_fault(INNO_LIDAR_IN_FAULT_RAWDATA_STREAM, true,
                               "pre galvos_number is 0, "
                               "not_enough_g_0 counts: %ld",
                                stats_not_enough_g_0_);
        } else {
          lidar_->heal_raw_fault(INNO_LIDAR_IN_FAULT_RAWDATA_STREAM, true,
                                "INNO_LIDAR_IN_FAULT_RAWDATA_STREAM heals");
        }
      }
      ret = 0;
    }
  }
  trigger_timeout_ = false;
  galvo_timeout_ = false;
  polygon_timeout_ = false;
  rawdata_fault_ = false;
  last_g_encoder_count_ = stats_g_encoder_count_;
  last_trigger_count_ = stats_trigger_count_;
  if (job_count_ % 10 == 0) {
    last_pr_encoder_count_ = stats_pr_encoder_count_;
  }
  return ret;
}


int StageSignal::add_to_raw_deque_(StageSignalJob *job) {
  std::unique_lock<std::mutex> lk(raw2_mutex_);
  raw_deque_.push_back(job);

  while (raw_deque_.size() > kRawDequeSize) {
    StageSignalJob *p = raw_deque_[0];
    raw_deque_.pop_front();

    inno_log_verify(p, "job from deque");
    if (p->dec_ref()) {
      lidar_->free_signal_job(p);
    }
  }

  return 0;
}


void StageSignal::clear_deque_wo_lock_(std::deque<StageSignalJob *> *q) {
  inno_log_verify(q, "q");
  while (q->size()) {
    StageSignalJob *p = (*q)[0];
    q->pop_front();
    if (p->dec_ref()) {
      lidar_->free_signal_job(p);
    }
  }
}

//
// handle INNO_RECORDER_CALLBACK_TYPE_RAW2
//
int StageSignal::raw2_recorder_(StageSignalJob *job) {
  if (lidar_->has_recorder_callback(INNO_RECORDER_CALLBACK_TYPE_RAW2)) {
    std::unique_lock<std::mutex> lk(raw2_mutex_);

    if (raw2_thread_ == NULL) {
      // first call after set the callback
      inno_log_verify(raw2_send_deque_.size() == 0, "q=%" PRI_SIZELU,
                      raw2_send_deque_.size());
      for (uint32_t i = 0; i < raw_deque_.size(); i++) {
        raw2_send_deque_.push_back(raw_deque_[i]);
        raw_deque_[i]->inc_ref();
      }

      // start new thread
      raw2_call_count_ = 0;
      raw2_done_ = false;
      raw2_shutdown_ = false;
      inno_log_verify(raw2_thread_ == NULL, "raw2_thread");
      raw2_thread_ = new std::thread([this]() {
        InnoUtils::set_self_thread_priority(-20);
        raw2_send_();
      });
      inno_log_verify(raw2_thread_, "raw2_thread");
    }

    if (raw2_done_) {
      inno_log_verify(raw2_thread_, "raw2_thread");
      lk.unlock();
      raw2_thread_->join();
      {
        std::unique_lock<std::mutex> lk2(raw2_mutex_);
        delete raw2_thread_;
        raw2_thread_ = NULL;
        clear_deque_wo_lock_(&raw2_send_deque_);
      }
    } else {
      if (raw2_shutdown_ == false) {
        if (raw2_send_deque_.size() > kRawDequeSize) {
          inno_log_info("send_deque too big");
          raw2_shutdown_ = true;
        } else {
          job->inc_ref();
          raw2_send_deque_.push_back(job);
        }
        raw2_cond_.notify_all();
      }
    }
  } else {
    std::unique_lock<std::mutex> lk(raw2_mutex_);
    if (raw2_thread_) {
      if (raw2_done_) {
        lk.unlock();
        raw2_thread_->join();
        delete raw2_thread_;
        raw2_thread_ = NULL;
        lk.lock();
        clear_deque_wo_lock_(&raw2_send_deque_);
        lk.unlock();
      }
    }
  }
  return 0;
}


int StageSignal::raw2_send_() {
  while (1) {
    std::unique_lock<std::mutex> lk(raw2_mutex_);

    if (raw2_shutdown_) {
      lidar_->do_recorder_callback(INNO_RECORDER_CALLBACK_TYPE_RAW2, NULL, 0);
      clear_deque_wo_lock_(&raw2_send_deque_);
      break;
    }

    if (raw2_call_count_ == 0) {
      lidar_->recorder_write_yaml(INNO_RECORDER_CALLBACK_TYPE_RAW2);
    }

    raw2_call_count_++;

    while (raw2_send_deque_.size()) {
      StageSignalJob *p = raw2_send_deque_[0];
      raw2_send_deque_.pop_front();
      lk.unlock();

      int ret = lidar_->do_recorder_callback(
          INNO_RECORDER_CALLBACK_TYPE_RAW2, p->get_buffer_with_leftover(),
          p->get_buffer_with_leftover_len_without_tail());
      if (p->dec_ref()) {
        lidar_->free_signal_job(p);
      }

      lk.lock();
      if (ret != 0) {
        raw2_done_ = true;
        break;
      }
    }

    if (raw2_done_) {
      clear_deque_wo_lock_(&raw2_send_deque_);
      break;
    }

    raw2_cond_.wait(
        lk, [this] { return raw2_send_deque_.size() != 0 || raw2_shutdown_; });
  }

  raw2_done_ = true;
  raw2_cond_.notify_all();
  return 0;
}

//
// flag save / send raw data;
//
void StageSignal::set_save_raw_data_flag(std::string cause) {
  constexpr int kCauseMaxSize = 20;

  if (raw3_is_enable_() || raw4_is_enable_()) {
    std::unique_lock<std::mutex> lk(raw_cause_mutex_);
    if (this->raw_cause_.size() > kCauseMaxSize) {
      this->raw_cause_.pop_front();
    }

    this->raw_cause_.push_back(cause);
  }
}

//
// do save / send raw data;
//
void StageSignal::raw3_raw4_recorder_() {
  // debug
  // set_save_raw_data_flag("test1");
  // set_save_raw_data_flag("test2");
  // set_save_raw_data_flag("test3");

  {
    std::unique_lock<std::mutex> lk(raw_cause_mutex_);
    if (this->raw_cause_.empty()) {
      return;
    }
  }

  if (raw3_is_enable_()) {
    raw3_recorder_();
  }

  // raw4 thread will clear this->raw_cause_;
  if (!raw4_is_enable_() && !raw4_thread_) {
    std::unique_lock<std::mutex> lk(raw_cause_mutex_);
    this->raw_cause_.clear();
  }

  raw4_recorder_();
}


bool StageSignal::raw3_is_enable_() {
  return raw_data_record_enable_ &&
         lidar_->has_recorder_callback(INNO_RECORDER_CALLBACK_TYPE_RAW3);
}

//
// handle INNO_RECORDER_CALLBACK_TYPE_RAW3
//
int StageSignal::raw3_recorder_() {
  {
    std::unique_lock<std::mutex> lk(raw_cause_mutex_);
    for (auto &info : this->raw_cause_) {
      inno_log_info("raw data to write: %s.", info.c_str());
    }
  }

  // only save once.
  raw_data_record_enable_ = false;

  lidar_->recorder_write_yaml(INNO_RECORDER_CALLBACK_TYPE_RAW3);

  inno_log_info("dump %" PRI_SIZELU " buffer to raw3", raw_deque_.size());
  for (uint32_t i = 0; i < raw_deque_.size(); i++) {
    StageSignalJob *p = raw_deque_[i];
    lidar_->do_recorder_callback(
        INNO_RECORDER_CALLBACK_TYPE_RAW3, p->get_buffer_with_leftover(),
        p->get_buffer_with_leftover_len_without_tail());
  }

  inno_log_info("dump to raw3 done");
  if (current_job_) {
    lidar_->do_recorder_callback(INNO_RECORDER_CALLBACK_TYPE_RAW3,
                                 current_job_->get_buffer_with_leftover(),
                                 current_job_->get_buffer_with_leftover_len());
  }

  inno_log_info("dump current done");
  lidar_->do_recorder_callback(INNO_RECORDER_CALLBACK_TYPE_RAW3, NULL, 0);

  return 0;
}

bool StageSignal::raw4_is_enable_() {
  return lidar_->has_recorder_callback(INNO_RECORDER_CALLBACK_TYPE_RAW4);
}

//
// handle INNO_RECORDER_CALLBACK_TYPE_RAW4
//
int StageSignal::raw4_recorder_() {
  if (lidar_->has_recorder_callback(INNO_RECORDER_CALLBACK_TYPE_RAW4)) {
    if (raw4_thread_ == NULL) {
      // first call after set the callback
      inno_log_verify(raw4_send_deque_.empty(), "q=%" PRI_SIZELU,
                      raw4_send_deque_.size());

      for (uint32_t i = 0; i < raw_deque_.size(); i++) {
        raw4_send_deque_.push_back(raw_deque_[i]);
        raw_deque_[i]->inc_ref();
      }

      // start new thread
      raw4_done_ = false;
      inno_log_verify(raw4_thread_ == NULL, "raw4_thread exist.");

      // todo
      raw4_thread_ = new std::thread([this]() {
        raw4_send_();
      });

      inno_log_verify(raw4_thread_, "raw4_thread is null.");
    }
  }

  if (raw4_thread_) {
    std::unique_lock<std::mutex> lk(raw4_done_mutex_);
    if (raw4_done_) {
      lk.unlock();

      raw4_thread_->join();
      delete raw4_thread_;
      raw4_thread_ = NULL;
    }
  }

  return 0;
}


int StageSignal::raw4_send_() {
  InnoRaw4Packet raw_packet;
  raw_packet.idx = lidar_->get_monotonic_raw_time_sec();

  inno_log_info("raw4 send start, idx: %u", raw_packet.idx);

  // send sn
  {
    raw_packet.field_type = InnoRaw4Packet::TYPE_SN;
    raw_packet.is_field_last = true;
    raw_packet.buffer = this->sn_;
    raw_packet.buffer_size = sizeof(this->sn_);
    lidar_->do_recorder_callback(INNO_RECORDER_CALLBACK_TYPE_RAW4,
                                 (const char *)&raw_packet,
                                 sizeof(InnoRaw4Packet));
    inno_log_info("raw4 send sn.");
  }

  // send diagnostic
  {
    std::stringstream ss;

    {
      std::unique_lock<std::mutex> lk(raw_cause_mutex_);
      for (std::string &str : this->raw_cause_) {
        ss << str << "-";
      }
    }

    std::string s = ss.str();
    if (InnoUtils::ends_with(s.c_str(), "-")) {
      s = s.substr(0, s.length() - 1);
    }
    raw_packet.field_type = InnoRaw4Packet::TYPE_CAUSE;
    raw_packet.is_field_last = true;
    raw_packet.buffer = s.data();
    raw_packet.buffer_size = s.size();
    lidar_->do_recorder_callback(INNO_RECORDER_CALLBACK_TYPE_RAW4,
                                 (const char *)&raw_packet,
                                 sizeof(InnoRaw4Packet));
    inno_log_info("raw4 send diagnostic: %s", ss.str().c_str());
  }

  // send yaml
  {
    std::string yaml = lidar_->get_version_yaml();

    raw_packet.field_type = InnoRaw4Packet::TYPE_RAWDATA;
    raw_packet.is_field_last = false;
    raw_packet.buffer = yaml.data();
    raw_packet.buffer_size = yaml.size();
    lidar_->do_recorder_callback(INNO_RECORDER_CALLBACK_TYPE_RAW4,
                                 (const char *)&raw_packet,
                                 sizeof(InnoRaw4Packet));
    inno_log_info("raw4 send yaml.");
  }

  // send row data
  {
    inno_log_info("sent raw4 data: %" PRI_SIZELU ".", raw4_send_deque_.size());

    while (!raw4_send_deque_.empty()) {
      StageSignalJob *p = raw4_send_deque_[0];
      raw4_send_deque_.pop_front();

      raw_packet.field_type = InnoRaw4Packet::TYPE_RAWDATA;
      raw_packet.is_field_last = raw4_send_deque_.empty();
      raw_packet.buffer = p->get_buffer_with_leftover();
      raw_packet.buffer_size = p->get_buffer_with_leftover_len_without_tail();

      lidar_->do_recorder_callback(INNO_RECORDER_CALLBACK_TYPE_RAW4,
                                   (const char *)&raw_packet,
                                   sizeof(InnoRaw4Packet));
      if (p->dec_ref()) {
        lidar_->free_signal_job(p);
      }
    }
  }

  // done.
  {
    std::unique_lock<std::mutex> lk(raw_cause_mutex_);
    this->raw_cause_.clear();
  }

  {
    std::unique_lock<std::mutex> lk(raw4_done_mutex_);
    raw4_done_ = true;
  }

  inno_log_info("sent raw4 done.");
  return 0;
}

int StageSignal::inc_job_count_() {
  galvo_slope_ = params_->iv_params.g_scan_range /
                 (InnoConsts::kGalvoEncoderMax - InnoConsts::kGalvoEncoderMin);
  galvo_min_angle_ = params_->iv_params.g_center_angle -
                     params_->iv_params.g_scan_range / 2;
  for (uint32_t i = 0; i < kInnoChannelNumber; i++) {
    if (params_->iv_params.dist_corr_transition_high !=
        params_->iv_params.dist_corr_transition_low) {
      dis_corr_slope_[i] = (params_->iv_params.distance_correction_2[i] -
                            params_->iv_params.distance_correction[i]) /
                           (params_->iv_params.dist_corr_transition_high -
                            params_->iv_params.dist_corr_transition_low);
    } else {
      dis_corr_slope_[i] = 0;
    }
  }
  if (params_->iv_params.dist_corr_transition_intensity > ref_min_intensity_) {
    ref_max_intensity_ = params_->\
                         iv_params.dist_corr_transition_intensity;
  }
  if (job_count_ == 0) {
    inno_log_trace("galvo_slope_ %f %f",
                    galvo_slope_, galvo_min_angle_);
    inno_log_info("ref intensity threshold: %" PRI_SIZED "/%" PRI_SIZED,
                   ref_min_intensity_, ref_max_intensity_);
  }

  job_count_++;
  return 0;
}

void StageSignal::print_stats(void) const {
  char buf[1024];
  get_stats_string(buf, sizeof(buf));
  inno_log_info("%s", buf);
  return;
}

void StageSignal::get_stats_string(char *buf, size_t buf_size) const {
  int ret = snprintf(buf, buf_size,
                     "StageSignal: "
                     "g_encoder_count=%" PRI_SIZELU
                     "stats_pr_encoder_count=%" PRI_SIZELU
                     "pf_encoder_count=%" PRI_SIZELU
                     "padding_count=%" PRI_SIZELU
                     "trigger_count=%" PRI_SIZELU
                     "adc_count=%" PRI_SIZELU
                     "pulse_count=%" PRI_SIZELU
                     "ref_count=%" PRI_SIZELU
                     "ref_sample_count=%" PRI_SIZELU
                     "previous_pulse_0=%" PRI_SIZELU
                     "previous_pulse_1=%" PRI_SIZELU
                     "discarded_multi_return=%" PRI_SIZELU
                     "scatter=%" PRI_SIZELU
                     "road_points=%" PRI_SIZELU
                     "retro=%" PRI_SIZELU
                     "discard_pre_retro=%" PRI_SIZELU
                     "discard_post_retro=%" PRI_SIZELU
                     "below_min_intensity=%" PRI_SIZELU
                     "force_drop_0=%" PRI_SIZELU
                     "not_enough_g_0=%" PRI_SIZELU
                     "other_encoder_count=%" PRI_SIZELU
                     "bad_ts_0=%" PRI_SIZELU
                     "bad_ts_1=%" PRI_SIZELU
                     "discard_pulse=%" PRI_SIZELU
                     "discard_trigger=%" PRI_SIZELU
                     "discard_encoder_0=%" PRI_SIZELU
                     "discard_encoder_1=%" PRI_SIZELU
                     "discard_encoder_2=%" PRI_SIZELU
                     "galvo_delay=%" PRI_SIZELU
                     "ignore_polygon_no_active=%" PRI_SIZELU
                     "ignore_polygon_small_gap=%" PRI_SIZELU
                     "road_0=%" PRI_SIZELU
                     "road_1=%" PRI_SIZELU
                     "polygon_mean/std/min/max/delta_max/cnt="
                     "%.1f/%.1f/%.1f/%.1f/%.1f/%" PRI_SIZEU
                     "polygon_real_mean/std/min/max/delta_max/cnt="
                     "%.1f/%.1f/%.1f/%.1f/%.1f/%" PRI_SIZEU
                     "galvo_0_mean=%.1f/%.1f/%.1f/%.1f/%" PRI_SIZEU
                     "galvo_1_mean=%.1f/%.1f/%.1f/%.1f/%" PRI_SIZED
                     ,
                     stats_g_encoder_count_,
                     stats_pr_encoder_count_,
                     stats_pf_encoder_count_,
                     stats_padding_count_,
                     stats_trigger_count_,
                     stats_adc_count_,
                     stats_pulse_count_,
                     stats_ref_count_,
                     stats_ref_track_count_,
                     stats_previous_pulse_0_,
                     stats_previous_pulse_1_,
                     stats_discarded_multi_return_,
                     stats_scatter_,
                     stats_road_points_,
                     stats_retro_,
                     stats_discard_pre_retro_,
                     stats_discard_post_retro_,
                     stats_below_min_intensity_,
                     stats_force_drop_0_,
                     stats_not_enough_g_0_,
                     stats_other_encoder_count_,
                     stats_bad_ts_0_,
                     stats_bad_ts_1_,
                     stats_discard_pulse_,
                     stats_discard_trigger_,
                     stats_discard_encoder_0_,
                     stats_discard_encoder_1_,
                     stats_discard_encoder_2_,
                     stats_galvo_delay_,
                     stats_ignore_polygon_no_active_,
                     stats_ignore_polygon_small_gap_,
                     stats_road_0_,
                     stats_road_1_,
                     polygon_mean_.mean()/1000,
                     polygon_mean_.std_dev()/1000,
                     polygon_mean_.min()/1000,
                     polygon_mean_.max()/1000,
                     polygon_mean_.max_delta()/1000,
                     polygon_mean_.count(),
                     polygon_real_mean_.mean()/1000,
                     polygon_real_mean_.std_dev()/1000,
                     polygon_real_mean_.min()/1000,
                     polygon_real_mean_.max()/1000,
                     polygon_real_mean_.max_delta()/1000,
                     polygon_real_mean_.count(),
                     galvo_mean_[0].mean()/1000,
                     galvo_mean_[0].std_dev()/1000,
                     galvo_mean_[0].min()/1000,
                     galvo_mean_[0].max()/1000,
                     galvo_mean_[0].count(),
                     galvo_mean_[1].mean()/1000,
                     galvo_mean_[1].std_dev()/1000,
                     galvo_mean_[1].min()/1000,
                     galvo_mean_[1].max()/1000,
                     galvo_mean_[1].count());
  if (ret >= ssize_t(buf_size)) {
    buf[buf_size - 1] = 0;
    return;
  }
}

const char *StageSignal::get_name_() const {
  return lidar_->get_name();
}

void StageSignal::rate_control_(InnoTimestampUs last_data_us) {
  int64_t should_elapsed = 0;
  if (play_rate_x_ > 0) {
    if (first_data_us_) {
      if (last_data_us > first_data_us_) {
        should_elapsed = (last_data_us - first_data_us_) / play_rate_x_;
      } else {
        return;
      }
    } else {
      first_data_us_ = last_data_us;
      start_time_us_ = lidar_->get_monotonic_raw_time_us();
      return;
    }
  } else {
    return;
  }
  InnoTimestampUs now_us = lidar_->get_monotonic_raw_time_us();
  int64_t elapsed_us = now_us - start_time_us_;
  if (should_elapsed > elapsed_us && elapsed_us >= 0) {
    usleep(should_elapsed - elapsed_us);
  }
  return;
}

void StageSignal::do_optical_check_(size_t trigger_count) {
  optical_path_check_->inc_frame_count();
  uint64_t frame_counts = optical_path_check_->get_frame_count();
  if (frame_counts >= ignore_begin_frames_blockage_) {
    optical_path_check_->do_optical_and_blockage_diagnosis(trigger_count);
  }
  optical_path_check_->prepare_optical_and_blockage_check();
  if (frame_counts >= ignore_begin_frames_ref_intensity_) {
    optical_path_check_->do_ref_intensity_check();
  }
  optical_path_check_->prepare_ref_intensity_check_for_next_round();
}

void StageSignal::config_chA_ref_sample_rate_() {
  if (lidar_->is_live_lidar()) {
    uint32_t value = 0;
    int ret = lidar_->read_pl_reg(CHA_REFERENCE_SAMPLE_RATE_REG, &value);
    if (ret != 0) {
      inno_log_error("read CHA_REFERENCE_SAMPLE_RATE_REG error, "
                     "chA default sample rate: 1/%"
                     PRI_SIZELU, kRefFpgaSampleRate);
    } else {
      int kChARefSampleIndicator = 7;
      if (((value >> kChARefSampleIndicator) & 0x1) != 0) {
        // all reference pulse will send to ARM, set to 1/16
        value &= ~(1U << kChARefSampleIndicator);
        int r = lidar_->write_pl_reg(CHA_REFERENCE_SAMPLE_RATE_REG, value);
        if (r != 0) {
          ref_sample_rate_[0] *= kRefFpgaSampleRate;
          inno_log_error("CHA reference sample rate: 1/%" PRI_SIZELU ", "
                         "ref_sample_rate_: %" PRI_SIZELU
                          "/%" PRI_SIZELU "/%" PRI_SIZELU "/%" PRI_SIZELU "/",
                          kRefRealSampleRate / kRefFpgaSampleRate,
                          ref_sample_rate_[0],
                          ref_sample_rate_[1],
                          ref_sample_rate_[2],
                          ref_sample_rate_[3]);
          return;
        }
      }
      inno_log_info("CHA reference sample rate: 1/%"
                     PRI_SIZELU ", reg value: %u",
                     kRefFpgaSampleRate, value);
    }
  } else {
    inno_log_info("file play mode, chA ref sample rate: 1/%" PRI_SIZELU,
                   kRefRealSampleRate / kRefFpgaSampleRate);
  }
}

void StageSignal::find_reference_(size_t trigger_count) {
  // skip first unreliable frame
  if (stats_frame_counts_ <= 1) {
    for (uint32_t i = 0; i < kInnoChannelNumber; i++) {
      ref_finder_[i].reset_stats();
    }
    return;
  }
  bool need_print = false;
  bool routine_print = false;
  bool inited = stats_frame_counts_ >= kMaxFreqUpdateFrames;
  std::string ref_find_info("Ref_finder: ");
  for (uint32_t i = 0; i < kInnoChannelNumber; i++) {
    ref_finder_[i].add_trigger_count(trigger_count);
    ref_find_info += "channel: " + std::to_string(i);
    int64_t new_ref_time_sub_ns = -1;
    int sample_interval = ref_finder_[i].get_sample_interval();
    ref_find_info += " sample interval: " +
                       std::to_string(sample_interval) + " {";
    if (stats_frame_counts_ % sample_interval == 0) {
      routine_print = true;
      stats_total_ref_check_count_[i]++;
      int equiv_trigger_count = ref_finder_[i].get_trigger_count() /
                                (ref_sample_rate_[i] == 1 ?
                                kRefFpgaSampleRate : 1);
      int min_sample_index = 0;
      if (inited) {
        min_sample_index = ref_finder_[i].get_min_sample_index();
      }
      int ret = ref_finder_[i].find_ref_center(equiv_trigger_count,
                                              &new_ref_time_sub_ns,
                                              &ref_find_info);
      ref_finder_[i].reset_stats();
      // phase 1: update q
      if (ret != 0) {
        if (inited) {
          ref_finder_[i].pop();  // need pop?
        }
        ref_finder_[i].dec_sample_interval(min_sample_index);
        need_print = true;
      } else {
        inno_log_verify(new_ref_time_sub_ns > 0,
                       "channel: %u, new_ref_time_sub_ns(%"
                       PRI_SIZED ") invalid",
                        i, new_ref_time_sub_ns);
        if (ref_finder_[i].q_is_empty()) {
            ref_finder_[i].inc_sample_interval(min_sample_index);
            ref_finder_[i].get_stastic_info().add(new_ref_time_sub_ns);
        } else {
          double mean = ref_finder_[i].get_stastic_info().mean();
          double std_dev = ref_finder_[i].get_stastic_info().std_dev();
          double three_sigma = 3 * std_dev;  // 3 sigma
          if (three_sigma < config_.ref_window_half_width_max_by_histo) {
            three_sigma = config_.ref_window_half_width_max_by_histo;
          }
          if (new_ref_time_sub_ns <= mean - three_sigma) {
            // new reference appears, enable rapid q update
            ref_finder_[i].dec_sample_interval(min_sample_index, 10);
            ref_finder_[i].clear();  // bias strategy, shall we clear the q?
            ref_finder_[i].get_stastic_info().add(new_ref_time_sub_ns);
            inno_log_warning("new ref candicate(%" PRI_SIZED
                             ") comes for channel %u, "
                             "currenr mean: %0.2f, "
                             "std_dev: %0.2f, 3*sigma: %0.2f",
                              new_ref_time_sub_ns, i, mean,
                              std_dev, three_sigma);
          } else if (new_ref_time_sub_ns > mean + three_sigma) {
            ref_finder_[i].dec_sample_interval(min_sample_index);
            if (inited) {
              ref_finder_[i].pop();
            }
          } else {
            ref_finder_[i].inc_sample_interval(min_sample_index);
            ref_finder_[i].get_stastic_info().add(new_ref_time_sub_ns);
          }
        }
      }
      // phase 2: reference time regulation
      if (ref_finder_[i].q_is_empty()) {
        ref_find_info += "[queue is empty, will not check reference time] ";
        ref_find_info += "} ";
        need_print = true;
        // we can not regulate the reference time for current channel,
        // it should keep the old value
        continue;
      }
      double mean = ref_finder_[i].get_stastic_info().mean();
      double std_dev = ref_finder_[i].get_stastic_info().std_dev();
      double three_sigma = 3 * std_dev;  // 3 sigma
      if (three_sigma < config_.ref_window_half_width_max_by_histo) {
        three_sigma = config_.ref_window_half_width_max_by_histo;
      }
      double current_ref_time = ref_tracking_result_[i].\
                                get_virtual_ref_offset();
      // when current ref is too big, correct it immediately
      if (current_ref_time > mean + three_sigma) {  // or use raw 3 * sigma?
        stats_ref_unfit_count_[i]++;
        inno_log_warning("ref time(%0.2f) of channel[%u] is out of range, "
                         "will be regulated by histogram info, mean: %0.2f, "
                         "std_dev: %0.2f, 3 sigma: %0.2f",
                          current_ref_time, i,
                          mean, std_dev,
                          three_sigma);
        // correct the reference
        for (uint32_t j = 0; j < kRoiTypeCount; j++) {
          RefTracking &tracking = ref_tracking_[j][i];
          tracking.set_ref_center(mean);
          tracking.set_bound(config_.ref_window_half_width_max_by_histo);
          tracking.reset_stats(trigger_count_[j]);
        }
        ref_tracking_result_[i].update(mean, 1);
        if (stats_ref_unfit_count_[i] == 1) {
          int64_t ref_center_current = ref_tracking_result_[i].\
                                       get_virtual_ref_offset();
          int64_t ref_center_non_roi = ref_tracking_[0][i].\
                                       get_current_virtual_ref_offset_sub_ns();
          int64_t ref_center_roi = ref_tracking_[1][i].\
                                   get_current_virtual_ref_offset_sub_ns();
          inno_log_info("channel: %u, ref center: %"
                        PRI_SIZED "/%" PRI_SIZED "/%" PRI_SIZED ", "
                        "update bound to %" PRI_SIZED,
                         i, ref_center_non_roi,
                         ref_center_roi,
                         ref_center_current,
                         config_.ref_window_half_width_max_by_histo);
        }
        need_print = true;
      } else {  // use the sliding average
        stats_ref_fit_count_[i]++;
        if (current_ref_time < mean - three_sigma) {
          inno_log_with_level(inited ? INNO_LOG_LEVEL_ERROR :
                              INNO_LOG_LEVEL_INFO,
                              "channel(%u): current ref time(%0.2f) is smaller "
                              "than 3 sigma(%0.2f), mean: %0.2f",
                               i, current_ref_time,
                               mean - three_sigma, mean);
        }
        // only updates the bound in normal case
        for (uint32_t j = 0; j < kRoiTypeCount; j++) {
          RefTracking &tracking = ref_tracking_[j][i];
          tracking.set_bound(config_.ref_window_half_width_max_by_histo,
                             trigger_count_[j]);
        }
        if (stats_ref_fit_count_[i] == 1) {
          for (uint32_t j = 0; j < kRoiTypeCount; j++) {
            RefTracking &tracking = ref_tracking_[j][i];
            tracking.set_ref_center(mean);
            tracking.set_window_l(config_.\
                                  ref_window_half_width_max_by_histo / 2);
            tracking.set_window_r(config_.\
                                  ref_window_half_width_max_by_histo / 2);
            tracking.set_bound(config_.ref_window_half_width_max_by_histo,
                               trigger_count_[j]);
            tracking.reset_stats(trigger_count_[j]);
          }
          ref_finder_[i].set_inited();
          ref_tracking_result_[i].update(mean, 1);
          int64_t ref_center_current = ref_tracking_result_[i].\
                                       get_virtual_ref_offset();
          int64_t ref_center_non_roi = ref_tracking_[0][i].\
                                       get_current_virtual_ref_offset_sub_ns();
          int64_t ref_center_roi = ref_tracking_[1][i].\
                                   get_current_virtual_ref_offset_sub_ns();
          inno_log_info("channel: %u, ref center: %" PRI_SIZED
                        "/%" PRI_SIZED "/%" PRI_SIZED ", "
                        "update bound to %" PRI_SIZED ", "
                        "update search window to (%u/%u), (%u/%u)",
                         i, ref_center_non_roi,
                         ref_center_roi,
                         ref_center_current,
                         config_.ref_window_half_width_max_by_histo,
                         ref_tracking_[0][i].get_search_window_l(),
                         ref_tracking_[0][i].get_search_window_r(),
                         ref_tracking_[1][i].get_search_window_l(),
                         ref_tracking_[1][i].get_search_window_r());
        }
      }
    } else {
      // sampling, do nothing
    }
    ref_find_info += "} ";
  }
  if (need_print ||
     (routine_print &&
     (stats_frame_counts_ < 16 ||
      stats_frame_counts_ % 128 == 0))) {
    inno_log_with_level(need_print &&
                        stats_frame_counts_ > kIgnoreBeginFrameRefIntensity ?
                        INNO_LOG_LEVEL_ERROR :
                        INNO_LOG_LEVEL_INFO,
                        "reference stats: "
                        "total frames: %" PRI_SIZELD ", "
                        "sample interval: %d/%d/%d/%d, "
                        "stats_total_ref_check_count=%" PRI_SIZELD
                        "/%" PRI_SIZELD "/%" PRI_SIZELD "/%" PRI_SIZELD ", "
                        "stats_ref_fit_count=%" PRI_SIZELD "/%"
                        PRI_SIZELD "/%" PRI_SIZELD "/%" PRI_SIZELD ", "
                        "stats_ref_unfit_count=%" PRI_SIZELD "/%"
                        PRI_SIZELD "/%" PRI_SIZELD "/%" PRI_SIZELD ", "
                        "stats_q_size=%d/%d/%d/%d/, "
                        "mean=%0.2f/%0.2f/%0.2f/%0.2f, "
                        "std_deviation=%0.2f/%0.2f/%0.2f/%0.2f, "
                        "ref_tracking=(%" PRI_SIZED
                        "/%" PRI_SIZED "/%" PRI_SIZED
                        "), (%" PRI_SIZED "/%" PRI_SIZED
                        "/%" PRI_SIZED "), "
                        "(%" PRI_SIZED "/%" PRI_SIZED "/%" PRI_SIZED
                        ") (%" PRI_SIZED "/%" PRI_SIZED "/%" PRI_SIZED "), "
                        "search window=[(%u/%u)(%u/%u)], [(%u/%u)(%u/%u)], "
                        "[(%u/%u)(%u/%u)], [(%u/%u)(%u/%u)], %s",
                        stats_frame_counts_,
                        ref_finder_[0].get_sample_interval(),
                        ref_finder_[1].get_sample_interval(),
                        ref_finder_[2].get_sample_interval(),
                        ref_finder_[3].get_sample_interval(),
                        stats_total_ref_check_count_[0],
                        stats_total_ref_check_count_[1],
                        stats_total_ref_check_count_[2],
                        stats_total_ref_check_count_[3],
                        stats_ref_fit_count_[0],
                        stats_ref_fit_count_[1],
                        stats_ref_fit_count_[2],
                        stats_ref_fit_count_[3],
                        stats_ref_unfit_count_[0],
                        stats_ref_unfit_count_[1],
                        stats_ref_unfit_count_[2],
                        stats_ref_unfit_count_[3],
                        ref_finder_[0].get_stastic_info().size(),
                        ref_finder_[1].get_stastic_info().size(),
                        ref_finder_[2].get_stastic_info().size(),
                        ref_finder_[3].get_stastic_info().size(),
                        ref_finder_[0].get_stastic_info().mean(),
                        ref_finder_[1].get_stastic_info().mean(),
                        ref_finder_[2].get_stastic_info().mean(),
                        ref_finder_[3].get_stastic_info().mean(),
                        ref_finder_[0].get_stastic_info().std_dev(),
                        ref_finder_[1].get_stastic_info().std_dev(),
                        ref_finder_[2].get_stastic_info().std_dev(),
                        ref_finder_[3].get_stastic_info().std_dev(),
                        ref_tracking_[0][0].\
                        get_current_virtual_ref_offset_sub_ns(),
                        ref_tracking_[1][0].\
                        get_current_virtual_ref_offset_sub_ns(),
                        ref_tracking_result_[0].get_virtual_ref_offset(),
                        ref_tracking_[0][1].\
                        get_current_virtual_ref_offset_sub_ns(),
                        ref_tracking_[1][1].\
                        get_current_virtual_ref_offset_sub_ns(),
                        ref_tracking_result_[1].get_virtual_ref_offset(),
                        ref_tracking_[0][2].\
                        get_current_virtual_ref_offset_sub_ns(),
                        ref_tracking_[1][2].\
                        get_current_virtual_ref_offset_sub_ns(),
                        ref_tracking_result_[2].get_virtual_ref_offset(),
                        ref_tracking_[0][3].\
                        get_current_virtual_ref_offset_sub_ns(),
                        ref_tracking_[1][3].\
                        get_current_virtual_ref_offset_sub_ns(),
                        ref_tracking_result_[3].get_virtual_ref_offset(),
                        ref_tracking_[0][0].get_search_window_l(),
                        ref_tracking_[0][0].get_search_window_r(),
                        ref_tracking_[1][0].get_search_window_l(),
                        ref_tracking_[1][0].get_search_window_r(),
                        ref_tracking_[0][1].get_search_window_l(),
                        ref_tracking_[0][1].get_search_window_r(),
                        ref_tracking_[1][1].get_search_window_l(),
                        ref_tracking_[1][1].get_search_window_r(),
                        ref_tracking_[0][2].get_search_window_l(),
                        ref_tracking_[0][2].get_search_window_r(),
                        ref_tracking_[1][2].get_search_window_l(),
                        ref_tracking_[1][2].get_search_window_r(),
                        ref_tracking_[0][3].get_search_window_l(),
                        ref_tracking_[0][3].get_search_window_r(),
                        ref_tracking_[1][3].get_search_window_l(),
                        ref_tracking_[1][3].get_search_window_r(),
                        ref_find_info.c_str());
  }
}

void StageSignal::update_ref_info_(enum InnoLidarMode current_mode,
                                   enum InnoLidarMode pre_mode) {
  std::string str;
  bool need_print = false;
  for (uint32_t i = 0; i < kInnoChannelNumber; i++) {
    if (!ref_finder_[i].inited()) {
      continue;
    }
    need_print = true;
    InnoFpgaSubNs ref_center = ref_tracking_result_[i].\
                               get_virtual_ref_offset();
    str += "channel ";
    str += std::to_string(i);
    str += ": (";
    for (uint32_t j = 0; j < kRoiTypeCount; j++) {
      RefTracking &tracking = ref_tracking_[j][i];
      InnoFpgaSubNs old_center = tracking.set_ref_center(ref_center);
      str += std::to_string(old_center);
      str += "/";
      tracking.set_window_l(config_.\
                            ref_window_half_width_max_by_histo / 2);
      tracking.set_window_r(config_.\
                            ref_window_half_width_max_by_histo / 2);
      tracking.set_bound(config_.ref_window_half_width_max_by_histo,
                         trigger_count_[j]);
      tracking.reset_stats(trigger_count_[j]);
    }
    str += "), ";
  }
  if (need_print) {
    inno_log_with_level_no_discard(INNO_LOG_LEVEL_INFO,
                                  "reinit ref center to %" PRI_SIZED
                                  "/%" PRI_SIZED "/%" PRI_SIZED
                                  "/%" PRI_SIZED "from %s"
                                  "because of mode change from %d to %d",
                                  ref_tracking_result_[0].\
                                  get_virtual_ref_offset(),
                                  ref_tracking_result_[1].\
                                  get_virtual_ref_offset(),
                                  ref_tracking_result_[2].\
                                  get_virtual_ref_offset(),
                                  ref_tracking_result_[3].\
                                  get_virtual_ref_offset(),
                                  str.c_str(),
                                  pre_mode,
                                  current_mode);
  }
}

void StageSignal::expand_ref_window_() {
  static const size_t ref_check_period =
                      config_.ref_sample_num * kRefFpgaSampleRate /
                      RefTracking::get_ref_check_times_per_update();
  for (uint32_t i = 0; i < kRoiTypeCount; i++) {
    if (trigger_count_[i] - trigger_count_old_[i] >= ref_check_period) {
      trigger_count_old_[i] = trigger_count_[i];
      for (uint32_t ch = 0; ch < kInnoChannelNumber; ch++) {
        if (!ref_finder_[ch].inited()) {
          continue;
        }
        ref_tracking_[i][ch].dec_window_expand_flags();
        if (ref_tracking_[i][ch].need_expand_window_external()) {
          ref_tracking_[i][ch].expand_window(2);
          ref_tracking_[i][ch].reset_window_expand_flags();
          ref_tracking_[i][ch].set_bound();
        }
      }
    }
  }
}

StageSignal::RefIntCheckHelper::RefIntCheckHelper() {
  memset(this, 0, sizeof(*this));
  build_temperature_correct_table_();
  inno_log_info("ready for reference intensity check");
}

StageSignal::RefIntCheckHelper::~RefIntCheckHelper() {
  // do nothing
}

void StageSignal::RefIntCheckHelper::build_temperature_correct_table_() {
  // note: keep tf_pairs ascending order
  struct TempFactorPair tf_pairs[kInitTableSize] = {
    {60,  0.50},
    {70,  0.40},
    {80,  0.50},
    {90,  0.25},
    {100, 0.20},
    {115, 0.20}
  };
  int left = 0;
  int right = 0;
  for (int i = 0; i < kCorrectEndTemp - kCorrectBeginTemp; i++) {
    int current_index = i + kCorrectBeginTemp;
    if (current_index >= tf_pairs[right].temp) {
      int begin = 0;
      int end = kInitTableSize - 1;
      // find the 1st neighbor which is <= current_index
      while (begin <= end) {
        int mid = (begin + end) / 2;
        if (current_index > tf_pairs[mid].temp) {
          begin = mid + 1;
        } else if (current_index < tf_pairs[mid].temp) {
          end = mid - 1;
        } else {
          begin = mid + 1;
        }
      }
      right = end + 1;

      // find the 1st neighbor which is >= current_index
      begin = 0;
      end = kInitTableSize - 1;
      while (begin <= end) {
        int mid = (begin + end) / 2;
        if (current_index > tf_pairs[mid].temp) {
          begin = mid + 1;
        } else if (current_index < tf_pairs[mid].temp) {
          end = mid - 1;
        } else {
          end = mid - 1;
        }
      }
      left = begin;
    }
    inno_log_verify(left >= 0 && left < right && right < kInitTableSize,
                   "invalid index for interplot: %d/%d", left, right);
    temp_correct_table_[i] = tf_pairs[left].factor +
                            (tf_pairs[right].factor - tf_pairs[left].factor) *
                            (current_index - tf_pairs[left].temp) /
                            (tf_pairs[right].temp - tf_pairs[left].temp);
    inno_log_trace("temperature correection table: %d/%d/%d corr: %0.5f",
                    i, left, right, temp_correct_table_[i]);
  }
}

int StageSignal::RefIntCheckHelper::\
    get_diag_threshold(std::pair<int, double>* thresholds) {
  if (thresholds->first >= kCorrectEndTemp) {
    thresholds->second = 0.0;
    return -1;
  }
  if (thresholds->first < kCorrectBeginTemp) {
    thresholds->second = 1.0;
  } else {
    thresholds->second = temp_correct_table_\
                         [thresholds->first - kCorrectBeginTemp];
  }
  return 0;
}

}  // namespace innovusion
