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
#include "sdk/system_stats.h"

namespace innovusion {

inline InnoFpgaSubNs StageSignal::process_adc_data_clock_(
    const AdcComputedDataExt &adc) {
  int64_t time_stamp =
      ((int64_t)adc.time_stamp_0) |
      ((int64_t)adc.time_stamp_1 << 5);
  return fpga_clock_.receive_3rd((time_stamp << StageSignalClock::kShiftBits) +
                                 (int64_t)adc.time_shift,
                                 (int64_t)adc.offs_frac);
}

inline InnoFpgaNs StageSignal::process_trigger_data_clock_(
    const TriggerData &trigger) {
  int64_t time_stamp =
      ((int64_t)trigger.time_stamp_0) |
      ((int64_t)trigger.time_stamp_1 << 4) |
      ((int64_t)trigger.time_stamp_2 << 12);

  return fpga_clock_.receive_2nd(time_stamp << StageSignalClock::kShiftBits);
}

inline InnoFpgaNs StageSignal::process_encoder_data_clock_(
    const EncoderData &encoder) {
  if (encoder.g_tstamp_type == RAW_DATA_TIME_TYPE_HW) {
    int64_t time_stamp =
        ((int64_t)encoder.g_tstamp_0) |
        ((int64_t)encoder.g_tstamp_1 << 4) |
        ((int64_t)encoder.g_tstamp_2 << 12) |
        ((int64_t)encoder.g_tstamp_3 << 20) |
        ((int64_t)encoder.g_tstamp_4 << 28) |
        ((int64_t)encoder.g_tstamp_5 << 36) |
        ((int64_t)encoder.g_tstamp_6 << 44) |
        ((int64_t)encoder.g_tstamp_7 << 52);
    return fpga_clock_.receive_1st(time_stamp << StageSignalClock::kShiftBits,
                                   (encoder.v_galvo_enc << 2) |
                                   (encoder.vf_enc_I << 1));
  } else if (encoder.g_tstamp_type == RAW_DATA_TIME_TYPE_NO) {
    stats_bad_ts_0_++;
    inno_log_warning("suspicious tstamp %hd, clock issue? %" PRI_SIZELU,
                     encoder.g_tstamp_type, stats_bad_ts_0_);
    return InnoConsts::kInvalidInnoFpgaNs;
  } else {
    stats_bad_ts_1_++;
    inno_log_warning("suspicious tstamp %hd, clock issue? %" PRI_SIZELU,
                     encoder.g_tstamp_type, stats_bad_ts_1_);
    return InnoConsts::kInvalidInnoFpgaNs;
  }
}

inline void StageSignal::process_pulse_to_point_(
    const AdcComputedDataExt &adc,
    int32_t distance_0,
    int32_t distance_1,
    int32_t intensity_seq,
    int32_t intensity,
    int32_t point_type,
    RawChannelPoint *p) {
  p->set_intensity_seq(intensity_seq);
  p->set_raw_intensity(intensity);
  p->set_radius(0, distance_0);
  p->set_radius(1, distance_1);
  p->set_is_real(0, set_first_firing_cycle_real_);
  // p->refl
  // p->is_saturated
  // p->is_retro  xxx todo
  p->set_type(point_type);
  p->set_elongation(adc.pulse_width);
  ++stats_pulse_count_;
  return;
}

inline void StageSignal::process_adc_data_pulse_(
    const AdcComputedDataExt &adc,
    const InnoFpgaSubNs pulse_sub_ns,
    int32_t channel,
    int32_t intensity_seq,
    int32_t intensity,
    int32_t point_type,
    RawBlock *block) {
  // discard this pulse if retro is seen before
  if (config_.kill_retro_noise && block->has_retro & (1 << channel)) {
    stats_discard_post_retro_++;
    return;
  }

  const IvParams *params = &params_->iv_params;

  InnoSubNsOffset pulse_offset =
    pulse_sub_ns - block->ref_ts_sub_ns_off[channel];

  pulse_offset -= misc_tables_->get_delay_correction(channel, intensity_seq);

  // xxx todo: distance correction in yaml file is a double
  int32_t distance_0 = sub_ns_to_inno_distance_unit_(pulse_offset);
  if (intensity < params->dist_corr_transition_low) {
    distance_0 -= params->distance_correction[channel];
  } else if (intensity >= params->dist_corr_transition_high) {
    distance_0 -= params->distance_correction_2[channel];
  } else {
    distance_0 -= params->distance_correction[channel] +
      (intensity - params->dist_corr_transition_low) * dis_corr_slope_[channel];
  }

  // distance_0 += 10 * 200;

  if (config_.pulse_pick == 0) {
    if (distance_0 < config_.exclude_distance) {
#ifndef GALVO_TRACKING_DIRECT_SCALING
      block->scatter_intensity[channel] = intensity_seq;
#else
      block->scatter_intensity[channel] = intensity;
#endif
      stats_scatter_++;
      optical_path_check_->add_scatter_sample(channel);
      return;
    } else {
      optical_path_check_->add_return_sample(channel);
    }
  } else if (distance_0 < config_.min_distance ||
             distance_0 > config_.max_distance) {
    return;
  }
  if (cur_lines_) {
    cur_lines_->add_frame_points(channel, up_scan_);
  }
  InnoFpgaSubNs off = InnoConverts::ns_to_sub_ns(block->trigger_period);
  int32_t distance_1 = distance_0 + sub_ns_to_inno_distance_unit_(off);

  // road point check
  if (point_type == 1) {
    // here we only consider vertical ROI, if we want to consider
    // horizontal ROI too, then need to check it in delivery stage
    if (!in_roi_ || distance_0 < config_.sw_start ||
        distance_0 > config_.sw_end) {
      stats_road_0_++;
      return;
    }
    // don't consider road points in the 2nd firing cycle
    distance_1 = 0;
    stats_road_1_++;
  }

  // remove pre retro points
  if (intensity > params->retro_intensity) {
    stats_retro_++;
    block->has_retro |= 1 << channel;
    for (uint32_t i = 0; i < kInnoMaxMultiReturn; i++) {
      RawChannelPoint &p = block->points[channel][i];
      if (p.get_raw_intensity() == 0) {
        break;
      }
      if (config_.kill_retro_noise &&
          p.get_raw_intensity() < config_.pre_retro_min_intensity) {
        p.set_raw_intensity(0);
        p.set_radius(0, 0);
        p.set_radius(1, 0);
        stats_discard_pre_retro_++;
      }
    }

    // re-arrange the data after removing pre retro points
    uint32_t j = 0;
    for (uint32_t i = 0; i < kInnoMaxMultiReturn; i++) {
      RawChannelPoint &p = block->points[channel][i];
      if (p.get_raw_intensity() > 0) {
        if (i > j) {
          block->points[channel][j] = p;
          p.set_raw_intensity(0);
          p.set_radius(0, 0);
          p.set_radius(1, 0);
        }
        j++;
      }
    }
  }

  RawChannelPoint *p = &(block->points[channel][0]);
  for (uint32_t i = 0; i < kInnoMaxMultiReturn; i++, p++) {
    if (p->get_raw_intensity() == 0) {
      // found an empty slot
      process_pulse_to_point_(adc, distance_0, distance_1, intensity_seq,
                              intensity, point_type, p);
      return;
    }

    if (p->get_raw_intensity() >= (uint32_t)intensity) {
      continue;
    } else {
      // to keep the descending order, we need to move points
      for (uint32_t j = kInnoMaxMultiReturn - 1; j > i; j--) {
//        block->points[channel][j] = block->points[channel][j - 1];
        memcpy(&block->points[channel][j], &block->points[channel][j - 1],
                sizeof(RawChannelPoint));
      }
      process_pulse_to_point_(adc, distance_0, distance_1, intensity_seq,
                              intensity, point_type, p);
      return;
    }
  }
  if (lidar_->multiple_return_\
         == INNO_MULTIPLE_RETURN_MODE_2_STRONGEST_FURTHEST) {
    // IN INNO_MULTIPLE_RETURN_MODE_2_STRONGEST_FURTHEST MODE
    // intensity is very low, but it is the furthest point so far
    RawChannelPoint &p = block->points[channel][kInnoMaxMultiReturn-1];
    process_pulse_to_point_(adc, distance_0, distance_1, intensity_seq,
                            intensity, point_type, &p);
    return;
  }

  stats_discarded_multi_return_++;
  return;
}

void StageSignal::process_adc_data_(const AdcComputedDataExt &adc) {
  if (fpga_clock_.bad_state()) {
    return;
  }

  InnoFpgaSubNs fpga_sub_ns = process_adc_data_clock_(adc);
  InnoFpgaSubNs time_to_trigger = fpga_sub_ns - current_trigger_sub_ns_;

  int32_t point_type = 0;
  int32_t intensity;
  // the sequence number for this intensity among all possible intensity values
  int32_t intensity_seq;
  int32_t intensity_shift = (adc.intensity_shift_2 << 2) |
                             adc.intensity_shift_1;
  int intensity_base = (adc.intensity_base_2 << 6) |
                        adc.intensity_base_1;
  if (intensity_shift != 0xf) {
    intensity = intensity_base << (intensity_shift + 1);
    intensity_seq = intensity_shift * 64 + intensity_base;
    if (intensity_seq >= MiscTables::kIntensityTableSize) {
      intensity_seq = MiscTables::kIntensityTableSize - 1;
    }
  } else {
    // road point
    point_type = 1;
    intensity = intensity_base << 1;
    intensity_seq = intensity_base;
    if (config_.sw_road_mode == 0) {
      return;
    } else if (config_.sw_road_mode == 2) {
      // for highlighting road points purpose only,
      // may affect distance correction and crosstalk
      intensity += 40000;
    }
    stats_road_points_++;
  }

  if (intensity < config_.min_intensity) {
    stats_below_min_intensity_++;
    return;
  }

  stats_adc_count_++;

  if (time_to_trigger > kMaxToTriggerSubNs) {
    // miss trigger, discard
    stats_discard_pulse_++;
    return;
  }

  int32_t channel = adc.channel;

  // xxx todo: handle galvo delay!!!
  RefTracking &tracking = ref_tracking_[in_roi_][channel];
  if (tracking.is_after_window(time_to_trigger)) {
    // PULSE (likely), use virtual reference
    if (current_raw_block_) {
      process_adc_data_pulse_(adc,
                              time_to_trigger,
                              channel, intensity_seq,
                              intensity, point_type,
                              current_raw_block_);
    }
  } else {
    bool maybe_ref = intensity < ref_max_intensity_ &&
                     intensity > ref_min_intensity_;
      // use mutiply to avoid a if branch
      ref_finder_[channel].add_sample(time_to_trigger * maybe_ref);
    if (!tracking.is_before_window(time_to_trigger)) {
      if (maybe_ref && !tracking.has_ref()) {
        // REF (likely)
        tracking.set_ref(true);
        if (current_raw_block_) {
          ScanLines::set_ref_intensity(current_raw_block_,
                                       channel,
                                       intensity);
        }
        stats_ref_count_++;
        optical_path_check_->add_reference_sample(channel);
        optical_path_check_->add_reference_intensity(channel, intensity);
        stats_ref_intensity_sum_ += intensity;
        // remove this if branch if sample rate of all channels are the same
        if (tracking.need_add_sample(channel, ref_sample_rate_[channel])) {
          stats_ref_track_count_++;
          tracking.add_sample(time_to_trigger,
                              trigger_count_[in_roi_],
                              &ref_tracking_result_[channel]);
        }
        // finish handling ref
        return;
      } else {
        if (intensity > config_.ref_scatter_threshold) {
          // scatter (unlikely)
          process_adc_data_pulse_(adc,
                                  time_to_trigger,
                                  channel, intensity_seq,
                                  intensity, point_type,
                                  current_raw_block_);
          tracking.add_scatter();
          return;
        } else {
          // PRE-PULSE (unlikely) belongs to previous trigger
          stats_previous_pulse_1_++;
          if (previous_raw_block_) {
            process_adc_data_pulse_(adc,
                                    fpga_sub_ns - previous_trigger_sub_ns_,
                                    channel, intensity_seq,
                                    intensity, point_type,
                                    previous_raw_block_);
          }
        }
      }
      return;
    } else {
      // PRE-PULSE (unlikely)
      // tracking.is_before_window(time_to_trigger)
      stats_previous_pulse_0_++;
      if (previous_raw_block_) {
        process_adc_data_pulse_(adc,
                                fpga_sub_ns - previous_trigger_sub_ns_,
                                channel, intensity_seq,
                                intensity, point_type,
                                previous_raw_block_);
      }
    }
  }
}

void StageSignal::process_trigger_data_(const TriggerData &trigger) {
  if (fpga_clock_.bad_state()) {
    return;
  }

  InnoFpgaNs fpga_ns = process_trigger_data_clock_(trigger);

  if (trigger.sync_timestamp_only) {
    // inno_log_trace("fake trigger");
    return;
  } else {
    // inno_log_trace("real trigger");
  }

  RefTracking *tracking = &ref_tracking_[in_roi_][0];
  for (uint32_t i = 0; i < kInnoChannelNumber; i++, tracking++) {
    has_ref_old_[i] = tracking->has_ref();
    tracking->set_ref(false);
  }

  stats_trigger_count_++;
  trigger_count_[in_roi_]++;

  // if has a new cur_lines_:
  //    get a block from begining of cur_lines_
  //    if pre_lines_ has enought galvo encoder, send it to next stage
  // xxx todo: boundary check to avoid no encoder signal

  previous_raw_block_ = current_raw_block_;

  if (current_block_source_lines_ != cur_lines_) {
    // xxx todo: this will effectively increase our blindspot,
    // need to do it more gracefully
    previous_raw_block_ = NULL;
    current_raw_block_ = cur_lines_->allocate_first_block();
    set_first_firing_cycle_real_ = cur_lines_->in_calibration_mode() ||
                                   config_.raw_output == 1;

    inno_log_verify(current_block_source_lines_ == pre_lines_,
                    "invalid lines_ %p vs %p",
                    current_block_source_lines_, pre_lines_);
    if (pre_lines_) {
      inno_log_verify(pre_lines_->polygon_closed,
                      "not enough p encoder %u",
                      pre_lines_->polygons_number());
      inno_log_verify(!pre_lines_->adc_closed,
                      "adc_closed");
      const RawBlock& last_blcok = pre_lines_->get_last_active_block_const();
      int64_t diff_time = fpga_ns - last_blcok.trigger_ts_ns;
      if (config_.ingore_trigger_interval_drop ||
          diff_time < (int64_t)kMaxTimeBetwenTriggers) {
        pre_lines_->adc_closed = true;
        if (pre_lines_->galvo_closed) {
          // we have everything
          inno_log_trace("trigger close: b=%u p=%u, g=%u",
                        pre_lines_->active_blocks_number(),
                        pre_lines_->polygons_number(),
                        pre_lines_->galvos_number());
          pre_lines_->stage_ts[ScanLines::STAGE_TIME_S1] =
                              lidar_->get_monotonic_raw_time();
          lidar_->add_stage_angle_job(pre_lines_);
          pre_lines_ = NULL;
        }
        lidar_->heal_raw_fault(INNO_LIDAR_IN_FAULT_DATA_DROP4, true,
                              "INNO_LIDAR_IN_FAULT_DATA_DROP4 heals");
      } else {
        inno_log_warning("drop pre_lines_, "
                        "trigger interval too big(%" PRI_SIZED
                        " ns) counter: %" PRI_SIZEU
                        "polygon close: %d, galvo close: %d, adc close: %d",
                        diff_time, ++stats_trigger_interval_too_big_,
                        pre_lines_->polygon_closed,
                        pre_lines_->galvo_closed,
                        pre_lines_->adc_closed);
        inno_log_verify(pre_lines_->dec_ref(), "cannot free");
        lidar_->free_angle_job(pre_lines_);
        pre_lines_ = NULL;
        lidar_->set_raw_fault(INNO_LIDAR_IN_FAULT_DATA_DROP4, true,
                             "INNO_LIDAR_IN_FAULT_DATA_DROP4 sets");
      }
    }
    // so we know when cur_lines_ changed (because of polygon signal)
    current_block_source_lines_ = cur_lines_;
  } else {
    if (!cur_lines_->can_advance_block(current_raw_block_)) {
      stats_discard_trigger_++;
      // 32s for 458kHz in normal mode, the interval may vary if in other mode
      static const int discard_print_interval = 458000 * 32;
      if (stats_discard_trigger_ <= 25 ||
          stats_discard_trigger_ % discard_print_interval == 0) {
        inno_log_error("drop trigger for %" PRI_SIZELU
                       " times because overflow, "
                       "no encoder?", stats_discard_trigger_);
      }
      return;
    }
    cur_lines_->advance_block(&current_raw_block_);
  }

  previous_trigger_sub_ns_ = current_trigger_sub_ns_;
  current_trigger_sub_ns_ = InnoConverts::ns_to_sub_ns(fpga_ns);
  uint32_t trigger_period = fpga_ns -
    InnoConverts::sub_ns_to_ns(previous_trigger_sub_ns_);
  if (((int64_t)trigger_period > config_.trigger_highlimit_ns
               || (int64_t)trigger_period < config_.trigger_lowlimit_ns)
               && (!trigger_timeout_) && update_limit_
               && (stats_g_encoder_count_ > signal_fault_begin_detection_)) {
    trigger_period_ns_TO_ = trigger_period;
    trigger_timeout_ = true;
  }
  // reset
  ScanLines::reset_block(current_raw_block_, in_roi_, fpga_ns,
                         trigger_period, &ref_tracking_result_[0]);
}

inline int32_t StageSignal::galvo_find_angle_unit_(int32_t galvo_value) {
  double galvo_angle = (galvo_value - InnoConsts::kGalvoEncoderMin) *
    galvo_slope_ + galvo_min_angle_;
  return static_cast<int32_t>(galvo_angle / kDegreePerInnoAngleUnit);
}

void StageSignal::init_job_conf_seq_(ScanLines* job) {
  for (uint32_t i = 0; i < INNO_CONFIDENCE_LEVEL_MAX; i++) {
    enum ConfidenceLevel conf_level = ConfidenceLevel(i);
    job->set_confidence_seq(conf_level, conf_seq_num_[conf_level]);
  }
}

void StageSignal::process_encoder_galvo_(const EncoderData &encoder) {
  InnoFpgaNs fpga_ns = process_encoder_data_clock_(encoder);
  InnoFpgaSubNs fpga_sub_ns = InnoConverts::ns_to_sub_ns(fpga_ns);
  InnoFpgaSubNs fpga_sub_ns_adjusted = galvo_adjust_sub_ns_(fpga_sub_ns);

  this_round_g_encoder_count_++;
  stats_g_encoder_count_++;
  in_roi_ = encoder.galvo_rbit;
  up_scan_ = encoder.galvo_sbit ?
             INNO_FRAME_DIRECTION_UP :
             INNO_FRAME_DIRECTION_DOWN;
  if (up_scan_ != pre_galvo_direction_) {
    if (pre_galvo_direction_ != INNO_FRAME_DIRECTION_MAX) {
      if (last_galvo_change_direction_ns_ != 0) {
        galvo_mean_[up_scan_].add(fpga_ns -
                                 last_galvo_change_direction_ns_);
      }
      last_galvo_change_direction_ns_ = fpga_ns;
      stats_frame_counts_++;
      size_t frame_trigger_count = stats_trigger_count_ -
                                   stats_trigger_count_old_;
      stats_trigger_count_old_ = stats_trigger_count_;
      find_reference_(frame_trigger_count);
      do_optical_check_(frame_trigger_count);
    }
    pre_galvo_direction_ = up_scan_;
  }

  int32_t value = galvo_find_angle_unit_(encoder.galvo_position());

  if (cur_lines_->add_galvo(stats_g_encoder_count_,
                            fpga_sub_ns_adjusted, value, up_scan_) != 0) {
    stats_discard_encoder_2_++;
    if (stats_discard_encoder_2_ < 10 ||
        stats_discard_encoder_2_ % 10000 == 1) {
      inno_log_warning("too much gc %u, %" PRI_SIZELU,
                      cur_lines_->galvos_number(),
                      stats_discard_encoder_2_);
    }
    return;
  }
  if (cur_lines_->galvos_number() >= 2) {
    int64_t diff_galvo = cur_lines_->get_diff_galvo_sub_ns();
    int64_t diff_galvo_ns = InnoConverts::sub_ns_to_ns(diff_galvo);
    if ((diff_galvo_ns > config_.galvo_highlimit_ns
                 || diff_galvo_ns < config_.galvo_lowlimit_ns)
                 && (!galvo_timeout_) && update_limit_
                 && (stats_g_encoder_count_ > signal_fault_begin_detection_)) {
      galvo_timeout_ = true;
      diff_galvo_ns_TO_ = diff_galvo_ns;
    }
  }
  if (pre_lines_) {
    inno_log_verify(pre_lines_->polygon_closed,
                    "not enough p encoder %u",
                    pre_lines_->polygons_number());
    if (!pre_lines_->galvo_closed) {
      // duplicate galvo signal in previous scanlines
      if (pre_lines_->add_galvo(stats_g_encoder_count_,
                                fpga_sub_ns_adjusted, value, up_scan_) != 0) {
        stats_discard_encoder_0_++;
        if (stats_discard_encoder_0_ < 10 ||
            stats_discard_encoder_0_ % 1024 == 1) {
          inno_log_warning("too much gp %u, %" PRI_SIZELU,
                           pre_lines_->galvos_number(),
                           stats_discard_encoder_0_);
        }
        return;
      }
      const RawBlock &pb = pre_lines_->get_last_active_block_const();
      if (fpga_sub_ns_adjusted > InnoConverts::ns_to_sub_ns(pb.trigger_ts_ns)) {
        pre_lines_->galvo_closed = true;

        if (pre_lines_->adc_closed) {
          // we have everything
          inno_log_trace("galvo close: b=%u p=%u, g=%u",
                         pre_lines_->active_blocks_number(),
                         pre_lines_->polygons_number(),
                         pre_lines_->galvos_number());
          pre_lines_->stage_ts[ScanLines::STAGE_TIME_S1] =
              lidar_->get_monotonic_raw_time();
          lidar_->add_stage_angle_job(pre_lines_);
          pre_lines_ = NULL;
        }
      } else {
        stats_galvo_delay_++;
      }
    } else {
      inno_log_verify(!pre_lines_->adc_closed, "adc_close");
      stats_discard_encoder_1_++;
    }
  }
  check_and_insert_fake_polygon(next_fake_fpga_ns_, fpga_ns);
  return;
}

void StageSignal::check_and_insert_fake_polygon(InnoEpNs calc_ns,
                                                InnoEpNs curr_ns) {
  InnoEpNs diff  = calc_ns - curr_ns;
  if (diff < 0 && is_calc_fake_polygon_
      && fake_polygon_index_ < lidar_->get_encodes_per_polygon()) {
    // calc precise_angle_value
    InnoFpgaSubNs interval;
    int32_t precise_angle_value;
    // interval time from current real polygon encode to now
    interval = curr_ns - polygons_[0].fpga_ns;
    precise_angle_value = interval * 2 * kInnoAngleUnitPerPiRad
                                        / curr_polygon_pre_ns_;
    polygons_[fake_polygon_index_].angle_value = precise_angle_value;
    // insert fake polygon
    process_encoder_polygon_(curr_ns, fake_polygon_index_);
    fake_polygon_index_++;
    if (fake_polygon_index_ < lidar_->get_encodes_per_polygon()) {
      inno_log_verify(fake_polygon_index_ < kMaxPolygonEncodes,
      "polygons_ array bounds fake_polygon_index_ %d < kMaxPolygonEncodes %u",
      fake_polygon_index_, kMaxPolygonEncodes);
      next_fake_fpga_ns_ = polygons_[fake_polygon_index_].fpga_ns;
    } else {
      is_calc_fake_polygon_ = false;
    }
  }
}

void StageSignal::process_real_encoder_polygon_(const EncoderData &encoder) {
  InnoFpgaNs fpga_ns = process_encoder_data_clock_(encoder);
  if (encoder.f_enc_I == 0) {
    if (last_real_polygon_ns_ != 0) {
      curr_polygon_pre_ns_ = fpga_ns - last_real_polygon_ns_;
    } else {             // first rotation
      // period corresponding to mode speed
      curr_polygon_pre_ns_ = InnoConverts::sub_ns_to_ns
                            (InnoConsts::kNormalPolygonPeriodSubNs);
    }
    polygon_real_mean_.add(curr_polygon_pre_ns_);
    fake_polygon_index_ = 1;
    last_real_polygon_ns_ = fpga_ns;
    // calc fake polygon encode
    for (uint16_t i = 0; i < lidar_->get_encodes_per_polygon(); i++) {
      PolygonStamp poly_stamp;
      poly_stamp.fpga_ns = fpga_ns +
        (i * (curr_polygon_pre_ns_ / lidar_->get_encodes_per_polygon()));
      poly_stamp.angle_value = i * 2 * kInnoAngleUnitPerPiRad
                             / lidar_->get_encodes_per_polygon();
      polygons_[i] = poly_stamp;
    }
    if (lidar_->get_encodes_per_polygon() > 1) {
      is_calc_fake_polygon_ = true;
      next_fake_fpga_ns_ = polygons_[1].fpga_ns;
    }
    // real encoder_polygon
    process_encoder_polygon_(fpga_ns, 0);
  } else {
    // ignore falling edge
    stats_pf_encoder_count_++;
  }
  return;
}

void StageSignal::process_encoder_polygon_(InnoFpgaNs fpga_ns,
                                           uint16_t fake_index) {
  InnoFpgaSubNs fpga_sub_ns = InnoConverts::ns_to_sub_ns(fpga_ns);
  if (last_polygon_ns_ != 0) {
    polygon_mean_.add(fpga_ns - last_polygon_ns_);
  }

  last_polygon_ns_ = fpga_ns;
  stats_pr_encoder_count_++;

  // xxx todo: handle it gracefully
  inno_log_verify(cur_lines_->polygons_number() < 2, "impossible %u",
                  cur_lines_->polygons_number());
  inno_log_verify(fake_index < kMaxPolygonEncodes,
    "polygons_ array bounds fake_polygon_index_ %d < kMaxPolygonEncodes %u",
    fake_index, kMaxPolygonEncodes);
  int32_t fake_value = polygons_[fake_index].angle_value;
  if (cur_lines_->polygons_number() == 1) {
    if (cur_lines_->active_blocks_number() == 0) {
      stats_ignore_polygon_no_active_++;
      if (stats_ignore_polygon_no_active_ < 10 ||
          stats_ignore_polygon_no_active_ % 1024 == 1) {
        inno_log_warning("no active blocks and have 2 polygon, "
                          "update the old polygon %" PRI_SIZELU ", "
                          "galvo encodes removed: %u/%u",
                          stats_ignore_polygon_no_active_,
                          cur_lines_->galvos_number(),
                          cur_lines_->get_invalid_galvo_num() +
                          cur_lines_->galvos_number());
      }

      cur_lines_->reini_scanlines(stats_pr_encoder_count_, fpga_sub_ns,
                                  fake_value);
      // lidar_->print_stats();  // debug
      // lidar_->system_stats_->show();  // debug
      return;
    }

    InnoFpgaSubNs diff = fpga_sub_ns - cur_lines_->polygons[0].ts_sub_ns;
    if (diff < (InnoFpgaSubNs)(InnoConsts::kMinPolygonPeriodSubNs / 2
              / lidar_->get_encodes_per_polygon())) {
      // ignore wrong signal
      stats_ignore_polygon_small_gap_++;

      char warning[256]{0};
      ::snprintf(warning, sizeof(warning),
                  "polygon period %" PRI_SIZED
                  " ns is too small, ignore %" PRI_SIZELU,
                  diff >> InnoConsts::kSubNsBits,
                  stats_ignore_polygon_small_gap_);
      inno_log_warning("%s", warning);
      return;
    }
  }
  inno_log_trace("add_polygon fpga_sub_ns = %"
                  PRI_SIZED ", fake_value = %d",
                  fpga_sub_ns, fake_value);
  int r = cur_lines_->add_polygon(stats_pr_encoder_count_,
                                fpga_sub_ns, fake_value);
  inno_log_verify(r == 0, "add_polygon %u", cur_lines_->polygons_number());
  if (this_round_g_encoder_count_ > signal_fault_begin_detection_) {
    int64_t diff_polygon;
    int ret = cur_lines_->get_diff_polygon_sub_ns(&diff_polygon);
    if (ret == 0) {
      int64_t diff_polygon_ns = InnoConverts::sub_ns_to_ns(diff_polygon);
      if ((diff_polygon_ns > config_.polygon_highlimit_ns
                  || diff_polygon_ns < config_.polygon_lowlimit_ns)
                  && (!polygon_timeout_) && update_limit_) {
        polygon_timeout_ = true;
        diff_polygon_ns_TO_ = diff_polygon_ns;
      }
    }
  }

  if (cur_lines_->polygons_number() == 2) {
    // use a new scanlines
    if (pre_lines_) {
      inno_log_warning("force drop pre_lines_, lost triggers? %" PRI_SIZELU
                       "polygon close: %d, galvo close: %d, adc close: %d",
                       ++stats_force_drop_0_, pre_lines_->polygon_closed,
                       pre_lines_->galvo_closed, pre_lines_->adc_closed);
      inno_log_verify(pre_lines_->dec_ref(), "cannot free");
      lidar_->free_angle_job(pre_lines_);
      pre_lines_ = NULL;
      lidar_->set_raw_fault(INNO_LIDAR_IN_FAULT_DATA_DROP4, true,
                           "INNO_LIDAR_IN_FAULT_DATA_DROP4 sets");
    } else {
      lidar_->heal_raw_fault(INNO_LIDAR_IN_FAULT_DATA_DROP4, true,
                            "INNO_LIDAR_IN_FAULT_DATA_DROP4 heals");
    }

    pre_lines_ = cur_lines_;
    inno_log_verify(!pre_lines_->polygon_closed &&
                    !pre_lines_->galvo_closed &&
                    !pre_lines_->adc_closed,
                    "%d %d %d",
                    pre_lines_->polygon_closed,
                    pre_lines_->galvo_closed,
                    pre_lines_->adc_closed);
    pre_lines_->polygon_closed = true;

    // starting new lines
    cur_lines_ = lidar_->alloc_angle_job();
    inno_log_verify(cur_lines_, "cur_lines_");
    init_job_conf_seq_(cur_lines_);
    if (lidar_->in_calibration_mode()) {
      cur_lines_->set_calibration_mode();
    }
    /*
      xxx-todo: what if the cur_lines closed before another job come
    cur_lines_->stage_ts[ScanLines::STAGE_TIME_R1] =
        pre_lines_->stage_ts[ScanLines::STAGE_TIME_R1];
    */
    cur_lines_->stage_ts[ScanLines::STAGE_TIME_S0] =
        lidar_->get_monotonic_raw_time();

    double diff;
    enum InnoTimeSyncType state =
        lidar_->get_clock().get_sync_state_and_diff(&diff);

    // xxx todo: switch to a new cur_lines_ if state changes
    cur_lines_->time_sync_state = state;
    cur_lines_->host_lidar_time_diff_sec = diff;

    // duplicate the polygon signal in the new scanlines
    r = cur_lines_->add_polygon(stats_pr_encoder_count_,
                                fpga_sub_ns, fake_value);
    inno_log_verify(r == 0, "add_polygon %u",
                    cur_lines_->polygons_number());

    // we need to duplicate the last galvo signal as well
    if (pre_lines_->galvos_number() > 0) {
      r = cur_lines_->copy_galvo(
          pre_lines_->galvos[pre_lines_->galvos_number() - 1]);
      inno_log_verify(r == 0, "%u galvos",
                      cur_lines_->galvos_number());
    } else {
      rawdata_fault_ = true;
      stats_not_enough_g_0_++;
    }
  }
  return;
}

void StageSignal::process_encoder_data_(const EncoderData &encoder) {
  if (encoder.v_galvo_enc) {
    process_encoder_galvo_(encoder);
  }
  if (encoder.vf_enc_I) {
    process_real_encoder_polygon_(encoder);
  }
  return;
}

void StageSignal::process_padding_data_(const PaddingData &padding) {
  stats_padding_count_++;
  /* inno_log_warning("stats_padding_count_ = %" PRI_SIZEU " pad=%d which=%d drop=%d",
                   stats_padding_count_, padding.padding_0,
                   padding.detail, padding.drop); */
}
}  // namespace innovusion
