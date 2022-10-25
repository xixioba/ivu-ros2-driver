/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "sdk/stage_deliver.h"

#include <math.h>

#include "utils/consumer_producer.h"
#include "sdk/lidar.h"
#include "sdk/stage_deliver_job.h"

namespace innovusion {

StageDeliver::StageDeliver(InnoLidar *l) {
  lidar_ = l;
  params_ = &lidar_->get_params();
  misc_tables_ = &lidar_->get_misc_tables();
  lidar_->add_config(&config_base_);
  config_.copy_from_src(&config_base_);

  sub_frame_idx_ = 0;
  hori_roi_angle_ = 0;
  last_frame_idx_ = -1;

  // start with some non-zero value
  for (uint32_t i = 0; i < kInnoChannelNumber; i++) {
    ref_intensity_[i] = 1000;
  }
  memset(stats_stage_sum_ts_, 0, sizeof(stats_stage_sum_ts_));
  memset(stats_stage_sumq_ts_, 0, sizeof(stats_stage_sumq_ts_));
  memset(stats_stage_max_ts_, 0, sizeof(stats_stage_max_ts_));
  memset(retro_table_, 0, sizeof(retro_table_));
  stats_dropped_jobs_ = 0;
  stats_delivered_jobs_ = 0;
  stats_frames_ = 0;
  stats_points_ = 0;
  stats_frame_points_ = 0;
  stats_skipped_blocks_ = 0;
  frame_points_low_counter_ = 0;
  stats_lost_frames_ = 0;
  stats_last_frame_idx_ = -1;
  remap_last_origin_frame_idx_ = -1;
  remap_last_mapped_frame_idx_ = -1;
  stats_last_scan_direction_ = -1;
  send_to_next_stage_ = false;
  cali_data_pool_ = nullptr;
  blooming_distance_diff_ = 0;
  init_ts_ = lidar_->get_monotonic_raw_time_ms();
  max_distance_total_refl_ = 0;
  max_distance_point_size_ = 0;
  frame_sync_locked = 0;
}

StageDeliver::~StageDeliver(void) {
  print_stats();
  lidar_->remove_config(&config_base_);
}

void StageDeliver::print_stats(void) const {
  char buf[1024];
  get_stats_string(buf, sizeof(buf));
  inno_log_info("%s", buf);
  return;
}


void StageDeliver::get_stats(InnoStatusCounters *counters) const {
  inno_log_verify(counters, "InnoStatusCounters is null!");

  if (stats_delivered_jobs_ == 0) {
    return;
  }

  for (uint32_t i = 0; i + 1 < ScanLines::STAGE_TIME_MAX; i += 2) {
    double avg = stats_stage_sum_ts_[i] / stats_delivered_jobs_;
    double avg2 = stats_stage_sum_ts_[i + 1] / stats_delivered_jobs_;

    // double max = stats_stage_max_ts_[i];
    double max2 = stats_stage_max_ts_[i + 1];

    double dev = stats_stage_sumq_ts_[i] / stats_delivered_jobs_ - avg * avg;
    // double dev2 =
    //     stats_stage_sumq_ts_[i + 1] / stats_delivered_jobs_ - avg2 * avg2;

    dev = dev > 0 ? sqrt(dev) : 0;
    // dev2 = dev2 > 0 ? sqrt(dev2) : 0;

    uint32_t idx = 0;
    if (i == ScanLines::STAGE_TIME_R0) {
      idx = 0;
    } else if (i == ScanLines::STAGE_TIME_S0) {
      idx = 1;
    } else if (i == ScanLines::STAGE_TIME_A0) {
      idx = 2;
    } else if (i == ScanLines::STAGE_TIME_NA0) {
      idx = 3;
    } else if (i == ScanLines::STAGE_TIME_NB0) {
      idx = 4;
    } else if (i == ScanLines::STAGE_TIME_D0) {
      idx = 5;
    } else {
      inno_log_warning("unknown StageTime type: %u", i);
      continue;
    }

    //
    constexpr int SEC_TO_10US = 1000 * 100;
    counters->latency_10us_average[idx] =
        static_cast<uint16_t>(avg2 * SEC_TO_10US);
    counters->latency_10us_variation[idx] =
        static_cast<uint16_t>(dev * SEC_TO_10US);
    counters->latency_10us_max[idx] =
        static_cast<uint16_t>(max2 * SEC_TO_10US);
  }
}


void StageDeliver::get_stats_string(char *buf, size_t buf_size) const {
  int ret = snprintf(buf, buf_size, "StageDeliver: dropped=%lu "
                     "delivered=%lu mean/dev/max ",
                     stats_dropped_jobs_,
                     stats_delivered_jobs_);
  if (ret >= ssize_t(buf_size)) {
    buf[buf_size - 1] = 0;
    return;
  }
  if (stats_delivered_jobs_ == 0) {
    return;
  }

  //
  constexpr int SEC_TO_MS = 1000;

  size_t so_far = ret;
  double sum = 0;
  for (uint32_t i = 2; i + 1 < ScanLines::STAGE_TIME_MAX; i += 2) {
    double avg = stats_stage_sum_ts_[i] / stats_delivered_jobs_;
    double avg2 = stats_stage_sum_ts_[i + 1] / stats_delivered_jobs_;
    double max = stats_stage_max_ts_[i];
    double max2 = stats_stage_max_ts_[i + 1];
    double dev = stats_stage_sumq_ts_[i] / stats_delivered_jobs_ - avg * avg;
    double dev2 = stats_stage_sumq_ts_[i + 1] /
                  stats_delivered_jobs_ - avg2 * avg2;

    dev = dev > 0 ? sqrt(dev) : 0;
    dev2 = dev2 > 0 ? sqrt(dev2) : 0;

    if (avg > 0) {
      sum += avg;
    }
    if (avg2 > 0) {
      sum += avg2;
    }
    (void)sum;
    const char *stage = "";
    if (i == ScanLines::STAGE_TIME_R0) {
      stage = "read";
    } else if (i == ScanLines::STAGE_TIME_S0) {
      stage = "signal";
    } else if (i == ScanLines::STAGE_TIME_A0) {
      stage = "angle";
    } else if (i == ScanLines::STAGE_TIME_NA0) {
      stage = "n0";
    } else if (i == ScanLines::STAGE_TIME_NB0) {
      stage = "n1";
    } else if (i == ScanLines::STAGE_TIME_D0) {
      stage = "deliver";
    } else {
      stage = "unknown";
    }

    ret = snprintf(buf + so_far, buf_size - so_far,
                   "%s(wait=%.1f/%.1f/%.1f run=%.1f/%.1f/%.1f) ", stage,
                   avg * SEC_TO_MS, dev * SEC_TO_MS, max * SEC_TO_MS,
                   avg2 * SEC_TO_MS, dev2 * SEC_TO_MS, max2 * SEC_TO_MS);

    if (ret >= ssize_t(buf_size - so_far)) {
      buf[buf_size - 1] = 0;
      return;
    }
    so_far += ret;
  }

  double avg = stats_stage_sum_ts_[ScanLines::STAGE_TIME_MAX] /
               stats_delivered_jobs_;
  double max = stats_stage_max_ts_[ScanLines::STAGE_TIME_MAX];
  double dev = stats_stage_sumq_ts_[ScanLines::STAGE_TIME_MAX] /
               stats_delivered_jobs_ - avg * avg;
  dev = dev > 0 ? sqrt(dev) : 0;

  ret = snprintf(buf + so_far, buf_size - so_far,
                 "sum=%.1f/%.1f/%.1fms callback=%.2f/%.2f/%.2f/%"
                 PRI_SIZEU " frames=%" PRI_SIZELU
                 " lost-frames=%" PRI_SIZELU
                 " points=%" PRI_SIZELU
                 " missalignment=%.2f/%.2f/%.2f/%" PRI_SIZEU,
                 avg * SEC_TO_MS, dev * SEC_TO_MS, max * SEC_TO_MS,
                 callback_mean_ms_.mean(),
                 callback_mean_ms_.std_dev(),
                 callback_mean_ms_.max(),
                 callback_mean_ms_.count(),
                 stats_frames_,
                 stats_lost_frames_,
                 stats_points_,
                 max_distance_intensity_.mean(),
                 max_distance_intensity_.min(),
                 max_distance_intensity_.max(),
                 max_distance_intensity_.count());
  if (ret >= ssize_t(buf_size - so_far)) {
    buf[buf_size - 1] = 0;
    return;
  }
}

int StageDeliver::process(void *in_job, void *ctx,
                         bool prefer) {
  StageDeliver *s = reinterpret_cast<StageDeliver *>(ctx);
#ifdef __APPLE__
  INNER_BEGIN_LOG(StageDeliver_process, OS_LOG_CATEGORY_DYNAMIC_TRACING,
                  StageDeliver_process);
#endif
  int ret = s->process_job_(reinterpret_cast<StageAngleJob *>(in_job), prefer);
#ifdef __APPLE__
  INNER_END_LOG(StageDeliver_process);
#endif
  return ret;
}

#define ALLOCATE_NEW_PACKET(line_index) \
  inno_log_verify(packet_index < packet_count,                          \
                  "packet_index: %d", packet_index);                    \
  packet = packets_[packet_index++];                                     \
  packet->idx = job->lines[line_index].frame_idx;                       \
  if (last_frame_idx_ != (int64_t)(packet->idx)) {                      \
    last_frame_idx_ = packet->idx;                                     \
    hori_roi_angle_ = lidar_->hori_roi_angle_ * kInnoAngleUnitPerDegree; \
  }                                                                     \
  packet->sub_idx = sub_frame_idx_++;                                   \
  packet->scanner_direction = job->lines[line_index].direction;         \
  inno_block = reinterpret_cast<InnoBlock *>(&packet->inno_block2s[0]); \
  inno_points = &inno_block->points[0];                                 \
  current_block = 0;                                                    \
  start_trigger_time = raw_block->trigger_ts_ns;                        \
  packet->common.ts_start_us =                                          \
    LidarClock::to_epoch_us_(start_trigger_time, host_lidar_time_diff_sec);

// process_job_ exceeding 500 lines
// move some code in this function
int StageDeliver::un_prefer_process_job_(StageAngleJob *job) {
  if (job->dec_ref()) {
    lidar_->free_angle_job(job);
  }
  stats_dropped_jobs_++;
  if (stats_dropped_jobs_ % 16384 == 1) {  // 0x4000
    inno_log_warning("drop data in deliver stage total_dropped=%lu",
                       stats_dropped_jobs_);
  }
  if (stats_dropped_jobs_ % 1024 == 1) {
    print_stats();
    max_distance_intensity_.reset();
    lidar_->cp_deliver_->print_stats();
  }
  uint64_t cpu_usage[4] = {0, 0, 0, 0};
  lidar_->get_cpu_usage(cpu_usage);
  lidar_->set_raw_fault(INNO_LIDAR_IN_FAULT_DATA_DROP2, true,
                         "CPU usage: %" PRI_SIZEU
                         "/%" PRI_SIZEU
                         "/%" PRI_SIZEU
                         "/%" PRI_SIZEU "/",
                          cpu_usage[0], cpu_usage[1],
                          cpu_usage[2], cpu_usage[3]);
  return 0;
}

int StageDeliver::process_job_(StageAngleJob *job, bool prefer) {
  if (!prefer) {
    return un_prefer_process_job_(job);
  }
  lidar_->heal_raw_fault(INNO_LIDAR_IN_FAULT_DATA_DROP2, true,
                        "INNO_LIDAR_IN_FAULT_DATA_DROP2 heals");
  inno_log_assert(job, "job");

  job->stage_ts[ScanLines::STAGE_TIME_D0] = lidar_->get_monotonic_raw_time();

  config_.copy_from_src(&config_base_);

  int return_times =
      InnoDataPacketUtils::get_return_times(lidar_->multiple_return_);

  bool record_cali_data =
    lidar_->has_recorder_callback(INNO_RECORDER_CALLBACK_TYPE_CALI);
  InnoCaliDataBuffer *cali_data_buffer = NULL;
  uint32_t cali_data_count = 0;
  /**
   * [CST Bugfinder Defect] Reviewed
   *
   * PolySpace report a defect here:
   * Partial read: v_angle_offset[0] is never read.
   *
   * It is expected.
   *
   * Ignore
   */
  int v_angle_offset[kInnoChannelNumber];
  for (uint32_t ich = 0; ich < kInnoChannelNumber; ich++) {
    v_angle_offset[ich] = ich * kInnoVAngleDiffBase;
  }

  if (record_cali_data) {
    if (!cali_data_pool_) {
      cali_data_pool_ = new MemPool("cali_data_pool",
                                    kMaxCaliDataBufferSize_,
                                    kCaliDataPoolSize_, 32);
      inno_log_verify(cali_data_pool_, "cali_data_pool_");
    }
    cali_data_buffer =
      reinterpret_cast<InnoCaliDataBuffer *>(cali_data_pool_->alloc());
  }

  bool in_calibration_mode = job->in_calibration_mode();
  if (in_calibration_mode) {
    send_to_next_stage_ = true;
  }
  bool use_reflectance = /* (!in_calibration_mode) && */
                         (lidar_->reflectance_mode_ ==
                          INNO_REFLECTANCE_MODE_REFLECTIVITY);

  for (uint32_t i = 0; i < kInnoChannelNumber; i++) {
    refl_factor_[i] = params_->iv_params.refl_factor[i] * 100 / 3900;
  }

  uint32_t kill_distance = params_->iv_params.k_dis;
  uint32_t kill_intensity = params_->iv_params.k_int;
  uint32_t kill_distance_2 = params_->iv_params.k_dis_2;
  float kill_reflectance_2 = params_->iv_params.k_ref_2;
  uint32_t retro_intensity_2 = params_->iv_params.retro_intensity_2;

  bool use_block2 = false;
  int unit_size = 0;
  int frame_count = job->frame_count;
  // inno_log_verify(frame_count >= 1 && frame_count <= 2,
  //                 "frame count: %d", frame_count);

  // xxx todo: reflectance mode
  // xxx todo: raw_output
  // xxx todo: is fov done in this stage?

  if (return_times == 2) {
    use_block2 = true;
    unit_size = sizeof(InnoBlock2);
  } else {
    // return_times == 1
    use_block2 = false;
    unit_size = sizeof(InnoBlock1);
  }

  if (config_.max_packet_size > 65000) {
    inno_log_error("max_packet_size too big %u", config_.max_packet_size);
    config_.max_packet_size = 65000;
  }
  int max_block_number = (config_.max_packet_size -
                          sizeof(InnoDataPacket)) / unit_size;
  if (max_block_number <= 0) {
    inno_log_error("bad max_block_number %d", max_block_number);
    max_block_number = 1;
  }
  int active_blocks_number = job->active_blocks_number();
  int32_t packet_count = (active_blocks_number + max_block_number - 1) /
                         max_block_number;
  if (frame_count > 1) {
    // may cross frame boundary
    packet_count += 1;
  }

  inno_log_verify(ssize_t(kDeliverMaxPacketNumber) > packet_count,
                 "%lu > %d", kDeliverMaxPacketNumber, packet_count);
  enum InnoLidarMode mode = INNO_LIDAR_MODE_WORK_NORMAL;
  enum InnoLidarMode pre_mode = INNO_LIDAR_MODE_WORK_NORMAL;
  enum InnoLidarStatus status = INNO_LIDAR_STATUS_NORMAL;
  uint64_t in_transition_mode_ms = 0;
  if (lidar_->is_live_direct_memory_lidar_()) {
    lidar_->get_mode_status(&mode, &pre_mode, &status, &in_transition_mode_ms);
  }

  enum InnoGalvoMode galvo_mode = lidar_->get_galvo_mode();
  if (galvo_mode == INNO_GALVO_MODE_NONE) {
    // we are playing file, use auto detect mode
    galvo_mode = job->get_auto_galvo_mode();
  }
  int published_frames = config_.published_frames;
  if (published_frames == 0) {
    // use galvo mode to determine published frames
    published_frames = (galvo_mode == INNO_GALVO_MODE_FLYBACK) ? 1 : 3;
  }

  for (int i = 0; i < packet_count; i++) {
    init_packet_(job, use_reflectance, unit_size, packet_count,
                 mode, status, i, lidar_->multiple_return_,
       stats_delivered_jobs_ >= (size_t)lidar_->get_encodes_per_polygon()
                             ? INNO_FULL_CONFIDENCE : INNO_NO_CONFIDENCE);
  }

  // InnoBlock *prev_block = reinterpret_cast<InnoBlock *>(&paddingblock_);
  int packet_index = 0;
  InnoDataPacket* packet;
  InnoBlock *inno_block;
  InnoChannelPoint *prev_inno_points = &paddingblock_.points[0];
  InnoChannelPoint *inno_points;
  RawBlock *raw_block = &job->blocks[job->lines[0].active_blocks_start];
  RawChannelPoint *raw_point = NULL;
  double host_lidar_time_diff_sec = job->host_lidar_time_diff_sec;
  InnoEpNs start_trigger_time;
  int current_block = 0;
  size_t total_points = 0;
  bool end_of_frame = false;

  ALLOCATE_NEW_PACKET(0);

  for (uint32_t i = 0; i < job->lines_number; i++) {
    raw_block = &job->blocks[job->lines[i].active_blocks_start];
    bool new_frame_start = false;
    if (job->lines[i].frame_idx != stats_last_frame_idx_
        && (int64_t)stats_last_frame_idx_ >= 0) {
      end_of_frame = true;
      if (i == 0 && should_send_empty_packet_) {
        // we will send an empty packet in this case, so need one more packet
        init_packet_(job, use_reflectance, unit_size, packet_count,
                     mode, status, packet_count, lidar_->multiple_return_);
        packet->idx = stats_last_frame_idx_;
        packet->scanner_direction = stats_last_scan_direction_;
        packet_count += 1;
        end_of_frame = false;
        packet->is_last_sub_frame = 1;
      }
      // this frame is done
      packet->item_number = current_block;
      if (stats_last_scan_direction_ == INNO_FRAME_DIRECTION_DOWN &&
          config_.published_frames != 2) {
        // only reset table after a down scanning.
        // we don't reset table after an up scanning because next frame
        // processing depends on information from last frame
#if 0
        for (int i = 0; i < kRetroTableSize_; i++) {
          for (int j = 0; j < kRetroTableBins_; j++) {
            if (retro_table_[i][j].retro_count > 0) {
              inno_log_info("i/j: %d/%d %f c: %d d: %f/%f v: %f/%f",
                             i, j, -60 + i * (1 << kRetroTableShift_) * 0.0055,
                             retro_table_[i][j].retro_count,
                             retro_table_[i][j].min_retro_distance * 0.005,
                             retro_table_[i][j].max_retro_distance * 0.005,
                             retro_table_[i][j].min_retro_vert_angle * 0.0055,
                             retro_table_[i][j].max_retro_vert_angle * 0.0055);
            }
          }
        }
#endif
        memset(retro_table_, 0, sizeof(retro_table_));
        blooming_distance_diff_ = config_.blooming_up_distance_diff;
      } else {
        blooming_distance_diff_ = config_.blooming_down_distance_diff;
      }
      sub_frame_idx_ = 0;
      stats_lost_frames_ +=
          job->lines[i].frame_idx - stats_last_frame_idx_ - 1;
      // new packet
      ALLOCATE_NEW_PACKET(i);
      new_frame_start = true;

      if (record_cali_data) {
        if (cali_data_count >= kMaxCaliDataPoint_) {
          inno_log_warning("cali_data_count %u reaches maximum, "
                           "need to increase cali data buffer size",
                           cali_data_count);
        }
        cali_data_buffer->item_number = cali_data_count;
        lidar_->do_recorder_callback(INNO_RECORDER_CALLBACK_TYPE_CALI,
                reinterpret_cast<char *>(&cali_data_buffer->cali_data[0]),
                                     cali_data_count * sizeof(InnoCaliData));
        cali_data_count = 0;
        cali_data_buffer->item_number = 0;
      }
    }

    stats_last_frame_idx_ = job->lines[i].frame_idx;
    stats_last_scan_direction_ = job->lines[i].direction;
    if (!should_publish_(job, galvo_mode, published_frames, packet, i)) {
      continue;
    }

    trace_log_frame_start_ts_(packet, new_frame_start);

    for (uint32_t j = 0;
         j < job->lines[i].active_blocks_number; j++, raw_block++) {
      if (raw_block->header.in_roi) {
        int h_angle = raw_block->header.h_angle;
        if (config_.roi_pattern == 0) {
          // the whole vertical roi is roi
          raw_block->header.in_roi = 3;
        } else if ((h_angle < lidar_->hori_roi_angle_unit_ -
             (signed)config_.hori_roi_size) ||
            (h_angle > lidar_->hori_roi_angle_unit_ +
             (signed)config_.hori_roi_size)) {
          // in vertically roi, but horizontally non-roi region
          if (raw_block->header.scan_idx & 0x1) {
            // skip every other block
            continue;
          }
          if (config_.roi_pattern == 2 && raw_block->header.scan_id & 0x1) {
            // skip the line
            continue;
          }
        } else {
          // in center roi region
          raw_block->header.in_roi = 3;
        }
      }

      // need to keep last non-zero ref_intensity since for channel 1/2/3
      // it is zero most of the time
      for (uint32_t channel = 0; channel < kInnoChannelNumber; channel++) {
        uint32_t ref_intensity = raw_block->ref_intensity[channel];
        if (ref_intensity != 0) {
          ref_intensity_[channel] = ref_intensity;
        }
      }

      // skip the raw block only when this block has no point in the first
      // firing cycle and the next block has no point in the second firing cycle
      if (can_skip_(raw_block)) {
        ++stats_skipped_blocks_;
        continue;
      }

      if (current_block >= max_block_number) {
        // this InnoDataPacket is done
        packet->item_number = current_block;

        // new packet
        ALLOCATE_NEW_PACKET(i);
      }

      memcpy(&inno_block->header, &raw_block->header, sizeof(InnoBlockHeader));
      inno_block->header.ts_10us =
        (raw_block->trigger_ts_ns - start_trigger_time) / InnoConsts::kNsIn10Us;
      raw_point = &raw_block->points[0][0];

      int32_t h_angle[4];
      int32_t v_angle[4];
      get_angles_(v_angle_offset, raw_block, v_angle, h_angle);

      bool max_distance_check_enabled
        = need_to_check_max_distance_(raw_block->header.h_angle,
                                      raw_block->header.v_angle);
      for (uint32_t channel = 0; channel < kInnoChannelNumber; channel++) {
        if ((raw_point->get_raw_intensity() > retro_intensity_2) &&
          raw_point->is_real(0)) {
          set_retro_table_(h_angle[channel], v_angle[channel],
                           raw_point->get_radius(0));
        }

        if (!(config_.published_halves & (1 << channel))) {
          for (int rt = 0; rt < return_times; rt++) {
            inno_points[InnoBlock2::get_idx(channel, rt)].radius = 0;
          }
          for (uint32_t k = 0; k < kInnoMaxMultiReturn; k++, raw_point++) {
          }
          continue;
        }

        int ret = 0;
        for (int k = 0; k < return_times; k++, raw_point++) {
          if (raw_point->is_real(0)) {
            // it is possible the first point is filtered out so
            // the second point takes the first slot
            uint32_t radius = raw_point->get_radius(0);
            if (radius < kill_distance &&
                raw_point->get_raw_intensity() < kill_intensity) {
              continue;
            }
            if (ret < return_times &&
                radius > config_.min_distance &&
                radius < config_.max_distance) {
              uint32_t idx = InnoBlock2::get_idx(channel, ret);
              float reflectance = 0;
              inno_points[idx].radius = radius;
              inno_points[idx].type = raw_point->get_type();
              inno_points[idx].elongation = raw_point->get_elongation();
              inno_points[idx].refl = get_refl_(use_reflectance,
                                               record_cali_data,
                                               channel,
                                               raw_point->intensity_seq,
                                               raw_point->raw_intensity,
                                               ref_intensity_[channel],
                                               radius,
                                               h_angle[channel],
                                               &reflectance);
              double radius_value = radius * kMeterPerInnoDistanceUnit;
              uint32_t s_intensity
                      = get_scaled_intensity(raw_point->raw_intensity);
              if (max_distance_check_enabled
                 && s_intensity < kInnoMaxDistanceThMaxIntensity
                 && radius_value < kInnoMaxDistanceThMaxRadius
                 && radius_value > kInnoMaxDistanceThMinRadius) {
                max_distance_total_refl_ += s_intensity * radius_value;
                ++max_distance_point_size_;
              }
              // In single return mode, we only take the first point of
              // each channel such as ch0_r0, ch1_r0
              // mark point of this point is 1st return or not
              // xxx todo: should we consider other points' radius and
              // reflectances is > kill_xxx or not
              inno_points[idx].is_2nd_return = 0;
              for (uint32_t r = 0; r < kInnoMaxMultiReturn; r++) {
                if (raw_block->points[channel][r].is_real(0) && r != idx &&
                    raw_block->points[channel][r].get_radius(0) < radius) {
                  // If any other points in the same channel is closer,
                  // we consider this point is 2nd return
                  inno_points[idx].is_2nd_return = 1;
                  break;
                }
              }
              if ((use_reflectance || record_cali_data) &&
                  radius < kill_distance_2 &&
                  reflectance < kill_reflectance_2) {
                bool kill = kill_blooming_(h_angle[channel],
                                           v_angle[channel],
                                           radius);
                if (kill) {
                  // inno_points[idx].refl = 180;
                  continue;
                }
              }
              // cali data is needed in first firing cycle only
              if (record_cali_data && cali_data_count < kMaxCaliDataPoint_) {
                InnoCaliData *cali_data =
                  &cali_data_buffer->cali_data[cali_data_count++];
                cali_data->frame_id = packet->idx;
                cali_data->channel = channel;
                cali_data->facet = raw_block->header.facet;
                cali_data->direction = packet->scanner_direction;
                cali_data->roi = raw_block->header.in_roi;
                cali_data->poly_angle = raw_block->poly_angle;
                cali_data->galvo_angle = raw_block->galvo_angle;
                cali_data->h_angle = h_angle[channel];
                cali_data->v_angle = v_angle[channel];
                cali_data->ref_intensity = ref_intensity_[channel];
                cali_data->radius = radius;
                cali_data->intensity = raw_point->raw_intensity;
                cali_data->reflectance = reflectance;
              }
              ret++;
              total_points++;
            }
          } else if (raw_point->is_real(1) && config_.firing_cycle > 1 &&
                     raw_point->get_radius(1) > config_.min_distance &&
                     raw_point->get_radius(1) < config_.max_distance) {
            // the point belongs to the previous firing cycle.
            // only fill in when the slot is not filled yet,
            // so that we don't overwrite a point in the first firing cycle
            uint32_t idx0 = InnoBlock2::get_idx(channel, 0);
            uint32_t idx1 = InnoBlock2::get_idx(channel, 1);
            if (prev_inno_points[idx0].radius == 0) {
              prev_inno_points[idx0].radius = raw_point->get_radius(1);
              prev_inno_points[idx0].type = raw_point->get_type();
              prev_inno_points[idx0].elongation = raw_point->get_elongation();
              prev_inno_points[idx0].refl = get_refl_(use_reflectance,
                                                     false,
                                                     channel,
                                                     raw_point->intensity_seq,
                                                     raw_point->raw_intensity,
                                                     ref_intensity_[channel],
                                                     raw_point->get_radius(1),
                                                     h_angle[channel],
                                                     NULL);
              prev_inno_points[idx0].is_2nd_return = 0;
              for (uint32_t r = 0; r < kInnoMaxMultiReturn; r++) {
                if (raw_block->points[channel][r].is_real(1) && r != idx0 &&
                    raw_block->points[channel][r].get_radius(1) <
                    prev_inno_points[idx0].radius) {
                  prev_inno_points[idx0].is_2nd_return = 1;
                  break;
                }
              }
              total_points++;
            } else if (use_block2 &&
                       prev_inno_points[idx1].radius == 0) {
              prev_inno_points[idx1].radius = raw_point->get_radius(1);
              prev_inno_points[idx1].type = raw_point->get_type();
              prev_inno_points[idx1].elongation = raw_point->get_elongation();
              prev_inno_points[idx1].refl = get_refl_(use_reflectance,
                                                     false,
                                                     channel,
                                                     raw_point->intensity_seq,
                                                     raw_point->raw_intensity,
                                                     ref_intensity_[channel],
                                                     raw_point->get_radius(1),
                                                     h_angle[channel],
                                                     NULL);
              prev_inno_points[idx1].is_2nd_return = 0;
              for (uint32_t r = 0; r < kInnoMaxMultiReturn; r++) {
                if (raw_block->points[channel][r].is_real(1) && r != idx1 &&
                    raw_block->points[channel][r].get_radius(1) <
                    prev_inno_points[idx1].radius) {
                  prev_inno_points[idx1].is_2nd_return = 1;
                  break;
                }
              }
              total_points++;
            }
          }
        }

        for (uint32_t k = return_times;
             k < kInnoMaxMultiReturn; k++, raw_point++) {
        }

        // the memory is not initialized when allocated,
        // therefore need to set slots not used to 0
        for (; ret < return_times; ret++) {
          inno_points[InnoBlock2::get_idx(channel, ret)].radius = 0;
        }
      }

      prev_inno_points = inno_points;
      inno_block = reinterpret_cast<InnoBlock *>
                  (reinterpret_cast<char *>(inno_block) + unit_size);
      inno_points = &inno_block->points[0];
      current_block++;
    }
  }

  packet->item_number = current_block;

  // If this line is the end of frame, we also set the is_last_sub_frame of the
  // last packet in this line to true
  // We should do stats how many packets with points of same frame is received
  // after this sub frame.
  // if (job->lines_number > 0) {
  //  packet->is_last_sub_frame =
  //  job->lines[job->lines_number - 1].end_of_frame;
  // }
  if (job->lines_number > 0 && !end_of_frame) {
    end_of_frame = job->lines[job->lines_number - 1].end_of_frame;
    packet->is_last_sub_frame = end_of_frame;
  }
  should_send_empty_packet_ = true;
  if (end_of_frame) {
    should_send_empty_packet_ = false;
  }

  if (packet->item_number == 0 && !packet->is_last_sub_frame) {
    sub_frame_idx_--;
  }

  job->stage_ts[ScanLines::STAGE_TIME_D1] = lidar_->get_monotonic_raw_time();
  stage_ts_(job);

  if (job->dec_ref()) {
    lidar_->free_angle_job(job);
  }

  StageDeliver2Job *deliver2_job = NULL;
  if (send_to_next_stage_) {
    deliver2_job = lidar_->alloc_deliver2_job();
    inno_log_verify(deliver2_job, "alloc deliver2 job falied");
  }

  size_t frame_cnt = 0;
  size_t pkt_cnt = 0;
  size_t sent_bytes = 0;
  enum ConfidenceLevel conf = INNO_CONFIDENCE_LEVEL_FULL;
  for (int i = 0; i < INNO_CONFIDENCE_LEVEL_MAX; i++) {
    enum ConfidenceLevel conf_level = ConfidenceLevel(i);
    uint32_t job_seq_num = job->get_confidence_seq(conf_level);
    uint32_t deliver_seq_num = lidar_->\
             get_confidence_seq(INNO_PROCESS_STAGE_DELIVER, conf_level);
    if (job_seq_num != deliver_seq_num) {
      // faults of current confidence level have not been healed yet
      // so we take the current confidence and break the loop
      conf = conf_level;
      break;
    }
    inno_log_verify(job_seq_num <= deliver_seq_num,
        "deliver_seq_num shouldn't be smaller than job_seq_num: %u vs. %u",
        deliver_seq_num, job_seq_num);
  }
  for (int i = 0; i < packet_count; i++) {
    int cr = 0;
    bool send_last_sub_frame = false;
    if (packets_[i]->is_last_sub_frame) {
      int published_frame =
        (packets_[i]->scanner_direction == INNO_FRAME_DIRECTION_UP) ? 0 : 1;
      // send last subframe only when config allows
      if ((1 << published_frame) & published_frames) {
        send_last_sub_frame = true;
      }
    }
    // The packets have points or need send empty last sub frame
    if (packets_[i]->item_number || send_last_sub_frame) {
      packets_[i]->common.size = sizeof(*packets_[i]) +
                                packets_[i]->item_number *
                                packets_[i]->item_size;
      if (conf < packets_[i]->confidence_level) {
        packets_[i]->confidence_level = conf;
      }
      if (packets_[i]->is_last_sub_frame) {
        frame_cnt++;
        stats_frames_++;
        if (stats_frames_ % (15 * 30) == 10) {
          print_stats();
          max_distance_intensity_.reset();
        }
        check_stats_frame_points_();
      }
      sent_bytes += packets_[i]->common.size;

      // remap frame_id
      if (remap_last_origin_frame_idx_ != (ssize_t)packets_[i]->idx) {
        remap_last_origin_frame_idx_ = packets_[i]->idx;
        remap_last_mapped_frame_idx_++;
      }
      packets_[i]->idx = remap_last_mapped_frame_idx_;
      if (packets_[i]->is_last_sub_frame) {
        do_max_distance_check_callback_(packets_[i]->idx);
      }

      InnoPacketReader::set_packet_crc32(&packets_[i]->common);
      if (!send_to_next_stage_) {
        uint64_t start = InnoUtils::get_time_ns(CLOCK_MONOTONIC_RAW);
        if (lidar_->data_packet_callback_) {
          if (lidar_->force_xyz_pointcloud_) {
            if (InnoDataPacketUtils::convert_to_xyz_pointcloud(
                    *packets_[i],
                    &xyz_data_packet_,
                    sizeof(xyz_data_packet_buf_),
                    false)) {
              cr = lidar_->do_data_callback(&xyz_data_packet_);
            } else {
              inno_log_error("cannot convert data_packet to xyz");
            }
          } else {
            // cr = 0;
            cr = lidar_->do_data_callback(packets_[i]);
          }
        }
        callback_mean_ms_.add((InnoUtils::get_time_ns(CLOCK_MONOTONIC_RAW) -
                             start) / 1000000.0);
        lidar_->free_deliver_points_job(packets_[i]);
      } else {
        deliver2_job->packets[pkt_cnt] = packets_[i];
      }
      pkt_cnt++;
    } else {
      lidar_->free_deliver_points_job(packets_[i]);
    }
    inno_log_verify(cr == 0, "data_packet_callback return %d", cr);
  }

  if (send_to_next_stage_) {
    deliver2_job->packet_cnt = pkt_cnt;
    lidar_->add_stage_deliver2_job(deliver2_job);
  }

  if (record_cali_data) {
    if (cali_data_count >= kMaxCaliDataPoint_) {
      inno_log_warning("cali_data_count %u reaches maximum, "
                       "need to increase cali data buffer size",
                       cali_data_count);
    }
    cali_data_buffer->item_number = cali_data_count;
    lidar_->do_recorder_callback(INNO_RECORDER_CALLBACK_TYPE_CALI,
            reinterpret_cast<char *>(&cali_data_buffer->cali_data[0]),
                                 cali_data_count * sizeof(InnoCaliData));
    cali_data_pool_->free(cali_data_buffer);
  }

  lidar_->stats_update_packet_bytes(ResourceStats::PACKET_TYPE_DATA,
                                    pkt_cnt, sent_bytes);
  lidar_->stats_update_packet_bytes(ResourceStats::PACKET_TYPE_POINT,
                                    frame_cnt, total_points);
  stats_points_ += total_points;
  stats_frame_points_ += total_points;

  return 0;
}

void StageDeliver::get_angles_(const int *v_angle_offset,
                               const RawBlock *raw_block,
                               int32_t *v_angle,
                               int32_t *h_angle) const {
  h_angle[0] = raw_block->header.h_angle;
  h_angle[1] = h_angle[0] + raw_block->header.h_angle_diff_1;
  h_angle[2] = h_angle[0] + raw_block->header.h_angle_diff_2;
  h_angle[3] = h_angle[0] + raw_block->header.h_angle_diff_3;

  v_angle[0] = raw_block->header.v_angle;
  v_angle[1] = v_angle[0] + raw_block->header.v_angle_diff_1 +
               v_angle_offset[1];
  v_angle[2] = v_angle[0] + raw_block->header.v_angle_diff_2 +
               v_angle_offset[2];
  v_angle[3] = v_angle[0] + raw_block->header.v_angle_diff_3 +
               v_angle_offset[3];
}

void StageDeliver::trace_log_frame_start_ts_(InnoDataPacket *packet,
                                             bool new_frame_start) {
  if (new_frame_start) {
    lidar_->frame_start_time_statistic(packet);
  }
}

bool StageDeliver::should_publish_(const StageAngleJob *job,
                                   const InnoGalvoMode &galvo_mode,
                                   int published_frames,
                                   const InnoDataPacket *packet,
                                   uint32_t i) const {
  // in flyback mode, the end_of_frame line could be making transition
  // across frame boundary and cause obvious artifact
  if (galvo_mode == INNO_GALVO_MODE_FLYBACK && published_frames == 1 &&
      job->lines[i].end_of_frame && job->lines[i].galvo_angle_change >
      (int32_t) config_.flyback_angle_threshold) {
    return false;
  }

  int published_frame =
    (packet->scanner_direction == INNO_FRAME_DIRECTION_UP) ? 0 : 1;
  if (!((1 << published_frame) & published_frames)) {
    return false;
  }

  return true;
}

// process_job_ exceeding 500 lines
// move some code in this function
inline void StageDeliver::check_stats_frame_points_() {
  if (stats_frame_points_ < kFramePointsTooFew_ &&
      !lidar_->\
      get_current_fault_status(INNO_LIDAR_IN_FAULT_WINDOW_BLOCKAGE1) &&
      (frame_points_low_counter_++ < 30 ||
      frame_points_low_counter_ % 256 == 0)) {
    inno_log_warning("frame points too few(%lu) "
                     "for %u times, skipped blocks: %lu",
                     stats_frame_points_,
                     frame_points_low_counter_,
                     stats_skipped_blocks_);
  }
  stats_frame_points_ = 0;
  stats_skipped_blocks_ = 0;
}

inline void StageDeliver::init_packet_(const StageAngleJob *job,
                                       bool use_reflectance,
                                       int unit_size,
                                       int32_t packet_count,
                                       const InnoLidarMode &mode,
                                       const InnoLidarStatus &status,
                                       int i,
                                       int multi_return_mode,
                                       int confidence) {
  packets_[i] = lidar_->alloc_deliver_points_job();
  inno_log_verify(packets_[i], "calloc in StageDeliver failed %d/%d",
   i, packet_count);

  packets_[i]->common.version.magic_number = kInnoMagicNumberDataPacket;
  packets_[i]->common.version.major_version = kInnoMajorVersionDataPacket;
  packets_[i]->common.version.minor_version = kInnoMinorVersionDataPacket;
  packets_[i]->common.version.fw_sequence = lidar_->get_fw_sequence();
  // xxx todo
  packets_[i]->common.checksum = 0;
  packets_[i]->common.size = 0;
  packets_[i]->common.source_id = 0;
  packets_[i]->common.timestamp_sync_type = job->time_sync_state;
  packets_[i]->common.reserved = 0;
  packets_[i]->common.ts_start_us = 0;
  packets_[i]->common.lidar_status = status;
  packets_[i]->common.lidar_mode = mode;

  packets_[i]->type = INNO_ITEM_TYPE_SPHERE_POINTCLOUD;
  packets_[i]->item_size = unit_size;
  packets_[i]->item_number = 0;
  packets_[i]->topic = 0;

  packets_[i]->scanner_direction = 0;
  packets_[i]->use_reflectance = use_reflectance;
  packets_[i]->multi_return_mode = multi_return_mode;
  int conf_tmp = status == INNO_LIDAR_STATUS_NORMAL ?
                           INNO_CONFIDENCE_LEVEL_FULL :
                           INNO_CONFIDENCE_LEVEL_NONE;
  if (confidence < conf_tmp) {
    conf_tmp = confidence;
  }
  packets_[i]->confidence_level = conf_tmp;
  packets_[i]->is_last_sub_frame = 0;
  packets_[i]->is_last_sequence = 1;
  packets_[i]->has_tail = 0;
  packets_[i]->frame_sync_locked = 0;
  packets_[i]->reserved_flag = 0;
  packets_[i]->roi_h_angle = lidar_->hori_roi_angle_unit_;
  packets_[i]->roi_v_angle = lidar_->vertical_roi_angle_unit_;

  packets_[i]->idx = 0;
  packets_[i]->sub_idx = 0;
  packets_[i]->sub_seq = 0;
}

int StageDeliver::do_max_distance_check_callback_(const uint64_t idx) {
  char content_buf[512] = {0};
  double frame_mean_refl
         = max_distance_point_size_ > kInnoMaxDistanceMinPointSize
           ? max_distance_total_refl_ / max_distance_point_size_ : 0;
  size_t ret = snprintf(content_buf, sizeof(content_buf),
                "%u,%u,%lf,%lf,%lf,%lf,%" PRI_SIZEU ",%u,%lf",
                kInnoMaxDistanceThMaxTimes,
                kInnoMaxDistanceThMinValidTimes,
                kInnoMaxDistanceThNormalRefl,
                kInnoMaxDistanceThFaultRefl,
                kInnoMaxDistanceThMinSpeed,
                kInnoMaxDistanceThStartSpeed,
                idx,
                max_distance_point_size_ > kInnoMaxDistanceMinPointSize
                                         ? max_distance_point_size_ : 0,
                frame_mean_refl);
  if (ret < sizeof(content_buf)) {
    lidar_->do_message_callback(INNO_MESSAGE_LEVEL_INFO,
                                INNO_MESSAGE_CODE_MAX_DISTANCE_CHECK_RESULT,
                                content_buf);
  }
  if (frame_mean_refl > 0) {
    max_distance_intensity_.add(frame_mean_refl);
  }
  max_distance_total_refl_ = 0;
  max_distance_point_size_ = 0;
  return 0;
}

inline bool StageDeliver::can_skip_(RawBlock *raw_block) {
  RawChannelPoint *raw_point = &raw_block->points[0][0];
  for (uint32_t channel = 0; channel < kInnoChannelNumber; channel++) {
    for (uint32_t k = 0; k < kInnoMaxMultiReturn; k++, raw_point++) {
      if (raw_point->is_real(0) &&
          raw_point->get_radius(0) > config_.min_distance &&
          raw_point->get_radius(0) < config_.max_distance) {
        return false;
      }
    }
  }

  raw_block++;
  if (config_.roi_pattern > 0 && raw_block->header.in_roi == 0x01) {
    // in vertically roi, but horizontally non-roi region,
    // we skip every other block
    raw_block++;
  }
  raw_point = &raw_block->points[0][0];
  for (uint32_t channel = 0; channel < kInnoChannelNumber; channel++) {
    for (uint32_t k = 0; k < kInnoMaxMultiReturn; k++, raw_point++) {
      if (raw_point->is_real(1) &&
          raw_point->get_radius(1) > config_.min_distance &&
          raw_point->get_radius(1) < config_.max_distance) {
        return false;
      }
    }
  }

  return true;
}

void StageDeliver::stage_ts_(StageAngleJob *job) {
  stats_delivered_jobs_++;
  if (stats_delivered_jobs_ == 1) {
    // xxx todo: why the first STAGE_TIME_NA1/NB1 are 0
    return;
  }
  // if (stats_delivered_jobs_ % 100 == 0) {
    // inno_log_debug("!!!!!!!!!!!!");
  // }
  InnoEpSecondDouble sum = 0;
  for (uint32_t i = 3; i < ScanLines::STAGE_TIME_MAX; i++) {
    InnoEpSecondDouble diff = job->stage_ts[i] - job->stage_ts[i - 1];
    if (diff > 0) {
      sum += diff;
    }
    stats_stage_sum_ts_[i] += diff;
    stats_stage_sumq_ts_[i] += diff * diff;
    if (stats_stage_max_ts_[i] < diff) {
      stats_stage_max_ts_[i] = diff;
    }
    // if (stats_delivered_jobs_ % 100 == 0) {
      // inno_log_debug("%u: %p %.4f", i, job, job->stage_ts[i]);
    // }
  }
  stats_stage_sum_ts_[ScanLines::STAGE_TIME_MAX] += sum;
  stats_stage_sumq_ts_[ScanLines::STAGE_TIME_MAX] += sum * sum;
  if (stats_stage_max_ts_[ScanLines::STAGE_TIME_MAX] < sum) {
    stats_stage_max_ts_[ScanLines::STAGE_TIME_MAX] = sum;
  }
  sum_latency_.add(sum * 1000);  // ms
  if (stats_delivered_jobs_ % 1000 == 0) {  // 10s
    detect_latency_();
    sum_latency_.reset();
  }
}

const char *StageDeliver::get_name_(void) const {
  return lidar_->get_name();
}

int StageDeliver::detect_latency_() {
  enum InnoLidarMode current_mode_ = INNO_LIDAR_MODE_NONE;
  uint32_t current_duration_s = 0;
  double current_latency_high;
  enum InnoLidarMode pre_mode = INNO_LIDAR_MODE_NONE;
  enum InnoLidarStatus status = INNO_LIDAR_STATUS_NONE;

  if (lidar_->get_mode_status_fast(&current_mode_, &pre_mode,
                                 &status, &current_duration_s) != 0) {
    return -1;
  }
  if (status != INNO_LIDAR_STATUS_NORMAL) {
    return -1;
  }
  double time_diff_s = lidar_->get_monotonic_raw_time_ms() - init_ts_;
  time_diff_s /= 1000;
  if (((current_duration_s > lidar_->kStableTimeS) ||
       (current_duration_s == 0 && time_diff_s > lidar_->kStableTimeS)) &&
      ((current_mode_ == INNO_LIDAR_MODE_WORK_NORMAL)
      || (current_mode_ == INNO_LIDAR_MODE_WORK_SHORT_RANGE)
      || (current_mode_ == INNO_LIDAR_MODE_WORK_CALIBRATION))) {
    double u = sum_latency_.mean();
    double sigma = sum_latency_.std_dev();
    /*
     * P(u-sigma < M < u + sigma) = 68.26%
     * P(u-2*sigma < M < u + 2*sigma) = 95.45%
     *
     * About 68% of values drawn from a normal distribution are within
     * one standard deviation away from the mean; about 95% of the
     * values lie within two standard deviations; and about 99.7% are
     * within three standard deviations.[5] This fact is known as the
     * 68-95-99.7 (empirical) rule, or the 3-sigma rule.
     *
     * ref: https://en.wikipedia.org/wiki/Normal_distribution
     */
    current_latency_high = u + sigma;
    if (current_latency_high > config_.latency_high_threshold_ms) {
      lidar_->set_raw_fault(INNO_LIDAR_IN_FAULT_LATENCY_LONG, true,
                           "The latency is too long! u = %lf, sigma = %lf, "
                           "current_latency_high = %lf, "
                           "config_.latency_high_threshold_ms = %u", u, sigma,
                            current_latency_high,
                            config_.latency_high_threshold_ms);
    } else {
      lidar_->heal_raw_fault(INNO_LIDAR_IN_FAULT_LATENCY_LONG, true,
                            "INNO_LIDAR_IN_FAULT_LATENCY_LONG heals");
    }
    return 0;
  } else {
    return -1;
  }
}

}  // namespace innovusion
