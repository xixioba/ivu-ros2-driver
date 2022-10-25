/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "sdk/stage_angle.h"

#include "utils/consumer_producer.h"
#include "sdk/lidar.h"

namespace innovusion {
StageAngle::StageAngle(InnoLidar *l)
    : lidar_(l)
    , params_(l->get_params())
    , misc_tables_(&l->get_misc_tables())
    , pre_lines_(NULL)
    , pre_left_start_(0)
    , last_facet_(-1)
    , last_direction_(INNO_FRAME_DIRECTION_MAX)
    , last_angle_hori_(0)
    , last_angle_vert_(0)
    , last_galvo_value_(0)
    , auto_galvo_mode_(INNO_GALVO_MODE_NORMAL)
    , angle_lookup_(l)
    , cross_talk_(&l->get_misc_tables())
    , galvo_tracking_(l)
    , stats_frames_(0)
    , stats_facets_(0)
    , stats_drop_few_adc_(0)
    , stats_drop_few_polygon_(0)
    , stats_drop_few_galvo_(0)
    , stats_fov_filter_blocks_(0)
    , stats_polygon_too_long_(0)
    , stats_polygon_too_short_(0)
    , stats_polygon_ooo1_(0)
    , stats_polygon_ooo2_(0)
    , stats_no_copy_(0)
    , frame_idx_(-1)
    , line_idx_(-1)
    , scan_id_(-1)
    , last_frame_scan_id_(0) {
  memset(frame_total_ref_intensity_, 0, sizeof(frame_total_ref_intensity_));
  memset(frame_total_ref_cnt_, 0, sizeof(frame_total_ref_cnt_));
  memset(frame_total_points_, 0, sizeof(frame_total_points_));
  for (uint8_t chn_idx = 0; chn_idx < kInnoChannelNumber; chn_idx++) {
    lidar_->write_ps_reg(FRAME_REF_REG_BASE + 4 * chn_idx, 0);
  }
  fw_ipc_seq_begin_ = 0;
  fw_ipc_seq_end_ = 0;
  lidar_->add_config(&config_base_);
  config_.copy_from_src(&config_base_);
}

StageAngle::~StageAngle(void) {
  for (uint8_t chn_idx = 0; chn_idx < kInnoChannelNumber; chn_idx++) {
    lidar_->write_ps_reg(FRAME_REF_REG_BASE + 4 * chn_idx, 0);
  }
  lidar_->remove_config(&config_base_);
  if (pre_lines_ && pre_lines_->dec_ref()) {
    lidar_->free_angle_job(pre_lines_);
    pre_lines_ = NULL;
  }
}

StageAngle::GalvoTracking::GalvoTracking(InnoLidar *l)
    : lidar_(l)
    , misc_tables_(&l->get_misc_tables())
    , stats_total_strong_scatter_(0)
    , stats_strong_scatter_per_frame_(0)
    , stats_skip_bad_reference_frames_(0)
    , stats_unreliable_detection_frames_(0)
    , stats_big_error_ratio_frames_(0)
    , stats_strong_scatter_frames_(0)
    , stats_invalid_lines_frames_(0)
    , stats_total_detection_frames_(0)
    , stats_real_detection_frames_(0)
    , stats_total_test_frames_(0)
    , stats_invalid_test_frames_(0)
    , is_galvo_start_low_set_(false)
    , skip_galvo_tracking_(false)
    , save_scatter_cali_file_after_testing_(false) {
  memset(scatter_error_ratio_hist_, 0, sizeof(scatter_error_ratio_hist_));
  memset(galvo_min_angle_shift_hist_, 0, sizeof(galvo_min_angle_shift_hist_));
  memset(scatter_cali_sum_, 0, sizeof(scatter_cali_sum_));
  memset(scatter_cali_cnt_, 0, sizeof(scatter_cali_cnt_));
  memset(scatter_cali_table_, 0, sizeof(scatter_cali_table_));
  memset(scatter_normal_table_, 0, sizeof(scatter_normal_table_));

  low_angle_limit_0_[0] = -13.0 * kInnoAngleUnitPerDegree;
  low_angle_limit_1_[0] = -9.0 * kInnoAngleUnitPerDegree;
  up_angle_limit_[0] = 6.8 * kInnoAngleUnitPerDegree;
  low_angle_limit_0_[1] = -11.8 * kInnoAngleUnitPerDegree;
  low_angle_limit_1_[1] = -7.8 * kInnoAngleUnitPerDegree;
  up_angle_limit_[1] = 8.0 * kInnoAngleUnitPerDegree;
  low_angle_limit_0_[2] = -10.6 * kInnoAngleUnitPerDegree;
  low_angle_limit_1_[2] = -6.6 * kInnoAngleUnitPerDegree;
  up_angle_limit_[2] = 9.2 * kInnoAngleUnitPerDegree;
  low_angle_limit_0_[3] = -9.4 * kInnoAngleUnitPerDegree;
  low_angle_limit_1_[3] = -5.4 * kInnoAngleUnitPerDegree;
  up_angle_limit_[3] = 10.4 * kInnoAngleUnitPerDegree;

  // bottom one scanline version:
  // strong_scatter_angle_limit_[0] = -12.0 * kInnoAngleUnitPerDegree;
  // strong_scatter_angle_limit_[1] = -10.8 * kInnoAngleUnitPerDegree;
  // strong_scatter_angle_limit_[2] = -9.6 * kInnoAngleUnitPerDegree;
  // strong_scatter_angle_limit_[3] = -8.4 * kInnoAngleUnitPerDegree;

  // bottom two scanline version:
  strong_scatter_angle_limit_[0] = -11.0 * kInnoAngleUnitPerDegree;
  strong_scatter_angle_limit_[1] = -9.8 * kInnoAngleUnitPerDegree;
  strong_scatter_angle_limit_[2] = -8.6 * kInnoAngleUnitPerDegree;
  strong_scatter_angle_limit_[3] = -7.4 * kInnoAngleUnitPerDegree;

  check_scatter_cali_file_();
}

int StageAngle::GalvoTracking::check_scatter_cali_file_() {
  scatter_file_path_ = "";
  galvo_tracking_phase_ = GALVO_TRACKING_PHASE_NONE;

  if (!lidar_->is_live_lidar_()) {
    inno_log_info("disable galvo tracking in file play mode");
    return 0;
  }

  enum InnoLidarMode mode;
  enum InnoLidarMode pre_mode;
  enum InnoLidarStatus status;
  int ret = lidar_->get_mode_status(&mode, &pre_mode, &status);
  if (ret) {
    return ret;
  }
  if (mode != INNO_LIDAR_MODE_WORK_NORMAL &&
      mode != INNO_LIDAR_MODE_WORK_QUIET) {
    inno_log_warning("mode: %d, not normal mode or quiet mode, "
                     "no galvo tracking", mode);
    return -1;
  }

  if (lidar_->is_live_direct_memory_lidar_()) {
    scatter_file_path_ = "/mnt/pointcloud_log/scatter_cali_data";
  } else {
    if (!access("/tmp/", F_OK)) {
      scatter_file_path_ = "/tmp/scatter_cali_data";
    } else {
#ifndef __MINGW64__
      struct passwd pwd;
      char pwd_path[PATH_MAX + 1];
      struct passwd *pwd_result = NULL;
      uid_t uid = getuid();
      getpwuid_r(uid, &pwd,
               &pwd_path[0], sizeof(pwd_path), &pwd_result);
      if (pwd_result) {
        scatter_file_path_ = pwd_result->pw_dir;
#else
      {
      scatter_file_path_ = getenv("USERPROFILE");
#endif
        scatter_file_path_ += "/output";
        if (access(scatter_file_path_.c_str(), F_OK) != 0) {
#ifndef __MINGW64__
          if (mkdir(scatter_file_path_.c_str(), S_IRWXU) != 0) {
            inno_log_error("create %s failed", scatter_file_path_.c_str());
          }
#else
          if (::mkdir(scatter_file_path_.c_str()) != 0) {
            inno_log_error("create %s failed", scatter_file_path_.c_str());
          }
#endif
        }
        scatter_file_path_ += "/scatter_cali_data";
      }
    }
  }
  if (scatter_file_path_.empty()) {
    galvo_tracking_phase_ = GALVO_TRACKING_PHASE_NONE;
    inno_log_warning("empty scatter file path, set GALVO_TRACKING_PHASE_NONE");
  } else if (mode == INNO_LIDAR_MODE_WORK_NORMAL) {
    FILE *fp = fopen(scatter_file_path_.c_str(), "r");
    if (fp) {
      int channel, intensity;
      double angle;
      galvo_tracking_phase_ = GALVO_TRACKING_NORMAL_PHASE_3;
      for (uint32_t i = 0; i < kInnoChannelNumber &&
             galvo_tracking_phase_ == GALVO_TRACKING_NORMAL_PHASE_3; i++) {
        for (int j = 0; j < kScatterTableSize_; j++) {
          int r = fscanf(fp, "%d %lf %d",
                         &channel, &angle, &intensity);
          // inno_log_info("%d %lf %d", channel, angle, intensity);
          if (r != 3 || channel != (signed)i) {
            galvo_tracking_phase_ = GALVO_TRACKING_CALI_PHASE_1;
            inno_log_warning("incomplete scatter cali file, "
                             "set GALVO_TRACKING_CALI_PHASE_1");
            break;
          }
          scatter_cali_table_[i][j] = intensity;
        }
      }
      fclose(fp);

      if (galvo_tracking_phase_ == GALVO_TRACKING_NORMAL_PHASE_3) {
        if (lidar_->is_live_lidar_()) {
          // for live lidar, want to wait for apd cal to be done
          frame_idx_start_normal_ = kNormalWaitFrames_;
          galvo_tracking_phase_ = GALVO_TRACKING_NORMAL_PHASE_2;
          inno_log_info("live lidar, read scatter cali file, "
                        "set GALVO_TRACKING_NORMAL_PHASE_2");
        } else {
          inno_log_info("play file, read scatter cali file, "
                        "set GALVO_TRACKING_NORMAL_PHASE_3");
        }
      }
    } else {
      galvo_tracking_phase_ = GALVO_TRACKING_CALI_PHASE_1;
      inno_log_info("no scatter cali file, "
                    "set GALVO_TRACKING_CALI_PHASE_1");
    }
  } else if (mode == INNO_LIDAR_MODE_WORK_QUIET) {
      galvo_tracking_phase_ = GALVO_TRACKING_CALI_PHASE_1;
      inno_log_info("quiet mode, set GALVO_TRACKING_CALI_PHASE_1");
  }

  return 0;
}

void StageAngle::GalvoTracking::save_scatter_cali_file_() {
  FILE *fp = fopen(scatter_file_path_.c_str(), "w");
  if (fp) {
    for (uint32_t c = 0; c < kInnoChannelNumber; c++) {
      fprintf(fp, "%u %u\n", target_ref_intensity_[c], target_ref_power_[c]);
    }
    for (uint32_t c = 0; c < kInnoChannelNumber; c++) {
      for (int i = 0; i < kScatterTableSize_; i++) {
        int angle = kScatterMinAngle_ + (i << kScatterTableShift_);
        fprintf(fp, "%d %f %d\n", c, angle * kDegreePerInnoAngleUnit,
                scatter_cali_table_[c][i]);
      }
    }
    fclose(fp);
    inno_log_info("done saving scatter data to %s",
                  scatter_file_path_.c_str());
  }
}

inline void StageAngle::GalvoTracking::add_scatter_signal(RawBlock *block,
                                                          int16_t h_angle[],
                                                          int16_t v_angle[],
                                                          int scan_id) {
  for (uint32_t ich = 0; ich < kInnoChannelNumber; ich++) {
    int scat_intensity = block->scatter_intensity[ich];
    if (scat_intensity > 0) {
      if (h_angle[ich] > - kScatterAngleRange_ &&
          h_angle[ich] < kScatterAngleRange_) {
#ifndef GALVO_TRACKING_DIRECT_SCALING
          // scat_intensity represents intensity_seq
        if (scat_intensity <= kScatterMaxIntensitySeq_) {
          scatter_normal_table_[ich][scan_id].cnt++;
          scatter_normal_table_[ich][scan_id].vert_angle_sum += v_angle[ich];
          scatter_normal_table_[ich][scan_id].intensity_or_power_sum +=
            misc_tables_->convert_to_power_2(scat_intensity);
        } else {
          stats_total_strong_scatter_++;
          if (v_angle[ich] > strong_scatter_angle_limit_[ich]) {
            stats_strong_scatter_per_frame_++;
          }
        }
#else
        if (scat_intensity <= kScatterMaxIntensity_) {
          scatter_normal_table_[ich][scan_id].cnt++;
          scatter_normal_table_[ich][scan_id].vert_angle_sum += v_angle[ich];
          scatter_normal_table_[ich][scan_id].intensity_or_power_sum +=
            scat_intensity;
        } else {
          stats_total_strong_scatter_++;
          if (v_angle[ich] > strong_scatter_angle_limit_[ich]) {
            stats_strong_scatter_per_frame_++;
          }
        }
#endif
      }
    }
  }
}

inline void StageAngle::GalvoTracking::do_galvo_tracking_cali_phase_1_(\
  int64_t frame_idx) {
  if (lidar_->is_live_lidar_()) {
    enum InnoLidarMode pre_mode;
    enum InnoLidarStatus status;
    int ret = lidar_->set_mode(INNO_LIDAR_MODE_WORK_QUIET,
                               &pre_mode, &status);
    if (ret == 0) {
      frame_idx_start_scroll_ = frame_idx + kScrollWaitFrames_;
      galvo_tracking_phase_ = GALVO_TRACKING_CALI_PHASE_2;
      inno_log_info("frame idx: %" PRI_SIZED
        ", set GALVO_TRACKING_CALI_PHASE_2", frame_idx);
    } else {
      inno_log_warning("frame idx: %" PRI_SIZED
        ", remain GALVO_TRACKING_CALI_PHASE_1", frame_idx);
    }
  } else {
    frame_idx_start_cali_ = frame_idx + kScrollWaitFrames_ +
                                        kCaliWaitFrames_;
    galvo_tracking_phase_ = GALVO_TRACKING_CALI_PHASE_3;
    inno_log_info("frame idx: %" PRI_SIZED
      ", set GALVO_TRACKING_CALI_PHASE_3", frame_idx);
  }
}

inline void StageAngle::GalvoTracking::do_galvo_tracking_cali_phase_2_(\
  int64_t frame_idx) {
  if (frame_idx >= frame_idx_start_scroll_) {
    enum InnoLidarMode mode;
    enum InnoLidarMode pre_mode;
    enum InnoLidarStatus status;
    int ret = lidar_->get_mode_status(&mode, &pre_mode, &status);
    // check quiet mode transiton done
    if (ret == 0 && mode == INNO_LIDAR_MODE_WORK_QUIET &&
        status == INNO_LIDAR_STATUS_NORMAL) {
      ret = lidar_->set_galvo_sync(false);
      if (ret == 0) {
        frame_idx_start_cali_ = frame_idx + kCaliWaitFrames_;
        frame_idx_wait_apd_cal_ = frame_idx + kAPdCalWaitFrames_;
        galvo_tracking_phase_ = GALVO_TRACKING_CALI_PHASE_3;
        inno_log_info("frame idx: %" PRI_SIZED
                      ", set GALVO_TRACKING_CALI_PHASE_3",
                      frame_idx);
      } else {
        inno_log_warning("frame idx: %" PRI_SIZED ", "
                         "remain GALVO_TRACKING_CALI_PHASE_2",
                         frame_idx);
      }
    }
  }
}

inline void StageAngle::GalvoTracking::do_galvo_tracking_cali_phase_3_(\
  int64_t frame_idx,
  uint32_t frame_ave_ref_intensity[],
  uint32_t ref_sample_points[]) {
  if (frame_idx >= frame_idx_start_cali_) {
    int status = 0;
    int ret = lidar_->get_apd_cal_status(&status);
    bool done = false;
    if (ret == 0 && status == 1) {
      done = true;
      frame_idx_set_galvo_start_low_ = frame_idx + kCaliFrames_;
      frame_idx_galvo_start_low_transition_ =
        frame_idx + kCaliFrames_ + kCaliTransitionFrames_;
      frame_idx_stop_cali_ =
        frame_idx + kCaliFrames_ + kCaliTransitionFrames_ + kCaliFrames_;
      for (uint32_t i = 0; i < kInnoChannelNumber; i++) {
        if (ref_sample_points[i] < kCaliRefPointsThreshold_ ||
            frame_ave_ref_intensity[i] < kRefIntensityThreshold_) {
          inno_log_error("cali_phase_3: channel %u bad ref_sample_points %u or "
                         "ref_intensity %u, will skip galvo tracking",
                         i, ref_sample_points[i], frame_ave_ref_intensity[i]);
          skip_galvo_tracking_ = true;
        }
        target_ref_intensity_[i] = frame_ave_ref_intensity[i];
        target_ref_power_[i] =
          misc_tables_->intensity_to_power_2(target_ref_intensity_[i]);
      }
    } else if (frame_idx >= frame_idx_wait_apd_cal_) {
      done = true;
      inno_log_error("frame_idx: %" PRI_SIZED
        ", apd cal not completed", frame_idx);
      skip_galvo_tracking_ = true;
    }
    if (done) {
      if (skip_galvo_tracking_) {
        galvo_tracking_phase_ = GALVO_TRACKING_NORMAL_PHASE_1;
        inno_log_info("frame idx: %" PRI_SIZED
          ", set GALVO_TRACKING_NORMAL_PHASE_1", frame_idx);
      } else {
        galvo_tracking_phase_ = GALVO_TRACKING_CALI_PHASE_4;
        inno_log_info("frame idx: %" PRI_SIZED
          ", set GALVO_TRACKING_CALI_PHASE_4", frame_idx);
      }
    }
  }
}

inline void StageAngle::GalvoTracking::do_galvo_tracking_cali_phase_4_(\
  int64_t frame_idx,
  int max_id,
  uint32_t frame_ave_ref_intensity[],
  uint32_t ref_sample_points[]) {
  double power_scale[kInnoChannelNumber] = {1.0, 1.0, 1.0, 1.0};
  int valid_lines = 0;

  if (is_galvo_start_low_set_ &&
      frame_idx < frame_idx_galvo_start_low_transition_) {
    return;
  }

  for (uint32_t i = 0; i < kInnoChannelNumber; i++) {
    if (ref_sample_points[i] < kCaliRefPointsThreshold_ ||
        frame_ave_ref_intensity[i] < kRefIntensityThreshold_) {
      inno_log_error("cali_phase_4: channel %u bad ref_sample_points %u or "
                     "ref_intensity %u, will skip galvo tracking",
                     i, ref_sample_points[i], frame_ave_ref_intensity[i]);
      skip_galvo_tracking_ = true;
    }
  }

  if (stats_strong_scatter_per_frame_ > kMaxStrongScatterPerFrame_) {
    inno_log_error("cali_phase_4: strong scatter per frame %"
                   PRI_SIZELU " exceeds "
                   "max %u, will skip galvo tracking",
                   stats_strong_scatter_per_frame_,
                   kMaxStrongScatterPerFrame_);
    skip_galvo_tracking_ = true;
  }

  for (uint32_t i = 0; i < kInnoChannelNumber; i++) {
    uint32_t ref_power =
      misc_tables_->intensity_to_power_2(frame_ave_ref_intensity[i]);
    if (ref_power > 0) {
      power_scale[i] = 1.0 * target_ref_power_[i] / ref_power;
    }
    for (int j = 0; j < max_id; j++) {
      scatter_entry *sp = &scatter_normal_table_[i][j];
      if (sp->cnt > kScatterCaliMinCount_) {
        valid_lines++;
        int avg_vert_angle = sp->vert_angle_sum / sp->cnt;
        int index =
          (avg_vert_angle - kScatterMinAngle_) >> kScatterTableShift_;
        if (index < 0) {
          index = 0;
        } else if (index >= kScatterTableSize_) {
          index = kScatterTableSize_ - 1;
        }
#ifndef GALVO_TRACKING_DIRECT_SCALING
        uint32_t avg_power = sp->intensity_or_power_sum / sp->cnt;
        avg_power = static_cast<uint32_t>(avg_power * power_scale[i]);
        scatter_cali_sum_[i][index] +=
          misc_tables_->convert_to_intensity(avg_power);
#else
        scatter_cali_sum_[i][index] += (sp->intensity_or_power_sum << 8) /
          (sp->cnt * frame_ave_ref_intensity[i]);
        // scatter_cali_table_[i][index] = sp->intensity_or_power_sum / sp->cnt;
#endif
        scatter_cali_cnt_[i][index]++;
      }
    }
  }

  if (valid_lines < kCaliMinValidLines_) {
    inno_log_error("cali_phase_4: valid lines %d is less than "
                   "threshold %d, will skip galvo tracking",
                   valid_lines, kCaliMinValidLines_);
    skip_galvo_tracking_ = true;
  }

  if (frame_idx >= frame_idx_set_galvo_start_low_ &&
      !is_galvo_start_low_set_) {
    is_galvo_start_low_set_ = true;
    inno_log_info("cali_phase_4: set gavlo start position lower by 6 degrees");
    int ret = lidar_->set_galvo_start_low(true);
    if (ret != 0) {
      inno_log_error("cali_phase_4: set_galvo_start_low failed, "
                     "will skip galvo tracking");
      skip_galvo_tracking_ = true;
    }
  }

  if (skip_galvo_tracking_) {
    galvo_tracking_phase_ = GALVO_TRACKING_NORMAL_PHASE_1;
    inno_log_info("frame idx: %" PRI_SIZED
                  ", set GALVO_TRACKING_NORMAL_PHASE_1", frame_idx);
    return;
  }

  if (frame_idx >= frame_idx_stop_cali_) {
    // do moving average
    for (uint32_t c = 0; c < kInnoChannelNumber; c++) {
      int sum = 0;
      int cnt = 0;
      for (int i = 0; i < kScatterMoveAvgSize_; i++) {
        sum += scatter_cali_sum_[c][i];
        cnt += scatter_cali_cnt_[c][i];
      }
      for (int i = 0; i < kScatterTableSize_; i++) {
        int k = i + kScatterMoveAvgSize_;
        if (k < kScatterTableSize_) {
          sum += scatter_cali_sum_[c][k];
          cnt += scatter_cali_cnt_[c][k];
        }
        k = i - kScatterMoveAvgSize_ - 1;
        if (k >= 0) {
          sum -= scatter_cali_sum_[c][k];
          cnt -= scatter_cali_cnt_[c][k];
        }
        if (cnt > 0) {
          scatter_cali_table_[c][i] = sum / cnt;
        }
      }
    }

    // do interpolation in case there is no set value
    for (uint32_t c = 0; c < kInnoChannelNumber; c++) {
      int i_left = 0;
      for (int i = 0; i < kScatterTableSize_; i++) {
        if (scatter_cali_table_[c][i] == 0) {
          int j = i + 1;
          while (j < kScatterTableSize_ &&
                 scatter_cali_table_[c][j] == 0) {
            j++;
          }
          if (j == kScatterTableSize_) {
            if (i == 0) {
              inno_log_error("scatter_cali_table have all 0 values!");
            } else {
              for (int k = i; k < kScatterTableSize_; k++) {
                scatter_cali_table_[c][k] = scatter_cali_table_[c][i - 1];
              }
            }
            break;
          } else if (i == 0) {
            scatter_cali_table_[c][i] = scatter_cali_table_[c][j];
          } else {
            scatter_cali_table_[c][i] =
              ((j - i) * scatter_cali_table_[c][i_left] +
               (i - i_left) * scatter_cali_table_[c][j]) / (j - i_left);
          }
        } else {
          i_left = i;
        }
      }
    }

    if (lidar_->is_live_lidar_()) {
      save_scatter_cali_file_after_testing_ = true;
    } else {
      save_scatter_cali_file_();
    }

    galvo_tracking_phase_ = GALVO_TRACKING_NORMAL_PHASE_1;
    inno_log_info("frame idx: %" PRI_SIZED
                  ", set GALVO_TRACKING_NORMAL_PHASE_1", frame_idx);
  }
}

inline void StageAngle::GalvoTracking::do_galvo_tracking_normal_phase_1_(\
  int64_t frame_idx) {
  if (lidar_->is_live_lidar_()) {
    enum InnoLidarMode pre_mode;
    enum InnoLidarStatus status;
    int ret = lidar_->set_mode(INNO_LIDAR_MODE_WORK_NORMAL,
                               &pre_mode, &status);
    if (ret == 0) {
      frame_idx_start_normal_ = frame_idx + kQuietToNormalWaitFrames_;
      galvo_tracking_phase_ = GALVO_TRACKING_NORMAL_PHASE_2;
      inno_log_info("frame idx: %" PRI_SIZED
                    ", set GALVO_TRACKING_NORMAL_PHASE_2",
                    frame_idx);
    } else {
      inno_log_warning("frame idx: %" PRI_SIZED
                       ", remain GALVO_TRACKING_NORMAL_PHASE_1",
                       frame_idx);
    }
  } else {
    // playing file, directly jump to normal operation
    galvo_tracking_phase_ = GALVO_TRACKING_NORMAL_PHASE_3;
    inno_log_info("frame idx: %" PRI_SIZED
                  ", set GALVO_TRACKING_NORMAL_PHASE_3",
                  frame_idx);
  }
}

inline void StageAngle::GalvoTracking::do_galvo_tracking_normal_phase_2_(\
  int64_t frame_idx) {
  if (frame_idx >= frame_idx_start_normal_) {
    enum InnoLidarMode mode;
    enum InnoLidarMode pre_mode;
    enum InnoLidarStatus status;
    // xxx todo: check apd cal done
    // check normal mode transition done
    int ret = lidar_->get_mode_status(&mode, &pre_mode, &status);
    if (ret == 0 && mode == INNO_LIDAR_MODE_WORK_NORMAL &&
        status == INNO_LIDAR_STATUS_NORMAL) {
      galvo_tracking_phase_ = GALVO_TRACKING_NORMAL_PHASE_3;
      inno_log_info("frame idx: %" PRI_SIZED
                    ", set GALVO_TRACKING_NORMAL_PHASE_3",
                    frame_idx);
    }
  }
}

inline void StageAngle::GalvoTracking::do_galvo_tracking_normal_phase_3_(\
  int64_t frame_idx,
  int max_id,
  uint32_t frame_ave_ref_intensity[],
  uint32_t ref_sample_points[],
  int galvo_tracking_angle_threshold) {
  if (skip_galvo_tracking_) {
    return;
  }

  char scatter_buff[64*1024];
  int valid_lines = 0;
  bool record_scatter = lidar_->has_recorder_callback(\
    INNO_RECORDER_CALLBACK_TYPE_SCATTER);
  bool skip_this_time = false;
  for (uint32_t i = 0; i < kInnoChannelNumber; i++) {
    if (ref_sample_points[i] < kNormalRefPointsThreshold_ ||
        frame_ave_ref_intensity[i] < kRefIntensityThreshold_) {
      skip_this_time = true;
    }
  }

  if (skip_this_time) {
    stats_skip_bad_reference_frames_++;
    if (stats_skip_bad_reference_frames_ % 600 == 1) {
      inno_log_error("skip_bad_reference_frames: %" PRI_SIZELU,
                     stats_skip_bad_reference_frames_);
    }
    return;
  }

  double power_scale[kInnoChannelNumber];
  for (uint32_t i = 0; i < kInnoChannelNumber; i++) {
    power_scale[i] = 1.0;
    target_ref_power_[i] =
      misc_tables_->intensity_to_power_2(target_ref_intensity_[i]);
    uint32_t ref_power =
      misc_tables_->intensity_to_power_2(frame_ave_ref_intensity[i]);
    if (ref_power > 0) {
      power_scale[i] = 1.0 * target_ref_power_[i] / ref_power;
    }
#if 0
    inno_log_info("chan: %d target_ref_power: %d "
                  "ref_power: %d ref_intensity: %d",
                  i, target_ref_power_[i], ref_power,
                  frame_ave_ref_intensity[i]);
#endif
    for (int j = 0; j < max_id; j++) {
      scatter_entry *sp = &scatter_normal_table_[i][j];
      if (sp->cnt > 0) {
        sp->avg_vert_angle = sp->vert_angle_sum / sp->cnt;
#ifndef GALVO_TRACKING_DIRECT_SCALING
        sp->avg_power = sp->intensity_or_power_sum / sp->cnt;
#else
        sp->avg_power = (sp->intensity_or_power_sum << 8) /
          (sp->cnt * frame_ave_ref_intensity[i]);
        // sp->avg_intensity = sp->intensity_or_power_sum / sp->cnt;
#endif
      }
      if (sp->cnt > kScatterNormalMinCount_) {
        valid_lines++;
      }
    }
  }
#if 0
  inno_log_info("power_scale: %f %f %f %f",
                power_scale[0], power_scale[1],
                power_scale[2], power_scale[3]);
#endif

  size_t min_error = std::numeric_limits<std::size_t>::max();
  size_t min_error2 = std::numeric_limits<std::size_t>::max();
  int min_angle_shift = 0;
  int min_angle_shift2 = 0;
  int min_k = 0;
  double min_error_ratio = 2.0;
  // scale with ref intensity and not scale with ref_intensity
  double scatter_scale[2][4] = {{power_scale[0], power_scale[1],
                                power_scale[2], power_scale[3]},
                               {1.0, 1.0, 1.0, 1.0}};
  for (uint32_t k = 0; k < 2; k++) {
    for (uint32_t i = 0; i < kInnoChannelNumber; i++) {
      for (int j = 0; j < max_id; j++) {
        scatter_entry *sp = &scatter_normal_table_[i][j];
        if (sp->cnt > 0) {
#ifndef GALVO_TRACKING_DIRECT_SCALING
          sp->avg_intensity =
           misc_tables_->convert_to_intensity(\
             sp->avg_power * scatter_scale[k][i]);
#else
          sp->avg_intensity = sp->avg_power * scatter_scale[k][i];
#endif
          if (record_scatter) {
            char buff[1024];
            int n = snprintf(buff, sizeof(buff), "%" PRI_SIZED
                             " %d %f %d %d %d\n",
                             frame_idx, i,
                             sp->avg_vert_angle * kDegreePerInnoAngleUnit,
                             sp->avg_intensity, sp->cnt,
                             frame_ave_ref_intensity[i]);
            lidar_->do_recorder_callback(\
              INNO_RECORDER_CALLBACK_TYPE_SCATTER, buff, n);
          }
        }
      }
    }

    size_t min_error_t = std::numeric_limits<std::size_t>::max();
    size_t min_error2_t = std::numeric_limits<std::size_t>::max();
    int min_angle_shift_t = 0;
    int min_angle_shift2_t = 0;
    size_t prev_error = std::numeric_limits<std::size_t>::max();
    int prev_angle_shift = 0;
    bool go_down = false;
    for (int s = 0; s < kGalvoShiftSteps_; s++) {
      uint64_t error = 0;
      int angle_shift = kGalvoShiftMinAngle_ + (s << kGalvoBitsShift_);
      for (uint32_t i = 0; i < kInnoChannelNumber; i++) {
        for (int j = 0; j < max_id; j++) {
          scatter_entry *sp = &scatter_normal_table_[i][j];
          if (sp->cnt > kScatterNormalMinCount_ &&
              sp->avg_vert_angle < up_angle_limit_[i]) {
            int angle = sp->avg_vert_angle + angle_shift;
            int index = (angle - kScatterMinAngle_) >> kScatterTableShift_;
            if (index < 0) {
              index = 0;
            } else if (index >= kScatterTableSize_) {
              index = kScatterTableSize_ - 1;
            }
            int cali_intensity = scatter_cali_table_[i][index];
            int diff = cali_intensity - sp->avg_intensity;
            if  (sp->avg_vert_angle > low_angle_limit_0_[i]) {
              error += diff * diff;
            }
          }
        }
      }


      bool go_up = error > prev_error;
      if (go_down && go_up) {
        // local minimum
        if (prev_error < min_error_t) {
          // if the local minimum is too close to global minimum,
          // skip this local minimum
          int diff = prev_angle_shift - min_angle_shift_t;
          if (diff > kMinAngleDiffThreshold_) {
            min_error2_t = min_error_t;
            min_angle_shift2_t = min_angle_shift_t;
          }
          min_error_t = prev_error;
          min_angle_shift_t = prev_angle_shift;
        } else if (prev_error < min_error2_t) {
          int diff = prev_angle_shift - min_angle_shift_t;
          // if the local minimum is too close to global minimum,
          // skip this local minimum
          if (diff > kMinAngleDiffThreshold_) {
            min_error2_t = prev_error;
            min_angle_shift2_t = prev_angle_shift;
          }
        }
      }
      go_down = error < prev_error;
      prev_error = error;
      prev_angle_shift = angle_shift;
    }

    double error_ratio = 1.0 * min_error_t / min_error2_t;

#if 0
    inno_log_info("k: %d error_ratio: %f scale: %.2f/%.2f/%.2f/%.2f "
                  "error: %" PRI_SIZELU " %" PRI_SIZELU " angle: %f %f",
                  k, error_ratio,
                  scatter_scale[k][0], scatter_scale[k][1],
                  scatter_scale[k][2], scatter_scale[k][3],
                  min_error_t, min_error2_t,
                  min_angle_shift_t * 0.005493,
                  min_angle_shift2_t * 0.005493);
#endif

    if (error_ratio < min_error_ratio) {
      min_error_ratio = error_ratio;
      min_error = min_error_t;
      min_error2 = min_error2_t;
      min_angle_shift = min_angle_shift_t;
      min_angle_shift2 = min_angle_shift2_t;
      min_k = k;
#if 0
      // The current result is good enough
      if (min_error_ratio < 0.5) {
        break;
      }
#endif
    }
  }
  uint32_t error_ratio_index =
    static_cast<uint32_t>(min_error_ratio * kErrorRatioHistSize_);
  if (error_ratio_index >= kErrorRatioHistSize_) {
    error_ratio_index = kErrorRatioHistSize_ - 1;
  }
  scatter_error_ratio_hist_[error_ratio_index]++;

  uint32_t min_angle_shift_index =
    (min_angle_shift - kGalvoShiftMinAngle_) >> kGalvoBitsShift_;
  if (min_angle_shift_index >= kGalvoShiftSteps_) {
    min_angle_shift_index = kGalvoShiftSteps_ -1;
  }
  galvo_min_angle_shift_hist_[min_angle_shift_index]++;

  bool galvo_offset_detected = false;
  if (min_angle_shift < -galvo_tracking_angle_threshold ||
      min_angle_shift > galvo_tracking_angle_threshold) {
    galvo_offset_detected = true;
    stats_total_detection_frames_++;
  }

  if (save_scatter_cali_file_after_testing_) {
    stats_total_test_frames_++;
    if (min_error_ratio > kTestErrorRatioThreshold_ ||
        valid_lines < kNormalMinValidLines_ ||
        stats_strong_scatter_per_frame_ > kMaxStrongScatterPerFrame_ ||
        galvo_offset_detected) {
      stats_invalid_test_frames_++;
    }
    if (stats_total_test_frames_ >= kMaxTestFrames_) {
      double r = 1.0 * stats_invalid_test_frames_ /
                 stats_total_test_frames_;
      if (r < kMaxInvalidTestFramesRatio_) {
        save_scatter_cali_file_();
      } else {
        skip_galvo_tracking_ = true;
        inno_log_warning("galvo tracking test using scatter failed, "
                         "won't save scatter cali file, "
                         "total_test_frames: %" PRI_SIZELU
                         "invalid_test_frames: %" PRI_SIZELU
                         "strong_scatter_frames: %" PRI_SIZELU
                         "invalid_lines_frames: %" PRI_SIZELU,
                         stats_total_test_frames_,
                         stats_invalid_test_frames_,
                         stats_strong_scatter_frames_,
                         stats_invalid_lines_frames_);
      }
      save_scatter_cali_file_after_testing_ = false;
    }
  }

  if (frame_idx % 600 == 0) {
    char buff[16*1024];
    int used = 0;
    for (int i = 0; i < kErrorRatioHistSize_; i++) {
      int r = snprintf(buff + used, sizeof(buff) - used,
                       "%d: %f ", i,
                       1.0 * scatter_error_ratio_hist_[i] / frame_idx);
      used += r;
    }
    inno_log_info("frame: %" PRI_SIZED
                  " galvo error ratio histogram %s",
                  frame_idx, buff);

    used = 0;
    for (int i = 0; i < kGalvoShiftSteps_; i++) {
      if (galvo_min_angle_shift_hist_[i] != 0) {
        int r = snprintf(buff + used, sizeof(buff) - used,
                         "%.3f: %.3f ",
                         (kGalvoShiftMinAngle_ + (i << kGalvoBitsShift_)) *
                         kDegreePerInnoAngleUnit ,
                         1.0 * galvo_min_angle_shift_hist_[i] / frame_idx);
        used += r;
      }
    }
    inno_log_info("frame: %" PRI_SIZED
                  " galvo min angle shift histogram %s",
                  frame_idx, buff);
    inno_log_info("galvo_offset_tracking "
                  "frame: %" PRI_SIZED " min_angle_shift: %.2f delta: %"
                  PRI_SIZELU
                  "min_angle_shift2: %.2f delta2: %" PRI_SIZELU
                  "min_k: %d "
                  "delta_ratio: %.3f valid_lines: %d "
                  "strong_scatter_per_frame: %" PRI_SIZELU
                  "total_strong_scatter: %" PRI_SIZELU
                  "skip_bad_reference: %" PRI_SIZELU
                  "unreliable: %" PRI_SIZELU
                  "big_error_ratio: %" PRI_SIZELU
                  "strong_scatter_frames: %" PRI_SIZELU
                  "invalid_lines_frames: %" PRI_SIZELU
                  "total_detection: %" PRI_SIZELU
                  "real_detection: %" PRI_SIZELU
                  "ref_intensity %u %u %u %u",
                  frame_idx,
                  min_angle_shift * kDegreePerInnoAngleUnit,
                  min_error,
                  min_angle_shift2 * kDegreePerInnoAngleUnit,
                  min_error2,
                  min_k, min_error_ratio,
                  valid_lines,
                  stats_strong_scatter_per_frame_,
                  stats_total_strong_scatter_,
                  stats_skip_bad_reference_frames_,
                  stats_unreliable_detection_frames_,
                  stats_big_error_ratio_frames_,
                  stats_strong_scatter_frames_,
                  stats_invalid_lines_frames_,
                  stats_total_detection_frames_,
                  stats_real_detection_frames_,
                  frame_ave_ref_intensity[0],
                  frame_ave_ref_intensity[1],
                  frame_ave_ref_intensity[2],
                  frame_ave_ref_intensity[3]);
  }

  // the detection is not reliable enough
  if (min_error_ratio > kErrorRatioThreshold_ ||
    valid_lines < kNormalMinValidLines_ ||
    stats_strong_scatter_per_frame_ > kMaxStrongScatterPerFrame_) {
    stats_unreliable_detection_frames_++;
    if (min_error_ratio > kErrorRatioThreshold_) {
      stats_big_error_ratio_frames_++;
    }
    if (valid_lines < kNormalMinValidLines_) {
      stats_invalid_lines_frames_++;
    }
    if (stats_strong_scatter_per_frame_ > kMaxStrongScatterPerFrame_) {
      stats_strong_scatter_frames_++;
    }
  } else if (galvo_offset_detected) {
    stats_real_detection_frames_++;
    int len = 0;
    scatter_buff[0] = 0;
    for (uint32_t i = 0; i < kInnoChannelNumber; i++) {
      for (int j = 0; j < max_id; j++) {
        scatter_entry *sp = &scatter_normal_table_[i][j];
        sp->avg_intensity = misc_tables_->convert_to_intensity(\
          sp->avg_power * scatter_scale[min_k][i]);
        int r = snprintf(scatter_buff + len, sizeof(scatter_buff) - len,
                         "%d %f %d %d ",
                         i, sp->avg_vert_angle * kDegreePerInnoAngleUnit,
                         sp->avg_intensity, sp->cnt);
        len += r;
      }
    }
    lidar_->set_raw_fault(INNO_LIDAR_IN_FAULT_GALVO_OFFSET, true,
                          "galvo_offset_tracking [FAULT] "
                          "frame: %" PRI_SIZED
                          " min_angle_shift: %.2f delta: %" PRI_SIZELU
                          "min_angle_shift2: %.2f delta2: %"
                          PRI_SIZELU " min_k: %d "
                          "delta_ratio: %.3f valid_lines: %d "
                          "strong_scatter_per_frame: %" PRI_SIZELU
                          "total_strong_scatter: %" PRI_SIZELU
                          "skip_bad_reference: %" PRI_SIZELU
                          "unreliable: %" PRI_SIZELU
                          "big_error_ratio: %" PRI_SIZELU
                          "strong_scatter_frames: %" PRI_SIZELU
                          "invalid_lines_frames: %" PRI_SIZELU
                          "total_detection: %" PRI_SIZELU
                          "real_detection: %" PRI_SIZELU
                          "ref_intensity %u %u %u %u scatter %s",
                          frame_idx,
                          min_angle_shift * kDegreePerInnoAngleUnit,
                          min_error,
                          min_angle_shift2 * kDegreePerInnoAngleUnit,
                          min_error2,
                          min_k, min_error_ratio,
                          valid_lines,
                          stats_strong_scatter_per_frame_,
                          stats_total_strong_scatter_,
                          stats_skip_bad_reference_frames_,
                          stats_unreliable_detection_frames_,
                          stats_big_error_ratio_frames_,
                          stats_strong_scatter_frames_,
                          stats_invalid_lines_frames_,
                          stats_total_detection_frames_,
                          stats_real_detection_frames_,
                          frame_ave_ref_intensity[0],
                          frame_ave_ref_intensity[1],
                          frame_ave_ref_intensity[2],
                          frame_ave_ref_intensity[3],
                          scatter_buff);
  } else {
    lidar_->heal_raw_fault(INNO_LIDAR_IN_FAULT_GALVO_OFFSET, true,
                          "INNO_LIDAR_IN_FAULT_GALVO_OFFSET heals");
  }
}

void StageAngle::GalvoTracking::do_galvo_tracking(\
  int64_t frame_idx, int max_scan_id,
  uint32_t frame_ave_ref_intensity[],
  uint32_t ref_sample_points[],
  int galvo_tracking_angle_threshold) {
  int max_id = max_scan_id < kMaxScanLines_ ? max_scan_id : kMaxScanLines_;

  if (galvo_tracking_phase_ == GALVO_TRACKING_NORMAL_PHASE_3) {
    do_galvo_tracking_normal_phase_3_(frame_idx,
                                      max_id,
                                      frame_ave_ref_intensity,
                                      ref_sample_points,
                                      galvo_tracking_angle_threshold);
  } else if (galvo_tracking_phase_ == GALVO_TRACKING_CALI_PHASE_1) {
    do_galvo_tracking_cali_phase_1_(frame_idx);
  } else if (galvo_tracking_phase_ == GALVO_TRACKING_CALI_PHASE_2) {
    do_galvo_tracking_cali_phase_2_(frame_idx);
  } else if (galvo_tracking_phase_ == GALVO_TRACKING_CALI_PHASE_3) {
    do_galvo_tracking_cali_phase_3_(frame_idx,
                                    frame_ave_ref_intensity,
                                    ref_sample_points);
  } else if (galvo_tracking_phase_ == GALVO_TRACKING_CALI_PHASE_4) {
    do_galvo_tracking_cali_phase_4_(frame_idx,
                                    max_id,
                                    frame_ave_ref_intensity,
                                    ref_sample_points);
  } else if (galvo_tracking_phase_ == GALVO_TRACKING_NORMAL_PHASE_1) {
    do_galvo_tracking_normal_phase_1_(frame_idx);
  } else if (galvo_tracking_phase_ == GALVO_TRACKING_NORMAL_PHASE_2) {
    do_galvo_tracking_normal_phase_2_(frame_idx);
  }

  memset(scatter_normal_table_, 0, sizeof(scatter_normal_table_));
  stats_strong_scatter_per_frame_ = 0;
}

void StageAngle::init_lookup_table() {
    angle_lookup_.init_table();
}

int StageAngle::process(void *in_job, void *ctx,
                        bool prefer) {
  StageAngle *s = reinterpret_cast<StageAngle *>(ctx);
#ifdef __APPLE__
  INNER_BEGIN_LOG(StageAngle_process, OS_LOG_CATEGORY_DYNAMIC_TRACING,
                  StageAngle_process);
#endif
  int ret = s->process_job_(reinterpret_cast<StageAngleJob *>(in_job), prefer);
#ifdef __APPLE__
  INNER_END_LOG(StageAngle_process);
#endif
  return ret;
}

inline void StageAngle::prepare_polygon_whole_(const EncoderSignal *polygon_1st,
                                               const EncoderSignal *polygon_2nd,
                                               InnoFpgaSubNs *whole,
                                               int64_t *whole_value) {
  *whole = polygon_2nd->ts_sub_ns - polygon_1st->ts_sub_ns;
  inno_log_verify(*whole != 0,
                  "p whole should not be 0 %" PRI_SIZED,
                  polygon_2nd->ts_sub_ns);
  if (polygon_1st->ts_sub_ns != 0) {
    if (*whole > (InnoFpgaSubNs)
       (InnoConsts::kMaxPolygonPeriodSubNs /
       lidar_->get_encodes_per_polygon())) {
      stats_polygon_too_long_++;
      inno_log_warning("polygon period %" PRI_SIZED
                       " ns is too big %" PRI_SIZELU,
                       *whole >> InnoConsts::kSubNsBits,
                       stats_polygon_too_long_);
    }
    if (*whole < (InnoFpgaSubNs)
       (InnoConsts::kMinPolygonPeriodSubNs /
       lidar_->get_encodes_per_polygon())) {
      stats_polygon_too_short_++;
      inno_log_warning("polygon period %" PRI_SIZED
                       " ns is too small %" PRI_SIZELU,
                       *whole >> InnoConsts::kSubNsBits,
                       stats_polygon_too_short_);
    }
  }
  // 2Pi between 2 polygons encoder
  int32_t interval_angle = polygon_2nd->angle - polygon_1st->angle;
  *whole_value = interval_angle > 0 ? interval_angle :
                 (2 * kInnoAngleUnitPerPiRad + interval_angle);
  return;
}

inline void StageAngle::prepare_galvo_whole_(const EncoderSignal *galvo_1st,
                                             const EncoderSignal *galvo_2nd,
                                             InnoFpgaSubNs *whole,
                                             int64_t *whole_value) {
  *whole = galvo_2nd->ts_sub_ns - galvo_1st->ts_sub_ns;
  inno_log_verify(*whole != 0,
                  "g whole should not be 0 %" PRI_SIZED,
                  galvo_2nd->ts_sub_ns);
  *whole_value = galvo_2nd->angle - galvo_1st->angle;
  return;
}

inline void StageAngle::add_new_line_(StageAngleJob *cur_lines,
                                      uint32_t block_start,
                                      uint32_t block_so_far,
                                      bool end_of_frame,
                                      int32_t galvo_angle_change) {
  if (block_so_far == 0) {
    return;
  }
  ScanLine &line = cur_lines->allocate_line();
  line.frame_idx = frame_idx_;
  if (cur_lines->lines_number > 1 &&
      cur_lines->lines[cur_lines->lines_number-2].frame_idx != line.frame_idx) {
    // we may detect a frame boundary but may not include the final partial
    // line in this scanline group, so need to increment frame count here
    cur_lines->frame_count++;;
  }
  line.idx = line_idx_;
  line.facet_id = last_facet_;
  line.active_blocks_start =
      cur_lines->get_active_block_storage_idx(block_start);
  line.active_blocks_end = line.active_blocks_start + block_so_far;
  line.active_blocks_number = block_so_far;
  line.galvo_angle_change = galvo_angle_change;

  line.direction = last_direction_;
  line.end_of_frame = end_of_frame;

  // xxx todo error handling
  line.has_problematic_blocks = false;
  return;
}

int StageAngle::process_job_(StageAngleJob *cur_lines,
                             bool prefer) {
  inno_log_verify(prefer, "stage_angle job not prefer");
  inno_log_assert(cur_lines, "job");

  bool need_to_free = false;

  {
    std::unique_lock<std::mutex> lk(lidar_->params_mutex_);
    if (angle_lookup_.get_version() != params_.get_version()) {
      init_lookup_table();
    }
  }

  config_.copy_from_src(&config_base_);

  cur_lines->stage_ts[ScanLines::STAGE_TIME_A0] =
      lidar_->get_monotonic_raw_time();

  bool cal_mode = cur_lines->in_calibration_mode();

  if (cal_mode) {
    fov_up_ = config_.fov_up_cal;
    fov_bottom_ = config_.fov_bottom_cal;
    fov_right_ = config_.fov_right_cal;
    fov_left_ = config_.fov_left_cal;
  } else {
    fov_up_ = config_.fov_top_high_angle;
    fov_bottom_ = config_.fov_top_low_angle;
    fov_right_ = config_.fov_top_right_angle;
    fov_left_ = config_.fov_top_left_angle;
  }
  fov_top_half_p_angle_ = params_.iv_params.fov_top_half_p_angle *
                          kInnoAngleUnitPerDegree;
  fov_bottom_half_p_angle_ = params_.iv_params.fov_bottom_half_p_angle *
                             kInnoAngleUnitPerDegree;
  fov_top_half_p_angle_linear_coeff_ =
                          (params_.iv_params.fov_bottom_half_p_angle -
                           params_.iv_params.fov_top_half_p_angle) / 5.0;
  cross_talk_.set_params(params_.iv_params.retro_intensity,
                         config_.cross_talk_distance1,
                         config_.cross_talk_distance2,
                         &params_.iv_params.ctr[0]);

  if (cur_lines->active_blocks_number() == 0) {
    need_to_free = true;
    stats_drop_few_adc_++;
  } else if (cur_lines->polygons_number() < 2) {
    need_to_free = true;
    stats_drop_few_polygon_++;
  } else if (cur_lines->galvos_number() < 2) {
    need_to_free = true;
    stats_drop_few_galvo_++;
  }

  if (need_to_free) {
    inno_log_verify(cur_lines->dec_ref(), "cannot free");
    lidar_->free_angle_job(cur_lines);
    return 0;
  }

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
  int16_t v_angle_offset[kInnoChannelNumber];
  for (uint32_t ich = 0; ich < kInnoChannelNumber; ich++) {
    v_angle_offset[ich] = ich * kInnoVAngleDiffBase;

    galvo_angle_threshold_[ich] = static_cast<int32_t>
      ((params_.iv_params.f_gamma[ich] / 2 + 90 -
        StageAngleLookup::kPolygonSurfaceAngle) /
       kDegreePerInnoAngleUnit);
  }

  int copied = process_job_copy_pre_(cur_lines);

  int32_t polygon_idx = 0;
  int32_t galvo_idx = 0;

  EncoderSignal *polygon_1st = cur_lines->iterate_polygons(&polygon_idx);
  inno_log_verify(polygon_1st, "polygon_1st");
  EncoderSignal *polygon_2nd = cur_lines->iterate_polygons(&polygon_idx);
  inno_log_verify(polygon_2nd, "polygon_2nd");
  EncoderSignal *galvo_1st = cur_lines->iterate_galvos(&galvo_idx);
  inno_log_verify(galvo_1st, "galvo_1st");
  EncoderSignal *galvo_2nd = cur_lines->iterate_galvos(&galvo_idx);
  inno_log_verify(galvo_2nd, "galvo_2nd");

  InnoFpgaSubNs polygon_whole, galvo_whole;
  int64_t polygon_whole_value, galvo_whole_value;
  InnoFrameDirection direction = galvo_1st->up_scan;

  prepare_polygon_whole_(polygon_1st, polygon_2nd,
                         &polygon_whole, &polygon_whole_value);
  prepare_galvo_whole_(galvo_1st, galvo_2nd,
                       &galvo_whole, &galvo_whole_value);

  const RawBlock *last_block = &cur_lines->get_last_active_block_const();

  uint32_t block_start = 0;
  uint32_t block_so_far = copied;
  RawBlock *block = &cur_lines->get_active_block(copied);
  for (uint8_t chn_idx = 0; chn_idx < kInnoChannelNumber; chn_idx++) {
    frame_total_points_[direction][chn_idx] +=
                        cur_lines->get_frame_points(chn_idx, direction);
  }

  for (;
       block <= last_block;
       block++, block_so_far++) {
    InnoFpgaSubNs block_sub_ns =
        InnoConverts::ns_to_sub_ns(block->trigger_ts_ns);

    // prepare encoders
    while (polygon_2nd->ts_sub_ns < block_sub_ns) {
      bool reach_end = false;
      EncoderSignal *polygon_next = cur_lines->iterate_polygons(&polygon_idx);
      if (polygon_next == NULL) {
        // must have out-of-order
        stats_polygon_ooo1_++;
        if (stats_polygon_ooo1_ % 1000 == 1) {
          inno_log_error("out-of-order polygon_2nd, %u, %" PRI_SIZELU,
                       cur_lines->polygons_number(),
                       stats_polygon_ooo1_);
        }
        // inno_log_verify(polygon_2nd, "polygon_2nd, %d",
        //                 cur_lines->polygons_number());
        reach_end = true;
      } else {
        polygon_1st = polygon_2nd;
        polygon_2nd = polygon_next;
      }
      prepare_polygon_whole_(polygon_1st, polygon_2nd,
                             &polygon_whole, &polygon_whole_value);
      if (reach_end) {
        break;
      }
    }
    while (galvo_2nd->ts_sub_ns < block_sub_ns) {
      bool reach_end = false;
      EncoderSignal *galvo_next = cur_lines->iterate_galvos(&galvo_idx);
      if (galvo_next == NULL) {
        // must have out-of-order
        stats_polygon_ooo2_++;
        if (stats_polygon_ooo2_ % 1000 == 1) {
          inno_log_error("out-of-order galvo_2nd, %u %" PRI_SIZELU,
                       cur_lines->galvos_number(),
                       stats_polygon_ooo2_);
        }
        // inno_log_verify(galvo_2nd, "galvo_2nd, %d",
        //                cur_lines->galvos_number());
        reach_end = true;
      } else {
        galvo_1st = galvo_2nd;
        galvo_2nd = galvo_next;
      }
      prepare_galvo_whole_(galvo_1st, galvo_2nd,
                           &galvo_whole, &galvo_whole_value);
      direction = galvo_1st->up_scan;
      if (reach_end) {
        break;
      }
    }

    InnoFpgaSubNs partial;
    int64_t partial_value;

    // Interpolate polygon
    partial = block_sub_ns - polygon_1st->ts_sub_ns;
    partial_value = polygon_whole_value * partial / polygon_whole
                    + polygon_1st->angle;
    int32_t polygon_value = partial_value;
    if (polygon_value < 0) {
      // it could overflow when polygon_1st->ts_sub_ns is 0 and
      // block_sub_ns is a very big number
      polygon_value = 0;
    }
    // inno_log_verify(polygon_value >= 0,
    //                "bad polygon_value %ld %ld %ld", block_sub_ns,
    //                polygon_1st->ts_sub_ns, polygon_whole_value);

    // Interpolate galvo
    partial = block_sub_ns - galvo_1st->ts_sub_ns;
    partial_value = galvo_whole_value * partial / galvo_whole;
    int32_t galvo_value = partial_value + galvo_1st->angle;

    // table lookup to get the angle
    int16_t h_angle[kInnoChannelNumber];
    int16_t v_angle[kInnoChannelNumber];
    int32_t facet;
    int32_t polygon_mod;

    // inno_log_debug("p=%u g=%u", polygon_value, galvo_value);
    angle_lookup_.map_to_angles(direction, polygon_value, galvo_value,
                                h_angle, v_angle, kInnoChannelNumber,
                                &facet, &polygon_mod);
    inno_log_assert(kInnoChannelNumber == 4,
                    "kInnoChannelNumber %u", kInnoChannelNumber);
    block->poly_angle = polygon_mod;
    block->galvo_angle = galvo_value;
    block->header.facet = facet;
    block->header.h_angle = h_angle[0];
    block->header.h_angle_diff_1 = h_angle[1];
    block->header.h_angle_diff_2 = h_angle[2];
    block->header.h_angle_diff_3 = h_angle[3];
    block->header.v_angle = v_angle[0];
    block->header.v_angle_diff_1 = v_angle[1];
    block->header.v_angle_diff_2 = v_angle[2];
    block->header.v_angle_diff_3 = v_angle[3];

    for (uint32_t ich = 1; ich < kInnoChannelNumber; ich++) {
      h_angle[ich] += h_angle[0];
      v_angle[ich] += v_angle[0] + v_angle_offset[ich];
    }

    if (config_.enable_galvo_tracking) {
      galvo_tracking_.add_scatter_signal(block, h_angle, v_angle, scan_id_);
    }

    for (uint32_t i = 0; i < kInnoChannelNumber; i++) {
      int32_t p_angle_limit = fov_bottom_half_p_angle_;
      if (galvo_value < galvo_angle_threshold_[i]) {
        p_angle_limit -= fov_top_half_p_angle_linear_coeff_ *
          (galvo_angle_threshold_[i] - galvo_value);
      }
      // FOV filter, we have to do 4 if
      if (h_angle[i] < fov_left_ || h_angle[i] > fov_right_ ||
          v_angle[i] < fov_bottom_ || v_angle[i] > fov_up_ ||
          polygon_mod < -p_angle_limit || polygon_mod > p_angle_limit) {
        RawChannelPoint *pp = &block->points[i][0];
        for (uint32_t j = 0; j < kInnoMaxMultiReturn; j++, pp++) {
          pp->set_radius(0, 0);
          pp->set_radius(1, 0);
        }
        stats_fov_filter_blocks_++;
      }
    }

    if (config_.remove_cross_talk) {
      cross_talk_.remove_cross_talk(block);
    }

    // xxx todo: make new_line and new frame detection more robust
    bool new_line = false;
#ifdef SEP_NEW_FRAME_TO_TWO_LINES
    if (direction != last_direction_) {
      last_direction_ = direction;
      frame_idx_++;
      stats_frames_++;
      new_line = true;
      line_idx_++;
    } else if (facet != last_facet_) {
      new_line = true;
      line_idx_++;
    }
#else
    if (facet != last_facet_) {
      new_line = true;
      line_idx_++;
    }
#endif

    last_angle_hori_ = h_angle[0];
    last_angle_vert_ = v_angle[0];

    if (galvo_value > config_.filt_intensity_galvo_angle) {  // angle filter
      for (uint8_t chn_idx = 0; chn_idx < kInnoChannelNumber; chn_idx++) {
        frame_total_ref_intensity_[block->header.in_roi][chn_idx] +=
                                                 block->ref_intensity[chn_idx];
        if (block->ref_intensity[chn_idx] > 0) {
          frame_total_ref_cnt_[block->header.in_roi][chn_idx]++;
        }
      }
    }

    if (new_line) {
      bool end_of_frame = direction != last_direction_;
      add_new_line_(cur_lines, block_start, block_so_far, end_of_frame,
                    galvo_value - last_galvo_value_);
      last_galvo_value_ = galvo_value;
      block_start += block_so_far;
      block_so_far = 0;
      scan_id_++;
      last_facet_ = facet;
      stats_facets_++;
      if (stats_facets_ % 15 == 0) {
        send_ref_time_to_fw(last_block);
      }
#ifndef SEP_NEW_FRAME_TO_TWO_LINES
      // only change the frame after a new line
      if (end_of_frame) {
        // auto detect galvo mode
        double ratio = (last_frame_scan_id_ + 0.1) / (scan_id_ + 0.1);
        last_frame_scan_id_ = scan_id_;
        if (ratio > 2 || ratio < 0.5) {
          auto_galvo_mode_ = INNO_GALVO_MODE_FLYBACK;
        } else {
          auto_galvo_mode_ = INNO_GALVO_MODE_NORMAL;
        }

        uint32_t frame_ave_ref_intensity[kInnoChannelNumber] = {0};
        uint32_t ref_sample_points[kInnoChannelNumber] = {0};
        char apd_string[200];
        int apd_len = 0;

        for (uint8_t chn_idx = 0; chn_idx < kInnoChannelNumber; chn_idx++) {
          uint32_t combined_data_low = 0;
          uint32_t combined_data_high = 0;
          uint32_t combined_data_reg = 0;
          int64_t combined_data_ipc = 0;
          size_t sample_points = frame_total_ref_cnt_[0][chn_idx] +
                        frame_total_ref_cnt_[1][chn_idx];
          ref_sample_points[chn_idx] = sample_points;
          if (sample_points <= 0) {
            frame_ave_ref_intensity[chn_idx] = 0;
          } else {
            frame_ave_ref_intensity[chn_idx] =
                          (frame_total_ref_intensity_[0][chn_idx] +
                          frame_total_ref_intensity_[1][chn_idx]) /
                          sample_points;
          }

          // combined_data_reg[31:16] means pt number,
          // combined_data_reg[15:0] means frame average intensity
          combined_data_low = frame_ave_ref_intensity[chn_idx] > 65535 ?
                              65535 : frame_ave_ref_intensity[chn_idx];
          combined_data_high = frame_total_points_[last_direction_][chn_idx] >
                               65535 ? 65535 : frame_total_points_\
                               [last_direction_][chn_idx];
          combined_data_reg = (combined_data_high << 16) | combined_data_low;

          // combined_data_ipc[63:32] means pt number,
          // combined_data_ipc[31:0] means frame average intensity
          combined_data_low = frame_ave_ref_intensity[chn_idx];
          combined_data_high = frame_total_points_[last_direction_][chn_idx] >
                               kPtNumberMax ? kPtNumberMax :
                               frame_total_points_[last_direction_][chn_idx];
          combined_data_ipc = combined_data_high;
          combined_data_ipc = (combined_data_ipc << 32) | combined_data_low;

          if (lidar_->is_live_direct_memory_lidar_()) {
            lidar_->write_ps_reg(FRAME_REF_REG_BASE + 4 * chn_idx,
                                 combined_data_reg);

            lidar_->get_fw_ipc()->write((IpcItem)                       \
                                        (IPC_ITEM_PT_INTENSITY_CH0 + chn_idx),
                                        combined_data_ipc);
          } else if (lidar_->is_live_lidar_()) {
            int a = snprintf(apd_string + apd_len, sizeof(apd_string) - apd_len,
                             " 0x%" PRI_SIZEX, combined_data_ipc);
            inno_log_verify((unsigned int)(a) + apd_len < sizeof(apd_string),
                            "%d", a + apd_len);
            apd_len += a;
          }
          frame_total_points_[direction][chn_idx] +=
                              cur_lines->get_frame_points(chn_idx, direction);
          frame_total_points_[last_direction_][chn_idx] = 0;
        }

        if (apd_len > 0) {
          inno_log_verify(lidar_->is_live_lidar_() &&
                          !lidar_->is_live_direct_memory_lidar_(),
                          "impossible");
          uint64_t now = InnoUtils::get_time_ms(CLOCK_MONOTONIC_RAW);
          const uint64_t kMarginMs = 50;
          if (apd_report_last_ms_ == 0 ||
              now + kMarginMs >=
              apd_report_last_ms_ + lidar_->apd_report_interval_ms_) {
            StageHelpJob *job = new StageHelpJob(HELPER_JOB_TYPE_REPORT_APD,
                                                 std::string(apd_string),
                                                 now);
            inno_log_verify(job, "job");
            lidar_->cp_help_->add_job(job);
            apd_report_last_ms_ = now;
          }
        }

        if (config_.enable_galvo_tracking &&
            auto_galvo_mode_ == INNO_GALVO_MODE_NORMAL) {  // falcon I only
          galvo_tracking_.do_galvo_tracking(frame_idx_, scan_id_,
                                        frame_ave_ref_intensity,
                                        ref_sample_points,
                                        config_.galvo_tracking_angle_threshold);
        }

        scan_id_ = 0;
        frame_idx_++;
        stats_frames_++;
        last_direction_ = direction;
        reset_ref_cnt_and_intensity();
      }
#endif
    }

    block->header.scan_id = scan_id_;
    block->header.scan_idx = block_so_far;
  }

  pre_left_start_ = block_start;
  pre_lines_ = cur_lines;

  // now we can enqueue scanlines to the next stage
  cur_lines->set_auto_galvo_mode(auto_galvo_mode_);
  cur_lines->inc_ref();
  cur_lines->stage_ts[ScanLines::STAGE_TIME_A1] =
      lidar_->get_monotonic_raw_time();
  lidar_->add_stage_noise_filter_phase0_job(cur_lines);

  return 0;
}

const char *StageAngle::get_name_(void) const {
  return lidar_->get_name();
}

int32_t StageAngle::process_job_copy_pre_(StageAngleJob *job) {
  if (pre_lines_ == NULL) {
    pre_left_start_ = 0;
    last_facet_ = -1;
    return 0;
  }

  // xxx todo: handle it more gracefully
  int32_t to_copy = job->append_from_source(*pre_lines_, pre_left_start_);
  if (to_copy < 0) {
    // xxx todo: not enough space handle it more gracefully
    // effectively the the partial line will be discarded
    last_facet_ = -1;
    stats_no_copy_++;
    inno_log_warning("not enough space to copy %" PRI_SIZELU,
                     stats_no_copy_);
  }

  // we do not need pre_lines any more.
  if (pre_lines_->dec_ref()) {
    lidar_->free_angle_job(pre_lines_);
  }
  pre_lines_ = NULL;
  pre_left_start_ = 0;

  return to_copy;
}

void StageAngle::print_stats(void) const {
  char buf[1024];
  get_stats_string(buf, sizeof(buf));
  inno_log_info("%s", buf);
  return;
}

void StageAngle::get_stats_string(char *buf, size_t buf_size) const {
  int ret = snprintf(buf, buf_size,
                     "StageAngle: "
                     "frames=%" PRI_SIZELU
                     "facets=%" PRI_SIZELU
                     "drop_few_adc=%" PRI_SIZELU
                     "drop_few_polygon=%" PRI_SIZELU
                     "drop_few_galvo=%" PRI_SIZELU
                     "fov_filter_blocks=%" PRI_SIZELU
                     "polygon_too_long=%" PRI_SIZELU
                     "polygon_too_short=%" PRI_SIZELU
                     "polygon_ooo1=%" PRI_SIZELU
                     "polygon_ooo2=%" PRI_SIZELU
                     "no_copy_=%" PRI_SIZELU
                     ,
                     stats_frames_,
                     stats_facets_,
                     stats_drop_few_adc_,
                     stats_drop_few_polygon_,
                     stats_drop_few_galvo_,
                     stats_fov_filter_blocks_,
                     stats_polygon_too_long_,
                     stats_polygon_too_short_,
                     stats_polygon_ooo1_,
                     stats_polygon_ooo2_,
                     stats_no_copy_);
  if (ret >= ssize_t(buf_size)) {
    buf[buf_size - 1] = 0;
    return;
  }
}

void StageAngle::reset_ref_cnt_and_intensity() {
  for (uint8_t i = 0; i < kInnoRoiNumber; i++) {
    for (uint8_t j = 0; j < kInnoChannelNumber; j++) {
      frame_total_ref_intensity_[i][j] = 0;
      frame_total_ref_cnt_[i][j] = 0;
    }
  }
}

void StageAngle::send_ref_time_to_fw(const RawBlock* block) {
  if (!block) {
    return;
  }
  lidar_->get_fw_ipc()->write(IPC_ITEM_SEQUENCE_BEGIN,
                              fw_ipc_seq_begin_++);
  for (uint32_t i = 0; i < kInnoChannelNumber; i++) {
    lidar_->get_fw_ipc()->write((IpcItem)\
                               (IPC_ITEM_SEQUENCE_DATA0 + i),
                               block->ref_ts_sub_ns_off[i]);
  }
  lidar_->get_fw_ipc()->write(IPC_ITEM_SEQUENCE_END, fw_ipc_seq_end_++);
}
}  // namespace innovusion
