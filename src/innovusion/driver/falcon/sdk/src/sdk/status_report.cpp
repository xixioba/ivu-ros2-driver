/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "sdk/status_report.h"

#if (defined(__APPLE__) || defined(__MINGW64__))
#include <unistd.h>
#else
#include <sys/sysinfo.h>
#endif

#include <string>
#include <utility>

#include "sdk/lidar.h"
#include "utils/config.h"
#include "utils/inno_thread.h"
#include "utils/utils.h"

namespace innovusion {
StatusReport::StatusReport(InnoLidar *l, uint32_t interval)
  : laser_overheat_set_thres_(kLaserOverheatThreshold3)
  , laser_overheat_recover_thres_(kLaserOverheatThresholdHeal3) {
  lidar_ = l;
  inno_log_verify(l, "lidar");
  lidar_->add_config(&config_base_);
  config_.copy_from_src(&config_base_);
  interval_overide_ms_ = interval;
  stats_status_sent_ = 0;
  status_id_ = 0;
  last_drop_noise_ = 0;
  last_drop_deliever_ = 0;
  last_frame_noise_ = 0;
  reset_fw_faults_ = false;
  memset(sn_, 0, sizeof(sn_));

  memset(&status_packet_, 0, sizeof(status_packet_));
  status_packet_.common.version.magic_number = kInnoMagicNumberStatusPacket;
  status_packet_.common.version.major_version = kInnoMajorVersionStatusPacket;
  status_packet_.common.version.minor_version = kInnoMinorVersionStatusPacket;
  status_packet_.common.size = sizeof(status_packet_);
  status_packet_.common.source_id = 0;

  int ret = lidar_->get_sn(sn_, sizeof(sn_));
  if (ret != 0) {
    inno_log_error("get_sn %d", ret);
  }
  char fw_seq_buf[64];
  memset(fw_seq_buf, 0, sizeof(fw_seq_buf));
  ret = lidar_->get_fw_sequence_buf(fw_seq_buf, sizeof(fw_seq_buf));
  if (ret != 0) {
    inno_log_error("get_fw_sequence_buf ret = %d", ret);
    status_packet_.common.version.fw_sequence = 0;
  } else {
    status_packet_.common.version.fw_sequence = atoi(fw_seq_buf);
  }

  inno_log_verify(sizeof(status_packet_.sn) == sizeof(sn_),
                  "%lu vs %lu", sizeof(status_packet_.sn), sizeof(sn_));
  memcpy(&status_packet_.sn, sn_, sizeof(sn_));
  lidar_->init_sys_stats_config(&config_);
  if (lidar_->is_live_lidar()) {
    int16_t temp[2];
    int ret = lidar_->get_overheat_thresholds(temp);
    if (ret == 0) {
      laser_overheat_set_thres_ = temp[0];
      laser_overheat_recover_thres_ = temp[1];
    }
  }
  inno_log_info("overheat threshold : %d/%d",
                 laser_overheat_set_thres_,
                 laser_overheat_recover_thres_);
}

StatusReport::~StatusReport() {
  lidar_->remove_config(&config_base_);
}

void *StatusReport::report(void *ctx) {
  StatusReport *s = reinterpret_cast<StatusReport *>(ctx);
  return s->loop_();
}

void StatusReport::set_read_fw_faults(bool need_read) {
  std::unique_lock<std::mutex> lk(mutex_);
  reset_fw_faults_ = need_read;
}

//
//
//
void *StatusReport::loop_() {
  uint64_t target_ms =
      interval_overide_ms_ > 0 ? interval_overide_ms_ : config_.interval_ms;

  //
  if (target_ms <= 0) {
    inno_log_info("Will NOT send status message every %"
      PRI_SIZEU " ms.", target_ms);
    return NULL;
  }

  //
  inno_log_info("Will send status message every%" PRI_SIZEU "ms.", target_ms);

  uint64_t target_us = 1000 * target_ms;
  status_packet_.status_packet_interval_ms = target_ms;

  int debug_run_count = 0;
  constexpr int debug_print_period = 100;  // 50 ms * 100

  //
  while (!lidar_->it_status_->has_shutdown()) {
    uint64_t time_s1 = InnoUtils::get_time_ns(CLOCK_MONOTONIC_RAW);
    gen_status_();
    send_status_();

    // for debug
    ++debug_run_count;
    if (debug_run_count % debug_print_period == 0) {
      print_status();
    }

    uint64_t time_s2 = InnoUtils::get_time_ns(CLOCK_MONOTONIC_RAW);
    uint64_t elapsed_us = (time_s2 - time_s1) / 1000;
    if (target_us > elapsed_us) {
      usleep(target_us - elapsed_us);
    } else {
      if (!this->enable_warring_) {
        this->enable_warring_ =
            status_packet_.counters.process_up_time_in_second >=
            kEnableAfterProcessUpSec;
        inno_log_info(
            "enable after process up %d sec, current %u sec, %s warring.",
            kEnableAfterProcessUpSec,
            status_packet_.counters.process_up_time_in_second,
            this->enable_warring_ ? "enable" : "disable");
      }

      if (this->enable_warring_) {
        inno_log_warning("can not finish status callback in %"
          PRI_SIZEU " ms", target_ms);
      }
    }
  }

  inno_log_info("status_report stopped");
  return NULL;
}

//
//
//
void StatusReport::gen_status_() {
  double diff;

  status_packet_.idx = status_id_++;
  status_packet_.common.timestamp_sync_type =
      lidar_->get_clock().get_sync_state_and_diff(&diff);
  status_packet_.common.ts_start_us =
      InnoUtils::get_time_ns(CLOCK_REALTIME) / 1000;  // frame header
  set_mode_();
  set_counters_();
  set_sensor_readings_();
  set_in_fault_();
  set_ex_fault_();

  InnoPacketReader::set_packet_crc32(&status_packet_.common);
  return;
}

void StatusReport::send_status_() {
  lidar_->do_status_callback(&status_packet_);
  return;
}

void StatusReport::set_mode_() {
  enum InnoLidarMode mode = INNO_LIDAR_MODE_WORK_NORMAL;
  enum InnoLidarMode pre_mode = INNO_LIDAR_MODE_WORK_NORMAL;
  enum InnoLidarStatus ss = INNO_LIDAR_STATUS_NORMAL;
  uint64_t in_transition_mode_ms = 0;

  if (lidar_->is_live_direct_memory_lidar_()) {
    lidar_->get_mode_status(&mode, &pre_mode, &ss,
                            &in_transition_mode_ms);
  }
  status_packet_.common.lidar_mode = mode;
  status_packet_.common.lidar_status = ss;
  status_packet_.pre_lidar_mode = pre_mode;
  status_packet_.in_transition_mode_ms = in_transition_mode_ms;
}

void StatusReport::set_in_fault_() {
  lidar_->update_data_packet(&status_packet_);
  return;
}

void StatusReport::set_ex_fault_() {
  status_packet_.ex_faults.clear_faults();
}

void StatusReport::set_counters_() {
  lidar_->set_status_counters(&status_packet_.counters,
                              status_packet_.idx == 0);
}

void StatusReport::set_sensor_readings_() {
  static const int16_t kInvalidTemp = -10000;
  if (lidar_->is_live_direct_memory_lidar_()) {
    if (!lidar_->read_fw_ipc_is_ready()) {
      inno_log_info("firmware IPC is not ready");
      return;
    }
    InnoStatusSensorReadings &s = status_packet_.sensor_readings;
    s.temperature_fpga_10th_c =
        lidar_->read_fw_ipc(IPC_ITEM_TEMPERATURE_FPGA_10th_C, kInvalidTemp);
    s.temperature_laser_10th_c =
        lidar_->read_fw_ipc(IPC_ITEM_TEMPERATURE_LASER_10th_C, kInvalidTemp);
    s.temperature_adc_10th_c =
        lidar_->read_fw_ipc(IPC_ITEM_TEMPERATURE_ADC_10th_C, kInvalidTemp);
    s.temperature_board_10th_c =
        lidar_->read_fw_ipc(IPC_ITEM_TEMPERATURE_BOARD_10th_C, kInvalidTemp);
    {
      static const int kTempRangeLo = -400;     // -40degC
      static const int kTempRangeHi = 1200;     // 120degC
      static const int kTempRangeMaxDiff = 50;  // 5degC
      static const int kDetectorTempHysteresis = 20;  // 2degC
      int max_temp = kTempRangeLo - 1;
      int min_temp = kTempRangeHi + 1;
      for (int i = 0; i < 4; i++) {
        s.temperature_det_10th_c[i] =
            lidar_->read_fw_ipc(
                innovusion::IpcItem(IPC_ITEM_TEMPERATURE_DET_0_10th_C + i),
                kInvalidTemp);
        if (s.temperature_det_10th_c[i] == kInvalidTemp) {
          continue;
        }
        if (s.temperature_det_10th_c[i] > max_temp) {
          max_temp = s.temperature_det_10th_c[i];
        }
        if (s.temperature_det_10th_c[i] < min_temp) {
          min_temp = s.temperature_det_10th_c[i];
        }
      }
      int hysteresis = 0;
      if (lidar_->get_current_fault_status(INNO_LIDAR_IN_FAULT_DBTEMP)) {
        hysteresis = kDetectorTempHysteresis;
      }
      if (max_temp > kTempRangeHi - hysteresis ||
          min_temp < kTempRangeLo + hysteresis ||
          max_temp - min_temp > kTempRangeMaxDiff - hysteresis) {
        lidar_->set_raw_fault(INNO_LIDAR_IN_FAULT_DBTEMP, true,
                            "max_temp: %d, min_temp: %d, %d/%d/%d/%d/",
                              max_temp, min_temp,
                              s.temperature_det_10th_c[0],
                              s.temperature_det_10th_c[1],
                              s.temperature_det_10th_c[2],
                              s.temperature_det_10th_c[3]);
      } else {
        if (max_temp >= min_temp) {
          lidar_->heal_raw_fault(INNO_LIDAR_IN_FAULT_DBTEMP, true,
                                "INNO_LIDAR_IN_FAULT_DBTEMP heals");
        } else {
          // keep old state
        }
      }
    }
    for (int i = 0; i < 3; i++) {
      s.temperature_other_10th_c[i] =
          lidar_->read_fw_ipc(
              innovusion::IpcItem(IPC_ITEM_TEMPERATURE_OTHER_0_10th_C + i),
              kInvalidTemp);
    }

    s.heater_current_ma = lidar_->read_fw_ipc(
              innovusion::IpcItem(IPC_ITEM_HEATER_CURRENT_MA),
              0);
    s.motor_rpm_1000th = lidar_->read_fw_ipc(
        innovusion::IpcItem(IPC_ITEM_MOTOR_RPM_1000th),
        0);
    s.galvo_fpm_1000th = lidar_->read_fw_ipc(
        innovusion::IpcItem(IPC_ITEM_GALVO_FPM_1000th),
        0);

    s.motor_rotation_total =
        lidar_->read_fw_ipc(IPC_ITEM_MOTOR_ROTATION_TOTAL, 0);
    s.galvo_round_total =
        lidar_->read_fw_ipc(IPC_ITEM_GALVO_ROUND_TOTAL, 0);

    for (int i = 0; i < 6; i++) {
      s.motor[i] = lidar_->read_fw_ipc(
          innovusion::IpcItem(IPC_ITEM_MOTOR_0 + i), 0);
    }
    for (int i = 0; i < 6; i++) {
      s.galvo[i] = lidar_->read_fw_ipc(
          innovusion::IpcItem(IPC_ITEM_GALVO_0 + i), 0);
    }
    for (int i = 0; i < 6; i++) {
      s.laser[i] = lidar_->read_fw_ipc(
          innovusion::IpcItem(IPC_ITEM_LASER_0 + i), 0);
    }
#if (defined(__APPLE__ ) || defined(__MINGW64__))
    uint32_t info_uptime =
        std::chrono::steady_clock::now().time_since_epoch().count() /
        1000000000;
    if (info_uptime > kLaserOverheatStartTimeS) {
      sensor_fault_detect_();
    }
#else
    struct sysinfo info;
    sysinfo(&info);
    if (info.uptime > kLaserOverheatStartTimeS) {
      sensor_fault_detect_();
    }
#endif
  }
}

void StatusReport::sensor_fault_detect_() {
  // laser temperature
  // xxx todo(@Yahui Hu): Use Ipc to tell firmware to shutdown lidar
  InnoStatusSensorReadings &s = status_packet_.sensor_readings;
  if (s.temperature_laser_10th_c > laser_overheat_set_thres_) {
    lidar_->set_raw_fault(INNO_LIDAR_IN_FAULT_OVERHEAT3, true,
                         "current temperature: %d, threshold: %d",
                          s.temperature_laser_10th_c,
                          laser_overheat_set_thres_);
    if (lidar_->get_current_fault_status(INNO_LIDAR_IN_FAULT_OVERHEAT3)) {
      lidar_->set_raw_fault(INNO_LIDAR_TEMPHIGH_INHIBIT, true,
                           "INNO_LIDAR_TEMPHIGH_INHIBIT sets");
    }
  } else if (s.temperature_laser_10th_c <= laser_overheat_recover_thres_) {
    lidar_->heal_raw_fault(INNO_LIDAR_IN_FAULT_OVERHEAT3, true,
                          "INNO_LIDAR_IN_FAULT_OVERHEAT3 heals");
    if (!lidar_->get_current_fault_status(INNO_LIDAR_IN_FAULT_OVERHEAT3)) {
      lidar_->heal_raw_fault(INNO_LIDAR_TEMPHIGH_INHIBIT, true,
                            "INNO_LIDAR_TEMPHIGH_INHIBIT heals");
    }
  }
  if (s.temperature_laser_10th_c > kLaserOverheatThreshold2) {
    lidar_->set_raw_fault(INNO_LIDAR_IN_FAULT_OVERHEAT2, true,
                         "current temperature: %d, threshold: %d",
                          s.temperature_laser_10th_c,
                          kLaserOverheatThreshold2);
  } else if (s.temperature_laser_10th_c <= kLaserOverheatThresholdHeal2) {
    lidar_->heal_raw_fault(INNO_LIDAR_IN_FAULT_OVERHEAT2, true,
                          "INNO_LIDAR_IN_FAULT_OVERHEAT2 heals");
  }
  if (s.temperature_laser_10th_c > kLaserOverheatThreshold1) {
    lidar_->set_raw_fault(INNO_LIDAR_IN_FAULT_OVERHEAT1, true,
                         "current temperature: %d, threshold: %d",
                          s.temperature_laser_10th_c,
                          kLaserOverheatThreshold1);
  } else if (s.temperature_laser_10th_c <= kLaserOverheatThresholdHeal1) {
    lidar_->heal_raw_fault(INNO_LIDAR_IN_FAULT_OVERHEAT1, true,
                          "INNO_LIDAR_IN_FAULT_OVERHEAT1 heals");
  }
}

void StatusReport::print_status() {
  constexpr size_t buf_size = 2048;
  char buf[buf_size]{0};

  int ret =
      InnoDataPacketUtils::printf_status_packet(status_packet_, buf, buf_size);
  if (ret > 0) {
    inno_log_info("%s", buf);
  } else {
    inno_log_error("InnoDataPacketUtils::printf_status_packet error %d.", ret);
  }
}

}  // namespace innovusion
