/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_STATUS_REPORT_H_
#define SDK_STATUS_REPORT_H_

#include <string>
#include <unordered_map>

#include "sdk_common/inno_lidar_packet.h"
#include "utils/config.h"
#include "sdk/firmware_ipc.h"

namespace innovusion {
class InnoLidar;

class StatusReportConfig: public Config {
 public:
  StatusReportConfig() : Config() {
    interval_ms = 50;
    interface_name = "eth0";
  }

  const char* get_type() const override {
    return "Lidar_StatusReport";
  }

  int set_key_value_(const std::string &key,
                     double value) override {
    SET_CFG(interval_ms);
    return -1;
  }

  int set_key_value_(const std::string &key,
                     const std::string value) override {
    SET_CFG(interface_name);
    return -1;
  }

  BEGIN_CFG_MEMBER()
  uint32_t interval_ms;
  std::string interface_name;
  uint32_t latency_set_threshold_10us;
  uint32_t latency_heal_threshold_10us;
  END_CFG_MEMBER()
};

class StatusReport {
 public:
  StatusReport(InnoLidar *lidar, uint32_t interval);
  ~StatusReport();

 public:
  static void *report(void *ctx);

 public:
  void set_read_fw_faults(bool need_read);

 private:
  void *loop_();
  void gen_status_();
  void send_status_();
  void set_mode_();
  void set_in_fault_();
  void set_ex_fault_();
  void set_counters_();
  void set_sensor_readings_();
  void sensor_fault_detect_();

  void print_status();

 private:
  // TODO(@Yahui Hu): thresholds TBD, unit is 1/10 °C
  static const int16_t kLaserOverheatThreshold1 = 95 * 10;
  static const int16_t kLaserOverheatThreshold2 = 99 * 10;
  static const int16_t kLaserOverheatThreshold3 = 100 * 10;
  // heal thresholds of overheat1/2/3 are both 94 °C for now,
  // but may be different in future
  static const int16_t kLaserOverheatThresholdHeal1 = 94 * 10;
  static const int16_t kLaserOverheatThresholdHeal2 = 94 * 10;
  static const int16_t kLaserOverheatThresholdHeal3 = 94 * 10;
  static const int16_t kLaserOverheatStartTimeS = 30;

  //
  static const int16_t kEnableAfterProcessUpSec = 10;

  InnoLidar *lidar_;

  StatusReportConfig config_base_;
  StatusReportConfig config_;

  uint32_t interval_overide_ms_;
  uint64_t stats_status_sent_;
  uint64_t status_id_;
  uint64_t last_drop_noise_;
  uint64_t last_drop_deliever_;
  uint64_t last_frame_noise_;
  int16_t laser_overheat_set_thres_;
  int16_t laser_overheat_recover_thres_;

  char sn_[InnoStatusPacket::kSnSize];

  InnoStatusPacket status_packet_;

  bool enable_warring_ {false};
  bool reset_fw_faults_;
  std::mutex mutex_;
};

}  // namespace innovusion

#endif  // SDK_STATUS_REPORT_H_
