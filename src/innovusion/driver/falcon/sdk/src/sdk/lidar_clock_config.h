/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_LIDAR_CLOCK_CONFIG_H_
#define SDK_LIDAR_CLOCK_CONFIG_H_

#include <string>
#include "utils/config.h"
#include "utils/log.h"

namespace innovusion {

//
//
//
class LidarClockConfig : public Config {
 public:
  LidarClockConfig() : Config() {
    lost_check_after_machine_up_ms = 5 * 60 * 1000;  // 5 minutes
    lost_check_after_progress_up_ms = 5 * 1000;      // 5s
  }

  const char *get_type() const override { return "Lidar_Clock"; }

  int set_key_value_(const std::string &key, double value) override {
    SET_CFG(lost_check_after_machine_up_ms);
    SET_CFG(lost_check_after_progress_up_ms);
    return -1;
  }

  int set_key_value_(const std::string &key, const std::string value) override {
    // no string attribute
    return -1;
  }

  BEGIN_CFG_MEMBER()
  uint32_t lost_check_after_machine_up_ms;
  uint32_t lost_check_after_progress_up_ms;
  END_CFG_MEMBER()

 public:
  void print() {
    inno_log_info(
        "lost_check_after_machine_up_ms=%u lost_check_after_progress_up_ms=%u",
        lost_check_after_machine_up_ms, lost_check_after_progress_up_ms);
  }
};

//
//
//
class PTPConfig : public Config {
 public:
  PTPConfig() : Config() {
    lost_timeout_ms = 30 * 1000;  // 30s
    print_interval = 40;
  }

  const char *get_type() const override { return "PTP"; }

  int set_key_value_(const std::string &key, double value) override {
    SET_CFG(lost_timeout_ms);
    SET_CFG(print_interval);

    return -1;
  }

  int set_key_value_(const std::string &key, const std::string value) override {
    // no string attribute
    return -1;
  }

  BEGIN_CFG_MEMBER()
  uint32_t lost_timeout_ms;
  uint32_t print_interval;
  END_CFG_MEMBER()

 public:
  void print() {
    inno_log_info("PTP config: lost_timeout_ms %u, print_interval %u",
                  lost_timeout_ms, print_interval);
  }
};

//
//
//
class NTPConfig : public Config {
 public:
  NTPConfig() : Config() {
    lost_timeout_ms = 1024 * 1000;  // 1024s
    print_interval = 1;
  }

  const char *get_type() const override { return "NTP"; }

  int set_key_value_(const std::string &key, double value) override {
    SET_CFG(lost_timeout_ms);
    SET_CFG(print_interval);

    return -1;
  }

  int set_key_value_(const std::string &key, const std::string value) override {
    // no string attribute
    return -1;
  }

  BEGIN_CFG_MEMBER()
  uint32_t lost_timeout_ms;
  uint32_t print_interval;
  uint16_t offset_threshold_us;
  END_CFG_MEMBER()

 public:
  void print() {
    inno_log_info(
        "NTP config: lost_timeout_ms %u, print_interval %u, "
        "offset_threshold_us %u",
        lost_timeout_ms, print_interval, offset_threshold_us);
  }
};

}  // namespace innovusion
#endif  // SDK_LIDAR_CLOCK_CONFIG_H_
