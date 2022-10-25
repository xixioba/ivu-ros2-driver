/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "sdk/lidar_clock.h"
#include "sdk/lidar_clock_ptp.h"
#include "sdk/lidar_clock_ntp.h"
#include "sdk/lidar_clock_gps.h"

#include <math.h>
#include <limits>

#ifdef __MINGW64__
#define timegm _mkgmtime
#endif

namespace innovusion {

//
//
//
LidarClock::LidarClock() {
}


LidarClock::~LidarClock() {
}


void LidarClock::reset(enum InnoTimeSyncConfig sync_config,
                       const LidarClockConfig &config) {
  this->config_.copy_from_src(const_cast<LidarClockConfig *>(&config));
  this->config_.print();

  update_sync_config_(sync_config);
}


//
//
//
void LidarClock::update_config(const LidarClockConfig &config) {
  this->config_.copy_from_src(const_cast<LidarClockConfig *>(&config));
}

//
// for human readable
//
static const char *SyncConfigName[INNO_TIME_SYNC_CONFIG_MAX + 1]{
    "0-HOST", "1-PTP", "2-GPS", "3-FILE", "4-NTP", "5-MAX"};

static const char *SyncTypeName[INNO_TIME_SYNC_TYPE_MAX + 1]{
    "0-NONE",          "1-RECORDED",     "2-HOST",      "3-GPS_INIT",
    "4-GPS_LOCKED",    "5-GPS_UNLOCKED", "6-PTP_INIT",  "7-PTP_LOCKED",
    "8-PTP_UNLOCKED",  "9-FILE_INIT",    "10-NTP_INIT", "11-NTP_LOCKED",
    "12-NTP_UNLOCKED", "13-GPS_LOST",    "14-PTP_LOST", "15-NTP_LOST",
    "16-MAX"};


//
//
//
bool LidarClock::init_clock_info(ClockData *clock, uint64_t fpga_clock,
                                 unsigned char y1, unsigned char y2,
                                 unsigned char M1, unsigned char M2,
                                 unsigned char d1, unsigned char d2,
                                 unsigned char h1, unsigned char h2,
                                 unsigned char m1, unsigned char m2,
                                 unsigned char s1, unsigned char s2) {
  if (y1 == 0 && y2 == 0 && M1 == 0 && M2 == 0 && d1 == 0 && d2 == 0 &&
      h1 == 0 && h2 == 0 && m1 == 0 && m2 == 0 && s1 == 0 && s2 == 0) {
    inno_log_info("%s NOT LOCKED since started", clock->log_token);
    return false;
  }

  // 1970 ~ 2069
  // 70 ~ 99 : year - 1900
  // 00 ~ 69 : year - 2000
  bool year_2000 = y1 < 7;

  // digits to timet
  struct tm t;
  t.tm_sec = s1 * 10 + s2;
  t.tm_min = m1 * 10 + m2;
  t.tm_hour = h1 * 10 + h2;
  t.tm_mday = d1 * 10 + d2;
  t.tm_mon = M1 * 10 + M2 - 1;  // Jan is zero!!!
  t.tm_year = y1 * 10 + y2 + (year_2000 ? 100 : 0);
  t.tm_wday = t.tm_yday = t.tm_isdst = 0;

  snprintf(clock->utc_str, sizeof(clock->utc_str),
           "%u%u-%u%u-%u%u %u%u:%u%u:%u%u", y1 + (year_2000 ? 200 : 190), y2,
           M1, M2, d1, d2, h1, h2, m1, m2, s1, s2);

  clock->utc = timegm(&t);
  if (clock->utc == -1) {
    inno_log_warning("%s ignore invalid, ts=%s timegm_error=%d",
                     clock->log_token, clock->utc_str, errno);
    return false;
  }

  // calculate fpga time diff
  clock->bootup_utc =
      clock->utc -
      fpga_clock * 8.0 / static_cast<double>(InnoConsts::kNsInSecond);

  //
  clock->fpga_clock = fpga_clock;
  return true;
}

//
//
//
void LidarClock::check_clock_(const ClockData &clock) {
  // check bootup offset
  bool check_bootup_offset = this->utc_ > 0 && clock.update_time;
  if (check_bootup_offset == false) {
    return;
  }

  double bootup_offset_sec = clock.bootup_utc - this->bootup_utc_;
  if (bootup_offset_sec < 0) {
    bootup_offset_sec = -bootup_offset_sec;
  }

  if (bootup_offset_sec > 0.1) {
    this->stat_.bootup_100ms_++;
  } else if (bootup_offset_sec > 0.01) {
    this->stat_.bootup_10ms_++;
  } else if (bootup_offset_sec > 0.001) {
    this->stat_.bootup_1ms_++;
  } else if (bootup_offset_sec > 0.0001) {
    this->stat_.bootup_100us_++;
  }

  static constexpr double BOOTUP_OFFSET_SEC = 0.1;  // 100 ms
  bool bootup_offset_too_big = bootup_offset_sec > BOOTUP_OFFSET_SEC;
  if (bootup_offset_too_big) {
    inno_log_info("%s bootup time big change %f ms.", clock.log_token,
                  bootup_offset_sec * 1000);
  }

  // check ppm
  time_t utc_duration_sec = clock.utc - this->utc_;
  if (utc_duration_sec > 0) {
    double drift_ppm = bootup_offset_sec / utc_duration_sec * 1000000;
    if (drift_ppm > 1000) {
      this->stat_.ppm_1000_++;
    } else if (drift_ppm > 500) {
      this->stat_.ppm_500_++;
    } else if (drift_ppm > 100) {
      this->stat_.ppm_100_++;
    } else if (drift_ppm > 40) {
      this->stat_.ppm_40_++;
    }
  }

  // print
  this->stat_.update_time_counter++;
  if (this->stat_.update_time_counter % 80 == 0) {
    print_status_();
  }
}

//
//
//
void LidarClock::print_status_() const {
  inno_log_info(
      "update time=%" PRI_SIZELU ", config=%s state=%s, fpga_clock=%"
      PRI_SIZEU " ts=%s utc=%ld bootup_utc=%f, "
      "bootup_utc offset >100ms(%u) >10ms(%u) >1ms(%u) "
      ">100us(%u), ppm >1000(%u) >500(%u) >100(%u) >40(%u).",
      this->stat_.update_time_counter, SyncConfigName[this->sync_config_],
      SyncTypeName[this->sync_type_], this->fpga_clock_, this->utc_str_,
      this->utc_, this->bootup_utc_, this->stat_.bootup_100ms_,
      this->stat_.bootup_10ms_, this->stat_.bootup_1ms_,
      this->stat_.bootup_100us_, this->stat_.ppm_1000_, this->stat_.ppm_500_,
      this->stat_.ppm_100_, this->stat_.ppm_40_);
}

//
//
//
bool LidarClock::update_sync_config_(enum InnoTimeSyncConfig target) {
  if (sync_config_ == target) {
    return true;
  }

  // prevent log storm
  if (ignore_from_[sync_config_] || ignore_to_[target]) {
    return false;
  }

  inno_log_info("sync config switch --- from %s to %s",
                SyncConfigName[sync_config_], SyncConfigName[target]);

  // check "from"
  if (sync_config_ != INNO_TIME_SYNC_CONFIG_MAX &&
      sync_config_ != INNO_TIME_SYNC_CONFIG_FILE &&
      sync_config_ != INNO_TIME_SYNC_CONFIG_HOST) {
    inno_log_error(
        "sync config switch --- failed, can't switch from %s, "
        "please restart progress to enable %s",
        SyncConfigName[sync_config_], SyncConfigName[target]);

    ignore_from_[sync_config_] = true;
    return false;
  }

  // check "to"
  enum InnoTimeSyncType init_sync_type = INNO_TIME_SYNC_TYPE_NONE;
  switch (target) {
    case INNO_TIME_SYNC_CONFIG_GPS:
      init_sync_type = INNO_TIME_SYNC_TYPE_GPS_INIT;
      break;
    case INNO_TIME_SYNC_CONFIG_PTP:
      init_sync_type = INNO_TIME_SYNC_TYPE_PTP_INIT;
      break;
    case INNO_TIME_SYNC_CONFIG_NTP:
      init_sync_type = INNO_TIME_SYNC_TYPE_NTP_INIT;
      break;
    case INNO_TIME_SYNC_CONFIG_FILE:
      init_sync_type = INNO_TIME_SYNC_TYPE_FILE_INIT;
      break;
    case INNO_TIME_SYNC_CONFIG_HOST:
      init_sync_type = INNO_TIME_SYNC_TYPE_HOST;
      break;
    default:
      inno_log_error(
          "sync config switch --- failed, can't switch to %s(unsupported), "
          "please check setting",
          SyncConfigName[target]);

      ignore_to_[target] = true;
      return false;
  }

  // update
  this->sync_config_ = target;

  this->stat_.reset();

  {
    std::unique_lock<std::mutex> lk(mutex_);
    this->sync_type_ = init_sync_type;

    this->bootup_utc_ = 0;
    this->fpga_clock_ = 0;
    this->utc_ = 0;
    utc_str_[0] = '\0';
  }

  return true;
}

//
//
//
void LidarClock::update_sync_type_(InnoTimeSyncType sync_type) {
  enum InnoTimeSyncType old_sync_type = INNO_TIME_SYNC_TYPE_MAX;

  {
    std::unique_lock<std::mutex> lk(mutex_);
    old_sync_type = this->sync_type_;
    this->sync_type_ = sync_type;
  }

  if (old_sync_type != sync_type) {
    inno_log_info("sync_type switch -- from %s to %s",
                  SyncTypeName[old_sync_type], SyncTypeName[sync_type]);
    print_status_();
  }
}

//
//
//
void LidarClock::update_sync_state_(const ClockData &clock) {
  if (!clock.update_type && !clock.update_time) {
    return;
  }

  enum InnoTimeSyncType old_sync_type = INNO_TIME_SYNC_TYPE_MAX;

  {
    std::unique_lock<std::mutex> lk(mutex_);
    old_sync_type = this->sync_type_;
    this->sync_type_ = clock.new_sync_type;
    if (clock.update_time) {
      this->utc_ = clock.utc;
      this->fpga_clock_ = clock.fpga_clock;
      strncpy(this->utc_str_, clock.utc_str, sizeof(clock.utc_str));

      this->bootup_utc_ = clock.bootup_utc;
    }
  }

  if (old_sync_type != clock.new_sync_type) {
    inno_log_info("sync_type switch -- from %s to %s",
                  SyncTypeName[old_sync_type],
                  SyncTypeName[clock.new_sync_type]);
    print_status_();
  }
}

//
//
//
void LidarClock::process_gps_data(const GpsData &data, uint64_t data_ms) {
  if (!update_sync_config_(INNO_TIME_SYNC_CONFIG_GPS)) {
    return;
  }

  // Not Implemented
}

//
//
//
void LidarClock::process_ptp_data(const PtpData &data, uint64_t data_ms) {
  if (!update_sync_config_(INNO_TIME_SYNC_CONFIG_PTP)) {
    return;
  }

  ClockData clock;
  PTP::process(data, data_ms, &clock);

  check_clock_(clock);
  update_sync_state_(clock);
}

//
//
//
void LidarClock::process_ntp_data(const NtpData &data, uint64_t data_ms) {
  if (!update_sync_config_(INNO_TIME_SYNC_CONFIG_NTP)) {
    return;
  }

  ClockData clock;
  NTP::process(data, data_ms, &clock);

  check_clock_(clock);
  update_sync_state_(clock);
}

///////////////////////////////////////////////////////////////////////////////
//

uint32_t LidarClock::get_stat_ptp() const { return PTP::get_ptp_stat(); }

//
//
//
enum InnoTimeSyncType LidarClock::get_sync_state_and_diff(double *diff) {
  inno_log_verify(diff, "invalid address");

  enum InnoTimeSyncType ret;
  {
    std::unique_lock<std::mutex> lk(mutex_);
    ret = sync_type_;
    *diff = bootup_utc_;
  }

  return ret;
}

//
//
//
LidarClock::LostType LidarClock::check_sync_lost(uint64_t time_now_ms,
                                                 uint64_t time_progress_up_ms) {
  if (time_now_ms <= config_.lost_check_after_machine_up_ms) {
    return LostType::NOP;
  }

  if (time_now_ms - time_progress_up_ms <=
      config_.lost_check_after_progress_up_ms) {
    return LostType::NOP;
  }

  LidarClock::LostType ret = LostType::NOP;
  switch (sync_config_) {
    case INNO_TIME_SYNC_CONFIG_GPS:
      ret = GPS::check_sync_lost(time_now_ms, time_progress_up_ms);
      if (ret == LostType::LOST) {
        update_sync_type_(INNO_TIME_SYNC_TYPE_GPS_LOST);
      }
      break;
    case INNO_TIME_SYNC_CONFIG_PTP:
      ret = PTP::check_sync_lost(time_now_ms, time_progress_up_ms);
      if (ret == LostType::LOST) {
        update_sync_type_(INNO_TIME_SYNC_TYPE_PTP_LOST);
      }
      break;
    case INNO_TIME_SYNC_CONFIG_NTP:
      ret = NTP::check_sync_lost(time_now_ms, time_progress_up_ms);
      if (ret == LostType::LOST) {
        update_sync_type_(INNO_TIME_SYNC_TYPE_NTP_LOST);
      }
      break;

    default:
      break;
  }

  return ret;
}

}  // namespace innovusion
