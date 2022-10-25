/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_LIDAR_CLOCK_H_
#define SDK_LIDAR_CLOCK_H_

#include <mutex>  // NOLINT
#include <string>
#include "sdk/rawdata_type.h"
#include "sdk/lidar_clock_config.h"
#include "sdk_common/inno_lidar_packet.h"
#include "utils/config.h"
#include "utils/log.h"
#include "utils/types_consts.h"

namespace innovusion {
static constexpr int UTC_STR_LEN = 64;

//
//
//
struct ClockData {
  // clock
  bool update_time{false};
  time_t utc{0};
  uint64_t fpga_clock{0};
  double bootup_utc{0};

  // runtime
  bool update_type{false};
  InnoTimeSyncType old_sync_type;
  InnoTimeSyncType new_sync_type;

  // log
  char utc_str[UTC_STR_LEN]{0};
  char log_token[64]{0};
};

//
//
//
struct LidarClockStatistic {
  size_t update_time_counter{0};
  uint32_t type_switch_counter{0};

  // bootup_offset
  uint32_t bootup_100us_{0};
  uint32_t bootup_1ms_{0};
  uint32_t bootup_10ms_{0};
  uint32_t bootup_100ms_{0};

  // ppm
  uint32_t ppm_40_{0};
  uint32_t ppm_100_{0};
  uint32_t ppm_500_{0};
  uint32_t ppm_1000_{0};

  //
  //
  //
  void reset() {
    type_switch_counter = 0;

    bootup_100us_ = 0;
    bootup_1ms_ = 0;
    bootup_10ms_ = 0;
    bootup_100ms_ = 0;

    //
    ppm_40_ = 0;
    ppm_100_ = 0;
    ppm_500_ = 0;
    ppm_1000_ = 0;
  }
};

//
//
//
class LidarClock {
 public:
  LidarClock();
  ~LidarClock();

  void reset(enum InnoTimeSyncConfig sync_config,
             const LidarClockConfig &config);

  //
  void update_config(const LidarClockConfig &config);
  void reset(enum InnoTimeSyncConfig target);

  //
  void process_gps_data(const GpsData &r, uint64_t data_ms);
  void process_ptp_data(const PtpData &r, uint64_t data_ms);
  void process_ntp_data(const NtpData &r, uint64_t data_ms);

  //
  enum InnoTimeSyncType get_sync_state_and_diff(double *diff);

  //
  enum class LostType { NOP, HEAL, LOST };
  LidarClock::LostType check_sync_lost(uint64_t time_now_ms,
                                       uint64_t time_start_ms);

  //
  uint32_t get_stat_ptp() const;

  //
  static inline InnoTimestampUs to_epoch_us_(InnoEpNs time, double diff) {
    return (time / static_cast<double>(InnoConsts::kNsInSecond) + diff) *
           InnoConsts::kUsInSecond;
  }

  //
  static bool init_clock_info(ClockData *clock, uint64_t fpga_clock,
                              unsigned char y1, unsigned char y2,
                              unsigned char M1, unsigned char M2,
                              unsigned char d1, unsigned char d2,
                              unsigned char h1, unsigned char h2,
                              unsigned char m1, unsigned char m2,
                              unsigned char s1, unsigned char s2);

 private:
  void check_clock_(const ClockData &clock);
  void print_status_() const;

  //
  bool update_sync_config_(enum InnoTimeSyncConfig target);
  void update_sync_type_(InnoTimeSyncType sync_type);
  void update_sync_state_(const ClockData &clock);

 private:
  LidarClockConfig config_;
  LidarClockStatistic stat_;

  //
  enum InnoTimeSyncConfig sync_config_ { INNO_TIME_SYNC_CONFIG_MAX };

  //
  std::mutex mutex_;
  enum InnoTimeSyncType sync_type_ { INNO_TIME_SYNC_TYPE_NONE };
  double bootup_utc_{0};

  uint64_t fpga_clock_{0};
  time_t utc_{0};
  char utc_str_[UTC_STR_LEN]{0};

  //
  bool ignore_from_[INNO_TIME_SYNC_CONFIG_MAX + 1]{false};
  bool ignore_to_[INNO_TIME_SYNC_CONFIG_MAX + 1]{false};
};

}  // namespace innovusion
#endif  // SDK_LIDAR_CLOCK_H_
