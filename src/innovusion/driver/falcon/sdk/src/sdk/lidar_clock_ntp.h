/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_LIDAR_CLOCK_NTP_H_
#define SDK_LIDAR_CLOCK_NTP_H_

#include <mutex>  // NOLINT
#include <string>
#include "sdk/rawdata_type.h"
#include "sdk/lidar_clock.h"
#include "sdk/lidar_clock_config.h"
#include "sdk_common/inno_lidar_packet.h"
#include "utils/log.h"
#include "utils/types_consts.h"


namespace innovusion {

//
//
//
struct NTPStatistic {
  size_t info_packet_counter{0};

  size_t packet_counter{0};
  uint64_t last_packet_ms{0};

  size_t consecutive_locked{0};
  size_t consecutive_unlocked{0};

  uint32_t uplocked_counter{0};

  uint32_t type_switch_counter{0};

  //
  size_t tp_offset_error_counter{0};
  size_t bootup_offset_error_counter{0};
  size_t ppm_error_counter{0};

  //
  //
  //
  void on_process_data(uint64_t data_ms, bool has_locked) {
    packet_counter++;
    last_packet_ms = data_ms;

    if (has_locked) {
      consecutive_locked++;
      consecutive_unlocked = 0;
    } else {
      consecutive_locked = 0;
      consecutive_unlocked++;

      uplocked_counter++;
    }
  }

  //
  //
  //
  void reset() {
    packet_counter = 0;
    last_packet_ms = 0;

    consecutive_locked = 0;
    consecutive_unlocked = 0;

    uplocked_counter = 0;
    type_switch_counter = 0;

    //
    tp_offset_error_counter = 0;
    bootup_offset_error_counter = 0;
    ppm_error_counter = 0;
  }
};

//
//
//
class NTP {
 public:
  NTP() = delete;
  ~NTP() = delete;

  static void init(const NTPConfig &config);
  static void process(const NtpData &r, uint64_t data_ms, ClockData *clock);

  static LidarClock::LostType check_sync_lost(uint64_t time_now_ms,
                                              uint64_t time_progress_up_ms);

  static void print_status(const NtpData &data, const ClockData &clock);

 private:
  static void check_time_offset(const ClockData &clock, int32_t offset_us);

 private:
  static NTPConfig config_;
  static NTPStatistic stat_;
};

}  // namespace innovusion
#endif  // SDK_LIDAR_CLOCK_NTP_H_
