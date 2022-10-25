/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "sdk/lidar_clock_ntp.h"
#include "sdk/lidar_clock.h"

#include <math.h>
#include <limits>

namespace innovusion {

NTPConfig NTP::config_;
NTPStatistic NTP::stat_;

//
//
//
void NTP::init(const NTPConfig &config) {
  NTP::stat_.reset();
  NTP::config_.copy_from_src(const_cast<NTPConfig *>(&config));
  NTP::config_.print();
}

//
//
//
void NTP::process(const NtpData &data, uint64_t data_ms, ClockData *clock) {
  snprintf(clock->log_token, sizeof(clock->log_token), "NTP [%lu]",
           NTP::stat_.packet_counter);

  if (!LidarClock::init_clock_info(
          clock, data.fpga_clock, data.y1, data.y2, data.M1, data.M2, data.d1,
          data.d2, data.h1, data.h2, data.m1, data.m2, data.s1, data.s2)) {
    clock->update_type = false;
    clock->update_time = false;
    return;
  }

  bool has_locked = data.has_locked;
  NTP::stat_.on_process_data(data_ms, has_locked);

  //
  clock->update_type = true;
  clock->update_time = has_locked;

  if (has_locked) {
    clock->new_sync_type = INNO_TIME_SYNC_TYPE_NTP_LOCKED;
  } else {
    clock->new_sync_type = INNO_TIME_SYNC_TYPE_NTP_UNLOCKED;

    if (NTP::stat_.uplocked_counter < 10 ||
        NTP::stat_.uplocked_counter % 10 == 0) {
      inno_log_info("%s unlocked, happened: %u, consecutive happened: %lu.",
                    clock->log_token, NTP::stat_.uplocked_counter,
                    NTP::stat_.consecutive_unlocked);
    }
  }

  int32_t offset_us = data.offset * (data.offset_10_100us ? 100 : 10);
  check_time_offset(*clock, offset_us);
}

//
// time protocol offset
//
void NTP::check_time_offset(const ClockData &clock, int32_t offset_us) {
  bool too_big = offset_us < -NTP::config_.offset_threshold_us ||
                 offset_us > NTP::config_.offset_threshold_us;

  uint32_t s_counter = 0;
  if (too_big) {
    s_counter++;
    if (s_counter < 30 || s_counter % 20 == 0) {
      inno_log_info("%s time protocol big time offset %d us, happened: %u.",
                    clock.log_token, offset_us, s_counter);
    }
  }
}

//
// print ntp clock status
//
void NTP::print_status(const NtpData &data, const ClockData &clock) {
  inno_log_info(
      "recv NTP [%" PRI_SIZELU "-%" PRI_SIZELU
      "] has_locked=%d fpga_clock=%" PRI_SIZEU " ts=%s utc=%ld "
      "offset_10_100us=%d offset=%d bootup_utc=%f",
      NTP::stat_.packet_counter, NTP::stat_.consecutive_locked, data.has_locked,
      data.fpga_clock, clock.utc_str, clock.utc, data.offset_10_100us,
      data.offset, clock.bootup_utc);
}

//
//
//
LidarClock::LostType NTP::check_sync_lost(uint64_t time_now_ms,
                                          uint64_t time_progress_up_ms) {
  uint64_t last_ms = NTP::stat_.last_packet_ms;

  // for pcs progress restart
  if (NTP::stat_.packet_counter <= 0 || NTP::stat_.last_packet_ms <= 0) {
    last_ms = time_progress_up_ms;
  }

  bool is_lost = time_now_ms - last_ms > config_.lost_timeout_ms;
  if (is_lost) {
    return LidarClock::LostType::LOST;
  } else {
    return LidarClock::LostType::HEAL;
  }
}

}  // namespace innovusion
