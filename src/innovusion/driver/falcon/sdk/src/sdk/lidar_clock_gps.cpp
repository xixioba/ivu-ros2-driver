/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "sdk/lidar_clock.h"
#include "sdk/lidar_clock_gps.h"

#include <math.h>
#include <limits>

namespace innovusion {

//
//
//
void GPS::init() {
}

//
//
//
void GPS::process(const GpsData &data, uint64_t data_ms) {
  ClockData clock;

  bool has_locked = data.pps && data.gprmc;
  if (!LidarClock::init_clock_info(
          &clock, data.fpga_clock, data.y1, data.y2, data.M1, data.M2, data.d1,
          data.d2, data.h1, data.h2, data.m1, data.m2, data.s1, data.s2)) {
    return;
  }

  //
  if (has_locked) {
    clock.new_sync_type = INNO_TIME_SYNC_TYPE_GPS_LOCKED;
  } else {
    clock.new_sync_type = INNO_TIME_SYNC_TYPE_GPS_UNLOCKED;
  }
}

//
// print gps clock status
//
void GPS::print_status(const GpsData &data, const ClockData &clock) {
}

//
//
//
LidarClock::LostType GPS::check_sync_lost(uint64_t time_now_ms,
                                          uint64_t time_progress_up_ms) {
  return LidarClock::LostType::NOP;
}

}  // namespace innovusion
