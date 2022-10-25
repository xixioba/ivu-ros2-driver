/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_LIDAR_CLOCK_GPS_H_
#define SDK_LIDAR_CLOCK_GPS_H_

#include <mutex>  // NOLINT
#include <string>
#include "sdk/rawdata_type.h"
#include "sdk_common/inno_lidar_packet.h"
#include "utils/config.h"
#include "utils/log.h"
#include "utils/types_consts.h"

namespace innovusion {

//
//
//
class GPS {
 public:
  GPS() = delete;
  ~GPS() = delete;

  void init();
  void process(const GpsData &r, uint64_t data_ms);

  static LidarClock::LostType check_sync_lost(uint64_t time_now_ms,
                                              uint64_t time_progress_up_ms);

  void print_status(const GpsData &data, const ClockData &clock);
};

}  // namespace innovusion
#endif  // SDK_LIDAR_CLOCK_GPS_H_
