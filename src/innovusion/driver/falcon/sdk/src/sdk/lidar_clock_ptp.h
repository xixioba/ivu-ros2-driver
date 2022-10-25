/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_LIDAR_CLOCK_PTP_H_
#define SDK_LIDAR_CLOCK_PTP_H_

#include <mutex>  // NOLINT
#include <string>
#include "sdk/rawdata_type.h"
#include "sdk_common/inno_lidar_packet.h"
#include "sdk/lidar_clock.h"
#include "sdk/lidar_clock_config.h"
#include "utils/config.h"
#include "utils/log.h"
#include "utils/types_consts.h"


namespace innovusion {

//
//
//
struct PTPStatistic {
  //
  uint64_t last_packet_ms{0};

  size_t info_packet_counter{0};
  size_t time_packet_counter{0};

  //
  size_t consecutive_locked{0};
  size_t unlocked_counter{0};

  //
  //
  //
  void on_receive_msg(bool has_locked) {
    if (has_locked) {
      consecutive_locked++;
    } else {
      consecutive_locked = 0;
      unlocked_counter++;
    }
  }

  //
  //
  //
  void reset() {
    //
    last_packet_ms = 0;
    info_packet_counter = 0;
    time_packet_counter = 0;

    //
    consecutive_locked = 0;
    unlocked_counter = 0;
  }
};

//
//
//
class PTP {
 public:
  PTP() = delete;
  ~PTP() = delete;

  static void init(const PTPConfig &config);
  static void update_config(const PTPConfig &config);

  static void process(const PtpData &r, uint64_t data_ms, ClockData *clock);

  static void print_rawdata(const PtpData &r, const ClockData &clock);

  static LidarClock::LostType check_sync_lost(uint64_t time_now_ms,
                                              uint64_t time_progress_up_ms);

  static uint32_t get_ptp_stat() {
    return PTP::stat_.unlocked_counter;
  }

 private:
  static void on_time_msg_(const PtpData::Msg::TimeMsg &time, ClockData *clock);
  static void on_info_msg_(const PtpData::Msg::InfoMsg &info, uint64_t data_ms,
                           ClockData *clock);

  //
  static void check_time_offset(const ClockData &clock, int32_t offset_us);

 private:
  static PTPConfig config_;
  static PTPStatistic stat_;
};

}  // namespace innovusion
#endif  // SDK_LIDAR_CLOCK_PTP_H_
