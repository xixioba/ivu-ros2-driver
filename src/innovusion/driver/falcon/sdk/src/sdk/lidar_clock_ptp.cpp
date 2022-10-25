/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "sdk/lidar_clock.h"
#include "sdk/lidar_clock_ptp.h"

#include <math.h>
#include <limits>

namespace innovusion {

//
PTPConfig PTP::config_;
PTPStatistic PTP::stat_;

//
//
//
void PTP::init(const PTPConfig &config) {
  PTP::stat_.reset();
  PTP::config_.copy_from_src(const_cast<PTPConfig *>(&config));
  PTP::config_.print();
}

//
// time protocol offset
//
void PTP::check_time_offset(const ClockData &clock, int32_t offset_us) {
  // bool too_big = offset_us < -PTP::config_.offset_threshold_us ||
  //                offset_us > PTP::config_.offset_threshold_us;

  // uint32_t s_counter = 0;
  // if (too_big) {
  //   s_counter++;
  //   if (s_counter < 30 || s_counter % 20 == 0) {
  //     inno_log_info("%s time protocol big time offset %d us, happened: %u.",
  //                   clock.log_token, offset_us, s_counter);
  //   }
  // }
}

//
//
//
void PTP::on_time_msg_(const PtpData::Msg::TimeMsg &time, ClockData *clock) {
  snprintf(clock->log_token, sizeof(clock->log_token), "PTP [%lu]",
           PTP::stat_.time_packet_counter);

  if (!LidarClock::init_clock_info(
          clock, time.fpga_clock, time.y1, time.y2, time.M1, time.M2, time.d1,
          time.d2, time.h1, time.h2, time.m1, time.m2, time.s1, time.s2)) {
    clock->update_type = false;
    clock->update_time = false;
    return;
  }

  clock->update_time = true;

  //
  clock->update_type = true;
  clock->new_sync_type = INNO_TIME_SYNC_TYPE_PTP_LOCKED;

  //
  PTP::stat_.on_receive_msg(true);

  //
  // check_time_offset(*clock, time.diff_10us * 10);
  if (PTP::stat_.time_packet_counter % 80 == 0) {
    inno_log_info(
        "recv time msg [%" PRI_SIZELU "-%" PRI_SIZELU
        "] fpga_clock=%" PRI_SIZEU
        " ts=%s diff_10us=%d "
        "bootup_utc=%f",
        PTP::stat_.time_packet_counter, PTP::stat_.consecutive_locked,
        time.fpga_clock, clock->utc_str, time.diff_10us, clock->bootup_utc);
  }
}

//
//
//
void PTP::on_info_msg_(const PtpData::Msg::InfoMsg &info, uint64_t data_ms,
                       ClockData *clock) {
  bool has_locked = info.gm && info.locked_once;
  if (has_locked == false) {
    clock->update_type = true;
    clock->new_sync_type = INNO_TIME_SYNC_TYPE_PTP_UNLOCKED;

    PTP::stat_.on_receive_msg(false);
  }

  //
  static uint32_t s_seq_num = 0;
  if ((s_seq_num != 0) && (info.seq_num - s_seq_num > 1)) {
    inno_log_warning("ptp lost sequence number: %u ~ %u", s_seq_num,
                     info.seq_num);
  }

  s_seq_num = info.seq_num;

  //
  if (s_seq_num % 80 == 0) {
    inno_log_info(
        "recv info msg [%lu] seq_num=%u timestamp_ms=%u gm=%d locked_once=%d "
        "fpga_clk_stoped=%d fpga_clk_resumed=%d",
        PTP::stat_.info_packet_counter, info.seq_num, info.timestamp_ms,
        info.gm, info.locked_once, info.fpga_clk_stoped, info.fpga_clk_resumed);
  }

  //
  static uint32_t s_timestamp_ms = 0;
  if ((s_timestamp_ms != 0) && (info.timestamp_ms - s_timestamp_ms > 300)) {
    inno_log_warning("ptp info packet interval: %u",
                     info.timestamp_ms - s_timestamp_ms);
  }

  s_timestamp_ms = info.timestamp_ms;
//  if (data_ms - info.timestamp_ms > 300) {
//    inno_log_warning("ptp info packet dealy: %"
//      PRI_SIZEU, data_ms - info.timestamp_ms);
//  }
}

///////////////////////////////////////////////////////////////////////////////
//

//
//
//
void PTP::process(const PtpData &data, uint64_t data_ms, ClockData *clock) {
  PTP::stat_.last_packet_ms = data_ms;

  if (data.msg.time.ptp) {
    PTP::stat_.time_packet_counter++;
    on_time_msg_(data.msg.time, clock);
  } else {
    PTP::stat_.info_packet_counter++;
    on_info_msg_(data.msg.info, data_ms, clock);
  }
}

//
//
//
LidarClock::LostType PTP::check_sync_lost(uint64_t time_now_ms,
                                          uint64_t time_progress_up_ms) {
  uint64_t last_ms = PTP::stat_.last_packet_ms;

  // for pcs progress restart
  if (PTP::stat_.info_packet_counter <= 0 || PTP::stat_.last_packet_ms <= 0) {
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
