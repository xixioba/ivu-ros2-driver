/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_STAGE_SIGNAL_CLOCK_H_
#define SDK_STAGE_SIGNAL_CLOCK_H_

#include <string>

#include "sdk/fpga_regs_collector.h"
#include "utils/log.h"
#include "utils/types_consts.h"

#define NO_FPGA_STRICT_ORDER

namespace innovusion {

class StageSignalClock {
 public:
  static const int kShiftBits = 3;
  static const int k1stLenBase = 8 + 8 + 4;
  static const int k1stLen = k1stLenBase + kShiftBits;
  static const int k2ndLenBase = 5 + 5;
  static const int k2ndLen = k2ndLenBase + kShiftBits;
  static const uint64_t k1stLowMax = (1 << k1stLen);
  static const uint64_t k1stLowMask = k1stLowMax - 1;
  static const uint64_t k1stHighMask = ~k1stLowMask;
  static const uint64_t k2ndLowMax = (1 << k2ndLen);
  static const uint64_t k2ndLowMask = k2ndLowMax - 1;
  static const uint64_t k2ndHighMask = ~k2ndLowMask;

  StageSignalClock() {
    set_bad_state_();

    base_1st_high0_ = 0;
    base_1st_high1_ = k1stLowMax;
    base_1st_low_ = 0;
    base_2nd_high0_ = 0;
    base_2nd_high1_ = k2ndLowMax;
    base_2nd_low_ = 0;

    first_chunck_ = false;
    first_chunck_cnt_ = 0;
    first_chunck_ns_ = 0;
    first_file_span_ns_ = 0;
    adjustment_for_file_rewind_ns_ = 0;
  }

  ~StageSignalClock() {
  }

  inline void set_lidar_live_direct_mode(bool live_direct_mode) {
    fpga_regs_collector_.set_lidar_live_direct_mode(live_direct_mode);
  }

  inline bool bad_state() const {
    return bad_state_;
  }

  inline InnoFpgaNs update_2nd_(int64_t o, int32_t flag) {
    // sanity check
    // inno_log_verify(o >= last_2nd_, "%ld vs %ld", o, last_2nd_);
    if (o < last_2nd_) {
#ifdef NO_FPGA_STRICT_ORDER
      disorder_error_counter_++;
      fpga_regs_collector_.\
      collect(std::string("out-of-order "),
              kInnoPLRegsSignalDisorder,
              sizeof(kInnoPLRegsSignalDisorder) / sizeof(struct RegInfo),
              kInnoPSRegsSignalDisorder,
              sizeof(kInnoPSRegsSignalDisorder) / sizeof(struct RegInfo));
      if (disorder_error_counter_ <= 32 ||
          disorder_error_counter_ % 1024 == 1) {
        inno_log_warning("out-of-order tg-%d 0x%" PRI_SIZEX " %" PRI_SIZED
                        "is smaller than 0x%" PRI_SIZEX " %" PRI_SIZED ", "
                        "reg info: %s",
                        flag, o, o * 32, last_2nd_, last_2nd_ * 32,
                        fpga_regs_collector_.get_info().c_str());
      }
      if (fpga_regs_collector_.get_reg_read_times() >= 2) {
        fpga_regs_collector_.reset();
      }
#else
      inno_log_panic("out-of-order tg=%d 0x%" PRI_SIZEX
                     " %" PRI_SIZELD
                     " is smaller than 0x%" PRI_SIZEX " %" PRI_SIZED,
                     flag, o, o * 32, last_2nd_, last_2nd_ * 32);
#endif
    }
    last_2nd_ = o;

    base_2nd_low_ = o & k2ndLowMask;
#ifdef NO_FPGA_STRICT_ORDER
    // base_2nd_low_ -= (1 << (k2ndLen - 1));
#endif
    base_2nd_high0_ = o & k2ndHighMask;
    base_2nd_high1_ = base_2nd_high0_ + k2ndLowMax;
    return o;
  }

  inline void set_bad_state_() {
    bad_state_ = true;
    last_1st_ = 0;
    last_2nd_ = 0;
    last_3rd_ = 0;
  }

  inline void set_bad_state() {
    set_bad_state_();
  }

  inline void set_is_first_chunck() {
    first_chunck_ = true;
    if (first_chunck_cnt_ == 1) {
      first_file_span_ns_ = last_2nd_ - first_chunck_ns_;
    }
    set_bad_state_();
  }

  inline InnoFpgaNs adjust_for_file_rewind_(InnoFpgaNs o) {
    return o + adjustment_for_file_rewind_ns_;
  }

  inline void handle_is_first_chunck_(int64_t o) {
    if (first_chunck_) {
      if (first_chunck_cnt_ == 0) {
        first_chunck_ns_ = o;
      }

      adjustment_for_file_rewind_ns_ = first_file_span_ns_ * first_chunck_cnt_;
      // if (!first_chunck_) {
      //   inno_log_info("file rewind adjustment %.03fs",
      //                 adjustment_for_file_rewind_ns_ / 1000.0 / 1000 / 1000);
      // }

      first_chunck_ = false;
      first_chunck_cnt_++;
    }
  }

  inline InnoFpgaNs receive_1st(int64_t o, int32_t flag) {  // encoder
    // xxx todo: what if we miss some 1st?
    // sanity check
    if (bad_state()) {
      handle_is_first_chunck_(o);
      bad_state_ = false;
    }
    // inno_log_trace("receive_1st 0x%lx", o);
    if (o < last_1st_) {
      inno_log_error("invalid ts, receive_1st: %"
        PRI_SIZED " vs %" PRI_SIZED, o, last_1st_);
      throw INNO_BAD_FPGA_DATA_INVALID_RECEIVE_1st;
    }

#ifdef THROW_TEST
    static int ran = 10;
    ran++;
    if (ran % 100000 == 1) {
      inno_log_error("====================================throw test");
      throw INNO_BAD_FPGA_DATA_INVALID_RECEIVE_1st;
    }
#endif

    last_1st_ = o;

    base_1st_low_ = o & k1stLowMask;
#ifdef NO_FPGA_STRICT_ORDER
    // base_1st_low_ -= (1 << (k1stLen - 1));
#endif
    base_1st_high0_ = o & k1stHighMask;
    base_1st_high1_ = base_1st_high0_ + k1stLowMax;

    update_2nd_(o, flag);
    return adjust_for_file_rewind_(o);
  }

  inline InnoFpgaNs receive_2nd(int64_t p) {  // trigger
    // inno_log_trace("receive_2nd 0x%lx", p);
    // xxx todo: what if we miss some 2nd? may need to force fpga
    // always start with 1st signal (encoder) after recover from
    // data drop
    if (p > (int64_t)k1stLowMask) {
      inno_log_error("invalid ts, receive_2nd, %"
          PRI_SIZED " vs %" PRI_SIZEU, p, k1stLowMask);
      throw INNO_BAD_FPGA_DATA_INVALID_RECEIVE_2nd;
    }

    int64_t o;
    if (p < base_1st_low_) {
      o = base_1st_high1_ + p;
    } else {
      o = base_1st_high0_ + p;
    }

    // inno_log_trace("2nd %lx", o >> 3);
    update_2nd_(o, 0);
    return adjust_for_file_rewind_(o);
  }

  inline InnoFpgaSubNs receive_3rd(int64_t p, int8_t fraction) {  // adc
    // inno_log_trace("receive_3rd 0x%lx", p);
    if (p > (int64_t)k2ndLowMask) {
      inno_log_error("invalid ts, receive_3rd, %"
          PRI_SIZED " vs %" PRI_SIZEU, p, k2ndLowMask);
      throw INNO_BAD_FPGA_DATA_INVALID_RECEIVE_3rd;
    }

    int64_t o;
    int64_t q = adjust_for_file_rewind_(p);
    if (p < base_2nd_low_) {
      o = InnoConverts::ns_to_sub_ns(base_2nd_high1_ + q);
    } else {
      o = InnoConverts::ns_to_sub_ns(base_2nd_high0_ + q);
    }
    /*
    inno_log_trace("p=0x%lx q=0x%lx o=0x%lx low=0x%lx high1=0x%lx high0=0x%lx",
                   p, q, o, base_2nd_low_, base_2nd_high1_, base_2nd_high0_);
    */

    // sanity check
    // inno_log_verify(o >= last_3rd_, "%ld vs %ld", o, last_3rd_);
    if (o < last_3rd_) {
#ifdef NO_FPGA_STRICT_ORDER
      static size_t s_error_counter = 0;
      s_error_counter++;

      bool interval_print = s_error_counter < 10 || s_error_counter % 1024 == 0;
      fpga_regs_collector_.\
      collect(std::string("out-of-order "),
              kInnoPLRegsSignalDisorder,
              sizeof(kInnoPLRegsSignalDisorder) / sizeof(struct RegInfo),
              kInnoPSRegsSignalDisorder,
              sizeof(kInnoPSRegsSignalDisorder) / sizeof(struct RegInfo));
      if (interval_print) {
        inno_log_warning(
            "out-of-order adc: 0x%" PRI_SIZEX
            " %" PRI_SIZED " is smaller than 0x%" PRI_SIZEX
            " %" PRI_SIZED ", happened: "
            "%" PRI_SIZELU ", reg info: %s",
            o, o, last_3rd_, last_3rd_, s_error_counter,
            fpga_regs_collector_.get_info().c_str());
      }
      if (fpga_regs_collector_.get_reg_read_times() >= 2) {
        fpga_regs_collector_.reset();
      }
#else
      inno_log_panic("out-of-order adc: 0x%" PRI_SIZEX
                    " %" PRI_SIZED " is smaller than 0x%"
                    PRI_SIZEX " %" PRI_SIZELD,
                     o, o, last_3rd_, last_3rd_);
#endif
    }
    last_3rd_ = o;

    o += fraction;
    return o;
  }

  inline InnoFpgaNs lastest_ns() const {
    return InnoConverts::sub_ns_to_ns(last_3rd_);
  }

 private:
  bool bad_state_;
  InnoFpgaNs last_1st_;
  InnoFpgaNs last_2nd_;
  InnoFpgaSubNs last_3rd_;

  InnoFpgaNs base_1st_high0_;
  InnoFpgaNs base_1st_high1_;
  InnoFpgaNs base_1st_low_;
  InnoFpgaNs base_2nd_high0_;
  InnoFpgaNs base_2nd_high1_;
  InnoFpgaNs base_2nd_low_;

  bool first_chunck_;
  size_t first_chunck_cnt_;
  InnoFpgaNs first_chunck_ns_;
  InnoFpgaNs first_file_span_ns_;
  InnoFpgaNs adjustment_for_file_rewind_ns_;
  ssize_t disorder_error_counter_ = 0;
  FpgaRegsCollector fpga_regs_collector_;
};

}  // namespace innovusion

#endif  // SDK_STAGE_SIGNAL_CLOCK_H_
