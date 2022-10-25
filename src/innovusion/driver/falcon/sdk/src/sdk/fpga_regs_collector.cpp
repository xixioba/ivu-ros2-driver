/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include <string>

#include "sdk/reg.h"
#include "sdk/fpga_regs_collector.h"
#include "utils/inno_lidar_log.h"


namespace innovusion {

FpgaRegsCollector::FpgaRegsCollector(bool live_direct_mode)
                  : live_direct_mode_(live_direct_mode) {
  reset();
}

FpgaRegsCollector::~FpgaRegsCollector() {
}

void FpgaRegsCollector::collect(const std::string& str,
                                const RegInfo pl_regs[], int pl_len,
                                const RegInfo ps_regs[], int ps_len) {
  if (!live_direct_mode_) {
    // xxx todo(WYY): add comm register read for live non direct mode
    return;
  }
  info_ += str;
  info_ += "registers read for " + std::to_string(reg_read_times_ + 1) +
           " time(s).\n";
  uint32_t val = 0;
  for (int i = 0; i < pl_len; i++) {
    int ret = Reg::pl_read(pl_regs[i].addr, &val);
    if (ret == 0) {
      info_ += pl_regs[i].name;
      info_ += ": ";
      info_ += std::to_string(val);
      info_ += ", ";
    } else {
      inno_log_error("reg read error for pl: %d, collected: %s",
                      ret, info_.c_str());
      reset();
      return;
    }
  }
  val = 0;
  for (int i = 0; i < ps_len; i++) {
    int ret = Reg::ps_read(ps_regs[i].addr, &val);
    if (ret == 0) {
      info_ += ps_regs[i].name;
      info_ += ": ";
      info_ += std::to_string(val);
      info_ += ", ";
    } else {
      inno_log_error("reg read error for ps: %d, collected: %s",
                      ret, info_.c_str());
      reset();
      return;
    }
  }
  info_ += "\n";
  reg_read_times_++;
}

void FpgaRegsCollector::reset() {
  info_.clear();
  reg_read_times_ = 0;
}

}  // namespace innovusion

