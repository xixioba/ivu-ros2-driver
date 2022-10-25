/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_FPGA_REGS_COLLECTOR_H_
#define SDK_FPGA_REGS_COLLECTOR_H_

#include <vector>
#include <string>

namespace innovusion {
struct RegInfo {
  int addr;
  const char* name;
};
static const int kPLResNum = 22;
static const int kPLRegsSigDisorder = 19;
static const int kPSResNum = 8;
static const int kPSRegsSigDisorder = 12;
// PL regs
static const struct RegInfo kInnoPLRegsReadTimeOut[kPLResNum] = {
  {0x00, "0x00"},
  {0x14, "0x14"},
  {0x18, "0x18"},
  {0x1c, "0x1c"},
  {0x84, "0x84"},
  {0x88, "0x88"},
  {0xe4, "0xe4"},
  {0xf4, "0xf4"},
  {0x118, "0x118"},
  {0x120, "0x120"},
  {0x130, "0x130"},
  {0x134, "0x134"},
  {0x138, "0x138"},
  {0x13c, "0x13c"},
  {0x150, "0x150"},
  {0x154, "0x154"},
  {0x158, "0x158"},
  {0x15c, "0x15c"},
  {0x400, "0x400"},
  {0x500, "0x500"},
  {0x600, "0x600"},
  {0x700, "0x700"}};

static const struct RegInfo kInnoPLRegsSignalDisorder[kPLRegsSigDisorder] = {
  {0x00, "0x00"},
  {0x14, "0x14"},
  {0x18, "0x18"},
  {0x1c, "0x1c"},
  {0xe4, "0xe4"},
  {0xf4, "0xf4"},
  {0x108, "0x108"},
  {0x110, "0x110"},
  {0x114, "0x114"},
  {0x118, "0x118"},
  {0x124, "0x124"},
  {0x130, "0x130"},
  {0x150, "0x150"},
  {0x24C, "0x24C"},
  {0x2A0, "0x2A0"},
  {0x400, "0x400"},
  {0x500, "0x500"},
  {0x600, "0x600"},
  {0x700, "0x700"}};

  // PS regs
static const struct RegInfo kInnoPSRegsReadTimeOut[kPSResNum] = {
  {0x04, "0x04"},
  {0x0c, "0x0c"},
  {0x10, "0x10"},
  {0x14, "0x14"},
  {0x34, "0x34"},
  {0x40, "0x40"},
  {0x44, "0x44"},
  {0x48, "0x48"}};

static const struct RegInfo kInnoPSRegsSignalDisorder[kPSRegsSigDisorder] = {
  {0x04, "0x04"},
  {0x2c, "0x2c"},
  {0x30, "0x30"},
  {0x34, "0x34"},
  {0x38, "0x38"},
  {0x40, "0x40"},
  {0xB0, "0xB0"},
  {0xB4, "0xB4"},
  {0x108, "0x108"},
  {0x128, "0x128"},
  {0x12C, "0x12C"},
  {0x140, "0x140"}};

class FpgaRegsCollector {
 public:
  explicit FpgaRegsCollector(bool live_direct_mode = false);
  ~FpgaRegsCollector();
  FpgaRegsCollector(const FpgaRegsCollector& rhs) = default;
  FpgaRegsCollector& operator= (const FpgaRegsCollector& rhs) = default;

 public:
  void collect(const std::string& str,
               const RegInfo pl_regs[], int pl_len,
               const RegInfo ps_regs[], int ps_len);
  void reset();
  inline const std::string& get_info() {
    return info_;
  }
  inline int get_reg_read_times() {
    return reg_read_times_;
  }
  inline void set_lidar_live_direct_mode(bool live_direct_mode) {
    live_direct_mode_ = live_direct_mode;
  }

 private:
  std::string info_;
  int reg_read_times_;
  bool live_direct_mode_;
};

}  // namespace innovusion

#endif  // SDK_FPGA_REGS_COLLECTOR_H_
