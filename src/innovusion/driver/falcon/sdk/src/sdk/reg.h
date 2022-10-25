/*
 *  Copyright (C) 2021 Innovusion Inc.
 *
 *  License: BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef SDK_REG_H_
#define SDK_REG_H_

#include <stdint.h>
#include <stddef.h>

#ifndef __MINGW64__
#include <sys/mman.h>
#else
#include "sdk/mman-win32/mman.h"
#endif

#include <mutex>  // NOLINT

#include "utils/log.h"

namespace innovusion {

#define PS_BASEADDR (0x43C0000000L)
#define PS_HIGHADDR (0x43C0000FFFL)
#define PS_LENGTH   ((int)(PS_HIGHADDR - PS_BASEADDR + 1))

#define PL_BASEADDR (0x43C2000000L)
#define PL_HIGHADDR (0x43C2000FFFL)
#define PL_LENGTH   ((int)(PL_HIGHADDR - PL_BASEADDR + 1))
#define ERR_REG (0x1F0)

#define ADC_DROP_PACKET_REG   (0x014)
#define MOTOR_DROP_PACKET_REG   (0x018)

// DMA Control Register: XPAR_AXILITE2REG_0_BASEADDR, offset 0x004
#define DMA_CONTROL_REGISTER (0x4)
#define DMA_RAW_DATA_SELECTED (0x2)  // bit 1

// Capabilities register: XPAR_AXILITE2REG_0_BASEADDR, offset 0x104
#define CAPABILITIES_REGISTER (0x104)
#define RAW_DATA_EXTRACT_SUPPORT (0x10)  // bit 4

// RAW DATA DMA Control Register: XPAR_AXILITE2REG_0_BASEADDR, offset 0x04C
#define RAW_DATA_DMA_CONTROL_REGISTER (0x4c)
#define DMA_RAW_DATA_TRIGGER (0xf0)      // bit 4-7
#define BOWL_MOTOR_INDEX_TRIGGER (0x10)  // bit 4
#define DMA_RAW_DATA_CHANNEL (0xf)       // bit 0-3

#define INNO_MODE_STATUS      (0x200)
#define INNO_MODE_DURATION_S  (0x204)
// bit[31:24] request mode
// bit[23:16] last mode
// bit[15: 8] current status
// bit[ 7: 0] current mode


// 32 bits packet received from DSP
#define PS_DSP_PACKET (0xB4)
#define PS_DSP_PACKET_FRAME_SYNC_STATUS (0x1 << 22)

// FPGA PULSE FILTER MODE CONTROL
// 4C[2:0] = 0 : disable filter
//         = 1 : pass strongest 1 pulse
//         = 2 : pass strongest 2 pulses
//         = 3 : pass strongest and furthest pulses
//         = 4 : normally send only the strongest pulse and will also send
//               second-strongest pulse only if second-strongest pulse has
//               shorter distance.
#define PL_PULSE_FILTER_CONTROL_REGISTER (0x4c)
#define PL_PULSE_FILTER_CONTROL_REGISTER_MODE (0x7)

// TIME SYNC
//  PL register 2E4 bit[31:4] is fsync_ts_low,
//  PL register 2E8[31:0] is fsync_ts_high,
//  when FPGA timestamp
//  fpga_ts[59:0] == {fsync_ts_high[31:0],fsync_ts_low[31:4]}
//  &&
//  {fsync_ts_high[31:0],fsync_ts_low[31:4]} != 0,
//  then FRAME_SYNC is set to high for 100us.

//  write fsync_ts registers:
//  While we write fsync_ts_low, fpga has an internal temporary register to save
//  this value, and fpga does not write fsync_ts_low register, it writes that
//  temporary register.
//  While we write fsync_ts_high, fpga will write new value to fsync_ts_high
//  register, and copy the temporary register value to fsync_ts_low register
//  at the same time. so fsync_ts 2 registers are changed at the same time
#define PL_FRAME_SYNC_LOW (0x2E4)
#define PL_FRAME_SYNC_HIGH (0x2E8)

// read fpga_ts:
// While we read fpga_ts_low, fpga return fpga_ts_low value, and save
// fpga_ts_high value to an internal temporary register at the same time.
// While we read fpga_ts_high, fpga will return the temporary register value.
#define PL_FPGA_TS_LOW (0x1E4)
#define PL_FPGA_TS_HIGH (0x1E8)

class Reg {
 private:
  static const uint32_t kMaxChannelNum = 4;
  static const uint16_t kChaBaseAddr = 0x400;
  static const uint16_t kChbBaseAddr = kChaBaseAddr + 0x100;
  static const uint16_t kChcBaseAddr = kChaBaseAddr + 0x200;
  static const uint16_t kChdBaseAddr = kChaBaseAddr + 0x300;
  static const uint16_t ch_base_addr_[kMaxChannelNum];

 private:
  static Reg *r_;
  static std::mutex static_mutex_;

 protected:
  Reg();
  Reg(const Reg&) = delete;             // disallow assign
  Reg& operator=(const Reg&) = delete;  // disallow copy

 private:
  static Reg *get_instance_() {
    std::unique_lock<std::mutex> lk(static_mutex_);
    if (r_ == nullptr) {
      r_ = new Reg();
      inno_log_verify(r_, "reg");
    }
    return r_;
  }

  ~Reg();

 public:
  static int ps_read(uint16_t off, uint32_t *value) {
    return get_instance_()->ps_read_(off, value);
  }

  static int ps_write(uint16_t off, uint32_t data) {
    return get_instance_()->ps_write_(off, data);
  }

  static int pl_read(uint16_t off, uint32_t *value) {
    return get_instance_()->pl_read_(off, value);
  }

  static int pl_write(uint16_t off, uint32_t data) {
    return get_instance_()->pl_write_(off, data);
  }

  static uint32_t read_error_reg() {
    return get_instance_()->read_error_reg_();
  }

  static int pl_rd_mask_mod(uint16_t offset, uint32_t val, uint32_t mask) {
    return get_instance_()->pl_rd_mask_mod_(offset, val, mask);
  }
  static uint16_t get_chn_addr_(int chn, uint32_t off);

 private:
  int ps_read_(uint16_t off, uint32_t *value);
  int ps_write_(uint16_t off, uint32_t data);
  int pl_read_(uint16_t off, uint32_t *value);
  int pl_write_(uint16_t off, uint32_t data);
  uint32_t read_error_reg_();
  int pl_rd_mask_mod_(uint16_t offset, uint32_t val, uint32_t mask);
  bool is_pl_invalid_() const {
    return MAP_FAILED == pl_map_base_;
  }
  bool is_ps_invalid_() const {
    return MAP_FAILED == ps_map_base_;
  }

 private:
  int mem_fd_;
  union {
    void *ps_map_base_void_;
    volatile char *ps_map_base_;
  };
  union {
    void *pl_map_base_void_;
    volatile char *pl_map_base_;
  };
};

}  // namespace innovusion
#endif  // SDK_REG_H_
