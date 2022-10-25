/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_FIRMWARE_IPC_H_
#define SDK_FIRMWARE_IPC_H_

#include <assert.h>

#include "utils/shared_memory.h"

namespace innovusion {
enum IpcItem {
  IPC_ITEM_FW_INITIALIZED = 0,
  IPC_ITEM_SW_INITIALIZED,
  IPC_ITEM_TEMPERATURE_FPGA_10th_C,
  IPC_ITEM_TEMPERATURE_LASER_10th_C,
  IPC_ITEM_TEMPERATURE_ADC_10th_C,
  IPC_ITEM_TEMPERATURE_BOARD_10th_C,
  IPC_ITEM_TEMPERATURE_DET_0_10th_C,
  IPC_ITEM_TEMPERATURE_DET_1_10th_C,
  IPC_ITEM_TEMPERATURE_DET_2_10th_C,
  IPC_ITEM_TEMPERATURE_DET_3_10th_C,
  IPC_ITEM_TEMPERATURE_OTHER_0_10th_C,
  IPC_ITEM_TEMPERATURE_OTHER_1_10th_C,
  IPC_ITEM_TEMPERATURE_OTHER_2_10th_C,
  IPC_ITEM_HEATER_CURRENT_MA,
  IPC_ITEM_MOTOR_RPM_1000th,
  IPC_ITEM_GALVO_FPM_1000th,
  IPC_ITEM_MOTOR_ROTATION_TOTAL,
  IPC_ITEM_GALVO_ROUND_TOTAL,
  IPC_ITEM_MOISTURE_INDEX_0,
  IPC_ITEM_MOISTURE_INDEX_1,
  IPC_ITEM_WINDOW_BLOCKAGE_INDEX_0,
  IPC_ITEM_WINDOW_BLOCKAGE_INDEX_1,
  IPC_ITEM_MOTOR_0,
  IPC_ITEM_MOTOR_1,
  IPC_ITEM_MOTOR_2,
  IPC_ITEM_MOTOR_3,
  IPC_ITEM_MOTOR_4,
  IPC_ITEM_MOTOR_5,
  IPC_ITEM_GALVO_0,
  IPC_ITEM_GALVO_1,
  IPC_ITEM_GALVO_2,
  IPC_ITEM_GALVO_3,
  IPC_ITEM_GALVO_4,
  IPC_ITEM_GALVO_5,
  IPC_ITEM_LASER_0,
  IPC_ITEM_LASER_1,
  IPC_ITEM_LASER_2,
  IPC_ITEM_LASER_3,
  IPC_ITEM_LASER_4,
  IPC_ITEM_LASER_5,
  IPC_ITEM_LIFELONG_UPTIME = 44,

  // one fault uses 2 items
  // 1. set fault
  // 2. heal fault
  // 256 / 2 = 128 faults
  // InnoLidarInFault::INNO_LIDAR_IN_FAULT_MAX <= 128
  IPC_ITEM_FAULTS_BEGIN = 256,
  IPC_ITEM_FAULTS_END   = 511,

  IPC_MAX_FROM_FIRMWARE = 511,

  // Items sent from PCS to Firmware
  IPC_ITEM_PT_INTENSITY_CH0 = 512,
  IPC_ITEM_PT_INTENSITY_CH1,
  IPC_ITEM_PT_INTENSITY_CH2,
  IPC_ITEM_PT_INTENSITY_CH3,
  IPC_ITEM_SEQUENCE_BEGIN,
  IPC_ITEM_SEQUENCE_END,
  IPC_ITEM_SEQUENCE_DATA0,
  IPC_ITEM_SEQUENCE_DATA1,
  IPC_ITEM_SEQUENCE_DATA2,
  IPC_ITEM_SEQUENCE_DATA3,
  IPC_ITEM_SEQUENCE_DATA4,
  IPC_ITEM_SEQUENCE_DATA5,
  IPC_ITEM_SEQUENCE_DATA6,
  IPC_ITEM_SEQUENCE_DATA7,
  IPC_ITEM_SEQUENCE_DATA8,
  IPC_ITEM_SEQUENCE_DATA9,
  IPC_ITEM_SEQUENCE_DATA10,
  IPC_ITEM_SEQUENCE_DATA11,
  IPC_ITEM_SEQUENCE_DATA12,
  IPC_ITEM_SEQUENCE_DATA13,
  IPC_ITEM_MAX
};

class FirmwareIpc {
 private:
  static const constexpr char *path_ = "/bin/sh";
  static const int kProjId_ = 82;
  static const size_t kMemSize_ = 8192;
  static const size_t kMagic_ = 0x3754765200000000UL;
  static const size_t kVersion_ = 1;
  static const size_t kVersionAndMagic_ = kMagic_ | kVersion_;

 public:
  FirmwareIpc() : shared_memory_(path_, kProjId_, kMemSize_) {
    assert(IPC_ITEM_MAX < kMemSize_ / sizeof(uint64_t));
    write(IPC_ITEM_SW_INITIALIZED, kVersionAndMagic_);
  }

  ~FirmwareIpc() {
  }

  inline int64_t read(enum IpcItem item, int64_t *value) const {
    if (shared_memory_.is_valid()) {
      const volatile void *v = shared_memory_.get_memory();
      const volatile int64_t *v64 =
          reinterpret_cast<const volatile int64_t *>(v);
      *value = v64[item];
      return true;
    }
    return false;
  }

  inline bool write(enum IpcItem item, int64_t value) {
    if (shared_memory_.is_valid()) {
      volatile void *v = shared_memory_.get_memory();
      volatile int64_t *v64 = reinterpret_cast<volatile int64_t *>(v);
      v64[item] = value;
      return true;
    }
    return false;
  }

  bool fw_is_ready() {
    return is_ready_(IPC_ITEM_FW_INITIALIZED);
  }

  bool sw_is_ready() {
    return is_ready_(IPC_ITEM_SW_INITIALIZED);
  }

 private:
  bool is_ready_(enum IpcItem item) {
    if (shared_memory_.is_valid()) {
      const volatile void *v = shared_memory_.get_memory();
      const volatile int64_t *v64 =
          reinterpret_cast<const volatile int64_t *>(v);
      return is_right_version_(v64, item);
    }
    return false;
  }

  static bool is_right_version_(const volatile int64_t *mem,
                                enum IpcItem item) {
    return mem[item] == kVersionAndMagic_;
  }

 private:
  SharedMemory shared_memory_;
};

}  // namespace innovusion

#endif  // SDK_FIRMWARE_IPC_H_
