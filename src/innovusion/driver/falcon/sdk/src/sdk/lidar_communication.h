/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_LIDAR_COMMUNICATION_H_
#define SDK_LIDAR_COMMUNICATION_H_

#include <mutex>  // NOLINT

#include "sdk_common/inno_lidar_api.h"
#include "utils/net_manager.h"
#include "utils/inno_lidar_log.h"

namespace innovusion {

class LidarCommunication: public NetManager {
 public:
  class StatusCounters {
   public:
    StatusCounters() {
      memset(this, 0, sizeof(*this));
    }
    ~StatusCounters();
    int set(const char *input);

    enum InnoTimeSyncConfig clock_config;
    uint64_t uptime;
    uint32_t stream_count;
    enum InnoLidarState stream_status;
    size_t data_sent;
    int32_t error_code;
    uint32_t idle_loop;
    uint32_t lose_counter_1;
    uint32_t lose_counter_2;
  };

 public:
  static const uint32_t kCapabilitiesRegister = 0x1104;
  static const uint32_t kDmaControlRegister = 0x1004;
  static const uint32_t kDmaRawDataSelected = 0x2;  // bit 1
  static const uint32_t kExtendedPacketFormat = 0x20000;
  static const int kNOPRoi = 10000;

 public:
  LidarCommunication(const char *ip,
                     uint16_t port,
                     uint16_t service_port,
                     double timeout_sec);
  ~LidarCommunication();

  double get_config_section_key_value(const char *section,
                                      const char *key,
                                      double default_value);
  int set_config_section_key_value(const char *section,
                                   const char *key,
                                   const char *value);
  int reg_read(uint32_t addr, uint32_t *data);
  int reg_write(uint32_t addr, uint32_t data);
  inline int read_ps_reg(uint32_t addr, uint32_t *data) {
    uint32_t ps_addr = 0x1000 | addr;
    return reg_read(ps_addr, data);
  }

  inline int write_ps_reg(uint32_t addr, uint32_t data) {
    uint32_t ps_addr = 0x1000 | addr;
    return reg_write(ps_addr, data);
  }

  double get_frame_rate();
  int get_galvo_mode();
  int get_board_name(char *buffer, size_t buffer_len);
  int get_did(char *buffer, size_t buffer_len, const char* did);
  int get_fw_sequence(char *buffer, size_t buffer_len);
  int get_fw_version(char *buffer, size_t buffer_len);
  int get_sn(char *buffer, size_t buffer_len);
  int get_model(char *buffer, size_t buffer_len);
  int get_temperature(char *buffer, size_t buffer_len);
  int get_detector_temps(char *buffer, size_t buffer_len);
  int get_motor_speeds(char *buffer, size_t buffer_len);
  int get_geo_yaml(char *buffer, size_t buffer_len);
  int get_ext_packet_format(bool *format);
  int get_status(StatusCounters *counters, bool get_diff);
  int get_mode_status(enum InnoLidarMode *mode,
                      enum InnoLidarMode *pre_mode,
                      enum InnoLidarStatus *status);
  int get_apd_cal_status(int *status);
  int get_calibration_mode_id(int *calibration_id);
  const char *get_mode_section(InnoLidarMode mode);
  const char *get_current_mode_section();
  int clear_fw_inner_faults(uint32_t fid);
  double get_motor_speed_config();


  int set_raw_capture_mode(bool);
  int set_vertical_roi(double angle);
  int get_vertical_roi(double *angle);
  int set_reboot(int value);

  int open_streaming(void);
  int start_streaming(bool is_tcp);
  void close_streaming();
  int get_streaming_fd();
  int send_stop();
  int set_mode(enum InnoLidarMode mode,
               enum InnoLidarMode *pre_mode,
               enum InnoLidarStatus *status);

 private:
  int send_command_and_save_reply_(char *buffer,
                                   size_t buffer_len,
                                   const char *cmd);
  int get_reply_info_(char *recv_buffer,
                      size_t recv_buffer_len,
                      const char *send_buffer);

 private:
  uint16_t service_port_;
  uint32_t first_ctrlor_lose_1_;
  uint32_t first_ctrlor_lose_2_;
  uint32_t last_ctrlor_lose_1_;
  uint32_t last_ctrlor_lose_2_;
  size_t get_status_called_cnt_;
  std::mutex mutex_;
  int streaming_fd_;
};
}  // namespace innovusion
#endif  // SDK_LIDAR_COMMUNICATION_H_
