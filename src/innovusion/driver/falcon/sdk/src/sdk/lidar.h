
/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_LIDAR_H_
#define SDK_LIDAR_H_

#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <unordered_map>
#include <cstddef>
#include <map>
#include <mutex>  // NOLINT
#include <string>

#include "sdk_common/inno_lidar_api.h"
#include "sdk_common/lidar_base.h"
#include "sdk_common/resource_stats.h"
#include "sdk/firmware_ipc.h"
#include "sdk/params.h"
#include "sdk/misc_tables.h"
#include "sdk/lidar_clock.h"
#include "sdk/stage_read.h"
#include "sdk/stage_signal.h"
#include "sdk/stage_angle.h"
#include "sdk/stage_angle_job.h"
#include "sdk/stage_deliver.h"
#include "sdk/stage_deliver_job.h"
#include "sdk/stage_deliver2.h"
#include "sdk/stage_help.h"
#include "sdk/stage_noise_filter.h"
#include "sdk/stage_signal_job.h"
#include "utils/config.h"
#include "utils/log.h"
#include "sdk/system_stats.h"
#include "sdk/status_report.h"
#include "sdk/fault_manager.h"
#include "sdk/frame_sync.h"

namespace innovusion {
class ConsumerProducer;
class GpsData;
class LidarClock;
class LidarCommunication;
class MemPool;
class PtpData;
class NtpData;
class StageRead;
class StageSignal;
class StageAngle;
class StageDeliver;
class StageDeliver2;
class StageDeliverPointsJobPool;
class StageDeliverMessageJobPool;
class StageDeliverStatusJobPool;
class StageHelp;
class StageNoiseFilter;
class SystemStats;
class StatusReport;
class InnoThread;
class FaultManager;
class FwFaultSynchronizer;

class LidarConfig: public Config {
 public:
  LidarConfig() : Config() {
    min_v_roi = -25;
    max_v_roi = 25;
    min_h_roi = -60;
    max_h_roi = 60;
    // todo zhuhe
    encodes_per_polygon = 5;
  }
    const char* get_type() const override {
    return "Lidar_Lidar";
  }

  int set_key_value_(const std::string &key,
                             double value) override {
    SET_CFG(min_v_roi);
    SET_CFG(max_v_roi);
    SET_CFG(min_h_roi);
    SET_CFG(max_h_roi);
    SET_CFG(encodes_per_polygon);
    return -1;
  }

  int set_key_value_(const std::string &key,
                     const std::string value) override {
    // no string attribute
    return -1;
  }

  BEGIN_CFG_MEMBER()
  double min_v_roi;
  double max_v_roi;
  double min_h_roi;
  double max_h_roi;
  uint32_t encodes_per_polygon;
  END_CFG_MEMBER()
};

struct StatAdcDataDrop {
  uint32_t last;
  uint32_t diff;
  uint64_t total;
};

class InnoLidar : public InnoLidarBase {
  friend StageRead;
  friend StageSignal;
  friend StageAngle;
  friend StageNoiseFilter;
  friend StageDeliver;
  friend StageDeliver2;
  friend StageHelp;
  friend SystemStats;
  friend StatusReport;
  friend FaultManager;
  friend FwFaultSynchronizer;
  friend FrameSync;

 private:
  enum LidarModel {
    LIDAR_MODEL_NONE = 0,
    LIDAR_MODEL_A,
    LIDAR_MODEL_B,
    LIDAR_MODEL_C,
    LIDAR_MODEL_D,
    LIDAR_MODEL_E,
    LIDAR_MODEL_F,
    LIDAR_MODEL_G,
    LIDAR_MODEL_H,
    LIDAR_MODEL_I,
    LIDAR_MODEL_J,
    LIDAR_MODEL_K,
    LIDAR_MODEL_MAX,
  };

  enum LidarSource {
    LIDAR_SOURCE_NONE = 0,
    LIDAR_SOURCE_FILE,
    LIDAR_SOURCE_LIVE,
    LIDAR_SOURCE_MAX,
  };

  enum LidarModeStatusFlag {
    FLAG_HAS_STREAM,
  };

 public:
  // xxx todo: pick the right value
  static const size_t kSignalJobPoolSizeReserve = 30;
  static const size_t kSignalJobPoolSizeMinReserve = 20;
  static const size_t kSignalJobPoolSize = 1200;
  static const size_t kAngleJobPoolSize = 45;
  static const size_t kDeliverMessageMaxSize = 65536;
  static const size_t kDeliverPointsMaxBlockNumber = 1600;
  static const size_t kDeliverMessageJobPoolSize = 50;
  static const size_t kDeliverPointsJobPoolSize = 1000;
  static const size_t kDeliverStatusJobPoolSize = 10;
  static const size_t kDeliver2JobPoolSize = 40;
  static const size_t kMaxYamlSize = 1024 * 1024;
  static const size_t kStableTimeS = 10;   // 10s

 private:
  static const int kInvalidDetTemp = -10000;  //  -1000degC
  static const int kCPUNumber = 4;

 public:  // static methods
  static int reader_func(void *job, void *ctx, bool prefer);

 public:
  InnoLidar(const char *name, const char *lidar_ip,
            uint16_t port, bool use_tcp);
  InnoLidar(const char *name, const char *filename, int play_rate,
            int rewind, int64_t skip);
  ~InnoLidar();

 public:
  void print_stats(void);
  int set_mode(enum InnoLidarMode mode,
               enum InnoLidarMode *mode_before_change,
               enum InnoLidarStatus *status_before_change);
  int set_galvo_sync(bool enable);
  int set_galvo_start_low(bool enable);

  bool check_mode_status_fast(LidarModeStatusFlag flag);

  int get_mode_status_fast(enum InnoLidarMode *mode,
                           enum InnoLidarMode *pre_mode,
                           enum InnoLidarStatus *status,
                           uint32_t *duration_s = NULL);
  int get_mode_status(enum InnoLidarMode *mode,
                      enum InnoLidarMode *pre_mode,
                      enum InnoLidarStatus *status,
                      uint64_t *in_transition_mode_ms = NULL);
  bool in_calibration_mode();
  int get_attribute(const char *attribute, double *value);
  int get_attribute_string(const char *attribute,
                           char *buf, size_t buf_size);
  int set_attribute_string(const char *attribute,
                           const char *buf);
  int set_faults_save_raw(uint64_t value);
  int read_ps_reg(uint16_t off, uint32_t *value);
  int read_pl_reg(uint16_t off, uint32_t *value);
  int write_ps_reg(uint16_t off, uint32_t value);
  int write_pl_reg(uint16_t off, uint32_t value);
  int set_params_file(const char *lidar_model,
                      const char *yaml_filename);
  int set_config_name_value(const char *name,
                            const char *value);
  int set_reflectance_mode(enum InnoReflectanceMode);
  int set_return_mode(InnoMultipleReturnMode ret_mode);
  int set_roi(double hori_angle, double v_angle);
  int get_roi(double *h_angle, double *v_angle);
  int get_apd_cal_status(int *status);
  int set_motion_compensation(double velocity[3],
                              double angular_velocity[3]);
  int thread_setaffinity_np(size_t cpusetsize,
                            const cpu_set_t *cpuset,
                            int exclude_callback_thread);
  int get_fw_state(enum InnoLidarState *state,
                   int *error_code);
  int get_fw_version(char *buffer, int buffer_len);
  int get_fw_sequence_buf(char *buffer, int buffer_len);
  int get_sn(char *buffer, int buffer_len);
  int get_model(char *buffer, int buffer_len);
  int get_temperature(char *buffer, int buffer_len);
  int get_detector_temps(char *buffer, int buffer_len);
  int get_detector_temps(int64_t (&temp)[kInnoChannelNumber]);
  int get_motor_speeds(char *buffer, int buffer_len);
  int get_inner_faults_info(char* buffer,
                            int buffer_len,
                            bool from_network = false);
  int get_inner_faults_all_info(char* buffer, int buffer_len);
  int get_sync_cycle_status(char* buffer, int buffer_len);
  enum InnoGalvoMode get_galvo_mode();
  int get_dsp_packet_reg(uint32_t *value);
  int get_fpga_ts(uint64_t *ts_ns);
  int set_fpga_frame_sync_ts(uint64_t ts_ns);
  void frame_start_time_statistic(InnoDataPacket *packet);
  double get_motor_speed_config();

  void start();
  void stop();

  void set_status_counters(InnoStatusCounters *counters,
                           bool first_call);
  int get_overheat_thresholds(int16_t (&threshold)[2]);

  uint32_t get_encodes_per_polygon() {
    return encodes_per_polygon_;
  }

  const LidarParams &get_params() const {
    return params_;
  }

  const MiscTables &get_misc_tables() const {
    return misc_tables_;
  }

  std::string get_version_yaml();
  void recorder_write_yaml(enum InnoRecorderCallbackType type);

  void set_save_raw_data_flag(std::string cause) {
    if (stage_signal_) {
      stage_signal_->set_save_raw_data_flag(cause);
    }
  }
  inline bool is_live_lidar() const {
    return is_live_lidar_();
  }
  inline int get_invalid_temp() {
    return kInvalidDetTemp;
  }

  void init_sys_stats_config(const StatusReportConfig *config) {
    inno_log_verify(system_stats_,
                    "init system_stats_ config before create it?");
    system_stats_->init_config(config);
  }

  void set_raw_fault(enum InnoSubInFaults sub_fault_id,
                     bool is_sub_fault,
                     bool condition,
                     const char *format, ...)
    __attribute__((format(printf, 5, 6)));

  void set_raw_fault(enum InnoLidarInFault fid,
                     bool condition,
                     const char *format, ...)
    __attribute__((format(printf, 4, 5)));

  void set_history_fault(int fid,
                         const char *format, ...)
    __attribute__((format(printf, 3, 4)));

  void heal_raw_fault(enum InnoSubInFaults sub_fault_id,
                      bool is_sub_fault,
                      bool condition,
                      const char *format, ...)
    __attribute__((format(printf, 5, 6)));

  void heal_raw_fault(enum InnoLidarInFault fid,
                      bool condition,
                      const char *format, ...)
    __attribute__((format(printf, 4, 5)));

  inline void update_data_packet(InnoStatusPacket* pkt) {
    fault_manager_->update_data_packet(pkt);
  }

  inline bool get_current_fault_status(int fid) {
    return fault_manager_->get_current_fault_status(fid);
  }

  inline bool get_fault_inhibit_status(enum InnoLidarInFault fid) {
    return fault_manager_->get_fault_inhibit_status(fid);
  }

  inline bool get_fault_enable_status(enum InnoLidarInFault fid,
                                      bool is_sub_fault = false) {
    return fault_manager_->get_fault_enable_status(static_cast<int>(fid),
                                                   is_sub_fault);
  }

  inline uint32_t get_confidence_seq(enum ProcessStage stage,
                                     enum ConfidenceLevel conf_level) {
    return conf_subscriber_.get_conf_seq_info(stage, conf_level);
  }

  inline ConfidenceSubscriber* get_conf_subscriber() {
    return &conf_subscriber_;
  }

  inline bool overheat_check() {
    return fault_manager_->overheat_check();
  }

  inline double get_streaming_start_ts() {
    return stage_read_->get_streaming_start_ts();
  }

  inline void get_cpu_usage(uint64_t (&cpu_usage)[kCPUNumber]) {
    if (system_stats_) {
      system_stats_->get_cpu_usage(cpu_usage);
    }
  }

  void set_apd_report_interval(int value) {
    apd_report_interval_ms_ = value;
  }

  void set_read_fw_faults(bool need_read) {
    if (status_report_) {
      status_report_->set_read_fw_faults(need_read);
    }
  }

  inline bool noise_level_high() {
    return stage_noise_filter_phase0_->noise_level_high() ||
           stage_noise_filter_phase1_->noise_level_high();
  }

  inline uint16_t get_fw_sequence() {
    return fw_sequence_;
  }

 protected:
  StageSignalJob *alloc_signal_job(void);
  StageAngleJob *alloc_angle_job(void);
  StageDeliverStatusJob *alloc_deliver_status_job(void);
  StageDeliverMessageJob *alloc_deliver_message_job(void);
  StageDeliverPointsJob *alloc_deliver_points_job(void);
  StageDeliver2Job *alloc_deliver2_job(void);

  void free_signal_job(StageSignalJob *job);
  void free_angle_job(StageAngleJob *job);
  void free_deliver_status_job(StageDeliverStatusJob *job);
  void free_deliver_message_job(StageDeliverMessageJob *job);
  void free_deliver_points_job(StageDeliverPointsJob *job);
  void free_deliver2_job(StageDeliver2Job *job);
  void add_stage_signal_job(StageSignalJob *job);
  void add_stage_angle_job(StageAngleJob *job);
  void add_stage_noise_filter_phase0_job(StageAngleJob *job);
  void add_stage_noise_filter_phase1_job(StageAngleJob *job);
  void add_stage_deliver_job(StageAngleJob *job);
  void add_stage_deliver2_job(StageDeliver2Job *job);
  int before_read_start(void);
  void add_config(Config *c);
  void remove_config(Config *c);
  void update_config();

  void stats_update_packet_bytes(enum ResourceStats::PacketType type,
                                 size_t packet, size_t byte);
  void stats_update_packet_bytes(size_t ref_count_sum,
                                 uint64_t intensity_sum);

  bool read_fw_ipc_is_ready() {
    return fw_ipc_.fw_is_ready();
  }
  int64_t read_fw_ipc(enum IpcItem item, int64_t default_v);
  bool has_window_correction() {
    return lidar_model_ == LIDAR_MODEL_I;
  }

  FirmwareIpc* get_fw_ipc() {
    return &fw_ipc_;
  }

  LidarClock& get_clock();

  // void update_status_check_time_();

 private:
  void init_();
  void setup_job_pools_();
  void free_job_pools_();
  InnoLidarBase::State get_state_();
  bool is_live_lidar_() const;
  bool is_live_direct_memory_lidar_() const;
  char *get_yaml_buffer_();
  int parse_lidar_yaml_();

  int set_hori_roi_(double hori_roi_angle);
  int set_vertical_roi_(double vertical_roi_angle);
  int get_vertical_roi_(double *vertical_roi_angle);
  int get_from_uds_rest(const char *did_name, char *buffer, size_t buf_size);
  int hw_part_from_board(char *buffer, int buf_len);
  int mfg_date_from_sn(const char *sn_buff, char *mfg_date, int buf_len);
  int print_to_json(const char *json_key, const char *json_val,
                    char *buffer, size_t buf_size);
  int set_ivparams_(const char *name, const char *value);
  int set_raw_channel_(int32_t channel);
  int start_frame_sync_with_lock_(double sync_time, bool restart_force);

 private:
  /* source info */
  enum LidarSource lidar_source_;
  char *ip_;
  uint16_t port_;
  uint16_t service_port_;
  bool use_tcp_;

  char *filename_;

  std::mutex mode_mutex_;
  enum InnoLidarMode work_mode_;
  enum InnoLidarMode last_work_mode_;
  enum InnoLidarMode pre_work_mode_;
  enum InnoLidarStatus work_status_;
  uint64_t last_in_transition_mode_epoch_ms_;

  LidarConfig config_base_;
  LidarConfig config_;

  /* communication with lidar */
  LidarCommunication *comm_;

  LidarClock clock_;
  LidarClockConfig clock_config_;

  /* lidar basic configs */
  enum LidarModel lidar_model_;
  char *yaml_filename_;
  char *yaml_from_live_;
  char *yaml_from_yaml_file_;
  char *yaml_from_data_file_;
  LidarParams params_base_;
  LidarParams params_;
  std::mutex params_mutex_;
  std::condition_variable params_cond_;
  enum InnoTimeSyncConfig lidar_time_config_;
  bool use_extended_format_;

  /* controlled by API */
  enum InnoReflectanceMode reflectance_mode_;
  enum InnoMultipleReturnMode multiple_return_;
  double hori_roi_angle_;      // in degree
  int hori_roi_angle_unit_;    // in unit
  double vertical_roi_angle_;  // in degree
  int vertical_roi_angle_unit_;    // in unit
  InnoGalvoMode galvo_mode_;
  uint32_t encodes_per_polygon_;

  /* processing pipeline */
  ConsumerProducer *cp_read_;
  ConsumerProducer *cp_signal_;
  ConsumerProducer *cp_angle_;
  ConsumerProducer *cp_noise_filter_phase0_;
  ConsumerProducer *cp_noise_filter_phase1_;
  ConsumerProducer *cp_deliver_;
  ConsumerProducer *cp_deliver2_;
  ConsumerProducer *cp_help_;
  InnoThread *it_status_;

  StageRead *stage_read_;
  StageSignal *stage_signal_;
  StageAngle *stage_angle_;
  StageNoiseFilter *stage_noise_filter_phase0_;
  StageNoiseFilter *stage_noise_filter_phase1_;
  StageDeliver *stage_deliver_;
  StageDeliver2 *stage_deliver2_;
  bool force_xyz_pointcloud_;
  StageHelp *stage_help_;
  StatusReport *status_report_;

  StageAngleJobPool *stage_angle_job_pool_;
  StageSignalJobPool *stage_signal_job_pool_;
  StageDeliverMessageJobPool *stage_deliver_message_job_pool_;
  StageDeliverPointsJobPool *stage_deliver_points_job_pool_;
  StageDeliverStatusJobPool *stage_deliver_status_job_pool_;
  StageDeliver2JobPool *stage_deliver2_job_pool_;

  SystemStats *system_stats_;
  FirmwareIpc fw_ipc_;

  struct StatAdcDataDrop stat_adc_data_drop_[2];

  FaultManager* fault_manager_;
  LidarModePublisher mode_publisher_;
  ConfidenceSubscriber conf_subscriber_;
  MiscTables misc_tables_;

  int apd_report_interval_ms_ {0};

  uint16_t fw_sequence_ {0};
  FrameSync *frame_sync_{NULL};
  bool frame_sync_enable_{false};
  double frame_sync_time_{0};
  std::mutex frame_sync_mutex_;
};

}  // namespace innovusion
#endif  // SDK_LIDAR_H_
