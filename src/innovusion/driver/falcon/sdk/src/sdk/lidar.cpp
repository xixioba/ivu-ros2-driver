/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "sdk/lidar.h"

#include <unistd.h>
#include <sys/time.h>

#include <limits>

#include "sdk/lidar_clock.h"
#include "sdk/lidar_communication.h"
#include "sdk/reg.h"
#include "sdk/status_report.h"
#include "sdk/system_stats.h"
#include "utils/consumer_producer.h"
#include "utils/inno_thread.h"
#include "utils/utils.h"
#include "utils/mem_allocator.h"
#include "sdk/frame_sync.h"

namespace innovusion {

#ifdef __MINGW64__
char *strndup(const char *str, int chars) {
  char *buffer;
  int n;

  buffer = (char *)malloc(chars + 1); // NOLINT
  if (buffer) {
    for (n = 0; ((n < chars) && (str[n] != 0)); n++) buffer[n] = str[n];
    buffer[n] = 0;
  }

  return buffer;
}
#endif

/**********************
 * constructor + destructor
 **********************/

InnoLidar::InnoLidar(const char *name,
                     const char *lidar_ip,
                     uint16_t port,
                     bool use_tcp)
    : InnoLidarBase("Lidar_", name)
    , fault_manager_(NULL) {
  init_();
  const char *sep = strchr(lidar_ip, ':');
  if (sep) {
    unsigned int len = sep - lidar_ip;
    ip_ = strndup(lidar_ip, len);
    service_port_ = atoi(sep + 1);
    inno_log_verify(service_port_ > 0, "invalid lidar ip %s", lidar_ip);
  } else {
#ifndef __MINGW64__
    ip_ = strdup(lidar_ip);
#else
    ip_ = _strdup(lidar_ip);
#endif
    service_port_ = 0;
  }
  port_ = port;
  use_tcp_ = use_tcp;
  lidar_source_ = LIDAR_SOURCE_LIVE;

  // start with default timeout value (2.5 seconds)
  double time_out_s = 2.5;
  if (is_live_direct_memory_lidar_()) {
    time_out_s = 5;
  }
  comm_ = new LidarCommunication(ip_, port_, service_port_, time_out_s);
  inno_log_verify(comm_,
                  "%s cannot allocate comm",
                  name_);
  if (is_live_direct_memory_lidar_()) {
    size_t base_addr = 0x10000000;
    size_t high_addr = 0x1DFFEFFF;
    // TODO(@Yahui Hu): get value from configuration file
    MemAllocDelegate::get_instance()->setup_live_direct_memory_mode(base_addr,
                                                                    high_addr);
  }

  stage_read_ = new StageRead(this, comm_, use_tcp, 1);
  inno_log_verify(stage_read_,
                  "%s cannot allocate stage_read",
                  name_);
  inno_log_info("%s uses live lidar at %s, "
                "service port is %hu, use_tcp=%d",
                name_, ip_, service_port_, use_tcp);
  return;
}

InnoLidar::InnoLidar(const char *name, const char *filename,
                     int play_rate, int rewind, int64_t skip)
    : InnoLidarBase("Lidar_", name) {
  init_();
  filename_ = strdup(filename);
  lidar_source_ = LIDAR_SOURCE_FILE;
  comm_ = NULL;

  lidar_time_config_ = INNO_TIME_SYNC_CONFIG_FILE;

  set_play_rate_(play_rate);

  char *yaml = NULL;
  size_t data_start_off = 0;
  int ret = StageRead::load_yaml_from_data_file(filename, &yaml,
                                                &data_start_off);
  if (ret == 0) {
    inno_log_info("load yaml from data file");
    yaml_from_data_file_ = yaml;
  } else {
    data_start_off = 0;
    yaml_from_data_file_ = NULL;
  }

  stage_read_ = new StageRead(this, filename, play_rate_,
                              rewind, skip, data_start_off);
  inno_log_verify(stage_read_,
                  "%s cannot allocate stage_read",
                  name_);
  inno_log_info("%s open file %s, rate=%dMB/s %fX",
                name_, filename_,
                play_rate_, play_rate_x_);
  return;
}

InnoLidar::~InnoLidar() {
  inno_log_info("%s close", name_);
  bool still_up;
  {
    std::unique_lock<std::mutex> lk(last_stage_mutex_);
    still_up = last_stage_is_up_;
  }
  inno_log_verify(!still_up,
                  "forgot to call lidar stop?");

  remove_config(&config_base_);

  if (ip_) {
    free(ip_);
    ip_ = NULL;
  }
  if (filename_) {
    free(filename_);
    filename_ = NULL;
  }
  if (yaml_filename_) {
    free(yaml_filename_);
    yaml_filename_ = NULL;
  }
  if (yaml_from_live_) {
    free(yaml_from_live_);
    yaml_from_live_ = NULL;
  }
  if (yaml_from_data_file_) {
    free(yaml_from_data_file_);
    yaml_from_data_file_ = NULL;
  }
  if (yaml_from_yaml_file_) {
    free(yaml_from_yaml_file_);
    yaml_from_yaml_file_ = NULL;
  }
  if (comm_) {
    delete comm_;
    comm_ = NULL;
  }
  if (cpuset_) {
    free(cpuset_);
    cpuset_ = NULL;
  }
  if (fault_manager_) {
    delete fault_manager_;
    fault_manager_ = NULL;
  }

  delete(stage_read_);
  stage_read_ = NULL;

  return;
}


/**********************
 * private methods
 **********************/
void InnoLidar::init_() {
  // handle_
  inno_log_verify(handle_ == 0,
                  "%s cannot init twice handle_=%d", name_, handle_);
  {
    std::unique_lock<std::mutex> lk(static_mutex_s);
    handle_ = ++max_handle_s;
  }

  lidar_model_ = LIDAR_MODEL_NONE;
  yaml_filename_ = NULL;
  yaml_from_live_ = NULL;
  yaml_from_yaml_file_ = NULL;
  yaml_from_data_file_ = NULL;

  comm_ = NULL;
  ip_ = NULL;
  port_ = 0;
  service_port_ = 0;
  use_tcp_ = false;
  filename_ = NULL;

  play_rate_ = 0;
  play_rate_x_ = 0;
  galvo_mode_ = INNO_GALVO_MODE_NONE;

  work_mode_ = INNO_LIDAR_MODE_NONE;
  last_work_mode_ = INNO_LIDAR_MODE_NONE;
  pre_work_mode_ = INNO_LIDAR_MODE_NONE;
  work_status_ = INNO_LIDAR_STATUS_NONE;
  last_in_transition_mode_epoch_ms_ = 0;

  multiple_return_ = INNO_MULTIPLE_RETURN_MODE_NONE;
  reflectance_mode_ = INNO_REFLECTANCE_MODE_NONE;
  hori_roi_angle_ = 0;
  hori_roi_angle_unit_ = 0;
  vertical_roi_angle_ = 0;
  vertical_roi_angle_unit_ = 0;

  fw_sequence_ = 0;

  // internal states
  cp_read_ = NULL;
  cp_signal_ = NULL;
  cp_angle_ = NULL;
  cp_noise_filter_phase0_ = NULL;
  cp_noise_filter_phase1_ = NULL;
  cp_deliver_ = NULL;
  cp_deliver2_ = NULL;
  cp_help_ = NULL;
  it_status_ = NULL;

  stage_signal_ = NULL;
  stage_angle_ = NULL;
  stage_noise_filter_phase0_ = NULL;
  stage_noise_filter_phase1_ = NULL;
  stage_deliver_ = NULL;
  stage_deliver2_ = NULL;
  stage_help_ = NULL;
  status_report_ = NULL;

  force_xyz_pointcloud_ = false;

  system_stats_ = NULL;

  memset(stat_adc_data_drop_, 0, sizeof(stat_adc_data_drop_));

  add_config(&config_base_);
  add_config(&this->clock_config_);

  config_.copy_from_src(&config_base_);
  encodes_per_polygon_ = config_.encodes_per_polygon;
  fault_manager_ = new FaultManager(this);
  inno_log_verify(fault_manager_, "fault_manager_");
  LidarModeSubscriber* subscriber = fault_manager_->get_mode_subscriber();
  mode_publisher_.add_subscriber(subscriber);
  frame_sync_ = nullptr;
}

void InnoLidar::setup_job_pools_() {
  stage_signal_job_pool_ = new StageSignalJobPool(
      "SignalJobPool",
      kSignalJobPoolSize,
      StageSignal::kWriteBlockSize,
      StageSignal::kMaxLeftoverSize);
  inno_log_verify(stage_signal_job_pool_, "stage_signal_job_pool_");
  unsigned int block_number = InnoConsts::kMaxTriggerPerSecond *
          InnoConsts::kSecondInMinute / InnoConsts::kMinPolygonRPM /
          config_.encodes_per_polygon + 1 + ScanLine::kMaxBlocksInScanLine * 2;
  stage_angle_job_pool_ = new StageAngleJobPool("AngleJobPool",
                          kAngleJobPoolSize *
                          (config_.encodes_per_polygon > 2 ?
                          config_.encodes_per_polygon - 1 :
                          config_.encodes_per_polygon),
                          block_number,
                          config_.encodes_per_polygon);
  inno_log_verify(stage_angle_job_pool_, "stage_angle_job_pool_");
  if (is_live_direct_memory_lidar_()) {
    // verify every RawChannelPoint array is 64 bytes align
    // (size of one cache line of A53)
    // that is AngleJobPool is 64 bytes align
    // and uint size of pool is multiple of 64 bytes
    // and sizeof(StageAngleJob) is multiple of 64 bytes
    // and sizeof(RawBlock) is multiple of 64 bytes
    // and sizeof(RawChannelPoint array in block) is multiple of 64 bytes.
    inno_log_verify((sizeof(StageAngleJob) & 0x3f) == 0,
                    "StageAngleJob size should be multiple of 64 bytes");
    inno_log_verify((sizeof(RawBlock) & 0x3f) == 0,
                    "RawBlock size should be multiple of 64 bytes");
    inno_log_verify(((sizeof(RawChannelPoint)
    * kInnoChannelNumber * kInnoMaxMultiReturn) & 0x3f) == 0,
                    "array of RawChannelPoint size should be multiple"
                    " of 64 bytes");
  }

  stage_deliver_points_job_pool_ =
      new StageDeliverPointsJobPool("DeliverPointsJobPool",
                                    kDeliverPointsJobPoolSize,
                                    kDeliverPointsMaxBlockNumber);
  inno_log_verify(stage_deliver_points_job_pool_,
                  "stage_deliver_points_job_pool_");

  stage_deliver_message_job_pool_ =
      new StageDeliverMessageJobPool("DeliverMessageJobPool",
                                     kDeliverMessageJobPoolSize,
                                     kDeliverMessageMaxSize);
  inno_log_verify(stage_deliver_message_job_pool_,
                  "stage_deliver_message_job_pool_");

  stage_deliver_status_job_pool_ =
      new StageDeliverStatusJobPool("DeliverStatusJobPool",
                                    kDeliverStatusJobPoolSize);
  inno_log_verify(stage_deliver_status_job_pool_,
                  "stage_deliver_status_job_pool_");

  stage_deliver2_job_pool_ =
      new StageDeliver2JobPool("Deliver2JobPool",
                               kDeliver2JobPoolSize,
                               StageDeliver::kDeliverMaxPacketNumber);
  inno_log_verify(stage_deliver2_job_pool_,
                  "stage_deliver2_job_pool_");

  // stage_help_job_pool_ = new StageHelpJobPool();
  // inno_log_verify(stage_help_job_pool_, "stage_help_job_pool_");
}

void InnoLidar::free_job_pools_() {
  delete stage_angle_job_pool_;
  stage_angle_job_pool_ = NULL;
  delete stage_signal_job_pool_;
  stage_signal_job_pool_ = NULL;
  delete stage_deliver_message_job_pool_;
  stage_deliver_message_job_pool_ = NULL;
  delete stage_deliver_points_job_pool_;
  stage_deliver_points_job_pool_ = NULL;
  delete stage_deliver_status_job_pool_;
  stage_deliver_status_job_pool_ = NULL;
  delete stage_deliver2_job_pool_;
  stage_deliver2_job_pool_ = NULL;
}

InnoLidarBase::State InnoLidar::get_state_() {
  return stage_read_->get_state();
}

bool InnoLidar::is_live_lidar_() const {
  return comm_ != NULL;
}

bool InnoLidar::is_live_direct_memory_lidar_() const {
  return is_live_lidar_() && !use_tcp_;
}

int InnoLidar::set_mode(enum InnoLidarMode mode,
                        enum InnoLidarMode *mode_before_change,
                        enum InnoLidarStatus *status_before_change) {
  if (is_live_lidar_()) {
    enum InnoLidarMode pre_mode;
    int ret = get_mode_status(mode_before_change, &pre_mode,
                              status_before_change);
    //  notify stage_read_ to stop reading if streaming will stop
    if (is_live_direct_memory_lidar_()
        &&
        (mode == INNO_LIDAR_MODE_PROTECTION ||
        mode == INNO_LIDAR_MODE_SLEEP ||
        mode == INNO_LIDAR_MODE_STANDBY)
        &&
        stage_read_->is_started()) {
      stage_read_->streaming_stop();
    }
    if (ret) {
      return ret;
    }

    ret = comm_->send_command_and_free_reply("set_mode %d", mode);
    if (ret == 0) {
      struct ModeChangeContent mode_change = {0, INNO_LIDAR_MODE_NONE};
      {
        std::unique_lock<std::mutex> lk(mode_mutex_);
        work_mode_ = mode;
        pre_work_mode_ = *mode_before_change;
        work_status_ = INNO_LIDAR_STATUS_TRANSITION;
        last_in_transition_mode_epoch_ms_ = get_monotonic_raw_time_ms();
        mode_change.timestamp = last_in_transition_mode_epoch_ms_;
        mode_change.target_mode = work_mode_;
      }
      if (mode_change.target_mode != INNO_LIDAR_MODE_NONE) {
        mode_publisher_.publish_mode_change(&mode_change);
      }
      // restart frame sync
      {
        std::unique_lock<std::mutex> lk(frame_sync_mutex_);
        if (frame_sync_enable_) {
          start_frame_sync_with_lock_(frame_sync_time_, true);
        }
      }
    }
    return ret;
  } else {
    if (mode_before_change) {
      *mode_before_change = INNO_LIDAR_MODE_WORK_NORMAL;
    }
    if (status_before_change) {
      *status_before_change = INNO_LIDAR_STATUS_NORMAL;
    }
    inno_log_warning("file replay cannot set mode");
    return 0;
  }
}

int InnoLidar::set_galvo_sync(bool enable) {
  if (!is_live_lidar_()) {
    return 0;
  }

  int ret;
  if (enable) {
    ret = comm_->send_command_and_free_reply("scanh_tran STW01200001ND");
    ret |= comm_->send_command_and_free_reply("scanh_tran STW02000001ND");
  } else {
    ret = comm_->send_command_and_free_reply("scanh_tran STW01200000ND");
    ret |= comm_->send_command_and_free_reply("scanh_tran STW02000001ND");
  }
  return ret;
}

// enable to set FOV 6 degrees lower
int InnoLidar::set_galvo_start_low(bool enable) {
  if (!is_live_lidar_()) {
    return 0;
  }

  int ret;
  if (enable) {
    ret = comm_->send_command_and_free_reply("scanh_tran STW00301965ND");
    ret |= comm_->send_command_and_free_reply("scanh_tran STW02000001ND");
  } else {
    ret = comm_->send_command_and_free_reply("scanh_tran STW00304424ND");
    ret |= comm_->send_command_and_free_reply("scanh_tran STW02000001ND");
  }
  return ret;
}

//
//
//
bool InnoLidar::check_mode_status_fast(LidarModeStatusFlag flag) {
  enum InnoLidarMode cur_mode;
  enum InnoLidarMode pre_mode;
  enum InnoLidarStatus status;

  int ret = get_mode_status_fast(&cur_mode, &pre_mode, &status);
  if (ret != 0) {
    return false;
  }

  if (flag == FLAG_HAS_STREAM) {
    if (status != INNO_LIDAR_STATUS_NORMAL) {
      return false;
    }

    if (cur_mode == INNO_LIDAR_MODE_WORK_NORMAL ||
        cur_mode == INNO_LIDAR_MODE_WORK_SHORT_RANGE ||
        cur_mode == INNO_LIDAR_MODE_WORK_CALIBRATION ||
        cur_mode == INNO_LIDAR_MODE_WORK_QUIET ||
        cur_mode == INNO_LIDAR_MODE_WORK_INTERNAL_1) {
      return true;
    }
  }

  return false;
}


//
//
//
int InnoLidar::get_mode_status_fast(enum InnoLidarMode *mode,
                                    enum InnoLidarMode *pre_mode,
                                    enum InnoLidarStatus *status,
                                    uint32_t *duration_s) {
  if (!is_live_direct_memory_lidar_()) {
    return -1;
  }

  //
  uint32_t reg = 0;
  int ret = Reg::ps_read(INNO_MODE_STATUS, &reg);
  if (ret != 0) {
    inno_log_error("ps_read return %d", ret);
    return ret;
  }

  //
  uint32_t ds_reg = 0;

  if (duration_s != NULL) {
    ret = Reg::ps_read(INNO_MODE_DURATION_S, &ds_reg);
    if (ret != 0) {
      inno_log_error("ps_read return %d", ret);
      return ret;
    }
  }

  //
  std::unique_lock<std::mutex> lk(mode_mutex_);
  work_mode_ = *mode = InnoLidarMode(reg & 0xff);
  work_status_ = *status = InnoLidarStatus((reg & (0xff << 8)) >> 8);
  pre_work_mode_ = *pre_mode = InnoLidarMode((reg & (0xff << 16)) >> 16);

  if (duration_s != NULL) {
    *duration_s = ds_reg;
  }

  if (last_work_mode_ != work_mode_) {
    inno_log_info("mode change from %d to %d", last_work_mode_, work_mode_);
    last_work_mode_ = work_mode_;
  }

  return 0;
}

//
//
//
int InnoLidar::get_mode_status(enum InnoLidarMode *mode,
                               enum InnoLidarMode *pre_mode,
                               enum InnoLidarStatus *status,
                               uint64_t *in_transition_mode_ms) {
  if (is_live_lidar_()) {
    if (!is_live_direct_memory_lidar_()) {
      int ret = comm_->get_mode_status(mode, pre_mode, status);
      if (ret == 0) {
        std::unique_lock<std::mutex> lk(mode_mutex_);
        work_mode_ = *mode;
        pre_work_mode_ = *pre_mode;
        work_status_ = *status;
      } else {
        return ret;
      }
    } else {
      int ret = get_mode_status_fast(mode, pre_mode, status);
      if (ret != 0) {
        return ret;
      }
    }
    if (work_status_ == INNO_LIDAR_STATUS_TRANSITION) {
      if (last_in_transition_mode_epoch_ms_ == 0) {
        last_in_transition_mode_epoch_ms_ = get_monotonic_raw_time_ms();
      }
      if (in_transition_mode_ms) {
        *in_transition_mode_ms = get_monotonic_raw_time_ms() -
                                 last_in_transition_mode_epoch_ms_;
      }
    } else if (in_transition_mode_ms) {
      *in_transition_mode_ms = 0;
    }
    return 0;
  } else {
    if (mode) {
      *mode = INNO_LIDAR_MODE_WORK_NORMAL;
    }
    if (pre_mode) {
      *pre_mode = INNO_LIDAR_MODE_WORK_NORMAL;
    }
    if (status) {
      *status = INNO_LIDAR_STATUS_NORMAL;
    }
    if (in_transition_mode_ms) {
      *in_transition_mode_ms = 0;
    }
    return 0;
  }
}

bool InnoLidar::in_calibration_mode() {
  enum InnoLidarMode mode = INNO_LIDAR_MODE_WORK_NORMAL;
  enum InnoLidarMode pre_mode = INNO_LIDAR_MODE_WORK_NORMAL;
  enum InnoLidarStatus status = INNO_LIDAR_STATUS_NORMAL;

  if (get_mode_status_fast(&mode, &pre_mode, &status) == 0) {
    if (mode == INNO_LIDAR_MODE_WORK_CALIBRATION) {
      return true;
    }
  }
  return false;
}

int InnoLidar::get_attribute(const char *attribute,
                             double *value) {
  inno_log_verify(attribute && value,
                  "%p, %p", attribute, value);
  if (strcmp(attribute, "frame_rate") == 0) {
    /*get frame_rate from firmeware if is_live_lidar_,
    or just return a default value 10*/
    if (is_live_lidar_()) {
      *value = comm_->get_frame_rate();
    } else {
      *value = kDefaultFrameRate;
    }
  } else if (strcmp(attribute, "reflectance_mode") == 0) {
    if (reflectance_mode_ == INNO_REFLECTANCE_MODE_NONE) {
      if (is_live_lidar_()) {
        int rv = comm_->get_config_section_key_value("manufacture",
                                                     "reflectance_mode",
                                                      2);
        reflectance_mode_ = InnoReflectanceMode(rv);
      } else {
        reflectance_mode_ = INNO_REFLECTANCE_MODE_REFLECTIVITY;
      }
    }
    *value = reflectance_mode_;
  } else if (strcmp(attribute, "multiple_return") == 0 ||
             strcmp(attribute, "return_mode") == 0) {
    if (multiple_return_ == INNO_MULTIPLE_RETURN_MODE_NONE) {
      if (is_live_lidar_()) {
        int rv = comm_->get_config_section_key_value("manufacture",
                                                     "multiple_return_mode",
                                                      1);
        multiple_return_ = InnoMultipleReturnMode(rv);
      } else {
        multiple_return_ = INNO_MULTIPLE_RETURN_MODE_SINGLE;
      }
    }
    *value = multiple_return_;
  } else if (strcmp(attribute, "enabled") == 0) {
    if (is_live_lidar_()) {
      *value = comm_->get_config_section_key_value("manufacture",
                                                   "internal_server",
                                                   -1);
    } else {
      *value = 1;
    }
  } else {
    return -1;
  }
  return 0;
}

int InnoLidar::get_attribute_string(const char *attribute,
                                    char *buf, size_t buf_size) {
  inno_log_verify(attribute && buf && buf_size > 0,
                  "%p, %p, %lu", attribute, buf, buf_size);

  if (buf_size < 2) {
    inno_log_warning("buf_size too small %lu", buf_size);
    return -1;
  }

  if (strcmp(attribute, "system_stats") == 0) {
    system_stats_->get_last_info_buffer(buf, buf_size);
  } else if (strcmp(attribute, "output_stats") == 0) {
    system_stats_->get_last_output_info_buffer(buf, buf_size);
  } else if (strcmp(attribute, "cpu_read") == 0) {
    cp_read_->get_stats_string(buf, buf_size);
  } else if (strcmp(attribute, "cpu_signal") == 0) {
    cp_signal_->get_stats_string(buf, buf_size);
  } else if (strcmp(attribute, "cpu_angle") == 0) {
    cp_angle_->get_stats_string(buf, buf_size);
  } else if (strcmp(attribute, "cpu_n0") == 0) {
    cp_noise_filter_phase0_->get_stats_string(buf, buf_size);
  } else if (strcmp(attribute, "cpu_n1") == 0) {
    cp_noise_filter_phase1_->get_stats_string(buf, buf_size);
  } else if (strcmp(attribute, "cpu_deliver") == 0) {
    cp_deliver_->get_stats_string(buf, buf_size);
  } else if (strcmp(attribute, "cpu_deliver2") == 0) {
    cp_deliver2_->get_stats_string(buf, buf_size);
  } else if (strcmp(attribute, "stage_read") == 0) {
    stage_read_->get_stats_string(buf, buf_size);
  } else if (strcmp(attribute, "stage_signal") == 0) {
    stage_signal_->get_stats_string(buf, buf_size);
  } else if (strcmp(attribute, "stage_angle") == 0) {
    stage_angle_->get_stats_string(buf, buf_size);
  } else if (strcmp(attribute, "stage_n0") == 0) {
    stage_noise_filter_phase0_->get_stats_string(buf, buf_size);
  } else if (strcmp(attribute, "stage_n1") == 0) {
    stage_noise_filter_phase1_->get_stats_string(buf, buf_size);
  } else if (strcmp(attribute, "stage_deliver") == 0) {
    stage_deliver_->get_stats_string(buf, buf_size);
  } else if (strcmp(attribute, "stage_deliver2") == 0) {
    stage_deliver2_->get_stats_string(buf, buf_size);
  } else if (strcmp(attribute, "temperature") == 0) {
    get_temperature(buf, buf_size);
  } else if (strcmp(attribute, "detector_temps") == 0) {
    get_detector_temps(buf, buf_size);
  } else if (strcmp(attribute, "motor_speeds") == 0) {
    get_motor_speeds(buf, buf_size);
  } else if (strcmp(attribute, "yaml") == 0) {
    int r = snprintf(buf, buf_size, "%s", get_yaml_buffer_());
    if (r >= ssize_t(buf_size)) {
      buf[buf_size - 1] = 0;
    }
  } else if (strcmp(attribute, "inner_faults_info") == 0) {
    get_inner_faults_info(buf, buf_size);
  } else if (strcmp(attribute, "inner_faults_info_network") == 0) {
    get_inner_faults_info(buf, buf_size, true);
  } else if (strcmp(attribute, "inner_faults_all_info") == 0) {
    get_inner_faults_all_info(buf, buf_size);
  } else if (strcmp(attribute, "sync_cycle_status") == 0) {
    get_sync_cycle_status(buf, buf_size);
  } else if (strcasecmp(attribute, "DID_HW_PART_NUM") == 0
             || strcasecmp(attribute, "0xf110") == 0) {
    return get_from_uds_rest(attribute, buf, buf_size);
  } else if (strcasecmp(attribute, "DID_SW_PART_NUM") == 0
             || strcasecmp(attribute, "0xf118") == 0) {
    return get_from_uds_rest(attribute, buf, buf_size);
  } else if (strcasecmp(attribute, "DID_ECU_MFG_DATE") == 0
             || strcasecmp(attribute, "0xf18b") == 0) {
    return get_from_uds_rest(attribute, buf, buf_size);
  } else if (strcasecmp(attribute, "DID_ECU_SN") == 0
             || strcasecmp(attribute, "0xf18c") == 0) {
    return get_from_uds_rest(attribute, buf, buf_size);
  } else if (strcasecmp(attribute, "DID_SUPPLIER_HW_VER") == 0
             || strcasecmp(attribute, "0xf193") == 0) {
    return get_from_uds_rest(attribute, buf, buf_size);
  } else if (strcasecmp(attribute, "DID_SUPPLIER_SW_VER") == 0
             || strcasecmp(attribute, "0xf195") == 0) {
    return get_from_uds_rest(attribute, buf, buf_size);
  } else if (strcmp(attribute, "source_is_live_lidar") == 0) {
    if (is_live_lidar_()) {
      buf[0] = '1';
    } else {
      buf[0] = '0';
    }
    buf[1] = '\0';
  } else if (strcmp(attribute, "is_live_direct_memory") == 0) {
    if (is_live_direct_memory_lidar_()) {
      buf[0] = '1';
    } else {
      buf[0] = '0';
    }
    buf[1] = '\0';
  } else if (strcmp(attribute, "frame_sync_stats") == 0) {
    std::unique_lock<std::mutex> lk(frame_sync_mutex_, std::try_to_lock);
    if (lk.owns_lock() && frame_sync_) {
      frame_sync_->get_stats_string(buf, buf_size);
    }
  } else if (strcmp(attribute, "is_live_lidar") == 0) {
    int retv = snprintf(buf, buf_size,
                        is_live_lidar_() ? "yes" : "no");
    if (retv >= ssize_t(buf_size)) {
      buf[buf_size - 1] = 0;
    }
  } else if (strcmp(attribute, "is_pc_server") == 0) {
    int retv = snprintf(buf, buf_size, "server");
    if (retv >= ssize_t(buf_size)) {
      buf[buf_size - 1] = 0;
    }
  } else {
    inno_log_info("No match for REST API request '%s'", attribute);
    return -1;
  }
  return 0;
}

int InnoLidar::get_from_uds_rest(const char *did_name,
                                 char *buffer,
                                 size_t buf_size) {
  if (strcasecmp(did_name, "DID_HW_PART_NUM") == 0
             || strcasecmp(did_name, "0xf110") == 0) {
    // Read from FW
    // f110 should be written by manufacturing at EOL to FW
    char hw_part_num[64];
    if (comm_->get_did(hw_part_num, sizeof(hw_part_num), "f1 10") == 0) {
      // Stored as hex values of characters -- convert to a string
      std::string hw_part_str = "";
      char *tok;
      char *remainder = hw_part_num;
      while ((tok = strtok_r(remainder, " ", &remainder))) {
        hw_part_str += std::stoi(tok, 0, 16);
      }
      return print_to_json("DID_HW_PART_NUM", hw_part_str.c_str(),
                          buffer, buf_size);
    } else {
      // Older sensor that didn't get f110 written by manufacturing.
      int ret = hw_part_from_board(hw_part_num, sizeof(hw_part_num));
      if (ret == 0) {
        return print_to_json("DID_HW_PART_NUM", hw_part_num, buffer, buf_size);
      } else {
        return ret;
      }
    }
  } else if (strcasecmp(did_name, "DID_SW_PART_NUM") == 0
             || strcasecmp(did_name, "0xf118") == 0) {
    // Hard-code, to match UDS value
    return print_to_json("DID_SW_PART_NUM", "P0235753 BO", buffer, buf_size);
  } else if (strcasecmp(did_name, "DID_ECU_MFG_DATE") == 0
             || strcasecmp(did_name, "0xf18b") == 0) {
    // Read serial # from FW & compute mfg date
    char sn_buff[128];
    int buf_len = sizeof(sn_buff);
    int ret = comm_->get_sn(sn_buff, buf_len);
    if (ret != 0) {
      return ret;
    }
    char mfg_date[64];
    buf_len = sizeof(mfg_date);
    ret = mfg_date_from_sn(sn_buff, mfg_date, buf_len);
    if (ret == 0) {
      return print_to_json("DID_ECU_MFG_DATE", mfg_date, buffer, buf_size);
    } else {
      return ret;
    }
  } else if (strcasecmp(did_name, "DID_ECU_SN") == 0
             || strcasecmp(did_name, "0xf18c") == 0) {
    // Read serial # from FW
    char sn_buff[128];
    int ret = comm_->get_sn(sn_buff, sizeof(sn_buff));
    if (ret != 0) {
      return ret;
    }
    return print_to_json("DID_ECU_SN", sn_buff, buffer, buf_size);
  } else if (strcasecmp(did_name, "DID_SUPPLIER_HW_VER") == 0
             || strcasecmp(did_name, "0xf193") == 0) {
    // Read board type from FW (get_hw_part_name)
    char board_name[32];
    int ret = comm_->get_board_name(board_name, sizeof(board_name));
    if (ret != 0) {
      return ret;
    }
    char hw_ver_num[32];
    if (sscanf(board_name, "%31s\nDone", hw_ver_num) != 1) {
      inno_log_warning("unexpected board name from FW: %s", board_name);
      return -50;
    }
    return print_to_json("DID_SUPPLIER_HW_VER", hw_ver_num, buffer, buf_size);
  } else if (strcasecmp(did_name, "DID_SUPPLIER_SW_VER") == 0
             || strcasecmp(did_name, "0xf195") == 0) {
    // Read build # from FW (get_fw_seq_number)
    char sw_ver[64];
    char sw_ver_num[32];
    int ret = comm_->get_fw_sequence(sw_ver, sizeof(sw_ver));
    if (ret != 0) {
      return ret;
    }
    if (sscanf(sw_ver, "%31s\nDone", sw_ver_num) != 1) {
      inno_log_warning("unexpected sw_ver returned from FW: %s", sw_ver);
      return -60;
    }
    return print_to_json("DID_SUPPLIER_SW_VER", sw_ver_num, buffer, buf_size);
  }
  inno_log_warning("unrecognized DID %s passed to get_from_uds_rest", did_name);
  return -10;
}

int InnoLidar::print_to_json(const char *json_key, const char *json_val,
                             char *buffer, size_t buf_size) {
  char ret_json[128];
  int len = sizeof(ret_json);
  int ret = snprintf(ret_json, len, "{\"%s\": \"%s\"}", json_key, json_val);
  if (ret >= len) {
    inno_log_warning("could not print full string to json buffer");
    return -20;
  }
  if (strlen(ret_json) >= buf_size) {
    inno_log_warning("json string (%s) too big for provided buffer %d",
                     ret_json, static_cast<int>(buf_size));
    return -30;
  }
  memcpy(buffer, ret_json, strlen(ret_json));
  buffer[strlen(ret_json)] = 0;
  return 0;
}

int InnoLidar::hw_part_from_board(char *buffer, int buf_len) {
  // Default value.
  int ret = snprintf(buffer, buf_len, "P0229775 AF");

  // Try to use board type to determine non-default (older) HW_PART_NUM value.
  char board_name[32];
  if (comm_->get_board_name(board_name, sizeof(board_name)) == 0) {
    char hw_ver_num[32];
    if (sscanf(board_name, "%31s\nDone", hw_ver_num) == 1) {
      if (strncmp(hw_ver_num, "B1", 2) == 0) {
        ret = snprintf(buffer, buf_len, "P0229775 AB");
      } else if (strncmp(hw_ver_num, "B2", 2) == 0) {
        ret = snprintf(buffer, buf_len, "P0229775 AC");
      } else if (strncmp(hw_ver_num, "B4", 2) == 0) {
        ret = snprintf(buffer, buf_len, "P0229775 AD");
      } else if (strncmp(hw_ver_num, "C0", 2) == 0) {
        ret = snprintf(buffer, buf_len, "P0229775 AE");
      }
    } else {
      inno_log_warning("using default; can't parse: %s", board_name);
    }
  } else {
    inno_log_warning("using default; DID not written and unknown board name");
  }
  if (ret >= buf_len) {
    inno_log_warning("could not print full hw part # to buffer %d", buf_len);
    return -20;
  }
  return 0;
}

int InnoLidar::mfg_date_from_sn(const char *sn_buff,
                                char *mfg_date, int buf_len) {
  // From Confluence doc MFG main comp SN format https://tinyurl.com/4uw3k4vb
  // 12 digit format: TTDDDYYXXXXX
  // TT: component short code. 36 for Falcon-i
  // DDD: day of year. 001-366
  // YY: last 2 digits of year. 21-99
  // XXXXX: Unique identity number of given day
  int day_of_year = 1;
  int day = 1;
  int month = 0;
  int year = 22;

  // report in BCD (binary coded decimal)
  int d_bcd = 1;
  int m_bcd = 1;
  int y_bcd = 16*(year/10) + year%10;
  try {
    std::string serial_num(sn_buff);
    day_of_year = std::stoi(serial_num.substr(2, 3));
    year = std::stoi(serial_num.substr(5, 2));
    int feb = 28;
    int days_in_year = 365;
    if (year % 4 == 0) {
        feb = 29;
        days_in_year = 366;
    }
    if (year < 21) {
      inno_log_warning("invalid SN: year=%d", year);
      return -45;
    }
    if (day_of_year == 0 || day_of_year > days_in_year) {
      inno_log_warning("invalid SN: d.o.y.=%d", day_of_year);
      return -40;
    } else {
      int mon_days[12] = { 31, feb, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
      int rem_days = day_of_year;
      while (rem_days > 0) {
        if (month > 11) {
          inno_log_warning("invalid SN: d.o.y.=%d", day_of_year);
          return -40;
        }
        day = rem_days;
        rem_days -= mon_days[month];
        ++month;
      }
    }
    y_bcd = 16*(year/10) + year%10;
    m_bcd = 16*(month/10) + month%10;
    d_bcd = 16*(day/10) + day%10;
  }
  catch(...) {
  }
  int ret = snprintf(mfg_date, buf_len, "%d %d %d", y_bcd, m_bcd, d_bcd);
  if (ret >= buf_len) {
    inno_log_warning("could not print full mfg date to buffer %d", buf_len);
    return -20;
  }
  return 0;
}

int InnoLidar::set_faults_save_raw(uint64_t value) {
  inno_log_info("update faults save raw: %" PRI_SIZEU, value);
  fault_manager_->update_client_save_raw_enable(value);
  return 0;
}

int InnoLidar::set_attribute_string(const char *attribute,
                                    const char *buf) {
  int ret = 0;
  inno_log_verify(attribute && buf, "%p, %p", attribute, buf);

  if (strcmp(attribute, "force_xyz_pointcloud") == 0) {
    if (strcmp(buf, "1") == 0) {
      inno_log_info("force_xyz_pointcloud to true");
      force_xyz_pointcloud_ = true;
      return 0;
    } else if (strcmp(buf, "0")) {
      inno_log_info("force_xyz_pointcloud to false");
      force_xyz_pointcloud_ = false;
      return 0;
    } else {
      inno_log_panic("invalid force_xyz_pointcloud %s", buf);
    }
  }

  if (!is_live_lidar_()) {
    inno_log_info("skip setting %s to %s for non-live",
                  attribute, buf);
    return 0;
  }

  if (strcmp(attribute, "enabled") == 0) {
    ret = comm_->set_config_section_key_value("manufacture",
                                              "internal_server",
                                              buf);
  } else if (strcmp(attribute, "command") == 0) {
    if (strcmp(buf, "get_udp_raw_data") == 0) {
      this->set_save_raw_data_flag("get_udp_raw_data command.");
    }
  } else if (strcmp(attribute, "reboot") == 0) {
    if (strcmp(buf, "1") == 0) {
      ret = comm_->set_reboot(1);
    } else {
      ret = -1;
    }
  } else if (strcmp(attribute, "heal_faults_external") == 0) {
    uint32_t value = atoi(buf);
    inno_log_verify(value >= INNO_LIDAR_IN_FAULT_OTHER &&
                    value <= INNO_LIDAR_IN_FAULT_MAX, "value: %u", value);
    int r = comm_->clear_fw_inner_faults(value);
    if (r != 0) {
      inno_log_error("fail to clear fw faults, got %d", r);
    }
    InnoLidarInFault k = InnoLidarInFault(value);
    fault_manager_->set_faults_external(k, INNO_FAULT_OPERATION_HEAL);
  } else if (strcmp(attribute, "set_faults_external") == 0) {
    uint32_t value = atoi(buf);
    inno_log_verify(value >= INNO_LIDAR_IN_FAULT_OTHER &&
                    value <= INNO_LIDAR_IN_FAULT_MAX, "value: %u", value);
    InnoLidarInFault k = InnoLidarInFault(value);
    fault_manager_->set_faults_external(k);
  } else if (strcmp(attribute, "init_faults_info") == 0) {
    fault_manager_->init_faults_info(buf);
  } else if (strcmp(attribute, "raw_channel") == 0) {
    {
      std::unique_lock<std::mutex> lk(last_stage_mutex_);
      if (last_stage_is_up_) {
        inno_log_warning("cannot set raw_channel while lidar is running");
        ret = -1;
        return ret;
      }
    }

    int32_t value = atoi(buf);
    ret = set_raw_channel_(value);
  } else if (strcmp(attribute, "frame_sync") == 0) {
    std::unique_lock<std::mutex> lk(frame_sync_mutex_);
    return start_frame_sync_with_lock_(atof(buf), false);
  } else if (strcmp(attribute, "frame_sync_disable") == 0) {
    std::unique_lock<std::mutex> lk(frame_sync_mutex_);
     if (frame_sync_) {
      inno_log_info("stopping current frame sync thread.");
      frame_sync_->stop();
      delete frame_sync_;
      frame_sync_ = nullptr;
    }
  } else {
    // xx todo(WYY): use strncmp()
    std::string str(attribute);
    if (str.find("SET_INNO_LIDAR_IN_FAULT_") != std::string::npos) {
      InnoLidarInFault fid =
      InnoLidarInFault(atoi(str.substr(str.rfind("_") + 1).c_str()));
      uint32_t value = atoi(buf);
      if (fid >= INNO_LIDAR_IN_FAULT_OTHER && fid < INNO_LIDAR_IN_FAULT_MAX) {
        set_raw_fault(fid, value, "fid(%d) sets", fid);
      }
    } else if (str.find("HEAL_INNO_LIDAR_IN_FAULT_") != std::string::npos) {
      InnoLidarInFault fid =
      InnoLidarInFault(atoi(str.substr(str.rfind("_") + 1).c_str()));
      uint32_t value = atoi(buf);
      if (fid >= INNO_LIDAR_IN_FAULT_OTHER && fid < INNO_LIDAR_IN_FAULT_MAX) {
        heal_raw_fault(fid, value, "fid(%d) heals", fid);
      }
    } else if (str.find("SET_HISTORY_INNO_LIDAR_IN_FAULT_") !=
               std::string::npos) {
      InnoLidarInFault fid =
          InnoLidarInFault(std::stoi(str.substr(str.rfind("_") + 1)));
      // uint32_t value = atoi(buf);
      if (fid >= INNO_LIDAR_IN_FAULT_OTHER && fid < INNO_LIDAR_IN_FAULT_MAX) {
        set_history_fault(fid, "history fault");
      }
    } else if (strcmp(attribute, "faults_save_raw") == 0) {
      char *end;
#define NUM_LEN 16  // same with client
      #define MAX_NUM_LEN 16  // same with server
      uint64_t value = strtoul(buf, &end, 16);
      if (end - buf > MAX_NUM_LEN) {
        inno_log_error("invalid parameter: %s", buf);
        return -1;
      }

      ret = set_faults_save_raw(value);
    } else if (strcmp(attribute, "sync_fw_faults") == 0) {
      ret = -1;
      if (fault_manager_) {
        fault_manager_->sync_fw_faults();
        ret = 0;
      }
    } else {
      // no matched string
    }
  }
  return ret;
}

int InnoLidar::read_ps_reg(uint16_t off, uint32_t *value) {
  if (!is_live_direct_memory_lidar_()) {
    return -1;
  }
  return Reg::ps_read(off, value);
}

int InnoLidar::read_pl_reg(uint16_t off, uint32_t *value) {
  if (!is_live_direct_memory_lidar_()) {
    return -1;
  }
  return Reg::pl_read(off, value);
}

int InnoLidar::write_ps_reg(uint16_t off, uint32_t value) {
  if (!is_live_direct_memory_lidar_()) {
    return -1;
  }
  return Reg::ps_write(off, value);
}

int InnoLidar::write_pl_reg(uint16_t off, uint32_t value) {
  if (!is_live_direct_memory_lidar_()) {
    return -1;
  }
  return Reg::pl_write(off, value);
}

char *InnoLidar::get_yaml_buffer_() {
  if (yaml_from_yaml_file_) {
    inno_log_info("use yaml file");
    return yaml_from_yaml_file_;
  } else if (yaml_from_data_file_) {
    inno_log_info("use yaml from data file");
    return yaml_from_data_file_;
  } else if (yaml_from_live_) {
    inno_log_info("use yaml from live lidar");
    return yaml_from_live_;
  }
  return NULL;
}

int InnoLidar::parse_lidar_yaml_() {
  char *yaml_buffer = get_yaml_buffer_();
  if (yaml_buffer) {
    // already yaml buffer
  } else if (is_live_lidar_()) {
    char *buffer = new char[kMaxYamlSize];
    if (!buffer) {
      inno_log_error("allocate memory failed.");
      return -1;
    }
    int ret = comm_->get_geo_yaml(buffer, kMaxYamlSize);
    if (ret) {
      inno_log_error("%s cannot get live lidar yaml %d",
                     name_, ret);
      do_message_callback(INNO_MESSAGE_LEVEL_ERROR,
                          INNO_MESSAGE_CODE_BAD_CONFIG_YAML,
                          "cannot download from controller");
      delete[] buffer;
      return -1;
    }
    yaml_from_live_ = strdup(buffer);
    delete[] buffer;
    inno_log_verify(yaml_from_live_, "yaml_from_live_");
    yaml_buffer = yaml_from_live_;
  } else {
    inno_log_error("%s no yaml file", name_);
    return -2;
  }

  if (params_.parse(yaml_buffer) == 0) {
    inno_log_info("%s load valid params from lidar", name_);
  } else {
    if (yaml_buffer == yaml_from_live_) {
      inno_log_error("%s invalid params from lidar", name_);
      do_message_callback(INNO_MESSAGE_LEVEL_ERROR,
                          INNO_MESSAGE_CODE_BAD_CONFIG_YAML,
                          "file downloaded from controller is corrupted");
      return -3;
    } else if (yaml_buffer == yaml_from_yaml_file_) {
      inno_log_error("%s invalid params from yaml file", name_);
      do_message_callback_fmt(INNO_MESSAGE_LEVEL_ERROR,
                              INNO_MESSAGE_CODE_BAD_CONFIG_YAML,
                              "file %s is corrupted", yaml_filename_);
      return -4;
    } else {
      inno_log_verify(yaml_buffer == yaml_from_data_file_, "yaml_buffer");
      inno_log_error("%s invalid params from data file", name_);
      do_message_callback_fmt(INNO_MESSAGE_LEVEL_ERROR,
                              INNO_MESSAGE_CODE_BAD_CONFIG_YAML,
                              "data file is corrupted");
      return -5;
    }
  }
  params_.set_default_for_missing();

  return 0;
}

LidarClock& InnoLidar::get_clock() {
  return this->clock_;
}

int InnoLidar::set_params_file(const char *lidar_model,
                               const char *yaml_filename) {
  int ret = 0;
  {
    std::unique_lock<std::mutex> lk(params_mutex_);
    if (lidar_model && strlen(lidar_model)) {
      if (strcasecmp(lidar_model, "rev_i") == 0 ||
          strcasecmp(lidar_model, "i") == 0) {
        lidar_model_ = LIDAR_MODEL_I;
      } else if (strcasecmp(lidar_model, "rev_k") == 0 ||
          strcasecmp(lidar_model, "k") == 0) {
        lidar_model_ = LIDAR_MODEL_K;
      } else if (strcasecmp(lidar_model, "rev_g") == 0 ||
          strcasecmp(lidar_model, "g") == 0) {
        lidar_model_ = LIDAR_MODEL_G;
      } else {
        inno_log_panic("%s invalid lidar_model %s. Valid model list: "
                       "i k",
                       name_, lidar_model);
      }
    }

    if (yaml_filename && strlen(yaml_filename)) {
      if (yaml_filename_) {
        free(yaml_filename_);
      }
      if (yaml_from_yaml_file_) {
        yaml_from_yaml_file_ = NULL;
      }
      yaml_filename_ = strdup(yaml_filename);
      inno_log_verify(yaml_filename_, "yaml_filename");
      size_t yaml_size = 0;
      char *yaml = NULL;
      ret = StageRead::load_yaml_from_yaml_file(yaml_filename,
                                                &yaml,
                                                &yaml_size);
      if (ret == 0) {
        inno_log_info("load yaml file %s", yaml_filename);
        yaml_from_yaml_file_ = yaml;
      } else {
        inno_log_error("cannot load yaml file %s, %d", yaml_filename, ret);
      }
    }

    inno_log_info("%s set_params_file model=%c config=%s",
                  name_, lidar_model_ != LIDAR_MODEL_NONE ?
                  lidar_model_ - LIDAR_MODEL_I + 'I' : ' ',
                  yaml_filename_);
  }

  inno_log_panic_if_not(get_state_() == InnoLidarBase::STATE_INIT,
                        "%s inno_lidar_set_parameters() must be called before "
                        "calling inno_lidar_start()",
                        name_);
  return ret;
}

int InnoLidar::set_raw_channel_(int32_t channel) {
  if (!is_live_direct_memory_lidar_()) {
    inno_log_info("cannot set raw_channel for non-live");
    return 0;
  }

  uint32_t data;
  int ret;
  if (channel >= 4) {
    inno_log_warning("invalid channel %d", channel);
    return -1;
  } else if (channel < 0) {
    if ((ret = read_ps_reg(DMA_CONTROL_REGISTER, &data)) < 0) {
      inno_log_warning("cannot read DMA_CONTROL_REGISTER %d", ret);
      return -1;
    }
    data = data & ~DMA_RAW_DATA_SELECTED;
    if ((ret = write_ps_reg(DMA_CONTROL_REGISTER, data)) < 0) {
      inno_log_warning("cannot write DMA_CONTROL_REGISTER %d", ret);
      return -1;
    } else {
      return 0;
    }
  } else {
    if ((ret = read_ps_reg(CAPABILITIES_REGISTER, &data)) < 0) {
      inno_log_warning("cannot read CAPABILITIES_REGISTER %d", ret);
      return -1;
    }
    if (!(data & RAW_DATA_EXTRACT_SUPPORT)) {
      inno_log_warning("raw capture mode is not supported in firmware %u",
                       data);
      return -1;
    }
    if ((ret = read_ps_reg(RAW_DATA_DMA_CONTROL_REGISTER,
                           &data)) < 0) {
      inno_log_warning("cannot read RAW_DATA_DMA_CONTROL %d", ret);
      return -1;
    }

    data = data & ~(DMA_RAW_DATA_TRIGGER | DMA_RAW_DATA_CHANNEL);
    data = data | channel | BOWL_MOTOR_INDEX_TRIGGER;

    if ((ret = write_ps_reg(RAW_DATA_DMA_CONTROL_REGISTER, data)) < 0) {
      inno_log_warning("cannot write RAW_DATA_DMA_CONTROL %d", ret);
      return -1;
    }

    if ((ret = read_ps_reg(DMA_CONTROL_REGISTER, &data)) < 0) {
      inno_log_warning("cannot read DMA_CONTROL %d", ret);
      return -1;
    }
    data = data | DMA_RAW_DATA_SELECTED;
    if ((ret = write_ps_reg(DMA_CONTROL_REGISTER, data)) < 0) {
      inno_log_warning("cannot write DMA_RAW_DATA_SELECTED %d", ret);
      return -1;
    }
    return 0;
  }
}

#define SET_PARAM(field)                          \
  do {                                            \
     if (strcmp(name, #field) == 0) {             \
       params_base_.iv_params.field = v;          \
       return 0;                                  \
     }                                            \
  } while (0)

#define SET_PARAM_2(rqt_name, field)              \
  do {                                            \
     if (strcmp(name, #rqt_name) == 0) {          \
       params_base_.iv_params.field = v;          \
       return 0;                                  \
     }                                            \
  } while (0)

#define SET_PARAM_3(rqt_name, field, index)       \
  do {                                            \
     if (strcmp(name, #rqt_name) == 0) {          \
       params_base_.iv_params.field[index] = v;   \
       return 0;                                  \
     }                                            \
  } while (0)

int InnoLidar::set_ivparams_(const char *name, const char *value) {
  if (strcmp(name, "end") == 0) {
    std::unique_lock<std::mutex> lk(params_mutex_);
    bool change_table = params_.copy_from_src(&params_base_);
    if (change_table) {
      misc_tables_.setup_aperture_correction_table(\
        params_.iv_params.aperture_offset);
      misc_tables_.setup_power_distance_correction_table(\
        params_.iv_params.aperture_offset_2,
        params_.iv_params.aperture_offset_3);
    }
    if (!stage_angle_->is_lookup_table_inited()) {
      inno_log_panic_if_not(lidar_model_ != LIDAR_MODEL_NONE,
                     "no lidar_model, cannot init lookup table");
      stage_angle_->init_lookup_table();
      inno_log_info("angle lookup table initialized, send notify");
      params_cond_.notify_all();
    }
    return 0;
  }

  double v;
  if (sscanf(value, "%lf", &v) != 1) {
    inno_log_error("invalid param value %s (name=%s)", value, name);
    return -1;
  }

  SET_PARAM(p_offset);
  SET_PARAM(p_off_center);
  SET_PARAM(g_scan_range);
  SET_PARAM(g_center_angle);
  SET_PARAM(g_tilt);
  SET_PARAM(g_tilt2);
  SET_PARAM(tilt_angle);
  SET_PARAM(shift_angle);
  SET_PARAM(n_p);
  SET_PARAM(retro_intensity);
  SET_PARAM(retro_intensity_2);
  SET_PARAM_3(cross_talk_ratio_1, ctr, 0);
  SET_PARAM_3(cross_talk_ratio_2, ctr, 1);
  SET_PARAM_3(cross_talk_ratio_3, ctr, 2);
  SET_PARAM_2(max_cross_talk_intensity, mcti);
  SET_PARAM_3(chan0_v_adjustment, v_adjustment, 0);
  SET_PARAM_3(chan1_v_adjustment, v_adjustment, 1);
  SET_PARAM_3(chan2_v_adjustment, v_adjustment, 2);
  SET_PARAM_3(chan3_v_adjustment, v_adjustment, 3);
  SET_PARAM_3(chan0_h_adjustment, ho_adjustment, 0);
  SET_PARAM_3(chan1_h_adjustment, ho_adjustment, 1);
  SET_PARAM_3(chan2_h_adjustment, ho_adjustment, 2);
  SET_PARAM_3(chan3_h_adjustment, ho_adjustment, 3);
  SET_PARAM_3(chan0_alpha, f_alpha, 0);
  SET_PARAM_3(chan1_alpha, f_alpha, 1);
  SET_PARAM_3(chan2_alpha, f_alpha, 2);
  SET_PARAM_3(chan3_alpha, f_alpha, 3);
  SET_PARAM_3(chan0_gamma, f_gamma, 0);
  SET_PARAM_3(chan1_gamma, f_gamma, 1);
  SET_PARAM_3(chan2_gamma, f_gamma, 2);
  SET_PARAM_3(chan3_gamma, f_gamma, 3);
  SET_PARAM_3(chan0_f_int_vbr0, f_int_vbr0, 0);
  SET_PARAM_3(chan1_f_int_vbr0, f_int_vbr0, 1);
  SET_PARAM_3(chan2_f_int_vbr0, f_int_vbr0, 2);
  SET_PARAM_3(chan3_f_int_vbr0, f_int_vbr0, 3);
  SET_PARAM_3(p0_tilt, p_tilt, 0);
  SET_PARAM_3(p1_tilt, p_tilt, 1);
  SET_PARAM_3(p2_tilt, p_tilt, 2);
  SET_PARAM_3(p3_tilt, p_tilt, 3);
  SET_PARAM_3(p4_tilt, p_tilt, 4);
  SET_PARAM_3(p5_tilt, p_tilt, 5);
  SET_PARAM_3(p5_tilt, p_tilt, 6);
  SET_PARAM_3(p0_shift, p_shift, 0);
  SET_PARAM_3(p1_shift, p_shift, 1);
  SET_PARAM_3(p2_shift, p_shift, 2);
  SET_PARAM_3(p3_shift, p_shift, 3);
  SET_PARAM_3(p4_shift, p_shift, 4);
  SET_PARAM_3(p5_shift, p_shift, 5);
  SET_PARAM_3(p6_shift, p_shift, 6);
  SET_PARAM_3(chan0_dist_correction, distance_correction, 0);
  SET_PARAM_3(chan1_dist_correction, distance_correction, 1);
  SET_PARAM_3(chan2_dist_correction, distance_correction, 2);
  SET_PARAM_3(chan3_dist_correction, distance_correction, 3);
  SET_PARAM_3(chan0_dist_correction_2, distance_correction_2, 0);
  SET_PARAM_3(chan1_dist_correction_2, distance_correction_2, 1);
  SET_PARAM_3(chan2_dist_correction_2, distance_correction_2, 2);
  SET_PARAM_3(chan3_dist_correction_2, distance_correction_2, 3);
  SET_PARAM_3(chan0_reference_time, reference_time, 0);
  SET_PARAM_3(chan1_reference_time, reference_time, 1);
  SET_PARAM_3(chan2_reference_time, reference_time, 2);
  SET_PARAM_3(chan3_reference_time, reference_time, 3);
  SET_PARAM_3(chan0_refl_factor, refl_factor, 0);
  SET_PARAM_3(chan1_refl_factor, refl_factor, 1);
  SET_PARAM_3(chan2_refl_factor, refl_factor, 2);
  SET_PARAM_3(chan3_refl_factor, refl_factor, 3);
  SET_PARAM(dist_corr_transition_intensity);
  SET_PARAM(dist_corr_transition_low);
  SET_PARAM(dist_corr_transition_high);
  SET_PARAM(dist_corr_max_intensity);
  SET_PARAM(aperture_offset);
  SET_PARAM(aperture_offset_2);
  SET_PARAM(aperture_offset_3);
  SET_PARAM_2(kill_distance, k_dis);
  SET_PARAM_2(kill_intensity, k_int);
  SET_PARAM_2(kill_reflectance, k_ref);
  SET_PARAM_2(kill_distance_2, k_dis_2);
  SET_PARAM_2(kill_reflectance_2, k_ref_2);
  SET_PARAM(fov_top_half_p_angle);
  SET_PARAM(fov_bottom_half_p_angle);

  return -1;
}

int InnoLidar::set_config_name_value(const char *name,
                                     const char *value) {
  if (strncmp(name, "Lidar_IvParams/", 15) == 0) {
    return set_ivparams_(name + 15, value);
  }
  return config_manage_.set_config_key_value(name, value, true);
}

int InnoLidar::set_reflectance_mode(enum InnoReflectanceMode mode) {
  if (mode <= INNO_REFLECTANCE_MODE_NONE ||
      mode >= INNO_REFLECTANCE_MODE_MAX) {
    inno_log_error("Invalid ReflectanceMode %d",
                   mode);
    return -1;
  }
  bool update_fw = reflectance_mode_ != mode;
  reflectance_mode_ = mode;
  if (is_live_lidar_() && update_fw) {
    return comm_->set_config_section_key_value("manufacture",
                                               "reflectance_mode",
                                                std::to_string(mode).c_str());
  }
  return 0;
}

int InnoLidar::set_return_mode(InnoMultipleReturnMode mode) {
  if (mode <= INNO_MULTIPLE_RETURN_MODE_NONE ||
      mode >= INNO_MULTIPLE_RETURN_MODE_MAX) {
    inno_log_error("Invalid MultipleReturnMode %d",
                   mode);
    return -1;
  }

  if (!is_live_lidar_()) {
    multiple_return_ = mode;
    return 0;
  }

  int ret;
  uint32_t reg_value;
  bool update_fw = false;
  // if is live lidar(not direct memory), we should send out command via comm_
  // set register 4C to right value according to mode.
  // Refer to definition of reg PL_PULSE_FILTER_CONTROL_REGISTER.
  if (!is_live_direct_memory_lidar_()) {
    ret = comm_->reg_read(PL_PULSE_FILTER_CONTROL_REGISTER, &reg_value);
  } else {
    ret = read_pl_reg(PL_PULSE_FILTER_CONTROL_REGISTER, &reg_value);
  }
  if (ret == 0) {
    // For INNO_MULTIPLE_RETURN_MODE_SINGLE, we set pulse filter mode to 2 to
    // make is_2nd_flag work.
    uint32_t kInvalidFilterMode = 0xffffffff;
    uint32_t return_mode_2_filter_mode[INNO_MULTIPLE_RETURN_MODE_MAX + 1] = {
        kInvalidFilterMode, 4, 2, 3, kInvalidFilterMode
    };
    uint32_t filter_mode = return_mode_2_filter_mode[mode];
    inno_log_verify(filter_mode != kInvalidFilterMode,
                    "invalid fpga pulse filter mode");
    reg_value &= ~(PL_PULSE_FILTER_CONTROL_REGISTER_MODE);
    reg_value |= filter_mode;
    if (!is_live_direct_memory_lidar_()) {
      ret = comm_->reg_write(PL_PULSE_FILTER_CONTROL_REGISTER, reg_value);
    } else {
      ret = write_pl_reg(PL_PULSE_FILTER_CONTROL_REGISTER, reg_value);
    }
    if (ret == 0) {
      update_fw = multiple_return_ != mode;
      multiple_return_ = mode;
    } else {
      inno_log_error("write reg error %d", ret);
      return ret;
    }
  } else {
    inno_log_error("read reg error %d", ret);
    return ret;
  }
  if (update_fw) {
    return comm_->\
           set_config_section_key_value("manufacture",
                                        "multiple_return_mode",
                                         std::to_string(mode).c_str());
  }
  return 0;
}

int InnoLidar::set_hori_roi_(double hori_roi_angle) {
  if (hori_roi_angle == kInnoNopROI) {
    return 0;
  }

  if (hori_roi_angle < config_.min_h_roi) {
    inno_log_error("horizontal_roi too small %f vs %f",
                   hori_roi_angle, config_.min_h_roi);
    return 1;
  }
  if (hori_roi_angle > config_.max_h_roi) {
    inno_log_error("horizontal_roi too big %f vs %f",
                   hori_roi_angle, config_.max_h_roi);
    return 1;
  }

  hori_roi_angle_ = hori_roi_angle;
  hori_roi_angle_unit_ = hori_roi_angle_ * kInnoAngleUnitPerDegree;

  return 0;
}

int InnoLidar::set_vertical_roi_(double vertical_roi_angle) {
  int ret = 0;
  if (vertical_roi_angle == kInnoNopROI) {
    return 0;
  }

  if (vertical_roi_angle < config_.min_v_roi) {
    inno_log_error("vertical_roi too small %f vs %f",
                   vertical_roi_angle, config_.min_v_roi);
    return 1;
  }
  if (vertical_roi_angle > config_.max_v_roi) {
    inno_log_error("vertical_roi too big %f vs %f",
                   vertical_roi_angle, config_.max_v_roi);
    return 1;
  }

  if (lidar_source_ == LIDAR_SOURCE_LIVE) {
    ret = comm_->set_vertical_roi(vertical_roi_angle);
    if (ret == 0) {
      vertical_roi_angle_ = vertical_roi_angle;
      vertical_roi_angle_unit_ = vertical_roi_angle_ * kInnoAngleUnitPerDegree;
    }
  }
  return ret;
}

int InnoLidar::get_vertical_roi_(double *vertical_roi_angle) {
  int ret = 0;
  inno_log_verify(vertical_roi_angle, "vertical_roi_angle");

  if (lidar_source_ == LIDAR_SOURCE_LIVE) {
    double v;
    ret = comm_->get_vertical_roi(&v);
    if (ret == 0) {
      *vertical_roi_angle = v;
    }
  }
  return ret;
}

int InnoLidar::set_roi(double h_angle, double v_angle) {
  if (h_angle == std::numeric_limits<double>::max()
    && v_angle == std::numeric_limits<double>::max()) {
    return 1;
  }
  if (h_angle != std::numeric_limits<double>::max()) {
    if (set_hori_roi_(h_angle) != 0) {
      return 4;
    }
  }
  if (v_angle != std::numeric_limits<double>::max()) {
    if (set_vertical_roi_(v_angle) != 0) {
      return 5;
    }
  }
  return 0;
}

int InnoLidar::get_roi(double *h_angle, double *v_angle) {
  inno_log_verify(h_angle, "h_angle");
  inno_log_verify(v_angle, "v_angle");
  *h_angle = hori_roi_angle_;
  *v_angle = vertical_roi_angle_;
  return 0;
}

int InnoLidar::get_apd_cal_status(int *status) {
  if (is_live_lidar_()) {
    return comm_->get_apd_cal_status(status);
  } else {
    *status = 1;
    return 0;
  }
}

int InnoLidar::set_motion_compensation(double velocity[3],
                                       double a_velocity[3]) {
  /*
  {
    std::unique_lock<std::mutex>(velocity_mutex_);

    for (int i = 0; i < 3; i++) {
      velocity_[i] = velocity[i];
      a_velocity_[i] = a_velocity[i];
    }
  }
  inno_log_trace("%p set comp %f %f %f %f %f %f",
                 this, velocity[0], velocity[1], velocity[2],
                 velocity_[0], velocity_[1], velocity_[2]);
  */
  return 0;
}

int InnoLidar::thread_setaffinity_np(size_t cpusetsize, const cpu_set_t *cpuset,
                                     int exclude_callback_thread) {
  inno_log_panic_if_not(cpusetsize > 0 && cpuset != NULL,
                        "invalid calling parameter");
  cpusetsize_ = cpusetsize;
  cpuset_ = reinterpret_cast<cpu_set_t *>(malloc(cpusetsize));
  exclude_callback_thread_ = exclude_callback_thread;

  if (cpuset_ == NULL) {
    inno_log_error("cannot allocate memory for cpuset");
    return 2;
  }
  memcpy(cpuset_, cpuset, cpusetsize);
  return 0;
}

int InnoLidar::get_fw_state(enum InnoLidarState *state,
                            int *error_code) {
  if (is_live_lidar_()) {
    LidarCommunication::StatusCounters counters;
    int ret = comm_->get_status(&counters, false);
    if (ret) {
      *state = INNO_LIDAR_STATE_UNKNOWN;
      *error_code = 0;
    } else {
      *state = counters.stream_status;
      *error_code = counters.error_code;
    }
    return 0;
  } else {
    // xxx todo: based on real streamin state
    *state = INNO_LIDAR_STATE_READY;
    *error_code = 0;
    return 0;
  }
}

int InnoLidar::get_fw_version(char *buffer, int buffer_len) {
  if (is_live_lidar_()) {
    return comm_->get_fw_version(buffer, buffer_len);
  } else {
    int ret = snprintf(buffer, buffer_len, "Inno Lidar Server");
    return ret < buffer_len ? 0 : -1;
  }
}

int InnoLidar::get_fw_sequence_buf(char *buffer, int buffer_len) {
  if (is_live_lidar_()) {
    return comm_->get_fw_sequence(buffer, buffer_len);
  } else {
    int ret = snprintf(buffer, buffer_len, "0");
    return ret < buffer_len ? 0 : -1;
  }
}

int InnoLidar::get_sn(char *buffer, int buffer_len) {
  if (is_live_lidar_()) {
    return comm_->get_sn(buffer, buffer_len);
  } else {
    int ret = snprintf(buffer, buffer_len, "s_file_replay");
    return ret < buffer_len ? 0 : -1;
  }
}

int InnoLidar::get_model(char *buffer, int buffer_len) {
  if (is_live_lidar_()) {
    return comm_->get_model(buffer, buffer_len);
  } else {
    int ret = snprintf(buffer, buffer_len,
                       "%c", (lidar_model_ - LIDAR_MODEL_A) + 'a');
    return ret < buffer_len ? 0 : -1;
  }
}

int InnoLidar::get_temperature(char *buffer, int buffer_len) {
  if (is_live_lidar_()) {
    return comm_->get_temperature(buffer, buffer_len);
  } else {
    int ret = snprintf(buffer, buffer_len, "Inno temperature");
    return ret < buffer_len ? 0 : -1;
  }
}

int InnoLidar::get_detector_temps(char *buffer, int buffer_len) {
  if (is_live_lidar_()) {
    return comm_->get_detector_temps(buffer, buffer_len);
  } else {
    int ret = snprintf(buffer, buffer_len, "Inno detector temperature");
    return ret < buffer_len ? 0 : -1;
  }
}

int InnoLidar::get_detector_temps(int64_t (&temp)[kInnoChannelNumber]) {
  if (is_live_direct_memory_lidar_() && read_fw_ipc_is_ready()) {
    for (uint32_t i = 0; i < kInnoChannelNumber; i++) {
      temp[i] = read_fw_ipc(IpcItem(IPC_ITEM_TEMPERATURE_DET_0_10th_C + i),
                            kInvalidDetTemp);
    }
    return 0;
  }
  return -1;
}

int InnoLidar::get_motor_speeds(char *buffer, int buffer_len) {
  if (is_live_lidar_()) {
    return comm_->get_motor_speeds(buffer, buffer_len);
  } else {
    int ret = snprintf(buffer, buffer_len, "Inno motor speeds");
    return ret < buffer_len ? 0 : -1;
  }
}

int InnoLidar::get_inner_faults_info(char* buffer,
                                     int buffer_len,
                                     bool from_network) {
  if (is_live_lidar_()) {
    inno_log_verify(fault_manager_, "fault_manager_ is NULL");
    return fault_manager_->get_inner_faults_info(buffer,
                                                 buffer_len,
                                                 from_network);
  } else {
    int ret = snprintf(buffer, buffer_len, "File play mode, no faults info");
    return ret < buffer_len ? 0 : -1;
  }
}

int InnoLidar::get_inner_faults_all_info(char* buffer, int buffer_len) {
  if (is_live_lidar_()) {
    return fault_manager_->get_inner_faults_all_info(buffer, buffer_len);
  } else {
    int ret = snprintf(buffer, buffer_len, "File play mode, no faults info");
    return ret < buffer_len ? 0 : -1;
  }
}

int InnoLidar::get_sync_cycle_status(char* buffer, int buffer_len) {
  if (is_live_lidar_()) {
    return fault_manager_->get_sync_cycle_status(buffer, buffer_len);
  } else {
    int ret = snprintf(buffer, buffer_len, "File play mode, no faults info");
    return ret < buffer_len ? 0 : -1;
  }
}

void InnoLidar::start() {
  if (strncmp(inno_api_version(), INNO_SDK_VERSION_IN_HEADER,
              strlen(INNO_SDK_VERSION_IN_HEADER)) != 0) {
    do_message_callback_fmt(
        INNO_MESSAGE_LEVEL_CRITICAL,
        INNO_MESSAGE_CODE_LIB_VERSION_MISMATCH,
        "Header file inno_lidar_api.h and "
        "libinnolidar.so file version mismatch: %s vs %s. "
        "Please use the right inno_lidar_api.h and recompile.",
        INNO_SDK_VERSION_IN_HEADER, inno_api_version());
  }
  inno_log_info("%s start", name_);
  inno_log_info("%s LIDAR SDK version is %s", name_, inno_api_version());
  inno_log_info("%s LIDAR SDK build tag is %s", name_, inno_api_build_tag());
  inno_log_info("%s LIDAR SDK build time is %s", name_, inno_api_build_time());
  inno_log_verify(!last_stage_is_up_,
                  "%s forget to call stop before restart?",
                  name_);

  setup_job_pools_();

  system_stats_ = new SystemStats(this);
  inno_log_verify(system_stats_, "cannot alloc system_stats_");

  stage_signal_ = new StageSignal(this, play_rate_x_);
  inno_log_verify(stage_signal_, "stage_signal_");
  stage_angle_ = new StageAngle(this);
  inno_log_verify(stage_angle_, "stage_angle_");
  stage_noise_filter_phase0_ = new StageNoiseFilter(this, 0);
  inno_log_verify(stage_noise_filter_phase0_, "stage_noise_filter_phase0_");
  stage_noise_filter_phase1_ = new StageNoiseFilter(this, 1);
  inno_log_verify(stage_noise_filter_phase1_, "stage_noise_filter_phase1_");
  stage_deliver_ = new StageDeliver(this);
  inno_log_verify(stage_deliver_, "stage_deliver_");
  stage_deliver2_ = new StageDeliver2(this);
  inno_log_verify(stage_deliver2_, "stage_deliver2_");
  stage_help_ = new StageHelp(this);
  inno_log_verify(stage_help_, "stage_help_");
  status_report_ = new StatusReport(this, 50);
  inno_log_verify(status_report_, "status_report");

  //
  it_status_ = new InnoThread("status", 41,
                              1, StatusReport::report, status_report_,
                              cpusetsize_, cpuset_);
  inno_log_verify(it_status_, "it_status");

  bool can_drop = is_live_lidar_() ||
                  play_rate_ != 0 || play_rate_x_ != 0;

  cp_help_ = new ConsumerProducer("help", 0,
                                  1, StageHelp::process, stage_help_,
                                  100, 0, 0,
                                  cpusetsize_, cpuset_);
  inno_log_verify(cp_help_, "helper");

  cp_deliver2_ = new ConsumerProducer("deliver2", 2,
                                      1, StageDeliver2::process,
                                      stage_deliver2_,
                                      10,
                                      can_drop ? 20 : 0,
                                      100,
                                      cpusetsize_,
                                      exclude_callback_thread_ ?
                                      NULL : cpuset_);
  inno_log_verify(cp_deliver2_, "deliver2");

  cp_deliver_ = new ConsumerProducer("deliver", 2,
                                     1, StageDeliver::process,
                                     stage_deliver_,
                                     10 * config_.encodes_per_polygon,
                                     can_drop ? 300 : 0,
                                     100,
                                     cpusetsize_,
                                     exclude_callback_thread_ ?
                                     NULL : cpuset_);
  inno_log_verify(cp_deliver_, "deliver");

  cp_noise_filter_phase1_ = new ConsumerProducer(
      "noise_filter_phase1", 1,
      1, StageNoiseFilter::process,
      stage_noise_filter_phase1_,
      20 * config_.encodes_per_polygon,
      can_drop ? 300 : 0,
      0,
      cpusetsize_, cpuset_);
  inno_log_verify(cp_noise_filter_phase1_, "noise_filter_phase1");

  cp_noise_filter_phase0_ = new ConsumerProducer(
      "noise_filter_phase0", 1,
      1, StageNoiseFilter::process,
      stage_noise_filter_phase0_,
      20 * config_.encodes_per_polygon,
      can_drop ? 300 : 0,
      0,
      cpusetsize_, cpuset_);
  inno_log_verify(cp_noise_filter_phase0_, "noise_filter_phase0");

  cp_angle_ = new ConsumerProducer("angle", 30,
                                   1, StageAngle::process,
                                   stage_angle_,
                                   3 * config_.encodes_per_polygon,
                                   0,  // no drop
                                   0,
                                   cpusetsize_, cpuset_);
  inno_log_verify(cp_angle_, "angle");

  cp_signal_ = new ConsumerProducer("signal", 35,
                                    1, StageSignal::process,
                                    stage_signal_,
                                    3,
                                    0,  // no drop
                                    0,
                                    cpusetsize_, cpuset_);
  inno_log_verify(cp_signal_, "signal");

  cp_read_ = new ConsumerProducer("read", 40,
                                  1, StageRead::process,
                                  stage_read_,
                                  2,
                                  2,
                                  0,
                                  cpusetsize_, cpuset_);
  inno_log_verify(cp_read_, "read");
  {
    std::unique_lock<std::mutex> lk(frame_sync_mutex_);
    if (frame_sync_enable_) {
      start_frame_sync_with_lock_(frame_sync_time_, false);
    }
  }
  it_status_->start();

  cp_deliver2_->start();
  {
    std::unique_lock<std::mutex> lk(last_stage_mutex_);
    last_stage_is_up_ = true;
  }
  cp_deliver_->start();
  cp_help_->start();
  cp_noise_filter_phase1_->start();
  cp_noise_filter_phase0_->start();
  cp_angle_->start();
  cp_signal_->start();
  cp_read_->start();

  cp_read_->add_job(NULL);
  inno_log_info("%s started", name_);

  do_message_callback_fmt(INNO_MESSAGE_LEVEL_INFO,
                          INNO_MESSAGE_CODE_NEW_START,
                          "%s started", name_);
}

void InnoLidar::stop() {
  inno_log_info("%s stop stage_read_", name_);
  print_stats();

  stage_read_->stop();

  // wait until stage change to stopped
  inno_log_info("%s shutdown read", name_);
  cp_read_->shutdown();
  inno_log_info("%s shutdown signal", name_);
  cp_signal_->shutdown();
  inno_log_info("%s shutdown angle", name_);
  cp_angle_->shutdown();
  inno_log_info("%s shutdown noise_filter_phase0", name_);
  cp_noise_filter_phase0_->shutdown();
  inno_log_info("%s shutdown noise_filter_phase1", name_);
  cp_noise_filter_phase1_->shutdown();
  inno_log_info("%s shutdown help", name_);
  cp_help_->shutdown();
  inno_log_info("%s shutdown deliver", name_);
  cp_deliver_->shutdown();
  inno_log_info("%s shutdown deliver2", name_);
  cp_deliver2_->shutdown();

  inno_log_info("%s final cleanup", name_);
  stage_read_->final_cleanup();

  {
    std::unique_lock<std::mutex> lk(last_stage_mutex_);
    last_stage_is_up_ = false;
  }

  // shut down frame sync thread after pipeline stop
  {
    std::unique_lock<std::mutex> lk(frame_sync_mutex_);
    if (frame_sync_) {
      frame_sync_->stop();
      delete frame_sync_;
      frame_sync_ = nullptr;
    }
  }

  inno_log_info("%s final cleanup done", name_);

  inno_log_info("%s shutdown status report", name_);
  it_status_->shutdown();

  delete(system_stats_);
  system_stats_ = NULL;

  delete cp_read_;
  cp_read_ = NULL;
  delete cp_signal_;
  cp_signal_ = NULL;
  delete cp_angle_;
  cp_angle_ = NULL;
  delete cp_noise_filter_phase0_;
  cp_noise_filter_phase0_ = NULL;
  delete cp_noise_filter_phase1_;
  cp_noise_filter_phase1_ = NULL;
  delete cp_deliver_;
  cp_deliver_ = NULL;
  delete cp_deliver2_;
  cp_deliver2_ = NULL;
  delete cp_help_;
  cp_help_ = NULL;
  delete it_status_;
  it_status_ = NULL;

  // DO NOT delete stage_read
  delete stage_signal_;
  stage_signal_ = NULL;
  delete stage_angle_;
  stage_angle_ = NULL;
  delete stage_noise_filter_phase0_;
  stage_noise_filter_phase0_ = NULL;
  delete stage_noise_filter_phase1_;
  stage_noise_filter_phase1_ = NULL;
  delete stage_deliver_;
  stage_deliver_ = NULL;
  delete stage_deliver2_;
  stage_deliver2_ = NULL;
  delete stage_help_;
  stage_help_ = NULL;
  delete status_report_;
  status_report_ = NULL;

  free_job_pools_();

  inno_log_info("%s stopped", name_);
}

void InnoLidar::set_raw_fault(enum InnoSubInFaults sub_fault_id,
                              bool is_sub_fault,
                              bool condition,
                              const char *format, ...) {
  inno_log_verify(fault_manager_, "fault_manager_ is NULL");
  bool valid = sub_fault_id > INNO_SUB_FAULT_BASE &&
               sub_fault_id < INNO_SUB_FAULT_MAX;
  inno_log_verify(is_sub_fault == valid,
                 "fid: %d is_sub_fault invalid: %d vs. %d",
                  sub_fault_id, is_sub_fault, valid);
  va_list valist;
  va_start(valist, format);
  fault_manager_->set_raw_fault(static_cast<int>(sub_fault_id),
                                is_sub_fault,
                                INNO_FAULT_OPERATION_SET,
                                condition,
                                format,
                                valist);
  va_end(valist);
}

void InnoLidar::set_raw_fault(enum InnoLidarInFault fid,
                              bool condition,
                              const char *format, ...) {
  inno_log_verify(fault_manager_, "fault_manager_ is NULL");
  va_list valist;
  va_start(valist, format);
  fault_manager_->set_raw_fault(static_cast<int>(fid),
                                false,
                                INNO_FAULT_OPERATION_SET,
                                condition,
                                format,
                                valist);
  va_end(valist);
}

void InnoLidar::set_history_fault(int fid,
                                  const char *format, ...) {
  inno_log_verify(fault_manager_, "fault_manager_ is NULL");
  va_list valist;
  va_start(valist, format);
  fault_manager_->set_history_fault(fid, format, valist);
  va_end(valist);
}

void InnoLidar::heal_raw_fault(enum InnoSubInFaults sub_fault_id,
                               bool is_sub_fault,
                               bool condition,
                               const char *format, ...) {
  bool valid = sub_fault_id > INNO_SUB_FAULT_BASE &&
               sub_fault_id < INNO_SUB_FAULT_MAX;
  inno_log_verify(is_sub_fault == valid,
                 "fid: %d is_sub_fault invalid: %d vs. %d",
                  sub_fault_id, is_sub_fault, valid);
  va_list valist;
  va_start(valist, format);
  fault_manager_->set_raw_fault(static_cast<int>(sub_fault_id),
                                is_sub_fault,
                                INNO_FAULT_OPERATION_HEAL,
                                condition,
                                format,
                                valist);
  va_end(valist);
}

void InnoLidar::heal_raw_fault(enum InnoLidarInFault fid,
                               bool condition,
                               const char *format, ...) {
  inno_log_verify(fid >= INNO_LIDAR_IN_FAULT_OTHER &&
                  fid < INNO_LIDAR_IN_FAULT_MAX, "fid invalid: %d", fid);
  va_list valist;
  va_start(valist, format);
  fault_manager_->set_raw_fault(static_cast<int>(fid),
                                false,
                                INNO_FAULT_OPERATION_HEAL,
                                condition,
                                format,
                                valist);
  va_end(valist);
}

void InnoLidar::print_stats(void) {
  std::unique_lock<std::mutex> lk(last_stage_mutex_);
  if (last_stage_is_up_) {
    cp_read_->print_stats();
    cp_signal_->print_stats();
    cp_angle_->print_stats();
    cp_noise_filter_phase0_->print_stats();
    cp_noise_filter_phase1_->print_stats();
    cp_deliver_->print_stats();
    cp_deliver2_->print_stats();

    stage_read_->print_stats();
    stage_signal_->print_stats();
    stage_angle_->print_stats();
    stage_noise_filter_phase0_->print_stats();
    stage_noise_filter_phase1_->print_stats();
    stage_deliver_->print_stats();
    stage_deliver2_->print_stats();
    {
      std::unique_lock<std::mutex> lk(frame_sync_mutex_, std::try_to_lock);
      if (lk.owns_lock() && frame_sync_) {
        frame_sync_->print_stats();
      }
    }
  } else {
    inno_log_panic("last_stage is not up");
  }

  return;
}

StageSignalJob *InnoLidar::alloc_signal_job(void) {
  return stage_signal_job_pool_->alloc();
}

StageAngleJob *InnoLidar::alloc_angle_job(void) {
  return stage_angle_job_pool_->alloc();
}

StageDeliverStatusJob *InnoLidar::alloc_deliver_status_job(void) {
  return stage_deliver_status_job_pool_->alloc();
}

StageDeliverMessageJob *InnoLidar::alloc_deliver_message_job(void) {
  return stage_deliver_message_job_pool_->alloc();
}

StageDeliverPointsJob *InnoLidar::alloc_deliver_points_job(void) {
  return stage_deliver_points_job_pool_->alloc();
}

StageDeliver2Job *InnoLidar::alloc_deliver2_job(void) {
  return stage_deliver2_job_pool_->alloc();
}

void InnoLidar::free_signal_job(StageSignalJob *job) {
  stage_signal_job_pool_->free(job);
}

void InnoLidar::free_angle_job(StageAngleJob *job) {
  stage_angle_job_pool_->free(job);
}

void InnoLidar::free_deliver_status_job(StageDeliverStatusJob *job) {
  return stage_deliver_status_job_pool_->free(job);
}

void InnoLidar::free_deliver_message_job(StageDeliverMessageJob *job) {
  return stage_deliver_message_job_pool_->free(job);
}

void InnoLidar::free_deliver_points_job(StageDeliverPointsJob *job) {
  return stage_deliver_points_job_pool_->free(job);
}

void InnoLidar::free_deliver2_job(StageDeliver2Job *job) {
  return stage_deliver2_job_pool_->free(job);
}

void InnoLidar::add_stage_signal_job(StageSignalJob *job) {
  cp_signal_->add_job(job);
}

void InnoLidar::add_stage_angle_job(StageAngleJob *job) {
  cp_angle_->add_job(job);
}

void InnoLidar::add_stage_noise_filter_phase0_job(StageAngleJob *job) {
  cp_noise_filter_phase0_->add_job(job);
}

void InnoLidar::add_stage_noise_filter_phase1_job(StageAngleJob *job) {
  cp_noise_filter_phase1_->add_job(job);
}

void InnoLidar::add_stage_deliver_job(StageAngleJob *job) {
  cp_deliver_->add_job(job);
}

void InnoLidar::add_stage_deliver2_job(StageDeliver2Job *job) {
  cp_deliver2_->add_job(job);
}

// xxx todo, deliver, help ...

void InnoLidar::update_config() {
  // set model specific config parameters as the example below
  // if (lidar_model_ == LIDAR_MODEL_K) {
  //   config_manage_.set_config_key_value(
  //     "Lidar_StageSignal/kill_retro_noise", "1", false);
  // }

  // Those config parameters passed from app override
  // model specific default config parameters
  config_manage_.play_config();
}

int InnoLidar::before_read_start(void) {
  inno_log_info("before_read_start");

  int ret;
  if (is_live_lidar_()) {
    // xxx todo: get_streamer
    // xxx todo: stop current streamer

    // get model
    char model[2];
    ret = comm_->get_model(model, sizeof(model));
    if (ret) {
      inno_log_error("cannot get model %d", ret);
      return ret;
    } else {
      enum LidarModel live_model;
      if (strcasecmp(model, "i") == 0) {
        live_model = LIDAR_MODEL_I;
      } else if (strcasecmp(model, "k") == 0) {
        live_model = LIDAR_MODEL_K;
      } else if (strcasecmp(model, "g") == 0) {
        live_model = LIDAR_MODEL_G;
      } else {
        inno_log_error("invalid live model %s", model);
        return -1;
      }
      if (lidar_model_ == LIDAR_MODEL_NONE) {
        inno_log_info("use live lidar_model %d", live_model);
        lidar_model_ = live_model;
      } else if (lidar_model_ != live_model) {
        inno_log_warning("live lidar_model %d diffs from command "
                         "line lidar_model %d, ignore live model",
                         live_model, lidar_model_);
      }
    }

    // fw version
    char buffer[1024];
    ret = comm_->get_fw_version(buffer, sizeof(buffer));
    if (ret) {
      inno_log_error("cannot get fw version %d", ret);
      return ret;
    } else {
      inno_log_info("%s fw version: %s",
                    get_name(),
                    buffer);
    }

    ret = comm_->get_fw_sequence(buffer, sizeof(buffer));
    if (ret) {
      inno_log_error("cannot get fw sequence %d", ret);
      return ret;
    } else {
      fw_sequence_ = atoi(buffer);
      inno_log_info("%s fw sequence: %u",
                    get_name(),
                    fw_sequence_);
    }

    // sn
    ret = comm_->get_sn(buffer, sizeof(buffer));
    if (ret) {
      inno_log_error("cannot get sn %d", ret);
      return ret;
    } else {
      inno_log_info("%s serial number: %s",
                    get_name(),
                    buffer);
    }

    // clear raw raw mode
    ret = set_raw_channel_(-1);
    if (ret) {
      inno_log_error("cannot set_raw_channel %d", ret);
      return ret;
    }
  }

  {
    std::unique_lock<std::mutex> lk(params_mutex_);
    ret = parse_lidar_yaml_();
    if (ret == 0) {
      if (lidar_model_ == LIDAR_MODEL_NONE) {
        if (params_.iv_params.device_model == 0) {
          lidar_model_ = LIDAR_MODEL_I;
          inno_log_info("no lidar_model in yaml file, force MODEL_I");
        } else if (params_.iv_params.device_model != 9 &&
                   params_.iv_params.device_model != 11 &&
                   params_.iv_params.device_model != 7) {
          inno_log_error("invalid lidar_model %d in yaml file. "
                         "Valid lidar_model list: 7(g) 9(i) 11(k)",
                         params_.iv_params.device_model);
          return -1;
        } else {
          lidar_model_ = (enum LidarModel)params_.iv_params.device_model;
          inno_log_info("use yaml file lidar_model %d", lidar_model_);
        }
      }

      // now that we have yaml file, initialize the angle lookup
      // table here before streaming starts
      misc_tables_.setup_tables_from_yaml(params_.iv_params);
      stage_angle_->init_lookup_table();
    } else {
      // wait for at most 2 seconds
      bool inited = params_cond_.wait_for(lk, std::chrono::seconds(2),
                  [this] { return stage_angle_->is_lookup_table_inited(); });
      if (!inited) {
        inno_log_panic("wait timeout, "
                       "angle lookup table is not initialized. "
                       "The raw file doesn't have a yaml header and "
                       "no yaml file is specified.");
        return -1;
      }
      inno_log_info("wait returns, angle lookup table is initialized");
    }
  }

  // now that we have lidar model, we can set
  // model-specific config parameters
  update_config();

  if (is_live_lidar_()) {
    LidarCommunication::StatusCounters counters;
    ret = comm_->get_status(&counters, false);
    if (ret) {
      inno_log_error("cannot get status");
      return ret;
    }

    // time sync method
    clock_.reset(counters.clock_config, this->clock_config_);

    // get galvo mode
    galvo_mode_ = static_cast<enum InnoGalvoMode>(comm_->get_galvo_mode());
    if (galvo_mode_ >= INNO_GALVO_MODE_MAX) {
      galvo_mode_ = INNO_GALVO_MODE_NORMAL;
    }

    // clear raw_capture
    ret = comm_->set_raw_capture_mode(false);
    if (ret) {
      inno_log_error("cannot clear raw capture mode");
      return ret;
    } else {
      inno_log_info("clear raw capture mode");
    }

    // get packet format
    bool is_extended_format;
    ret = comm_->get_ext_packet_format(&is_extended_format);
    if (ret) {
      inno_log_error("cannot get packet format");
      return ret;
    }
    use_extended_format_ = is_extended_format;

    // set vertical roi
    ret = get_vertical_roi_(&vertical_roi_angle_);
    if (ret) {
      inno_log_error("cannot set vertical roi");
      return ret;
    } else {
      vertical_roi_angle_unit_ = vertical_roi_angle_ * kInnoAngleUnitPerDegree;
    }
    return 0;
  } else {
    // new lidar_clock
    clock_.reset(INNO_TIME_SYNC_CONFIG_FILE, this->clock_config_);

    use_extended_format_ = true;
  }
  return 0;
}

void InnoLidar::add_config(Config *c) {
  inno_log_verify(c, "invalid config");
  config_manage_.add_config(c);
}

void InnoLidar::remove_config(Config *c) {
  inno_log_verify(c, "invalid config");
  config_manage_.remove_config(c);
}

void InnoLidar::stats_update_packet_bytes(enum ResourceStats::PacketType type,
                                          size_t packet, size_t byte) {
  if (system_stats_) {
    inno_log_verify(system_stats_, "system_stats");
    system_stats_->update_packet_bytes(type, packet, byte);
  }
}

void InnoLidar::stats_update_packet_bytes(size_t ref_count_sum,
                                          uint64_t intensity_sum) {
  if (system_stats_) {
    inno_log_verify(system_stats_, "system_stats_ref");
    system_stats_->update_packet_bytes\
                  (ResourceStats::PacketType::PACKET_TYPE_POINT, 0, 0,
                   ref_count_sum, intensity_sum);
  }
}

std::string InnoLidar::get_version_yaml() {
  std::string yaml("INNODATA1000");   // version

  char *yaml_buffer = get_yaml_buffer_();
  if (yaml_buffer) {
    uint32_t size = strlen(yaml_buffer);
    uint32_t n_size = htonl(size);

    inno_log_info("yaml size: %u, htonl: %u ", size, n_size);

    yaml.append(reinterpret_cast<const char *>(&n_size), sizeof(n_size));
    yaml.append(yaml_buffer, size);
  }

  return yaml;
}

void InnoLidar::recorder_write_yaml(enum InnoRecorderCallbackType type) {
  char version[] = "INNODATA1000";
  char *buffer = get_yaml_buffer_();

  /**
   * [CST Bugfinder Defect] Reviewed
   *
   * PolySpace report a defect here:
   * Unnecessary code, if-condition is always true.
   *
   * But buffer may be NULL,
   * and l < buffer_written_len may be true.
   *
   * So ignore this defect
   */
  if (buffer) {
    uint32_t size;
    char new_buffer[kMaxYamlSize]{0};
    size_t buffer_written_len =
        sizeof(new_buffer) - strlen(version) - sizeof(size);
    memset(new_buffer, 0, buffer_written_len);

    size_t l = strlen(buffer);
    size = buffer_written_len;
    size = htonl(size);

    if (l < buffer_written_len) {
      memcpy(new_buffer, buffer, l);
      do_recorder_callback(type, version, strlen(version));
      do_recorder_callback(type, (const char *)&size, sizeof(size));
      do_recorder_callback(type, new_buffer, buffer_written_len);
    }
  }
}

int64_t InnoLidar::read_fw_ipc(enum IpcItem item,
                               int64_t default_v) {
  int64_t v;
  bool r = fw_ipc_.read(item, &v);
  if (!r) {
    inno_log_warning("use default value %" PRI_SIZED " for ipc item %d",
                     default_v, item);
    v = default_v;
  }
  return v;
}


//
// for status packet
//
void InnoLidar::set_status_counters(InnoStatusCounters *counters,
                                    bool first_call) {
  inno_log_verify(counters, "counters is null!");

  // explicit check;
  if (!this->system_stats_ || !this->stage_read_ || !this->stage_signal_ ||
      !this->stage_angle_ || !this->stage_noise_filter_phase0_ ||
      !this->stage_noise_filter_phase1_ || !this->stage_deliver_ ||
      !this->stage_deliver2_) {
    return;
  }

  // actually sent;
  counters->point_data_packet_sent = stat_point_data_packet_sent_;
  counters->point_sent = stat_point_sent_;
  counters->message_packet_sent = stat_message_packet_sent_;

  // stage_read
  counters->raw_data_read = this->stage_read_->stats_raw_data_read_;

  // stage_signal
  memcpy(counters->in_signals, this->stage_signal_->stats_signals_,
         sizeof(counters->in_signals));

  counters->total_polygon_rotation =
      this->stage_signal_->stats_pr_encoder_count_ +
      this->stage_signal_->stats_pf_encoder_count_;

  counters->lose_ptp_sync = this->clock_.get_stat_ptp();

  // uint16_t motor[5];  /* std, min, max1, max2, mean */
  counters->motor[0] =
      static_cast<uint16_t>(this->stage_signal_->polygon_mean_.std_dev());
  counters->motor[1] =
      static_cast<uint16_t>(this->stage_signal_->polygon_mean_.min());
  counters->motor[2] =
      static_cast<uint16_t>(this->stage_signal_->polygon_mean_.max());
  counters->motor[3] =
      static_cast<uint16_t>(this->stage_signal_->polygon_mean_.max_delta());
  counters->motor[4] =
      static_cast<uint16_t>(this->stage_signal_->polygon_mean_.mean());

  // uint16_t galvo[5];  /* std, min, max1, max2, mean */
  for (auto &mean : this->stage_signal_->galvo_mean_) {
    counters->galvo[0] += static_cast<uint16_t>(mean.std_dev());
    counters->galvo[1] += static_cast<uint16_t>(mean.min());
    counters->galvo[2] += static_cast<uint16_t>(mean.max());
    counters->galvo[3] += static_cast<uint16_t>(mean.max_delta());
    counters->galvo[4] += static_cast<uint16_t>(mean.mean());
  }

  // stage_angle
  counters->total_frame = this->stage_angle_->stats_frames_;
  counters->total_polygon_facet = this->stage_angle_->stats_facets_;

  {
    counters->data_drop[0] = 0;
    counters->data_drop[1] = 0;

    // reg access only works when pcs is running inside the lidar.
    static uint32_t kInvalidUptime = UINT32_MAX;
    counters->lifelong_uptime = kInvalidUptime;
    if (is_live_direct_memory_lidar_()) {
      if (read_fw_ipc_is_ready()) {
        int64_t lifelong_uptime = read_fw_ipc(IPC_ITEM_LIFELONG_UPTIME,
                                              kInvalidUptime);
        // bit32 of lifelong_uptime means valid or not
        if (((lifelong_uptime >> 32) & 0x1) != 0) {
          counters->lifelong_uptime = lifelong_uptime;
        }
      }
#define DROPPED 0x1
#define UNKNOWN 0x2
      uint32_t fpga_dropped_flag = 0;
      for (uint32_t i = 0;
           i < sizeof(stat_adc_data_drop_) / sizeof(struct StatAdcDataDrop);
           i++) {
        uint32_t value = 0;
        int ret = 0;
        /**
         * [CST Bugfinder Defect] Reviewed
         *
         * PolySpace report a defect here:
         * If-condition is always true. "else {..}" is dead branch
         *
         * But we should verify here to make sure read_pl_reg return success.
         *
         * So ignore this defect
         *
         */
        if (i == 0) {
          ret = read_pl_reg(ADC_DROP_PACKET_REG, &value);
        } else if (i == 1) {
          ret = read_pl_reg(MOTOR_DROP_PACKET_REG, &value);
        } else {
          inno_log_verify(false, "invalid i %u", i);
        }
        if (ret == 0) {
          struct StatAdcDataDrop &drop = stat_adc_data_drop_[i];
          if (!first_call) {
            if (value >= drop.last) {
              drop.diff = value - drop.last;
            } else {
              drop.diff = ((uint64_t)value) + (1LU << 32) - drop.last;
            }
            drop.total += drop.diff;
          }
          drop.last = value;
          counters->data_drop[i] = drop.total;
          if (drop.diff > 0) {
            fpga_dropped_flag |= DROPPED;
          }
        } else {
          inno_log_warning("cannot get drop %u", i);
          fpga_dropped_flag |= UNKNOWN;
        }
      }
      set_raw_fault(INNO_LIDAR_IN_FAULT_DATA_DROP3,
                    fpga_dropped_flag & DROPPED,
                   "INNO_LIDAR_IN_FAULT_DATA_DROP3 sets");
      heal_raw_fault(INNO_LIDAR_IN_FAULT_DATA_DROP3,
                    !(fpga_dropped_flag & DROPPED) &&
                    !(fpga_dropped_flag & UNKNOWN),
                    "INNO_LIDAR_IN_FAULT_DATA_DROP3 heals");
    }

    counters->data_drop[2] =
        this->stage_noise_filter_phase0_->stats_jobs_dropped_;
    counters->data_drop[3] =
        this->stage_noise_filter_phase1_->stats_jobs_dropped_;

    counters->data_drop[4] = this->stage_deliver_->stats_dropped_jobs_;
    counters->data_drop[5] = this->stage_deliver2_->stats_dropped_jobs_;

    // reserved
    counters->data_drop[6] = 0;
    counters->data_drop[7] = 0;
  }

  // stage_deliver_;
  /*
   * uint16_t latency_10us_average[6];
   * uint16_t latency_10us_variation[6];
   * uint16_t latency_10us_max[6];
   **/
  this->stage_deliver_->get_stats(counters);

  // system_stats
  /*
   * uint32_t power_up_time_in_second;
   * uint32_t process_up_time_in_second;
   * uint16_t cpu_percentage;
   * uint16_t mem_percentage;
   **/
  this->system_stats_->get_sys_stats(counters);

  // TODO(Donglang): xxx.
  /*
   * uint32_t bad_data[4];
   * uint32_t big_latency_frame;
   * uint32_t bad_frame;
   * uint32_t big_gap_frame;
   * uint32_t small_gap_frame;
   **/
}

enum InnoGalvoMode InnoLidar::get_galvo_mode() {
  return galvo_mode_;
}

int InnoLidar::get_dsp_packet_reg(uint32_t *status) {
  if (!is_live_lidar_()) {
    inno_log_error("not support in file play mode");
    return -1;
  }
  int ret = 0;
  inno_log_verify(status, "status");
  if (is_live_direct_memory_lidar_()) {
    ret = read_ps_reg(PS_DSP_PACKET, status);
  } else {
    ret = comm_->read_ps_reg(PS_DSP_PACKET, status);
  }
  return ret;
}

int InnoLidar::get_fpga_ts(uint64_t *ts_ns) {
  if (!is_live_lidar_()) {
    inno_log_error("not support in file play mode");
    return -1;
  }
  inno_log_verify(ts_ns, "ts_ns");
  int ret = 0;
  uint32_t low_32;
  uint32_t high_32;
  if (is_live_direct_memory_lidar_()) {
    ret = read_pl_reg(PL_FPGA_TS_LOW, &low_32);
    if (ret) {
      return ret;
    }
    ret = read_pl_reg(PL_FPGA_TS_HIGH, &high_32);
    if (ret) {
      return ret;
    }
  } else {
    ret = comm_->reg_read(PL_FPGA_TS_LOW, &low_32);
    if (ret) {
      return ret;
    }
    ret = comm_->reg_read(PL_FPGA_TS_HIGH, &high_32);
    if (ret) {
      return ret;
    }
  }

  *ts_ns = 0;
  *ts_ns = ((*ts_ns | high_32) << 28) | (low_32 >> 4);
  *ts_ns <<= 3;

  return 0;
}

int InnoLidar::set_fpga_frame_sync_ts(uint64_t ts_ns) {
  if (!is_live_lidar_()) {
    inno_log_error("not support in file play mode");
    return -1;
  }
  int ret = 0;
  // set to {high_32[0-28], low_32[4, 32]} in terms of 8ns
  ts_ns <<= 1;
  uint32_t low_32 = ts_ns & 0xffffffff;
  uint32_t high_32 = ts_ns >> 32;
  if (is_live_direct_memory_lidar_()) {
    ret = write_pl_reg(PL_FRAME_SYNC_LOW, low_32);
    if (ret) {
      return ret;
    }
    ret = write_pl_reg(PL_FRAME_SYNC_HIGH, high_32);
    if (ret) {
      return ret;
    }
  } else {
    ret = comm_->reg_write(PL_FRAME_SYNC_LOW, low_32);
    if (ret) {
      return ret;
    }
    ret = comm_->reg_write(PL_FRAME_SYNC_HIGH, high_32);
    if (ret) {
      return ret;
    }
  }

  return 0;
}

void InnoLidar::frame_start_time_statistic(InnoDataPacket *packet) {
  std::unique_lock<std::mutex> lk(frame_sync_mutex_, std::try_to_lock);
  if (lk.owns_lock() && frame_sync_) {
    frame_sync_->frame_start_time_statistic(packet);
  }
}

int InnoLidar::start_frame_sync_with_lock_(double sync_time,
                                           bool restart_force) {
  int ret = 0;
  // client send a UTC_in_second_double, like 10000000.111s
  // pull up a high priority thread to generate sync-time in 8ns and set to
  // PL register 2E4(low 32 bits) 2E8(high 32 bits)
  // predicatively and periodically
  // If there is already a sync thread exit, stop it first
  if (frame_sync_) {
    if (!restart_force && frame_sync_->in_step_with(sync_time)) {
      inno_log_info("time %.3lf is in step with current", sync_time);
      return 0;
    }
    inno_log_info("stopping current frame sync thread.");
    frame_sync_->stop();
    delete frame_sync_;
  }
  frame_sync_ = FrameSync::create(sync_time, this);
  if (!frame_sync_) {
    inno_log_info("Start todo frame sync failed: create FrameSync failed");
    return 2;
  }
  frame_sync_->start();
  // for lidar restart
  frame_sync_time_ = sync_time;
  frame_sync_enable_ = true;
  inno_log_info("frame sync started. galvo_mode=%d", galvo_mode_);
  return ret;
}

double InnoLidar::get_motor_speed_config() {
  return comm_->get_motor_speed_config();
}

int InnoLidar::get_overheat_thresholds(int16_t (&threshold)[2]) {
  static const int16_t kInvalidOverhearThreshold = INT16_MAX;
  int ret = 0;
  for (int i = 0; i < 2; i++) {
    const char* key = i == 0 ? "overheat_set" : "overheat_heal";
    threshold[i] = comm_->\
                   get_config_section_key_value("laser", key,
                                                 kInvalidOverhearThreshold);
    if (threshold[i] == kInvalidOverhearThreshold) {
      ret = -1;
      inno_log_error("error getting %s threshold", key);
    }
  }
  return ret;
}
}  // namespace innovusion
