/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include <stdio.h>
#include <stdlib.h>
#if !(defined(__APPLE__) || defined(__MINGW64__))
#include "sys/sysinfo.h"
#endif
#include "sdk/lidar.h"
#include "sdk/fault_manager.h"
#include "thirdparty/nlohmann/json.hpp"

using json = nlohmann::json;

namespace innovusion {
bool FaultManager::laser_overheat_ = false;

FwFaultSynchronizer::FwFaultSynchronizer(InnoLidar* lidar) {
  lidar_ = lidar;
  inno_log_verify_simple(lidar_, "lidar_ is NULL");
  fw_fault_.clear();
  enum InnoLidarInFault fw_fault_list[] = {
    INNO_LIDAR_IN_FAULT_POWER_LOW,
    INNO_LIDAR_IN_FAULT_POWER_HIGH,
    INNO_LIDAR_IN_FAULT_LASER_INTERLOCK,
    INNO_LIDAR_IN_FAULT_COMM_LASER,
    INNO_LIDAR_IN_FAULT_LASER,
    INNO_LIDAR_IN_FAULT_COMM_DSP,
    INNO_LIDAR_IN_FAULT_CANFD_DSP,
    INNO_LIDAR_IN_FAULT_DSP,
    INNO_LIDAR_IN_FAULT_POLYGON_CONTROL,
    INNO_LIDAR_IN_FAULT_POLYGON_SENSOR,
    INNO_LIDAR_IN_FAULT_GALVO_CONTROL,
    INNO_LIDAR_IN_FAULT_GALVO_SENSOR,
    INNO_LIDAR_IN_FAULT_IIC_DSP,
    INNO_LIDAR_IN_FAULT_IIC_SOC,
    INNO_LIDAR_IN_FAULT_DSP_EXTWD,
    INNO_LIDAR_IN_FAULT_CHIPTEMP,
    INNO_LIDAR_IN_FAULT_HUMIDITY,
    INNO_LIDAR_IN_FAULT_COMM_ADC,
    INNO_LIDAR_IN_FAULT_FPGACLOCK,
    INNO_LIDAR_IN_FAULT_SOC,
    INNO_LIDAR_IN_FAULT_SOC_EXTWD,
    INNO_LIDAR_IN_FAULT_POWSUPL1,
    INNO_LIDAR_IN_FAULT_POWSUPL2,
    INNO_LIDAR_IN_FAULT_LPDDR4,
    INNO_LIDAR_IN_FAULT_FLASH,
    INNO_LIDAR_IN_FAULT_REPROGRAMMING
  };
  for (auto fault : fw_fault_list) {
    fw_fault_.insert(static_cast<int>(fault));
  }
  enum InnoLidarInFault history_fault_only[] = {
    INNO_LIDAR_IN_FAULT_DSP_EXTWD,
    INNO_LIDAR_IN_FAULT_SOC_EXTWD,
  };
  for (auto fault : history_fault_only) {
    history_fault_.insert(static_cast<int>(fault));
  }
}

FwFaultSynchronizer::~FwFaultSynchronizer() {
}

void FwFaultSynchronizer::sync_fw_faults() {
  if (!lidar_->is_live_direct_memory_lidar_() ||
      !lidar_->read_fw_ipc_is_ready()) {
    return;
  }
  static const int32_t kInvalidSeq = -1;
  int ipc_fault_base = IPC_ITEM_FAULTS_BEGIN;
  for (int i = 0; i < INNO_LIDAR_IN_FAULT_MAX; i++) {
    if (fw_fault_.count(i) == 0) {
      continue;
    }
    enum IpcItem fw_fault_set_item = IpcItem(i * 2 + ipc_fault_base);
    enum IpcItem fw_fault_heal_item = IpcItem(i * 2 + 1 + ipc_fault_base);
    int64_t fw_fault_set_seq =
            lidar_->read_fw_ipc(fw_fault_set_item, kInvalidSeq);
    int64_t fw_fault_heal_seq =
            lidar_->read_fw_ipc(fw_fault_heal_item, kInvalidSeq);
    if (fw_fault_set_seq == kInvalidSeq ||
        fw_fault_heal_seq == kInvalidSeq) {
      static uint32_t print_counter = 0;
      if (print_counter++ < 30 || print_counter % 256 == 0) {
        inno_log_error("read IPC invalid for item: %d/%d,"
                        "got: %" PRI_SIZED "/%" PRI_SIZED,
                        fw_fault_set_item, fw_fault_heal_item,
                        fw_fault_set_seq, fw_fault_heal_seq);
      }
      continue;
    }
    if (fw_fault_set_seq > fw_fault_heal_seq) {
      if (history_fault_.count(i)) {
        lidar_->set_history_fault((InnoLidarInFault)i,
                                  "fw history fault sets via shared memory, "
                                  "fid: %d, ipc_item: %d/%d, "
                                  "remote seq: %" PRI_SIZED
                                  "/%" PRI_SIZED,
                                   i, fw_fault_set_item,
                                   fw_fault_heal_item,
                                   fw_fault_set_seq,
                                   fw_fault_heal_seq);
      } else {
        lidar_->set_raw_fault((InnoLidarInFault)i, true,
                             "fw fault sets via shared memory, "
                             "fid: %d, ipc_item: %d/%d, "
                             "remote seq: %" PRI_SIZED
                             "/%" PRI_SIZED,
                              i, fw_fault_set_item,
                              fw_fault_heal_item,
                              fw_fault_set_seq,
                              fw_fault_heal_seq);
      }
    } else {
      lidar_->heal_raw_fault((InnoLidarInFault)i, true,
                            "fw fault heals via shared memory, "
                            "fid: %d, ipc_item: %d/%d, "
                            "remote seq: %" PRI_SIZED
                            "/%" PRI_SIZED,
                             i, fw_fault_set_item,
                             fw_fault_heal_item,
                             fw_fault_set_seq,
                             fw_fault_heal_seq);
    }
  }
}

FaultReactor::FaultReactor(FaultManager* fault_manager) {
  fault_manager_ = fault_manager;
  inno_log_verify_simple(fault_manager_, "fault_manager_ is NULL");
}

FaultReactor::~FaultReactor() {
}

void FaultReactor::react_to_overheat_(enum InnoFaultOperation operation) {
  if (fault_manager_) {
    enum InnoLidarMode pre_mode;
    enum InnoLidarMode current_mode;
    enum InnoLidarStatus status;
    int ret = fault_manager_->\
              get_mode_status_fast(&current_mode, &pre_mode, &status);
    if (ret != 0) {
      return;
    }
    if (current_mode == INNO_LIDAR_MODE_WORK_NORMAL ||
        current_mode == INNO_LIDAR_MODE_WORK_QUIET ||
        current_mode == INNO_LIDAR_MODE_WORK_CALIBRATION ||
        current_mode == INNO_LIDAR_MODE_PROTECTION) {
      enum InnoLidarMode target_mode;
      if (operation == INNO_FAULT_OPERATION_SET) {
        target_mode = INNO_LIDAR_MODE_PROTECTION;
      } else if (operation == INNO_FAULT_OPERATION_HEAL) {
        target_mode = INNO_LIDAR_MODE_WORK_NORMAL;
      } else {
        inno_log_error("invalid operation(%d), please check", operation);
        return;
      }
      ret = fault_manager_->set_mode(target_mode, &pre_mode, &status);
      inno_log_verify_simple(ret == 0,
                            "fail to switch mode from %d to %d, got %d",
                             current_mode, target_mode, ret);
      fault_manager_->\
      do_message_callback(INNO_MESSAGE_LEVEL_WARNING,
                          INNO_MESSAGE_CODE_OVERHEAT_PROTECTION,
                         "INNO_LIDAR_IN_FAULT_OVERHEAT3 happened, "
                         "will go to protection mode");
    }
  }
}

ConfidenceSubscriber::ConfidenceSubscriber()
                    : Subscriber() {
  for (int i = 0; i < INNO_PROCESS_STAGE_MAX; i++) {
    for (int j = 0; j < INNO_CONFIDENCE_LEVEL_MAX; j++) {
      conf_seq_num_[i][j] = 0;
    }
  }
}

ConfidenceSubscriber::~ConfidenceSubscriber() {
}

void ConfidenceSubscriber::update(void* content) {
  struct InnoConfidenceContent* conf =
         reinterpret_cast<struct InnoConfidenceContent*>(content);
  std::unique_lock<std::mutex> lk(mutex_);
  conf_seq_num_[conf->stage][conf->conf_level] = conf->seq_number;
}

ConfidencePublisher::ConfidencePublisher()
                   : Publisher() {
  for (uint32_t i = 0; i < INNO_CONFIDENCE_LEVEL_MAX; i++) {
    fault_counters_[i] = 0;
    fault_counters_old_[i] = 0;
    fault_set_seq_num_[i] = 0;
    fault_heal_seq_num_[i] = 0;
  }
}

ConfidencePublisher::~ConfidencePublisher() {
}

void ConfidencePublisher::notify_subscriber_(void* content) {
  if (content) {
    for (auto subscriber : subscribers_) {
      subscriber->update(content);
    }
  }
}

void ConfidencePublisher::\
     update_conf_and_publish(int fid,
                             enum InnoFaultOperation operation) {
  enum ConfidenceLevel conf_level = Inno_faults_def[fid].conf_level;
  if (conf_level == INNO_CONFIDENCE_LEVEL_FULL) {
    return;
  }
  std::unique_lock<std::mutex> lk(mutex_);
  if (operation == INNO_FAULT_OPERATION_SET) {
    fault_counters_[conf_level]++;
  } else if (operation == INNO_FAULT_OPERATION_HEAL) {
    fault_counters_[conf_level]--;
  } else {
    inno_log_verify_simple(operation == INNO_FAULT_OPERATION_SET ||
                           operation == INNO_FAULT_OPERATION_HEAL,
                           "invalid fault operation: %d", operation);
  }
  inno_log_verify_simple(fault_counters_[conf_level] >= 0,
                        "fault_counters_[%d] < 0: %d",
                         conf_level, fault_counters_[conf_level]);
  if (fault_counters_old_[conf_level] == 0 &&
      fault_counters_[conf_level] > 0) {
    fault_set_seq_num_[conf_level]++;
  } else if (fault_counters_old_[conf_level] > 0 &&
      fault_counters_[conf_level] == 0) {
    fault_heal_seq_num_[conf_level]++;
  } else {
    // do nothing
  }
  fault_counters_old_[conf_level] = fault_counters_[conf_level];

  for (uint32_t i = 0; i < INNO_CONFIDENCE_LEVEL_MAX; i++) {
    struct InnoConfidenceContent content;
    content.conf_level = ConfidenceLevel(i);
    if (fault_counters_[i] > 0) {
      // publish fault set info to deliver stage
      content.stage = INNO_PROCESS_STAGE_DELIVER;
      content.seq_number = fault_set_seq_num_[i];
    } else {
      // publish fault heal info to read stage
      content.stage = INNO_PROCESS_STAGE_READ;
      content.seq_number = fault_heal_seq_num_[i];
    }
    notify_subscriber_(&content);
  }
}

LidarModeSubscriber::LidarModeSubscriber(FaultManager* fault_manager) {
  fault_manager_ = fault_manager;
  inno_log_verify(fault_manager_, "fault_manager");
  init_mode_inhibit_table_();
}

LidarModeSubscriber::~LidarModeSubscriber() {
}

void LidarModeSubscriber::update(void* content) {
  struct ModeChangeContent* info =
         reinterpret_cast<struct ModeChangeContent*>(content);
  fault_manager_->\
  set_state_and_update_inhibit_list(INNO_FAULT_DETECTION_TRANSITION,
                                    info->timestamp,
                                    mode_inhibit_faults_[info->target_mode]);
}

void LidarModeSubscriber::init_mode_inhibit_table_() {
  memset(mode_inhibit_faults_, 0, sizeof(mode_inhibit_faults_));
  for (uint32_t i = 0; i < INNO_LIDAR_IN_FAULT_MAX; i++) {
    if (Inno_faults_def[i].mode_inhibit_mask == 0) {
      continue;
    }
    for (uint32_t j = 0; j < INNO_LIDAR_MODE_WORK_MAX; j++) {
      if (((Inno_faults_def[i].mode_inhibit_mask >> j) & 0x1) != 0) {
        mode_inhibit_faults_[j] |= (1UL << i);
      }
    }
  }
}

Fault::Fault(const char* fault_name,
             int fid,
             uint16_t set_fault_debounce,
             uint16_t heal_fault_debounce,
             uint16_t heal_history_cycle_threshold,
             bool fault_enable) {
  fid_ = fid;
  fault_name_ = std::string(fault_name);
  debounce_start_ts_ = -1.0;
  set_fault_debounce_ = set_fault_debounce;
  heal_fault_debounce_ = heal_fault_debounce;
  heal_history_cycle_threshold_ = heal_history_cycle_threshold;
  no_fault_cycle_so_far_ = 0;
  fault_status_ = 0;
  fault_enable_ = fault_enable;
  state_ = INNO_DIAG_INIT;
}

Fault::~Fault() {
}

void Fault::clear_bit(int position) {
  fault_status_ &= ~(1U << position);
}

void Fault::set_bit(int position) {
  fault_status_ |= (1U << position);
}

inline bool Fault::get_fault_enable_status() const {
  return fault_enable_;
}

inline uint16_t Fault::get_fault_status() const {
  return fault_status_;
}

double Fault::get_debounce_time(enum InnoFaultOperation op) {
  inno_log_verify_simple(op >= INNO_FAULT_OPERATION_SET &&
                         op < INNO_FAULT_OPERATION_MAX,
                        "invalid operation number %d", op);
  double ret = (op == INNO_FAULT_OPERATION_SET) ?
               set_fault_debounce_ : heal_fault_debounce_;
  return ret;
}

void Fault::add_no_fault_count() {
  no_fault_cycle_so_far_++;
  if (no_fault_cycle_so_far_ >= heal_history_cycle_threshold_) {
    clear_history_fault_bit();
    update_no_fault_cycle_count(0);
  }
}

inline uint16_t Fault::get_no_fault_cycle_count() const {
  return no_fault_cycle_so_far_;
}

inline void Fault::set_history_fault_bit() {
  fault_status_ |= (1U << 3);
  // reset the counter
  no_fault_cycle_so_far_ = 0;
}

void Fault::set_current_fault_bit() {
  // set fault bit
  fault_status_ |= (1U << 0);

  // set fault ever happened bit
  fault_status_ |= (1U << 1);

  // set indicator
  fault_status_ |= (1U << 7);

  no_fault_cycle_so_far_ = 0;
}

inline void Fault::clear_history_fault_bit() {
  fault_status_ &= ~(1U << 3);
}

void Fault::clear_current_fault_bit() {
  fault_status_ &= ~(1U << 0);
  fault_status_ &= ~(1U << 7);
}

inline void Fault::set_debounce_start_ts(double ts) {
  debounce_start_ts_ = ts;
}

inline double Fault::get_debounce_start_ts() {
  return debounce_start_ts_;
}

inline void Fault::set_debouncing_bit() {
  fault_status_ |= (1U << 6);
}

inline void Fault::clear_debouncing_bit() {
  fault_status_ &= ~(1U << 6);
}

inline bool Fault::current_cycle_fault_happened() const {
  return (fault_status_ & (1U << 1)) != 0;
}

inline void Fault::clear_current_cycle_fault_happened() {
  fault_status_ &= ~(1U << 1);
}

inline void Fault::set_status(uint16_t status) {
  fault_status_ = status;
}

inline void Fault::update_no_fault_cycle_count(uint16_t counter) {
  no_fault_cycle_so_far_ = counter;
}

inline void Fault::set_diag_state_(enum InnoFaultState state) {
  state_ = state;
}

inline enum InnoFaultState Fault::get_diag_state_() const {
  return state_;
}

uint32_t FaultManager::bootup_number = 1;

FaultManager::FaultManager(InnoLidar* lidar) : fw_fault_synchronizer_(lidar) {
  lidar_ = lidar;
  inno_log_verify_simple(lidar_, "lidar invalid");
  init_fault_configs_();
  init_fault_list_();
  faults_update_counter_ = 0;
  fault_manager_construct_ts_ = lidar_->get_monotonic_raw_time_ms();
  mode_change_timestamp_ = fault_manager_construct_ts_;
  temporary_inhibit_faults_ = 0;
  ConfidenceSubscriber* subscriber = lidar_->get_conf_subscriber();
  conf_publisher_.add_subscriber(subscriber);
  fault_reactor_ = new FaultReactor(this);
  inno_log_verify_simple(fault_reactor_, "fault_reactor_");
  memset(on_fault_, 0, sizeof(on_fault_));
  on_fault_[INNO_LIDAR_IN_FAULT_OVERHEAT3] = FaultReactor::react_to_overheat_s;
  for (const auto& fault : fault_list_) {
    fault->fault_set_ts_.tm_year = 0;
    fault->fault_set_ts_.tm_mon = 1;
    fault->fault_set_ts_.tm_mday = 1;
    fault->fault_set_ts_.tm_hour = 0;
    fault->fault_set_ts_.tm_min = 0;
    fault->fault_set_ts_.tm_sec = 0;
  }
  for (const auto& sub_fault : sub_fault_list_) {
    sub_fault->fault_set_ts_.tm_year = 0;
    sub_fault->fault_set_ts_.tm_mon = 1;
    sub_fault->fault_set_ts_.tm_mday = 1;
    sub_fault->fault_set_ts_.tm_hour = 0;
    sub_fault->fault_set_ts_.tm_min = 0;
    sub_fault->fault_set_ts_.tm_sec = 0;
  }
  mode_subscriber_ = new LidarModeSubscriber(this);
  inno_log_verify_simple(mode_subscriber_, "mode_subscriber_");
  init_fault_inhibit_graph_();
  fault_manager_state_ = INNO_FAULT_DETECTION_INIT;
  inno_log_info("Fault manager create completed");
}

FaultManager::~FaultManager() {
  std::unique_lock<std::mutex> lk(mutex_);
  inno_log_verify_simple(mode_subscriber_, "mode_subscriber_");
  delete mode_subscriber_;
  mode_subscriber_ = NULL;
  inno_log_verify_simple(fault_reactor_, "fault_reactor_");
  delete fault_reactor_;
  fault_reactor_ = NULL;

  void* p1 = fault_list_[0];
  void* p2 = config_list_base_[0];
  for (int i = 0; i < INNO_LIDAR_IN_FAULT_MAX; i++) {
    lidar_->remove_config(config_list_base_[i]);
    fault_list_[i] = NULL;
    config_list_base_[i] = NULL;
  }
  inno_log_verify_simple(p1, "fault member list invalid destructing");
  inno_log_verify_simple(p2, "fault config list invalid destructing");
  free(p1);
  free(p2);
}

int FaultManager::fault_message_with_stack_(int fid,
                                            enum InnoFaultOperation operation,
                                            const char *format,
                                            va_list valist) {
  static const int kMaxFaultMessageLen = 8192;
  char buffer[8192];
  int ret = vsnprintf(buffer, kMaxFaultMessageLen, format, valist);
  if (ret >= kMaxFaultMessageLen) {
    return ret;
  }
  buffer[kMaxFaultMessageLen - 1] = '\0';
  bool set_fault = operation == INNO_FAULT_OPERATION_SET;
  inno_log_with_level(set_fault ?
                      INNO_LOG_LEVEL_ERROR :
                      INNO_LOG_LEVEL_INFO,
                      "fault %s(%d) has been %s internally. "
                      "%s",
                      Inno_faults_def[fid].name,
                      fid,
                      set_fault ? "set" : "healed",
                      buffer);
  return 0;
}

void FaultManager::message_log_out_(int fid,
                                    enum InnoFaultOperation operation,
                                    const char *format, va_list valist) {
  int ret = fault_message_with_stack_(fid, operation, format, valist);
  if (ret > 0) {
    char* buffer = reinterpret_cast<char*>(malloc(ret));
    if (buffer) {
      vsnprintf(buffer, ret, format, valist);
      buffer[ret - 1] = '\0';
    } else {
      inno_log_error("malloc buffer for fault message failed");
    }
    bool set_fault = operation == INNO_FAULT_OPERATION_SET;
    inno_log_with_level_no_discard(set_fault ?
                                   INNO_LOG_LEVEL_ERROR :
                                   INNO_LOG_LEVEL_INFO,
                                   "fault %s(%d) has been %s internally. "
                                   "%s",
                                   Inno_faults_def[fid].name,
                                   fid,
                                   set_fault ? "set" : "healed",
                                   buffer);
    if (buffer) {
      free(buffer);
    }
  }
}

void FaultManager::set_history_fault(int fid,
                                     const char *format,
                                     va_list valist) {
  {
    std::unique_lock<std::mutex> lk(mutex_);
    if (fault_list_[fid]->has_history_fault()) {
      return;
    }
    fault_list_[fid]->set_history_fault_bit();
  }
  message_log_out_(fid, INNO_FAULT_OPERATION_SET, format, valist);
}

void FaultManager::set_raw_fault(int fid,
                                 bool is_sub_fault,
                                 enum InnoFaultOperation operation,
                                 bool condition,
                                 const char *format,
                                 va_list valist) {
  double now_ts = lidar_->get_monotonic_raw_time_ms();
  std::unique_lock<std::mutex> lk(mutex_);
  enum FaultManagerState state = fault_manager_state_;
  if (event_driven_faults_.count(fid) != 0 ||
     (fid == INNO_LIDAR_IN_FAULT_OVERHEAT3 &&
      operation == INNO_FAULT_OPERATION_HEAL) ||
     !lidar_->is_live_lidar()) {
    state = INNO_FAULT_DETECTION_NORMAL;
  }
  switch (state) {
    case INNO_FAULT_DETECTION_INIT: {
      if (now_ts - fault_manager_construct_ts_ >
                   kMinIntervalAfterConstruction) {
        fault_manager_state_ = INNO_FAULT_DETECTION_TRANSITION;
        if (temporary_inhibit_faults_ == 0) {
          enum InnoLidarMode mode;
          enum InnoLidarMode pre_mode;
          enum InnoLidarStatus status;
          uint64_t transition_ms;
          int ret = lidar_->get_mode_status(&mode, &pre_mode,
                                            &status, &transition_ms);
          if (ret == 0) {
            temporary_inhibit_faults_ = mode_subscriber_->\
                                        get_inhibit_faults(mode);
          } else {
            inno_log_warning("get_mode_status error, got: %d, "
                             "no faults will be inhibited until "
                             "mode switches", ret);
          }
        }
      } else {
        return;
      }
      // break;
    }
    case INNO_FAULT_DETECTION_TRANSITION: {
      if (now_ts - mode_change_timestamp_ < kMinIntervalAfterModeChange) {
        return;
      }
      enum InnoLidarMode mode;
      enum InnoLidarMode pre_mode;
      enum InnoLidarStatus status;
      uint64_t transition_ms;
      int ret = lidar_->get_mode_status(&mode, &pre_mode,
                                        &status, &transition_ms);
      if (ret == 0 && status == INNO_LIDAR_STATUS_NORMAL) {
        fault_manager_state_ = INNO_FAULT_DETECTION_NORMAL;
      } else {
        if (now_ts - mode_change_timestamp_ > kMaxIntervalAfterModeChange) {
          // transition timeout, set state to normal to enable fault detect
          inno_log_warning("mode switch not finished in %ds, "
                           "force enable fault detection",
                           kMaxIntervalAfterModeChange);
          fault_manager_state_ = INNO_FAULT_DETECTION_NORMAL;
        } else {
          return;
        }
      }
      // break;
    }
    case INNO_FAULT_DETECTION_NORMAL: {
      Fault* fault;
      bool inhibited = false;
      bool enabled = true;
      if (is_sub_fault) {
        fault = sub_fault_list_[fid - INNO_SUB_FAULT_BASE - 1];
        Fault* master_fault = get_fault_from_sub_fault_(fid);
        inhibited = inhibited_(master_fault->get_fid()) ||
                    fault_subscriber_[master_fault->get_fid()].is_inhibited();
        enabled = master_fault->get_fault_enable_status();
      } else {
        fault = fault_list_[fid];
        inhibited = inhibited_(fid) ||
                    fault_subscriber_[fid].is_inhibited();
        enabled = fault->get_fault_enable_status();
      }
      if (!enabled || inhibited) {
        return;
      }
      bool set_cond = operation == INNO_FAULT_OPERATION_SET && condition;
      bool heal_cond = operation == INNO_FAULT_OPERATION_HEAL && condition;
      inno_log_verify_simple(!set_cond || !heal_cond,
                             "both set and heal condition "
                             "are true for: %d", fid);
      if (!set_cond && !heal_cond) {
        return;
      }
      int ret = update_fault_status_(fault, set_cond, heal_cond, now_ts);
      if (is_sub_fault && ret == 0) {
        ret = -1;
        fid = get_fault_from_sub_fault_(fid)->get_fid();
        fault = fault_list_[fid];
        if (set_cond) {  // set fault if any sub-fault reports
          ret = update_fault_status_(fault, set_cond,
                                     heal_cond, now_ts);
        } else if (heal_cond) {  // heal fault when all sub-faults are healed
          int sub_fault_number = fault_to_sub_fault_[fault].size();
          for (auto& sub_fault : fault_to_sub_fault_[fault]) {
            sub_fault_number -= !sub_fault->get_current_fault_flag_();
          }
          if (sub_fault_number == 0) {
            ret = update_fault_status_(fault, set_cond,
                                       heal_cond, now_ts);
          }
        } else {
          // do nothing
        }
      }
      // fault changes from set->heal or heal->set
      if (ret == 0) {
        faults_update_counter_++;
        time_t local_time;
        local_time = time(&local_time);
#ifndef __MINGW64__
        localtime_r(&local_time, &fault->fault_set_ts_);
#else
        localtime_s(&fault->fault_set_ts_, &local_time);
#endif
        fault->fault_set_ts_.tm_year += 1900;
        if (fault->get_fid() == INNO_LIDAR_IN_FAULT_OVERHEAT3) {
          laser_overheat_ = operation == INNO_FAULT_OPERATION_SET;
        }
        fault_publisher_[fid].notify_subscriber_(&operation);
        lk.unlock();
        conf_publisher_.update_conf_and_publish(fault->get_fid(), operation);
        message_log_out_(fid, operation, format, valist);
        if (operation == INNO_FAULT_OPERATION_SET &&
            is_config_save_raw_enable_(fid)) {
          TRY_TO_SEND_RAW(fid);
        }
        if (on_fault_[fid]) {
          on_fault_[fid](fault_reactor_, operation);
        }
      }
      break;
    }
    default:
      inno_log_verify_simple(state == INNO_FAULT_DETECTION_INIT ||
                             state == INNO_FAULT_DETECTION_TRANSITION ||
                             state == INNO_FAULT_DETECTION_NORMAL,
                            "fault manager state invalid: %d/%d", state,
                             fault_manager_state_);
      break;
  }
}

int FaultManager::get_fault_enable_status(int fid, bool is_sub_fault) {
  if (is_sub_fault) {
    fid = get_fault_from_sub_fault_(fid)->get_fid();
  }
  return fault_list_[fid]->get_fault_enable_status();
}

bool FaultManager::\
     current_cycle_fault_happened_(enum InnoLidarInFault fid) const {
  return fault_list_[fid]->current_cycle_fault_happened();
}

void FaultManager::clear_fault_after_long_(enum InnoLidarInFault fid) {
  fault_list_[fid]->add_no_fault_count();
  bootup_number = 0;
}

void FaultManager::init_fault_configs_() {
  FaultConfig* p = reinterpret_cast<FaultConfig*>\
                   (calloc(INNO_LIDAR_IN_FAULT_MAX * 2,
                   sizeof(FaultConfig)));
  inno_log_verify_simple(p, "FaultConfig malloc failed");
  for (int i = 0; i < INNO_LIDAR_IN_FAULT_MAX; i++, p++) {
    config_list_base_[i] = new (p) FaultConfig(i);
    inno_log_verify_simple(config_list_base_[i], "config_list_base[%d]", i);
    config_list_[i] = new (p + INNO_LIDAR_IN_FAULT_MAX) FaultConfig(i);
    inno_log_verify_simple(config_list_[i], "config_list[%d]", i);
    lidar_->add_config(config_list_base_[i]);
    config_list_[i]->copy_from_src(config_list_base_[i]);
  }
  config_list_[INNO_LIDAR_IN_FAULT_OVERHEAT1]->\
              set_key_value_("set_fault_debounce", 10 * 1000);   // 10s
  config_list_[INNO_LIDAR_IN_FAULT_OVERHEAT1]->\
              set_key_value_("heal_fault_debounce", 10 * 1000);  // 10s
  config_list_[INNO_LIDAR_IN_FAULT_OVERHEAT2]->\
              set_key_value_("set_fault_debounce", 10 * 1000);   // 10s
  config_list_[INNO_LIDAR_IN_FAULT_OVERHEAT2]->\
              set_key_value_("heal_fault_debounce", 10 * 1000);  // 10s
  config_list_[INNO_LIDAR_IN_FAULT_OVERHEAT3]->\
              set_key_value_("set_fault_debounce", 5 * 1000);    // 5s
  config_list_[INNO_LIDAR_IN_FAULT_OVERHEAT3]->\
              set_key_value_("heal_fault_debounce", 10 * 1000);  // 10s
  config_list_[INNO_LIDAR_IN_FAULT_OPTIC1]->\
              set_key_value_("heal_fault_debounce", 0);  // use sub-fault
  config_list_[INNO_LIDAR_IN_FAULT_OPTIC2]->\
              set_key_value_("heal_fault_debounce", 0);  // use sub-fault
  config_list_[INNO_LIDAR_IN_FAULT_WINDOW_BLOCKAGE1]->\
              set_key_value_("set_fault_debounce", 2000);  // 2000ms
  config_list_[INNO_LIDAR_IN_FAULT_DBTEMP]->\
              set_key_value_("set_fault_debounce", 100);  // 100ms
  config_list_[INNO_LIDAR_IN_FAULT_EXCESSIVE_NOISE]->\
              set_key_value_("heal_fault_debounce", 0);
  config_list_[INNO_LIDAR_IN_FAULT_RAWDATA_STREAM]->\
              set_key_value_("set_fault_debounce", 300);  // 300ms
  config_list_[INNO_LIDAR_IN_FAULT_POLYGON_TO]->\
              set_key_value_("set_fault_debounce", 300);  // 300ms
  config_list_[INNO_LIDAR_IN_FAULT_GALVO_TO]->\
              set_key_value_("set_fault_debounce", 300);  // 300ms
  config_list_[INNO_LIDAR_IN_FAULT_TRIGGER_TO]->\
              set_key_value_("set_fault_debounce", 300);  // 300ms
  config_list_[INNO_LIDAR_IN_FAULT_OPTIC1_F]->\
              set_key_value_("set_fault_debounce", 300);  // 300ms
  config_list_[INNO_LIDAR_IN_FAULT_OPTIC2_F]->\
              set_key_value_("set_fault_debounce", 300);  // 300ms
  config_list_[INNO_LIDAR_IN_FAULT_REFINTENSITY]->\
              set_key_value_("heal_fault_debounce", 0);   // use sub-fault
  config_list_[INNO_LIDAR_IN_FAULT_NETWORK2]->\
              set_key_value_("set_fault_debounce", 200);  // 200ms
  config_list_[INNO_LIDAR_IN_FAULT_GALVO_OFFSET]->\
              set_key_value_("set_fault_debounce", 100);  // 100ms
  config_list_[INNO_LIDAR_IN_FAULT_RESERVED17]->\
              set_key_value_("set_fault_debounce", 2000);  // 2000ms

  InnoLidarInFault faults_save_raw_enable[] = {
      INNO_LIDAR_IN_FAULT_WINDOW_BLOCKAGE1,
      INNO_LIDAR_IN_FAULT_POLYGON_CONTROL,
      INNO_LIDAR_IN_FAULT_GALVO_CONTROL,
      INNO_LIDAR_IN_FAULT_OPTIC1,
      INNO_LIDAR_IN_FAULT_OPTIC2,
      INNO_LIDAR_IN_FAULT_RAWDATA_STREAM,
      INNO_LIDAR_IN_FAULT_POLYGON_TO,
      INNO_LIDAR_IN_FAULT_GALVO_TO,
      INNO_LIDAR_IN_FAULT_TRIGGER_TO,
      INNO_LIDAR_IN_FAULT_ASSERT_FAILURE,
      INNO_LIDAR_IN_FAULT_CPULOAD_HIGH,
      INNO_LIDAR_IN_FAULT_LATENCY_LONG,
      INNO_LIDAR_IN_FAULT_EXCESSIVE_NOISE,
      INNO_LIDAR_IN_FAULT_DATA_DROP1,
      INNO_LIDAR_IN_FAULT_DATA_DROP2,
      INNO_LIDAR_IN_FAULT_DATA_DROP3,
      INNO_LIDAR_IN_FAULT_REFINTENSITY,
      INNO_LIDAR_IN_FAULT_GALVO_MIRROR,
      INNO_LIDAR_IN_FAULT_MAX_DISTANCE,
      INNO_LIDAR_IN_FAULT_GALVO_OFFSET,
      INNO_LIDAR_IN_FAULT_RESERVED17
  };
  std::string info = "enable raw save for fault: [";
  for (InnoLidarInFault f : faults_save_raw_enable) {
    config_list_[f]->set_key_value_("save_raw_enable", true);
    if (config_list_[f]->save_raw_enable) {
      info += std::to_string(f) + ", ";
    }
  }
  info += "]";
  inno_log_info("%s", info.c_str());
  InnoLidarInFault event_trigged_faults[] = {
    INNO_LIDAR_IN_FAULT_ASSERT_FAILURE,
    INNO_LIDAR_IN_FAULT_CONFIG1,
    INNO_LIDAR_IN_FAULT_CONFIG2,
    INNO_LIDAR_IN_FAULT_POWER_LOW,
    INNO_LIDAR_IN_FAULT_POWER_HIGH,
    INNO_LIDAR_IN_FAULT_LASER_INTERLOCK,
    INNO_LIDAR_IN_FAULT_COMM_LASER,
    INNO_LIDAR_IN_FAULT_LASER,
    INNO_LIDAR_IN_FAULT_COMM_DSP,
    INNO_LIDAR_IN_FAULT_CANFD_DSP,
    INNO_LIDAR_IN_FAULT_DSP,
    INNO_LIDAR_IN_FAULT_POLYGON_CONTROL,
    INNO_LIDAR_IN_FAULT_POLYGON_SENSOR,
    INNO_LIDAR_IN_FAULT_GALVO_CONTROL,
    INNO_LIDAR_IN_FAULT_GALVO_SENSOR,
    INNO_LIDAR_IN_FAULT_IIC_DSP,
    INNO_LIDAR_IN_FAULT_IIC_SOC,
    INNO_LIDAR_IN_FAULT_DSP_EXTWD,
    INNO_LIDAR_IN_FAULT_CHIPTEMP,
    INNO_LIDAR_IN_FAULT_HUMIDITY,
    INNO_LIDAR_IN_FAULT_COMM_ADC,
    INNO_LIDAR_IN_FAULT_FPGACLOCK,
    INNO_LIDAR_IN_FAULT_SOC,
    INNO_LIDAR_IN_FAULT_SOC_EXTWD,
    INNO_LIDAR_IN_FAULT_POWSUPL1,
    INNO_LIDAR_IN_FAULT_POWSUPL2,
    INNO_LIDAR_IN_FAULT_LPDDR4,
    INNO_LIDAR_IN_FAULT_FLASH,
    INNO_LIDAR_IN_FAULT_RAWDATA_TO,
    INNO_LIDAR_IN_FAULT_REPROGRAMMING
  };
  for (auto fault : event_trigged_faults) {
    config_list_[fault]->set_key_value_("set_fault_debounce", 0);
    config_list_[fault]->set_key_value_("heal_fault_debounce", 0);
    event_driven_faults_.insert(fault);
  }
}

void FaultManager::init_fault_list_() {
  const int sub_fault_number = INNO_SUB_FAULT_MAX - INNO_SUB_FAULT_BASE - 1;
  Fault* p = reinterpret_cast<Fault*>(malloc(sizeof(Fault) *
             (INNO_LIDAR_IN_FAULT_MAX + sub_fault_number)));
  inno_log_verify_simple(p, "malloc error: %p", p);
  std::unique_lock<std::mutex> lk(mutex_);
  for (int i = 0; i < INNO_LIDAR_IN_FAULT_MAX; i++, p++) {
    fault_list_[i] = new (p) Fault(Inno_faults_def[i].name, i,
                             config_list_[i]->set_fault_debounce,
                             config_list_[i]->heal_fault_debounce,
                             config_list_[i]->heal_history_cycle_threshold,
                             config_list_[i]->fault_enable);
    if (config_list_[i]->save_raw_enable) {
      enable_config_save_raw_((enum InnoLidarInFault) i);
    }
  }
  // init sub-faults
  struct SubFault sub_faults[sub_fault_number] = {
    {"INNO_SUB_FAULT_OPTIC1_0",
     INNO_SUB_FAULT_OPTIC1_0,
     5 * 1000, 500},
    {"INNO_SUB_FAULT_OPTIC1_1",
     INNO_SUB_FAULT_OPTIC1_1,
     5 * 1000, 500},
    {"INNO_SUB_FAULT_OPTIC1_2",
     INNO_SUB_FAULT_OPTIC1_2,
     5 * 1000, 500},
    {"INNO_SUB_FAULT_OPTIC1_3",
     INNO_SUB_FAULT_OPTIC1_3,
     5 * 1000, 500},
    {"INNO_SUB_FAULT_OPTIC1_4",
     INNO_SUB_FAULT_OPTIC1_4,
     300, 500},
    {"INNO_SUB_FAULT_OPTIC2_0",
     INNO_SUB_FAULT_OPTIC2_0,
     2 * 1000, 500},
    {"INNO_SUB_FAULT_OPTIC2_1",
     INNO_SUB_FAULT_OPTIC2_1,
     2 * 1000, 500},
    {"INNO_SUB_FAULT_OPTIC2_2",
     INNO_SUB_FAULT_OPTIC2_2,
     2 * 1000, 500},
    {"INNO_SUB_FAULT_OPTIC2_3",
     INNO_SUB_FAULT_OPTIC2_3,
     2 * 1000, 500},
    {"INNO_SUB_FAULT_EXCESSIVE_NOISE_0",
     INNO_SUB_FAULT_EXCESSIVE_NOISE_0,
     500, 500},
    {"INNO_SUB_FAULT_EXCESSIVE_NOISE_1",
     INNO_SUB_FAULT_EXCESSIVE_NOISE_1,
     500, 500},
    {"INNO_SUB_FAULT_REFINTENSITY0",
     INNO_SUB_FAULT_REFINTENSITY0,
     300, 500},
    {"INNO_SUB_FAULT_REFINTENSITY4",
     INNO_SUB_FAULT_REFINTENSITY4,
     300, 500},
    {"INNO_SUB_FAULT_REFINTENSITY1_0",
     INNO_SUB_FAULT_REFINTENSITY1_0,
     300, 500},
    {"INNO_SUB_FAULT_REFINTENSITY1_1",
     INNO_SUB_FAULT_REFINTENSITY1_1,
     300, 500},
    {"INNO_SUB_FAULT_REFINTENSITY1_2",
     INNO_SUB_FAULT_REFINTENSITY1_2,
     300, 500},
    {"INNO_SUB_FAULT_REFINTENSITY1_3",
     INNO_SUB_FAULT_REFINTENSITY1_3,
     300, 500}
  };
  for (int i = 0; i < sub_fault_number; i++) {
    sub_fault_list_[i] = new (p + i) Fault(sub_faults[i].fault_name,
                              static_cast<int>(sub_faults[i].fid),
                              sub_faults[i].set_fault_debounce,
                              sub_faults[i].heal_fault_debounce,
                              30, true);
  }
  // initialize sub_fault_to_fault_, fault_to_sub_fault_
  for (int i = INNO_SUB_FAULT_OPTIC1_0;
           i <= INNO_SUB_FAULT_OPTIC1_4; i++) {
    sub_fault_to_fault_[sub_fault_list_[i - INNO_SUB_FAULT_BASE - 1]] =
        fault_list_[INNO_LIDAR_IN_FAULT_OPTIC1];
    fault_to_sub_fault_[fault_list_[INNO_LIDAR_IN_FAULT_OPTIC1]].\
        push_back(sub_fault_list_[i - INNO_SUB_FAULT_BASE - 1]);
  }
  for (int i = INNO_SUB_FAULT_EXCESSIVE_NOISE_0;
           i <= INNO_SUB_FAULT_EXCESSIVE_NOISE_1; i++) {
    sub_fault_to_fault_[sub_fault_list_[i - INNO_SUB_FAULT_BASE - 1]] =
        fault_list_[INNO_LIDAR_IN_FAULT_EXCESSIVE_NOISE];
    fault_to_sub_fault_[fault_list_[INNO_LIDAR_IN_FAULT_EXCESSIVE_NOISE]].\
        push_back(sub_fault_list_[i - INNO_SUB_FAULT_BASE - 1]);
  }
  for (int i = INNO_SUB_FAULT_OPTIC2_0;
           i <= INNO_SUB_FAULT_OPTIC2_3; i++) {
    sub_fault_to_fault_[sub_fault_list_[i - INNO_SUB_FAULT_BASE - 1]] =
        fault_list_[INNO_LIDAR_IN_FAULT_OPTIC2];
    fault_to_sub_fault_[fault_list_[INNO_LIDAR_IN_FAULT_OPTIC2]].\
        push_back(sub_fault_list_[i - INNO_SUB_FAULT_BASE - 1]);
  }
  for (int i = INNO_SUB_FAULT_REFINTENSITY0;
           i <= INNO_SUB_FAULT_REFINTENSITY1_3; i++) {
    sub_fault_to_fault_[sub_fault_list_[i - INNO_SUB_FAULT_BASE - 1]] =
        fault_list_[INNO_LIDAR_IN_FAULT_REFINTENSITY];
    fault_to_sub_fault_[fault_list_[INNO_LIDAR_IN_FAULT_REFINTENSITY]].\
        push_back(sub_fault_list_[i - INNO_SUB_FAULT_BASE - 1]);
  }
}

int FaultManager::\
    update_fault_status_external_(Fault* fault,
                                  enum InnoFaultOperation operation) {
  inno_log_verify_simple(operation == INNO_FAULT_OPERATION_SET ||
                         operation == INNO_FAULT_OPERATION_HEAL,
                         "operation: %d invalid", operation);
  bool update_timestamp = false;
  bool status_changed = true;
  enum InnoFaultState new_state = INNO_DIAG_INIT;
  if (operation == INNO_FAULT_OPERATION_HEAL) {
    if (fault->has_history_fault() ||
        fault->current_cycle_fault_happened()) {
      fault->clear_current_fault_bit();
      fault->clear_history_fault_bit();
      fault->update_no_fault_cycle_count(0);
      fault->clear_current_cycle_fault_happened();
    } else {
      status_changed = false;
    }
  } else {
    fault->set_current_fault_bit();
    new_state = INNO_DIAG_FAULT_SET;
    if (fault->get_fid() != INNO_LIDAR_IN_FAULT_OVERHEAT3 ||
       !laser_overheat_) {
      update_timestamp = true;
    }
  }
  fault->clear_debouncing_bit();
  fault->set_diag_state_(new_state);
  if (fault->get_fid() == INNO_LIDAR_IN_FAULT_OVERHEAT3) {
    laser_overheat_ = operation == INNO_FAULT_OPERATION_SET;
  }
  faults_update_counter_ += status_changed;
  if (update_timestamp) {
    time_t local_time;
    local_time = time(&local_time);
#ifndef __MINGW64__
    localtime_r(&local_time, &fault->fault_set_ts_);
#else
    localtime_s(&fault->fault_set_ts_, &local_time);
#endif
    fault->fault_set_ts_.tm_year += 1900;
  }
  return 0;
}
int FaultManager::update_fault_status_(Fault* fault,
                                       bool set_condition,
                                       bool heal_condition,
                                       double now_ts) {
  enum InnoFaultState new_state;
  enum InnoFaultState current_state;
  enum InnoFaultOperation operation;
  bool has_fault = fault->get_current_fault_flag_();
  if (has_fault) {
    current_state = INNO_DIAG_FAULT_SET;
    if (heal_condition) {
      new_state = INNO_DIAG_INIT;
      operation = INNO_FAULT_OPERATION_HEAL;
    } else {
      new_state = current_state;
      operation = INNO_FAULT_OPERATION_NONE;
    }
  } else {
    current_state = INNO_DIAG_INIT;
    if (set_condition) {
      new_state = INNO_DIAG_FAULT_SET;
      operation = INNO_FAULT_OPERATION_SET;
    } else {
      new_state = current_state;
      operation = INNO_FAULT_OPERATION_NONE;
    }
  }
  if (new_state == current_state) {
    fault->clear_debouncing_bit();
    fault->set_diag_state_(new_state);
    return -1;
  }
  int ret = -1;
  if (fault->get_diag_state_() != INNO_DIAG_DEBOUNCING) {
    fault->set_debounce_start_ts(now_ts);
  }
  if (now_ts - fault->get_debounce_start_ts() <
               fault->get_debounce_time(operation)) {
    fault->set_debouncing_bit();
    fault->set_diag_state_(INNO_DIAG_DEBOUNCING);
  } else {
    fault->clear_debouncing_bit();
    fault->set_diag_state_(new_state);
    bool is_sub_fault = is_sub_fault_(fault->get_fid());
    if (new_state == INNO_DIAG_FAULT_SET) {
      fault->set_current_fault_bit();
      // set all sub-faults
      if (!is_sub_fault) {
        for (auto sub_fault : fault_to_sub_fault_[fault]) {
          sub_fault->clear_debouncing_bit();
          sub_fault->set_diag_state_(new_state);
          sub_fault->set_current_fault_bit();
        }
      }
    } else {
      fault->clear_current_fault_bit();
      fault->set_history_fault_bit();
      // heal all sub-faults
      if (!is_sub_fault) {
        for (auto sub_fault : fault_to_sub_fault_[fault]) {
          sub_fault->clear_debouncing_bit();
          sub_fault->set_diag_state_(new_state);
          sub_fault->clear_current_fault_bit();
          sub_fault->set_history_fault_bit();
        }
      }
    }
    ret = 0;  // status changed
  }
  return ret;
}

int FaultManager::get_inner_faults_all_info(char* buffer, int buffer_len) {
  json joutput;
  {
    std::unique_lock<std::mutex> lk(mutex_);
    for (int i = 0, j = 0; i < INNO_LIDAR_IN_FAULT_MAX; i++) {
      if (!fault_list_[i]->get_fault_enable_status()) {
        continue;
      }
      json jitem;
      // info: items could be added here if needed
      jitem["fid"] = fault_list_[i]->get_fid();
      jitem["name"] = fault_list_[i]->get_name();
      jitem["no_fault_cycle_so_far"] = fault_list_[i]->\
                                      get_no_fault_cycle_count();
      jitem["fault_status"] = fault_list_[i]->get_fault_status();
      jitem["enable"] = fault_list_[i]->get_fault_enable_status();
      char ts_buffer[64];
      int ret = get_fault_timestamp_(i, ts_buffer, sizeof(ts_buffer));
      if (ret == 0) {
        jitem["ts"] = ts_buffer;
        joutput[j++] = jitem;
      } else {
        inno_log_error("snprintf error, ret: %d, %s", ret, ts_buffer);
      }
    }
  }
  int ret = snprintf(buffer, buffer_len, "%s", joutput.dump().c_str());
  return ret < buffer_len ? 0 : 1;
}

int FaultManager::get_inner_faults_info(char *buffer,
                                        int buffer_len,
                                        bool from_network) {
  json joutput;
  {
    std::unique_lock<std::mutex> lk(mutex_);
    for (int i = 0, j = 0; i < INNO_LIDAR_IN_FAULT_MAX; i++) {
      if (!fault_list_[i]->get_fault_enable_status() ||
          (!fault_list_[i]->has_history_fault() &&
           !fault_list_[i]->current_cycle_fault_happened())) {
        continue;
      }
      json jitem;
      jitem["fid"] = fault_list_[i]->get_fid();
      jitem["name"] = fault_list_[i]->get_name();
      jitem["no_fault_cycle_so_far"] = fault_list_[i]->\
                                       get_no_fault_cycle_count();
      //
      uint16_t status = fault_list_[i]->get_fault_status();
      if (from_network) {
        // only send the LSB
        status &= 0xff;
      }
      jitem["fault_status"] = status;
      char ts_buffer[64];
      int ret = get_fault_timestamp_(i, ts_buffer, sizeof(ts_buffer));
      if (ret == 0) {
        jitem["ts"] = ts_buffer;
        joutput[j++] = jitem;
      } else {
        inno_log_error("snprintf error, ret: %d, %s", ret, ts_buffer);
      }
    }
  }
  int ret = snprintf(buffer, buffer_len, "%s", joutput.dump().c_str());
  return ret < buffer_len ? 0 : 1;
}

void FaultManager::set_faults_external(int fid,
                                       enum InnoFaultOperation operation) {
  int lo = fid;
  int hi = fid + 1;
  std::string info;
  // heals all faults
  if (fid == INNO_LIDAR_IN_FAULT_MAX) {
    lo = 0;
    hi = INNO_LIDAR_IN_FAULT_MAX;
    info += "all faults";
  } else {
    info += "fault ";
    info += Inno_faults_def[fid].name;
    info += "(";
    info += std::to_string(fid);
    info += ")";
  }
  bool set_fault = operation == INNO_FAULT_OPERATION_SET;
  inno_log_with_level(set_fault ?
                      INNO_LOG_LEVEL_ERROR :
                      INNO_LOG_LEVEL_INFO,
                      "%s have/has been %s internally",
                      info.c_str(),
                      set_fault ? "set" : "healed");
  for (int i = lo; i < hi; i++) {
    Fault* fault = fault_list_[i];
    bool need_callback = false;
    {
      std::unique_lock<std::mutex> lk(mutex_);
      if ((fault->get_current_fault_flag_() &&
          operation == INNO_FAULT_OPERATION_HEAL) ||
          (!fault->get_current_fault_flag_() &&
          operation == INNO_FAULT_OPERATION_SET)) {
        need_callback = true;
        fault_publisher_[i].notify_subscriber_(&operation);
      }
      for (auto& sub_fault : fault_to_sub_fault_[fault]) {
        update_fault_status_external_(sub_fault, operation);
      }
      update_fault_status_external_(fault, operation);
    }
    if (operation == INNO_FAULT_OPERATION_SET) {
      if (is_config_save_raw_enable_(i)) {
        TRY_TO_SEND_RAW(i);
      }
    }
    if (need_callback) {
      conf_publisher_.update_conf_and_publish(fault->get_fid(), operation);
      if (on_fault_[i]) {
        on_fault_[i](fault_reactor_, operation);
      }
    }
  }
}

int FaultManager::init_faults_info(std::string str) {
  json input_str = json::parse(str);
  int i = 0;
  {
    std::unique_lock<std::mutex> lk(mutex_);
    for (const auto &item : input_str) {
      enum InnoLidarInFault fid = item["fid"].get<enum InnoLidarInFault>();
      Fault* fault = fault_list_[fid];
      fault->set_status(item["fault_status"].get<uint16_t>());
      fault->\
      update_no_fault_cycle_count(item["no_fault_cycle_so_far"].\
                                                 get<uint16_t>());
      fault->clear_current_cycle_fault_happened();
      fault->clear_current_fault_bit();
      fault->clear_debouncing_bit();
      fault->set_history_fault_bit();
      if (fid == INNO_LIDAR_IN_FAULT_OVERHEAT3 && laser_overheat_) {
        update_fault_status_external_(fault, INNO_FAULT_OPERATION_SET);
        conf_publisher_.update_conf_and_publish(fault->get_fid(),
                                                INNO_FAULT_OPERATION_SET);
      }
      if (item.find("ts") != item.end()) {
        std::string time = item["ts"].get<std::string>();
        int ret = sscanf(time.c_str(), "%x-%x-%x-%x-%x-%x",
                        &fault_list_[fid]->fault_set_ts_.tm_year,
                        &fault_list_[fid]->fault_set_ts_.tm_mon,
                        &fault_list_[fid]->fault_set_ts_.tm_mday,
                        &fault_list_[fid]->fault_set_ts_.tm_hour,
                        &fault_list_[fid]->fault_set_ts_.tm_min,
                        &fault_list_[fid]->fault_set_ts_.tm_sec);
        if (ret != 6) {
          inno_log_error("fault ts initialize error: %d "
                         "vs. 6 items, string: %s", ret, time.c_str());
          break;
        } else {
          if (fault_list_[fid]->fault_set_ts_.tm_year >= 0 &&
              fault_list_[fid]->fault_set_ts_.tm_year <= 99 &&
              fault_list_[fid]->fault_set_ts_.tm_mon >= 1 &&
              fault_list_[fid]->fault_set_ts_.tm_mon <= 12 &&
              fault_list_[fid]->fault_set_ts_.tm_mday >= 1 &&
              fault_list_[fid]->fault_set_ts_.tm_mday <= 31 &&
              fault_list_[fid]->fault_set_ts_.tm_hour >= 0 &&
              fault_list_[fid]->fault_set_ts_.tm_hour <= 23 &&
              fault_list_[fid]->fault_set_ts_.tm_min >= 0 &&
              fault_list_[fid]->fault_set_ts_.tm_min <= 59 &&
              fault_list_[fid]->fault_set_ts_.tm_sec >= 0 &&
              fault_list_[fid]->fault_set_ts_.tm_sec <= 59) {
            // format ok
            fault_list_[fid]->fault_set_ts_.tm_year += 2000;
            fault_list_[fid]->fault_set_ts_.tm_mon -= 1;
          } else {
            inno_log_error("timestamp of %s init failed: "
                           "%d-%d-%d-%d-%d-%d and abort fault init",
                            fault_list_[fid]->get_name().c_str(),
                            fault_list_[fid]->fault_set_ts_.tm_year,
                            fault_list_[fid]->fault_set_ts_.tm_mon,
                            fault_list_[fid]->fault_set_ts_.tm_mday,
                            fault_list_[fid]->fault_set_ts_.tm_hour,
                            fault_list_[fid]->fault_set_ts_.tm_min,
                            fault_list_[fid]->fault_set_ts_.tm_sec);
          }
        }
      }
      i++;
    }
    faults_update_counter_++;
  }
  inno_log_info("update %d internal faults from file", i);
  return 0;
}

// merge with before
int FaultManager::get_sync_cycle_status(char* buffer, int buffer_len) {
  json joutput;
  {
    std::unique_lock<std::mutex> lk(mutex_);
    for (int i = 0, j = 0; i < INNO_LIDAR_IN_FAULT_MAX; i++) {
      if (!fault_list_[i]->has_history_fault() &&
          !fault_list_[i]->current_cycle_fault_happened()) {
        continue;
      }
      InnoLidarInFault fid = InnoLidarInFault(i);
      if (!current_cycle_fault_happened_(fid) &&
                          bootup_number == 1) {
        clear_fault_after_long_(fid);
      }
      json jitem;
      jitem["fid"] = fault_list_[i]->get_fid();
      jitem["name"] = fault_list_[i]->get_name();
      jitem["no_fault_cycle_so_far"] = fault_list_[i]->\
                                       get_no_fault_cycle_count();
      jitem["fault_status"] = fault_list_[i]->get_fault_status();
      char ts_buffer[64];
      int ret = get_fault_timestamp_(i, ts_buffer, sizeof(ts_buffer));
      if (ret == 0) {
        jitem["ts"] = ts_buffer;
        joutput[j++] = jitem;
      } else {
        inno_log_error("snprintf error, ret: %d, %s", ret, ts_buffer);
      }
    }
  }
  int ret = snprintf(buffer, buffer_len, "%s", joutput.dump().c_str());
  return ret < buffer_len ? 0 : 1;
}

void FaultManager::update_data_packet(InnoStatusPacket* pkt) {
  std::unique_lock<std::mutex> lk(mutex_);
  for (uint32_t i = 0; i < INNO_LIDAR_IN_FAULT_MAX; i++) {
    enum InnoFaultState state = fault_list_[i]->get_diag_state_();
    if (state == INNO_DIAG_FAULT_SET) {
      pkt->in_faults.faults |= (1UL << i);
    } else if (state == INNO_DIAG_INIT) {
      pkt->in_faults.faults &= ~(1UL << i);
    } else {
      // if state is none of SET/INIT, do nothing
    }
  }
  pkt->fault_version = faults_update_counter_;
}

int FaultManager::set_mode(enum InnoLidarMode mode,
                           enum InnoLidarMode *mode_before_change,
                           enum InnoLidarStatus *status_before_change) {
  return lidar_->set_mode(mode, mode_before_change, status_before_change);
}

void FaultManager::do_message_callback(enum InnoMessageLevel error_level,
                                       enum InnoMessageCode code,
                                       const char *error_message) {
  lidar_->do_message_callback(error_level, code, error_message);
}

int FaultManager::get_mode_status_fast(enum InnoLidarMode *mode,
                                       enum InnoLidarMode *pre_mode,
                                       enum InnoLidarStatus *status) {
  return lidar_->get_mode_status_fast(mode, pre_mode, status);
}

int FaultManager::get_fault_timestamp_(int index, char* buffer, int len) {
  int year = fault_list_[index]->fault_set_ts_.tm_year;
  while (year > 2000) {
    year -= 2000;
  }
  if (year < 0 || year == 1970) {
    inno_log_info("invalid year: %d", year);
    year = 0;
  } else if (year > 99) {
    inno_log_info("invalid year: %d", year);
    year = 99;
  }
  int ret = snprintf(buffer, len,
                    "%x-%x-%x-%x-%x-%x",
                     year,
                     fault_list_[index]->fault_set_ts_.tm_mon + 1,
                     fault_list_[index]->fault_set_ts_.tm_mday,
                     fault_list_[index]->fault_set_ts_.tm_hour,
                     fault_list_[index]->fault_set_ts_.tm_min,
                     fault_list_[index]->fault_set_ts_.tm_sec);
  if (ret >= len) {
    buffer[len - 1] = '\0';
    return -1;
  }
  return 0;
}

void FaultManager::\
     set_state_and_update_inhibit_list(enum FaultManagerState state,
                                       uint64_t timestamp,
                                       uint64_t inhibit_fault_list) {
  std::unique_lock<std::mutex> lk(mutex_);
  mode_change_timestamp_ = timestamp;
  temporary_inhibit_faults_ = inhibit_fault_list;
  if (fault_manager_state_ == INNO_FAULT_DETECTION_INIT) {
    return;
  }
  fault_manager_state_ = state;
}

void FaultManager::init_fault_inhibit_graph_() {
  // creat the inhibit graph via adjacency table
  for (int i = 0; i < INNO_LIDAR_IN_FAULT_MAX; i++) {
    fault_subscriber_[i].init(i);
    if (Inno_faults_def[i].fault_inhibit_mask == 0) {
      continue;
    }
    for (int j = 0; j < INNO_LIDAR_IN_FAULT_MAX; j++) {
      if (((Inno_faults_def[i].fault_inhibit_mask >> j) & 0x1) != 0) {
        fault_publisher_[i].add_subscriber(&fault_subscriber_[j]);
      }
    }
  }
}

}  // namespace innovusion
