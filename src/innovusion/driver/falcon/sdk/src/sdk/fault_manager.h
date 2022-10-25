/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_FAULT_MANAGER_H_
#define SDK_FAULT_MANAGER_H_

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <mutex>  //NOLINT

#include <limits>

#include "utils/config.h"
#include "utils/inno_lidar_log.h"
#include "sdk_common/inno_lidar_packet.h"
#include "sdk_common/inno_faults_common.h"
#include "sdk/lidar.h"

namespace innovusion {
class InnoLidar;
class FaultManager;

class FaultConfig : public Config {
 public:
  explicit FaultConfig(int fid)
         : Config(), name("Lidar_") {
    set_fault_debounce = 0;  // ms, 0 means set fault immediately
    heal_fault_debounce = 500;
    heal_history_cycle_threshold = 30;
    fault_enable = true;
    save_raw_enable = false;
    name += Inno_faults_def[fid].name;
  }

  const char* get_type() const override {
    return name.c_str();
  }

  int set_key_value_(const std::string &key, double value) override {
    SET_CFG(set_fault_debounce);
    SET_CFG(heal_fault_debounce);
    SET_CFG(heal_history_cycle_threshold);
    SET_CFG(fault_enable);
    SET_CFG(save_raw_enable);
    return 0;
  }

  int set_key_value_(const std::string &key,
                     const std::string value) override {
    // no string attribute
    return -1;
  }

  BEGIN_CFG_MEMBER()
  std::string name;
  uint16_t set_fault_debounce;
  uint16_t heal_fault_debounce;
  uint16_t heal_history_cycle_threshold;
  bool fault_enable;
  bool save_raw_enable;
  END_CFG_MEMBER()
};

class FwFaultSynchronizer {
 public:
  explicit FwFaultSynchronizer(InnoLidar* lidar);
  ~FwFaultSynchronizer();
  void sync_fw_faults();

 private:
  InnoLidar* lidar_;
  std::unordered_set<int> fw_fault_;
  std::unordered_set<int> history_fault_;  // only report history status
};

class FaultReactor {
 public:
  explicit FaultReactor(FaultManager* fault_manager);
  ~FaultReactor();

  static void* react_to_overheat_s(void* context,
                                   enum InnoFaultOperation operation) {
    inno_log_verify_simple(context, "context is NULL");
    FaultReactor* fault_reactor =
                  reinterpret_cast<FaultReactor*>(context);
    fault_reactor->react_to_overheat_(operation);
    return NULL;
  }

 private:
  void react_to_overheat_(enum InnoFaultOperation operation);

 private:
  FaultManager* fault_manager_;
};

struct InnoConfidenceContent {
  enum ProcessStage stage;
  enum ConfidenceLevel conf_level;
  uint32_t seq_number;
};

struct ModeChangeContent {
  uint64_t timestamp;
  enum InnoLidarMode target_mode;
};

enum FaultManagerState {
  INNO_FAULT_DETECTION_INIT = 0,
  INNO_FAULT_DETECTION_TRANSITION = 1,
  INNO_FAULT_DETECTION_NORMAL = 2,
  INNO_FAULT_DETECTION_MAX
};

class ConfidenceSubscriber final : public Subscriber {
 public:
  ConfidenceSubscriber();
  ~ConfidenceSubscriber();

  void update(void* content) override;
  inline uint32_t get_conf_seq_info(enum ProcessStage stage,
                                    enum ConfidenceLevel conf_level) {
    std::unique_lock<std::mutex> lk(mutex_);
    return conf_seq_num_[stage][conf_level];
  }

 private:
  // conf_seq_num_[INNO_PROCESS_STAGE_READ] store info of faults heal
  // conf_seq_num_[INNO_PROCESS_STAGE_DELIVER] store info of faults set
  uint32_t conf_seq_num_[INNO_PROCESS_STAGE_MAX][INNO_CONFIDENCE_LEVEL_MAX];
  std::mutex mutex_;
};

class LidarModeSubscriber final : public Subscriber {
 public:
  LidarModeSubscriber(FaultManager* fault_manager);
  ~LidarModeSubscriber();
  void update(void* content) override;
  inline uint64_t get_inhibit_faults(enum InnoLidarMode mode) {
    return mode_inhibit_faults_[mode];
  }

 private:
  void init_mode_inhibit_table_();

 private:
  // mode_inhibit_faults_ holds the inhibit info for each lidar working mode
  uint64_t mode_inhibit_faults_[INNO_LIDAR_MODE_WORK_MAX];
  FaultManager* fault_manager_;
  std::mutex mutex_;
};

class LidarModePublisher final : public Publisher {
 public:
  LidarModePublisher() {}
  ~LidarModePublisher() {}
  inline void publish_mode_change(struct ModeChangeContent* info) {
    notify_subscriber_(info);
  }

 private:
  void notify_subscriber_(void* content) override {
    if (content) {
      std::unique_lock<std::mutex> lk(mutex_);
      for (auto subscriber : subscribers_) {
        lk.unlock();
        subscriber->update(content);
        lk.lock();
      }
    }
  }
};

class ConfidencePublisher final : public Publisher {
 public:
  explicit ConfidencePublisher();
  ~ConfidencePublisher();
  void update_conf_and_publish(int fid,
                               enum InnoFaultOperation operation);

 private:
  void notify_subscriber_(void* content) override;

 private:
  int32_t fault_counters_[INNO_CONFIDENCE_LEVEL_MAX];
  int32_t fault_counters_old_[INNO_CONFIDENCE_LEVEL_MAX];
  uint32_t fault_set_seq_num_[INNO_CONFIDENCE_LEVEL_MAX];
  uint32_t fault_heal_seq_num_[INNO_CONFIDENCE_LEVEL_MAX];
};

class FaultPublisher final : public Publisher {
 public:
  FaultPublisher() {}
  ~FaultPublisher() {}
  FaultPublisher(const FaultPublisher& rhs) = delete;
  FaultPublisher& operator= (const FaultPublisher& rhs) = delete;
  void notify_subscriber_(void* content) override {
    for (const auto& subscriber : subscribers_) {
      subscriber->update(content);
    }
  }
};

class FaultSubscriber final : public Subscriber {
 public:
  FaultSubscriber() : fid_(-1), indegree_(0) {}
  ~FaultSubscriber() {}
  FaultSubscriber(const FaultSubscriber& rhs) = delete;
  FaultSubscriber& operator= (const FaultSubscriber& rhs) = delete;
  void init(int fid) {
    if (fid >= 0 && fid < INNO_LIDAR_IN_FAULT_MAX) {
      fid_ = fid;
    }
  }
  void update(void* content) override {
    const enum InnoFaultOperation* operation =
               static_cast<enum InnoFaultOperation*>(content);
    std::unique_lock<std::mutex> lk(mutex_);
    if (*operation == INNO_FAULT_OPERATION_SET) {
      indegree_++;
    } else {
      indegree_--;
    }
    inno_log_verify_simple(indegree_ >= 0 &&
                           indegree_ < INNO_LIDAR_IN_FAULT_MAX,
                           "fid %d indegree_ invalid: %d", fid_, indegree_);
  }
  inline bool is_inhibited() {
    std::unique_lock<std::mutex> lk(mutex_);
    return indegree_ != 0;
  }

 private:
  int fid_;
  int indegree_;
  std::mutex mutex_;
};

class Fault {
  friend class FaultManager;

  Fault(const char* fault_name,
        int fid,
        uint16_t set_fault_debounce,
        uint16_t heal_fault_debounce,
        uint16_t heal_history_cycle_threshold,
        bool fault_enable = true);
  ~Fault();

 private:
  bool get_fault_enable_status() const;
  uint16_t get_fault_status() const;
  double get_debounce_time(enum InnoFaultOperation op);
  void add_no_fault_count();
  uint16_t get_no_fault_cycle_count() const;
  inline int get_fid() {
    return fid_;
  }
  void set_history_fault_bit();
  void set_current_fault_bit();
  inline bool get_current_fault_flag_() {
    return (fault_status_ & (1U << 0)) != 0;
  }
  void clear_history_fault_bit();
  void clear_current_fault_bit();
  void set_debounce_start_ts(double ts);
  double get_debounce_start_ts();
  void set_debouncing_bit();
  void clear_debouncing_bit();
  inline bool has_history_fault() {
    return (fault_status_ & (1U << 3)) != 0;
  }
  // fault ever happened this cycle
  bool current_cycle_fault_happened() const;
  void clear_current_cycle_fault_happened();
  void set_status(uint16_t status);
  void update_no_fault_cycle_count(uint16_t counter);
  const std::string& get_name() const {
    return fault_name_;
  }
  void set_diag_state_(enum InnoFaultState state);
  enum InnoFaultState get_diag_state_() const;
  void clear_bit(int position);
  void set_bit(int position);
  inline bool is_sub_fault() {
    return fid_ > INNO_LIDAR_IN_FAULT_OTHER &&
           fid_ < INNO_LIDAR_IN_FAULT_MAX;
  }

 private:
  std::string fault_name_;
  double debounce_start_ts_;
  struct tm fault_set_ts_;
  int fid_;
  uint16_t set_fault_debounce_;  // ms
  uint16_t heal_fault_debounce_;  // ms
  // continuous no fault cycles to heal history fault
  uint16_t heal_history_cycle_threshold_;
  uint16_t no_fault_cycle_so_far_;
  /*************************************************************************
   * fault_status_ header info: [15:8] reserved
   * others: T.B.D.
   * bit 7: warnning indicator, 0: indicator off, 1: indicator on
   * bit 6: debouncing flag, 0: not debouncing, 1: debouncing
   * bit 5:
   * bit 4:
   * bit 3: history faults, 0: no fault, 1: fault
   * bit 2:
   * bit 1: fault happens during current cycle, 0: no fault, 1: ever happened
   * bit 0: instant fault status, 0: no fault, 1: fault
   *************************************************************************/
  uint16_t fault_status_;
  enum InnoFaultState state_;
  bool fault_enable_;
};  // Fault

class FaultManager {  // internal fault manager
 private:
  struct SubFault {
    const char* fault_name;
    int fid;
    uint16_t set_fault_debounce;
    uint16_t heal_fault_debounce;
  };

 public:
  explicit FaultManager(InnoLidar* lidar);
  ~FaultManager();

 private:
  static const uint16_t kOverheatDebounceAfterBoot = 45;  // 45s
  static const uint16_t kInnoInternalFaultsMaxFileSize = 16 * 1024;
  static const uint16_t kMinIntervalAfterModeChange = 5 * 1000;  // 5s
  static const uint16_t kMaxIntervalAfterModeChange = 15 * 1000;  // 15s
  static const uint16_t kMinIntervalAfterConstruction = 5 * 1000;  // 5s

 private:
  static uint32_t bootup_number;
  static bool laser_overheat_;

 public:
  void set_raw_fault(int fid,
                     bool is_sub_fault,
                     enum InnoFaultOperation operation,
                     bool condition,
                     const char *format,
                     va_list valist);
  void set_history_fault(int fid,
                         const char *format,
                         va_list valist);
  // only return fault info of current and history faults
  int get_inner_faults_info(char *buffer,
                            int buffer_len,
                            bool from_network);
  // return info of all faults
  int get_inner_faults_all_info(char* buffer, int buffer_len);

  void set_faults_external(int fid,
                           enum InnoFaultOperation operation =
                           INNO_FAULT_OPERATION_SET);
  int init_faults_info(std::string str);
  // power cycle status
  int get_sync_cycle_status(char* buffer, int buffer_len);
  void update_data_packet(InnoStatusPacket* pkt);
  void on_mode_changed(double ts);
  void set_state_and_update_inhibit_list(enum FaultManagerState state,
                                         uint64_t timestamp,
                                         uint64_t inhibit_fault_list);

  inline bool overheat_check() {
    std::unique_lock<std::mutex> lk(mutex_);
    return !fault_list_[INNO_LIDAR_IN_FAULT_OVERHEAT3]->\
            get_current_fault_flag_();
  }

  inline int get_mode_status_fast(enum InnoLidarMode *mode,
                                  enum InnoLidarMode *pre_mode,
                                  enum InnoLidarStatus *status);

  int set_mode(enum InnoLidarMode mode,
               enum InnoLidarMode *mode_before_change,
               enum InnoLidarStatus *status_before_change);

  inline void do_message_callback(enum InnoMessageLevel error_level,
                                  enum InnoMessageCode code,
                                  const char *error_message);

  bool get_current_fault_status(int fid) {
    Fault* fault = NULL;
    if (is_sub_fault_(fid)) {
      fault = sub_fault_list_[fid - INNO_SUB_FAULT_BASE - 1];
    } else {
      fault = fault_list_[fid];
    }
    std::unique_lock<std::mutex> lk(mutex_);
    return fault->get_current_fault_flag_();
  }

  inline void enable_client_save_raw_with_lock(enum InnoLidarInFault fid) {
    client_save_raw_enable_ |= (1UL << fid);
  }

  inline void update_client_save_raw_enable(uint64_t value) {
    std::unique_lock<std::mutex> lk(mutex_save_raw_);
    client_save_raw_enable_ = value;
  }

  inline void disable_client_save_raw_with_lock(int fid) {
    client_save_raw_enable_ &= ~(1UL << fid);
  }

  inline LidarModeSubscriber* get_mode_subscriber() {
    return mode_subscriber_;
  }

  inline void sync_fw_faults() {
    fw_fault_synchronizer_.sync_fw_faults();
  }

  int get_fault_enable_status(int fid, bool is_sub_fault);
  inline bool get_fault_inhibit_status(enum InnoLidarInFault fid) {
    return fault_subscriber_[fid].is_inhibited();
  }

 private:
  void clear_fault_after_long_(enum InnoLidarInFault fid);
  void init_fault_configs_();
  void init_fault_list_();
  bool current_cycle_fault_happened_(enum InnoLidarInFault fid) const;
  int update_fault_status_(Fault* fault,
                           bool set_condition,
                           bool heal_condition,
                           double now_ts);
  int update_fault_status_external_(Fault* fault,
                           enum InnoFaultOperation operation);
  int get_fault_timestamp_(int index, char* buffer, int len);
  void message_log_out_(int fid,
                        enum InnoFaultOperation operation,
                        const char *format, va_list valist);
  int fault_message_with_stack_(int fid,
                                enum InnoFaultOperation operation,
                                const char *format, va_list valist);

  inline Fault* get_fault_from_sub_fault_(int sub_fid) {
    return sub_fault_to_fault_[sub_fault_list_\
                              [sub_fid - INNO_SUB_FAULT_BASE - 1]];
  }

  inline bool inhibited_(int fid) {
    return ((temporary_inhibit_faults_ >> fid) & 0x1);
  }

  inline bool is_sub_fault_(int fid) {
    return fid > INNO_SUB_FAULT_BASE &&
           fid < INNO_SUB_FAULT_MAX;
  }

 private:
#define TRY_TO_SEND_RAW(fid)                                        \
  do {                                                              \
    bool should_send;                                               \
    {                                                               \
      std::unique_lock<std::mutex> lock_save_raw_(mutex_save_raw_); \
      should_send = is_client_save_raw_enable_((fid));              \
      disable_client_save_raw_with_lock((fid));                     \
    }                                                               \
    if (should_send) {                                              \
      lidar_->set_save_raw_data_flag(std::to_string((fid)));        \
    }                                                               \
  } while (0)

  inline bool is_config_save_raw_enable_(int fid) const {
    return config_save_raw_enable_ & (1UL << fid);
  }

  inline void enable_config_save_raw_(InnoLidarInFault fid) {
    config_save_raw_enable_ |= (1UL << fid);
  }

  inline bool is_client_save_raw_enable_(int fid) const {
    return client_save_raw_enable_ & (1UL << fid);
  }

  void init_fault_inhibit_graph_();

 private:
  InnoLidar* lidar_;
  ConfidencePublisher conf_publisher_;
  LidarModeSubscriber* mode_subscriber_;
  FaultPublisher fault_publisher_[INNO_LIDAR_IN_FAULT_MAX];
  FaultSubscriber fault_subscriber_[INNO_LIDAR_IN_FAULT_MAX];
  FwFaultSynchronizer fw_fault_synchronizer_;
  FaultReactor* fault_reactor_;
  std::mutex mutex_;
  Fault* fault_list_[INNO_LIDAR_IN_FAULT_MAX];
  Fault* sub_fault_list_[INNO_SUB_FAULT_MAX - INNO_SUB_FAULT_BASE - 1];
  FaultConfig* config_list_base_[INNO_LIDAR_IN_FAULT_MAX];
  FaultConfig* config_list_[INNO_LIDAR_IN_FAULT_MAX];
  double mode_change_timestamp_;
  double fault_manager_construct_ts_;
  uint16_t faults_update_counter_;
  OnFaultCallback on_fault_[INNO_LIDAR_IN_FAULT_MAX];
  enum FaultManagerState fault_manager_state_;
  uint64_t temporary_inhibit_faults_;

  /**
   * predefined by the config;
   * indicate if the fault can trigger raw data sending.
   */
  uint64_t config_save_raw_enable_{0};
  /**
   * indicate if the fault has occurred in current dlb period(now is 14 days).
   * In one dlb period, pcs will send raw data 1 time at most for same fault.
   *
   * todo xxx: In fact we only need a save_raw_enable_ flag, and it should be
   * defined in cloud server. But this needs supporting from our customer(NIO).
   * Currently we will send raw data when
   * (config_save_raw_enable_ && client_save_raw_enable_) is true.
   */
  uint64_t client_save_raw_enable_{0};
  std::mutex mutex_save_raw_;
  std::unordered_set<int> event_driven_faults_;
  std::unordered_map<Fault*, Fault*> sub_fault_to_fault_;
  std::unordered_map<Fault*, std::vector<Fault*>> fault_to_sub_fault_;
};
}  // namespace innovusion

#endif  // SDK_FAULT_MANAGER_H_
