/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_STAGE_READ_H_
#define SDK_STAGE_READ_H_

#include <condition_variable>  // NOLINT
#include <mutex>               // NOLINT
#include <string>

#include "sdk_common/lidar_base.h"
#include "sdk/stage_signal_job.h"
#include "sdk/fpga_regs_collector.h"
#include "utils/config.h"
#include "utils/types_consts.h"

namespace innovusion {
class InnoLidar;
class LidarCommunication;
class StageSignalJob;

class StageReadConfig: public Config {
 public:
  StageReadConfig() : Config() {
    file_read_block = 64 * 1024;
    // xxx todo: reduce to 64KB to reduce latency, need FPGA work
    mem_read_block = 64 * 1024;
  }

  const char* get_type() const override {
    return "Lidar_StageRead";
  }

  int set_key_value_(const std::string &key, double value) override {
    SET_CFG(file_read_block);
    SET_CFG(mem_read_block);
    return -1;
  }

  int set_key_value_(const std::string &key,
                             const std::string value) override {
    // no string attribute
    return -1;
  }

  BEGIN_CFG_MEMBER()
  size_t file_read_block;
  size_t mem_read_block;
  END_CFG_MEMBER()
};

class StageRead {
  friend InnoLidar;

 private:
  enum Source {
    SOURCE_NO = 0,
    SOURCE_FILE,
    SOURCE_TCP,
    SOURCE_DIRECT_MEMORY,
    SOURCE_MAX,
  };

 public:
  static int process(void *job, void *ctx, bool prefer);
  static int load_yaml_from_data_file(const char *filename,
                                      char **yaml, size_t *off);
  static int load_yaml_from_yaml_file(const char *filename,
                                      char **yaml, size_t *off);

 public:
  StageRead(InnoLidar *l,
            LidarCommunication *lm,
            bool use_tcp,
            int max_retry);
  StageRead(InnoLidar *l,
            const char *filename,
            double play_rate,
            int rewind,
            int64_t skip,
            off_t data_start_off);
  ~StageRead(void);

 public:
  void stop(void);
  void final_cleanup(void);
  enum InnoLidarBase::State get_state();
  void print_stats(void) const;
  void get_stats_string(char *buf, size_t buf_size) const;
  int get_max_queue_len() const {
    return max_mem_queue_len_;
  }
  int get_queue_len() const {
    return mem_queue_len_;
  }
  double get_streaming_start_ts();

 private:
  void init_(InnoLidar *l);
  const char *get_name_(void) const;

 private:
  static const uint32_t kProtectionRecorverCounter = 1500;

  void start_reading_(void);
  int send_stop_lidar_(void);
  int send_start_lidar_(void);
  StageSignalJob *alloc_signal_job_(void);
  void free_signal_job_(StageSignalJob *);
  int call_recorder_(StageSignalJob *out_job);
  bool need_break_();
  int read_loop_(bool prefer);
  int open_file_and_seek_to_data_(void);
  int read_file_first_time_(void);
  int read_file_(StageSignalJob *job);
  void read_file_rate_control_(uint64_t spent_us, int r);
  int wait_until_allow_to_stream_();
  int read_lidar_first_time_(void);
  int read_lidar_(StageSignalJob *job);
  void streaming_stop(void);
  bool is_started(void);

 private:
  InnoLidar *lidar_;
  FpgaRegsCollector fpga_regs_collector_;

  StageReadConfig config_base_;
  StageReadConfig config_;

  enum Source source_;
  LidarCommunication *lidar_comm_;
  int max_retry_ = 0;

  const char *filename_;

  double play_rate_;
  int max_file_rewind_;
  int64_t skip_;
  off_t data_start_off_;

  bool rate_control_first_time_;
  size_t rate_control_total_byte_received_;
  InnoEpMs rate_control_start_time_ms_;

  int play_round_;
  int64_t skipped_;
  size_t call_recorder_counter_;

  int file_fd_;
  off_t data_location_;

  int mem_queue_len_;
  int max_mem_queue_len_;

  enum InnoLidarBase::State state_;
  std::mutex mutex_;
  std::condition_variable cond_;

  uint64_t stats_raw_data_read_ {0};
  double streaming_start_ts_;
  uint32_t recorver_counter_;
  bool streaming_stoped_;
};

}  // namespace innovusion

#endif  // SDK_STAGE_READ_H_
