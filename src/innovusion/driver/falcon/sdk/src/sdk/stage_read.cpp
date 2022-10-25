/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "sdk/stage_read.h"

#include <string>

#include "sdk/lidar.h"
#include "sdk/lidar_communication.h"
#include "sdk/direct_memory.h"

namespace innovusion {
StageRead::StageRead(InnoLidar *l,
                     LidarCommunication *lm,
                     bool use_tcp,
                     int max_retry) {
  init_(l);
  inno_log_verify(lm, "lm should nout be NULL");
  lidar_comm_ = lm;
  max_retry_ = max_retry;
  if (use_tcp) {
    source_ = SOURCE_TCP;
  } else {
    source_ = SOURCE_DIRECT_MEMORY;
  }
}

StageRead::StageRead(InnoLidar *l,
                     const char *filename,
                     double play_rate,
                     int rewind,
                     int64_t skip,
                     off_t data_start_off) {
  init_(l);
  lidar_comm_ = NULL;
  filename_ = filename;
  play_rate_ = play_rate;
  source_ = SOURCE_FILE;
  max_file_rewind_ = rewind;
  skip_ = skip;
  data_start_off_ = data_start_off;
}

StageRead::~StageRead(void) {
  lidar_comm_ = NULL;
  {
    std::unique_lock<std::mutex> lk(mutex_);
    inno_log_verify(state_ == InnoLidarBase::STATE_INIT,
                    "invalid state=%d",
                    state_);
    inno_log_verify(file_fd_ < 0,
                    "fd still open %d", file_fd_);
  }
  lidar_->remove_config(&config_base_);
}

void StageRead::init_(InnoLidar *l) {
  state_ = InnoLidarBase::STATE_INIT;
  lidar_ = l;
  file_fd_ = -1;
  filename_ = NULL;
  max_file_rewind_ = 0;
  skip_ = 0;
  data_start_off_ = 0;

  play_round_ = 1;
  skipped_ = 0;
  call_recorder_counter_ = 0;

  mem_queue_len_ = 0;
  max_mem_queue_len_ = 0;

  lidar_->add_config(&config_base_);
  config_.copy_from_src(&config_base_);

  streaming_stoped_ = false;
  streaming_start_ts_ = -1.0;
  recorver_counter_ = 0;
  fpga_regs_collector_.\
  set_lidar_live_direct_mode(lidar_->is_live_direct_memory_lidar_());
}

const char *StageRead::get_name_(void) const {
  return lidar_->get_name();
}

void StageRead::start_reading_(void) {
  {
    std::unique_lock<std::mutex> lk(mutex_);
    inno_log_verify(state_ == InnoLidarBase::STATE_INIT,
                    "%s state=%d", get_name_(), state_);
    state_ = InnoLidarBase::STATE_READING;
  }
  cond_.notify_all();
  config_.copy_from_src(&config_base_);
}

void StageRead::stop(void) {
  inno_log_info("%s stop", get_name_());
  std::unique_lock<std::mutex> lk(mutex_);
  if (state_ == InnoLidarBase::STATE_INIT) {
    cond_.wait(lk, [this] {
                   inno_log_info("%s wait for state %d", get_name_(), state_);
                   return state_ != InnoLidarBase::STATE_INIT;
                 });
  }
  inno_log_verify(state_ == InnoLidarBase::STATE_READING ||
                  state_ == InnoLidarBase::STATE_STOPPED ||
                  state_ == InnoLidarBase::STATE_STOPPING,
                  "%s state=%d, forget to call start before stop?",
                  get_name_(), state_);
  if (state_ == InnoLidarBase::STATE_READING) {
    state_ = InnoLidarBase::STATE_STOPPING;
    cond_.notify_all();
  }
  cond_.wait(lk, [this] {
                   inno_log_info("%s wait for state %d", get_name_(), state_);
                   return state_ != InnoLidarBase::STATE_STOPPING;
                 });
}

void StageRead::streaming_stop(void) {
  inno_log_info("streaming stoped, now shut down %s", get_name_());
  streaming_stoped_ = true;
  stop();
}

bool StageRead::is_started() {
  std::unique_lock<std::mutex> lk(mutex_);
  return state_ == InnoLidarBase::STATE_READING ||
         state_ == InnoLidarBase::STATE_STOPPED ||
         state_ == InnoLidarBase::STATE_STOPPING;
}

enum InnoLidarBase::State StageRead::get_state() {
  std::unique_lock<std::mutex> lk(mutex_);
  enum InnoLidarBase::State ret = state_;

  return ret;
}

void StageRead::final_cleanup(void) {
  if (source_ == SOURCE_FILE) {
    if (file_fd_ >= 0) {
      close(file_fd_);
      file_fd_ = -1;
    }
  } else {
    lidar_comm_->close_streaming();
  }

  {
    std::unique_lock<std::mutex> lk(mutex_);
    state_ = InnoLidarBase::STATE_INIT;
    cond_.notify_all();
  }
}

void StageRead::print_stats(void) const {
  char buf[1024];
  get_stats_string(buf, sizeof(buf));
  inno_log_info("%s", buf);
  return;
}

void StageRead::get_stats_string(char *buf, size_t buf_size) const {
  int ret = snprintf(buf, buf_size,
                     "StageRead: %s,"
                     "raw_data_read_: %" PRI_SIZEU,
                     "XXX TODO", stats_raw_data_read_);
  if (ret >= ssize_t(buf_size)) {
    buf[buf_size - 1] = 0;
    return;
  }
}

int StageRead::process(void *in_job, void *ctx,
                       bool prefer) {
  // in_job is ignored
  StageRead *s = reinterpret_cast<StageRead *>(ctx);
#ifdef __APPLE__
  INNER_BEGIN_LOG(StageRead_process, OS_LOG_CATEGORY_DYNAMIC_TRACING,
                  StageRead_process);
#endif
  int ret = s->read_loop_(prefer);
#ifdef __APPLE__
  INNER_END_LOG(StageRead_process);
#endif
  return ret;
}

StageSignalJob *StageRead::alloc_signal_job_(void) {
  StageSignalJob *job = lidar_->alloc_signal_job();
  inno_log_verify(job, "cannot alloc StageSignalJob");
  for (uint32_t i = 0; i < INNO_CONFIDENCE_LEVEL_MAX; i++) {
    enum ConfidenceLevel conf_level = ConfidenceLevel(i);
    uint32_t conf_seq = lidar_->\
             get_confidence_seq(INNO_PROCESS_STAGE_READ, conf_level);
    job->set_confidence_seq(conf_level, conf_seq);
  }
  job->set_write_block_size(source_ == SOURCE_FILE
                            ? config_.file_read_block
                            : config_.mem_read_block);
  job->before_read_ts = lidar_->get_monotonic_raw_time();
  return job;
}

void StageRead::free_signal_job_(StageSignalJob *job) {
  lidar_->free_signal_job(job);
  return;
}

int StageRead::call_recorder_(StageSignalJob *out_job) {
  if (!lidar_->has_recorder_callback(INNO_RECORDER_CALLBACK_TYPE_RAW)) {
    return 0;
  }
  if (call_recorder_counter_ == 0) {
    lidar_->recorder_write_yaml(INNO_RECORDER_CALLBACK_TYPE_RAW);
  }
  call_recorder_counter_++;
  lidar_->do_recorder_callback(INNO_RECORDER_CALLBACK_TYPE_RAW,
                               out_job->get_buffer_wo_leftover(),
                               out_job->get_buffer_wo_leftover_len());
  return 0;
}

bool StageRead::need_break_() {
  std::unique_lock<std::mutex> lk(mutex_);
  inno_log_verify(state_ == InnoLidarBase::STATE_READING ||
                  state_ == InnoLidarBase::STATE_STOPPING,
                  "%s reader state=%d", get_name_(), state_);
  if (state_ == InnoLidarBase::STATE_STOPPING) {
    inno_log_info("%s reader new state %d, stopping", get_name_(), state_);
    return true;
  } else {
    return false;
  }
}


//
//
//
int StageRead::read_loop_(bool prefer) {
  inno_log_panic_if_not(prefer, "%s reader prefer", get_name_());
  start_reading_();

  StageSignalJob *out_job = alloc_signal_job_();
  out_job->set_is_first_chunck();

  int cannot_read = false;
  int32_t timeout_count = 0;
  uint64_t read_count = 0;
  uint64_t stats_raw_data_read_this_round = 0;

  play_round_ = 1;
  skipped_ = 0;
  call_recorder_counter_ = 0;

  rate_control_first_time_ = true;

  while (1) {
#ifdef __APPLE__
    INNER_BEGIN_LOG(StageRead_read_loop_, OS_LOG_CATEGORY_DYNAMIC_TRACING,
                    StageRead_read_loop_);
#endif
    config_.copy_from_src(&config_base_);

    bool need_break = need_break_();

    if (!need_break) {
      int r;
      if (source_ == SOURCE_FILE) {
        r = read_file_(out_job);
      } else {
        r = read_lidar_(out_job);
      }

      if (r < 0) {
        need_break = true;
        cannot_read = true;
        inno_log_warning("%s reader got %d return", get_name_(), r);
      } else if (r == 0) {
        // timeout
        inno_log_warning("%s timeout", get_name_());
        timeout_count++;
        if (timeout_count > max_retry_) {
          need_break = true;
          cannot_read = true;
          inno_log_warning("%s reader got too many timeout %d",
                           get_name_(), timeout_count);
        } else {
          lidar_->do_message_callback_fmt(
              INNO_MESSAGE_LEVEL_WARNING,
              INNO_MESSAGE_CODE_READ_TIMEOUT,
              "read from controller timed out %d %d",
              timeout_count, max_retry_);
        }
      } else {
        timeout_count = 0;
        read_count++;
        {
          stats_raw_data_read_ += out_job->get_bytes_written();
          stats_raw_data_read_this_round += out_job->get_bytes_written();

          call_recorder_(out_job);
          out_job->enque_ts = lidar_->get_monotonic_raw_time();
          lidar_->add_stage_signal_job(out_job);

          // refill
          // inno_log_trace("%s reader add one job to stage1", get_name_());
          out_job = alloc_signal_job_();
        }
        lidar_->stats_update_packet_bytes(ResourceStats::PACKET_TYPE_SRC, 1, r);
      }
    }

    if (need_break) {
      {
        std::unique_lock<std::mutex> lk(mutex_);
        inno_log_panic_if_not(state_ == InnoLidarBase::STATE_READING ||
                              state_ == InnoLidarBase::STATE_STOPPING,
                              "%s state=%d", get_name_(), state_);
        state_ = InnoLidarBase::STATE_STOPPED;
        inno_log_info("%s reader new state %d", get_name_(), state_);
      }
      cond_.notify_all();
      break;
    }
#ifdef __APPLE__
    INNER_END_LOG(StageRead_read_loop_);
#endif
  }  // while (1)

  free_signal_job_(out_job);


  inno_log_with_level(
      cannot_read ? INNO_LOG_LEVEL_ERROR : INNO_LOG_LEVEL_INFO,
      "cannot read (%d), streaming_stoped_=%d "
      "read_count=%" PRI_SIZEU ", bytes=%"
      PRI_SIZEU "/%" PRI_SIZEU,
      cannot_read,
      streaming_stoped_,
      read_count,
      stats_raw_data_read_this_round,
      stats_raw_data_read_);

  if (streaming_stoped_) {
    lidar_->do_message_callback_fmt(INNO_MESSAGE_LEVEL_INFO,
                                    INNO_MESSAGE_CODE_TO_NON_WORKING_MODE,
                                    "%s change to non-working mode",
                                    get_name_());
  } else if (cannot_read) {
    if (source_ == SOURCE_FILE) {
      if (stats_raw_data_read_this_round > 0) {
        lidar_->do_message_callback_fmt(INNO_MESSAGE_LEVEL_INFO,
                                        INNO_MESSAGE_CODE_READ_FILE_END,
                                        "end of file %s", get_name_());
      } else {
        lidar_->do_message_callback_fmt(INNO_MESSAGE_LEVEL_ERROR,
                                        INNO_MESSAGE_CODE_CANNOT_READ,
                                        "cannot read file %s", get_name_());
      }
    } else {
      bool has_stream = (streaming_stoped_ == false) &&
          lidar_->check_mode_status_fast(InnoLidar::FLAG_HAS_STREAM);
      lidar_->do_message_callback_fmt(
          has_stream ? INNO_MESSAGE_LEVEL_CRITICAL : INNO_MESSAGE_LEVEL_INFO,
          INNO_MESSAGE_CODE_CANNOT_READ, "cannot read from lidar %s",
          get_name_());
    }
  }
  return 0;
}


//
//
//
int StageRead::send_start_lidar_(void) {
  int ret = lidar_comm_->start_streaming(source_ == SOURCE_TCP);
  if (ret < 0) {
    inno_log_error("%s cannot send start (%d)", get_name_(), ret);
    lidar_comm_->close_streaming();
    return -10;
  } else {
    double now_ts = lidar_->get_monotonic_raw_time_ms();
    {
      std::unique_lock<std::mutex> lk(mutex_);
      streaming_start_ts_ = now_ts;
    }
    inno_log_info("%s start command sent", get_name_());
    return 0;
  }
}

int StageRead::wait_until_allow_to_stream_() {
  if (source_ == SOURCE_DIRECT_MEMORY) {
    uint64_t check_count = 0;
    uint64_t query_count = 0;
    while (1) {
      double v = lidar_comm_->get_config_section_key_value("manufacture",
                                                           "internal_server",
                                                           0);
      bool overheat_pass = lidar_->overheat_check();
      enum InnoLidarMode mode;
      enum InnoLidarMode pre_mode;
      enum InnoLidarStatus status;
      uint64_t transition_ms;
      int ret = lidar_->get_mode_status(&mode, &pre_mode,
                                        &status, &transition_ms);
      bool mode_check_pass = ret == 0 &&
                             status == INNO_LIDAR_STATUS_NORMAL &&
                             (mode == INNO_LIDAR_MODE_WORK_NORMAL ||
                             mode == INNO_LIDAR_MODE_WORK_SHORT_RANGE ||
                             mode == INNO_LIDAR_MODE_WORK_CALIBRATION ||
                             mode == INNO_LIDAR_MODE_WORK_QUIET);
      if (v == 0 || !overheat_pass || !mode_check_pass) {
        if (check_count == 0 && v == 0) {
          inno_log_warning("internal_server is disabled. Waiting...");
        }
        if (query_count % (v == 0 ? 4 : 128) == 0) {
          inno_log_info("not allowed to read!!! "
                        "enable: %0.2f, overheat_pass: %d, "
                        "mode_check_pass: %d, ret: %d, status: %d, "
                        "mode: %d, recorver_counter: %u/%u",
                         v, overheat_pass, mode_check_pass, ret,
                         status, mode, recorver_counter_,
                         kProtectionRecorverCounter);
        }
        check_count += v == 0;
        query_count++;
        uint32_t sleep_slice_ms = 10 * 1000;  // 10ms
        if (v == 0) {
          sleep_slice_ms = 5 * 100 * 1000;  // 500ms
        }
        for (int i = 0; i < 10; i++) {
          if (need_break_()) {
            return -1;
          }
          usleep(sleep_slice_ms);
        }
        if (v != 0 && overheat_pass &&
            mode == INNO_LIDAR_MODE_PROTECTION  &&
           (++recorver_counter_) % kProtectionRecorverCounter == 0) {
          lidar_->set_mode(INNO_LIDAR_MODE_WORK_NORMAL, &pre_mode, &status);
          lidar_->\
          do_message_callback_fmt(INNO_MESSAGE_LEVEL_WARNING,
                                  INNO_MESSAGE_CODE_TO_NON_WORKING_MODE,
                                  "lidar tries self-recorvering "
                                  "from jammed status [mode %d], counter: %u",
                                  mode, recorver_counter_);
        }
      } else {
        inno_log_info("internal_server is enabled "
                      "and overheat check passed, %u", recorver_counter_);
        return 0;
      }
    }
  }
  return 0;
}

int StageRead::read_lidar_first_time_(void) {
  int ret = wait_until_allow_to_stream_();
  if (ret < 0) {
    return ret;
  }

  if (lidar_comm_->send_stop() != 0) {
    inno_log_error("%s cannot send stop command", get_name_());
    return -1;
  } else {
    inno_log_info("%s send stop command", get_name_());
  }

  if (lidar_->before_read_start()) {
    inno_log_info("before_read_start failed");
    return -1;
  }

  // establish streaming connection
  int o = lidar_comm_->open_streaming();
  if (o < 0) {
    inno_log_error("%s failed to open stream", get_name_());
    return -2;
  }
  inno_log_info("%s open stream", get_name_());

  // send start command
  return send_start_lidar_();
}

int StageRead::read_lidar_(StageSignalJob *job) {
  bool is_read_lidar_first_time = false;
  if (lidar_comm_->get_streaming_fd() < 0) {
    int ret = read_lidar_first_time_();
    if (ret < 0) {
      return ret;
    }
    is_read_lidar_first_time = true;
  }

  if (source_ == SOURCE_DIRECT_MEMORY) {
    DirectMemory *inst = DirectMemory::get_instance();
    int queue_len;
    int ret = inst->read(job->get_write_buffer(),
                         job->bytes_to_write(),
                         &queue_len,
                         &fpga_regs_collector_);
    // re-try read job 5 times, if this is the first time to read buffer
    // and ret is zero (read timeout).
    if (ret == 0 && is_read_lidar_first_time) {
      int retry_times = 5;
      while (retry_times--) {
        usleep(100000);  // wait 100ms
        inno_log_info("read raw data retry times remaining %d", retry_times);
        ret = inst->read(job->get_write_buffer(),
                         job->bytes_to_write(),
                         &queue_len,
                         &fpga_regs_collector_);
        if (ret != 0) {
          break;
        }
      }
    }

    if (ret < 0) {
      inno_log_error("%s read", get_name_());
      return -11;
    } else if (ret == 0) {
      inno_log_error("%s read from direct_memory return 0 when read %"
                      PRI_SIZELU, get_name_(), job->bytes_to_write());
      lidar_->set_raw_fault(INNO_LIDAR_IN_FAULT_RAWDATA_TO, true,
                           "stage_read timeout: %s",
                            fpga_regs_collector_.get_info().c_str());
      fpga_regs_collector_.reset();
      return -12;
    }
    bool has_fault_old = lidar_->\
         get_current_fault_status(INNO_LIDAR_IN_FAULT_RAWDATA_TO);
    lidar_->heal_raw_fault(INNO_LIDAR_IN_FAULT_RAWDATA_TO, true,
                          "stage_read timeout recovers");
    bool has_fault = lidar_->\
         get_current_fault_status(INNO_LIDAR_IN_FAULT_RAWDATA_TO);
    if (has_fault_old && !has_fault) {
      fpga_regs_collector_.\
      collect(std::string("can read raw data: "),
              kInnoPLRegsReadTimeOut,
              sizeof(kInnoPLRegsReadTimeOut) / sizeof(struct RegInfo),
              kInnoPSRegsReadTimeOut,
              sizeof(kInnoPSRegsReadTimeOut) / sizeof(struct RegInfo));
      inno_log_info("%s", fpga_regs_collector_.get_info().c_str());
    }
    fpga_regs_collector_.reset();

    if (max_mem_queue_len_ < queue_len) {
      max_mem_queue_len_ = queue_len;
    }
    mem_queue_len_ = queue_len;

    inno_log_trace("%s read %d", get_name_(), ret);
    job->add_bytes_written(ret);
  } else {
    while (job->bytes_to_write() > 0) {
      int ret;
      while (-1 == (ret = recv(lidar_comm_->get_streaming_fd(),
                               job->get_write_buffer(),
                               job->bytes_to_write(), 0)) &&
             errno == EINTR) {
      }
      if (ret < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          inno_log_warning("%s read timeout", get_name_());
          return 0;
        } else {
          inno_log_error_errno("%s read", get_name_());
          return -11;
        }
      } else if (ret == 0) {
        inno_log_error("%s read return 0 when read %" PRI_SIZELU,
                       get_name_(), job->bytes_to_write());
        return -12;
      } else {
        // inno_log_detail("%s read %d", get_name_(), ret);
        job->add_bytes_written(ret);
      }
    }
  }
  // inno_log_trace("%s read from return %d", get_name_(), job->len_);

  return job->get_bytes_written();
}

//
//
//
int StageRead::load_yaml_from_data_file(const char *filename,
                                        char **yaml, size_t *off) {
  inno_log_verify(yaml, "yaml");
  inno_log_verify(off, "off");

  char version[12];
  static const char version_with_yaml[] = "INNODATA1000";
  uint32_t size;

  int fd = InnoUtils::open_file(filename, O_RDONLY, 0);
  if (fd < 0) {
    return -1;
  }

  int ret = read(fd, version, sizeof(version));
  if (ret != sizeof(version)) {
    inno_log_error("cannot read version %d %s", ret, filename);
    close(fd);
    return -1;
  }

  if (memcmp(version, version_with_yaml, sizeof(version)) != 0) {
    inno_log_info("%s has no yaml info", filename);
    close(fd);
    return -1;
  }

  ret = read(fd, &size, sizeof(size));
  if (ret != sizeof(size)) {
    inno_log_error("cannot read size %d %s", ret, filename);
    close(fd);
    return -1;
  }

  size = ntohl(size);
  if (size > InnoLidar::kMaxYamlSize) {
    inno_log_error("yaml size too big %u %s", size, filename);
    close(fd);
    return -1;
  }

  char *buffer = reinterpret_cast<char *>(calloc(size, 1));
  inno_log_verify(buffer, "buffer %u", size);

  ret = read(fd, buffer, size);
  if (ret != int32_t(size)) {
    inno_log_error("cannot read size %d vs %u %s", ret, size, filename);
    close(fd);

    free(buffer);
    buffer = NULL;
    return -1;
  }

  *yaml = buffer;
  *off = size + sizeof(size) + sizeof(version);
  close(fd);

  return 0;
}

//
//
//
int StageRead::load_yaml_from_yaml_file(const char *filename,
                                        char **yaml, size_t *off) {
  inno_log_verify(yaml, "yaml");
  inno_log_verify(off, "off");
  int fd = InnoUtils::open_file(filename, O_RDONLY, 0);
  if (fd < 0) {
    return -1;
  }
  int size = lseek(fd, 0, SEEK_END);
  if (size > (ssize_t)InnoLidar::kMaxYamlSize) {
    inno_log_error("yaml size too big %d %s", size, filename);
    close(fd);
    return -1;
  }
  char *buffer = reinterpret_cast<char *>(calloc(size, 1));
  inno_log_verify(buffer, "buffer %d", size);
  lseek(fd, 0, SEEK_SET);
  int ret = read(fd, buffer, size);
  if (ret != int32_t(size)) {
    inno_log_error("cannot read size %d vs %d %s", ret, size, filename);
    close(fd);
    return -1;
  }
  *yaml = buffer;
  *off = size;
  close(fd);

  return 0;
}

int StageRead::open_file_and_seek_to_data_(void) {
  inno_log_verify(file_fd_ < 0, "fd %d", file_fd_);
  int ret = InnoUtils::open_file(filename_, O_RDONLY, 0);
  if (ret < 0) {
    return ret;
  }
  file_fd_ = ret;
  off_t r = lseek(file_fd_, data_start_off_, SEEK_SET);
  if (r < 0) {
    inno_log_error_errno("lseek error %s", filename_);
    return -1;
  } else {
    return 0;
  }
}

int StageRead::read_file_first_time_(void) {
  inno_log_verify(file_fd_ < 0, "file_fd_ = %d", file_fd_);
  inno_log_info("read_file_first_time_");
  int ret = open_file_and_seek_to_data_();
  if (ret) {
    return ret;
  } else {
    // file opened
    if (lidar_->before_read_start()) {
      inno_log_info("before_read_start failed");
      return -1;
    } else {
      inno_log_info("%s open %s", get_name_(), filename_);
      return 0;
    }
  }
}

int StageRead::read_file_(StageSignalJob *job) {
  if (file_fd_ < 0) {
    int ret = read_file_first_time_();
    if (ret < 0) {
      return ret;
    }
  }

  uint64_t start_ns = InnoUtils::get_time_ns(CLOCK_MONOTONIC_RAW);

  int r;
  for (int k = 0; k < 2; k++) {
    while (-1 == (r = read(file_fd_, job->get_write_buffer(),
                           job->bytes_to_write())) &&
           errno == EINTR) {
    }
    if (r == 0) {
      inno_log_info("%s reach end of %s",
                    get_name_(), filename_);
      if (k == 0 && (play_round_ < max_file_rewind_ + 1 ||
                     max_file_rewind_ < 0)) {
        inno_log_info("%s rewind file %s %d/%d",
                      get_name_(), filename_,
                      play_round_, max_file_rewind_);
        play_round_++;
        close(file_fd_);
        file_fd_ = -1;
        job->set_is_first_chunck();
        int ret = open_file_and_seek_to_data_();
        if (ret) {
          inno_log_info("%s cannot open %s %d",
                        get_name_(), filename_, ret);
          return -5;
        } else {
          continue;
        }
      } else {
        inno_log_info("%s no more rewind %s", get_name_(), filename_);
        return -4;
      }
    } else {
      break;
    }
  }
  if (r <= 0) {
    inno_log_error_errno("%s cannot read %s", get_name_(), filename_);
    return -5;
  } else {
    job->add_bytes_written(r);
    uint64_t end_ns = InnoUtils::get_time_ns(CLOCK_MONOTONIC_RAW);
    uint64_t elapsed_us = (end_ns - start_ns) / 1000;
    read_file_rate_control_(elapsed_us, r);

    inno_log_trace("%s read from %s return %d",
                   get_name_(), filename_, r);

    return r;
  }
}

void StageRead::read_file_rate_control_(uint64_t spent_us, int r) {
  if (play_rate_ == 0) {
    return;
  }

  if (rate_control_first_time_) {
    rate_control_start_time_ms_ = lidar_->get_monotonic_raw_time_ms();
    rate_control_total_byte_received_ = 0;
    rate_control_first_time_ = false;
  }

  rate_control_total_byte_received_ += r;
  int64_t sleep_us = rate_control_total_byte_received_ / play_rate_;
  int64_t elapsed_us = (lidar_->get_monotonic_raw_time_ms() -
                        rate_control_start_time_ms_)
                       * 1000;
  if (sleep_us > elapsed_us && elapsed_us >= 0) {
    usleep(sleep_us - elapsed_us);
  }
  return;
}

double StageRead::get_streaming_start_ts() {
  std::unique_lock<std::mutex> lk(mutex_);
  if (streaming_start_ts_ < 0) {
    return lidar_->get_monotonic_raw_time_ms();
  }
  return streaming_start_ts_;
}

}  // namespace innovusion
