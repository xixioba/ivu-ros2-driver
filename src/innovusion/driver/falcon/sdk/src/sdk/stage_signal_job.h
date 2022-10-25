/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_STAGE_SIGNAL_JOB_H_
#define SDK_STAGE_SIGNAL_JOB_H_

#include <mutex>  // NOLINT

#include "utils/log.h"
#include "utils/mem_pool_manager.h"
#include "utils/types_consts.h"
#include "sdk_common/inno_faults_common.h"

namespace innovusion {

class StageSignalJob {
 public:
  static const size_t kWriteBlockSize = 64 * 1024;
  static const size_t kLeftoverSize = 64;

 public:
  explicit StageSignalJob(size_t write_block_size,
                          size_t max_leftover_size) {
    write_block_size_ = write_block_size;
    max_leftover_size_ = max_leftover_size;
    leftover_size_ = 0;
    tail_size_ = 0;
    current_written_ = 0;
    is_first_chunck_ = false;
    ref_count_ = 1;
  }

  ~StageSignalJob() {
    inno_log_verify(ref_count_ == 0, "ref=%d", ref_count_);
  }

  inline void inc_ref(void) {
    std::unique_lock<std::mutex> lk(mutex_);
    ref_count_++;
  }

  inline bool dec_ref(void) {
    std::unique_lock<std::mutex> lk(mutex_);
    ref_count_--;
    bool reach_zero = ref_count_ == 0;
    return reach_zero;
  }

  void *get_write_buffer() {
    return &buffer_[current_written_ + max_leftover_size_];
  }

  size_t get_bytes_written() const {
    return current_written_;
  }

  size_t bytes_to_write() const {
    inno_log_verify(write_block_size_ >= current_written_,
                    "invalid bytes_to_write %" PRI_SIZELU " %" PRI_SIZELU,
                    write_block_size_, current_written_);
    return write_block_size_ - current_written_;
  }

  void add_bytes_written(size_t d) {
    current_written_ += d;
    inno_log_verify(current_written_ <= write_block_size_,
                    "%" PRI_SIZELU " vs %" PRI_SIZELU,
                    current_written_, write_block_size_);
    return;
  }

  char *get_buffer_wo_leftover() {
    return buffer_ + max_leftover_size_;
  }

  size_t get_buffer_wo_leftover_len() const {
    return current_written_;
  }

  char *get_buffer_with_leftover() {
    return get_buffer_wo_leftover() - leftover_size_;
  }

  size_t get_buffer_with_leftover_len() const {
    return get_buffer_wo_leftover_len() + leftover_size_;
  }

  size_t get_buffer_with_leftover_len_without_tail() const {
    return get_buffer_wo_leftover_len() + leftover_size_ - tail_size_;
  }

  void set_leftover(const char *src, size_t size) {
    inno_log_verify(size <= max_leftover_size_,
                    "leftover too big %" PRI_SIZELU " vs %" PRI_SIZELU,
                    size, max_leftover_size_);
    leftover_size_ = size;
    memcpy(get_buffer_with_leftover(), src, size);
  }

  void set_is_first_chunck(void) {
    is_first_chunck_ = true;
  }

  bool get_is_first_chunck(void) const {
    return is_first_chunck_;
  }

  void set_tail_size(size_t tail_size) {
    tail_size_ = tail_size;
  }

  void set_write_block_size(size_t write_block_size) {
    inno_log_verify(write_block_size_ >= write_block_size,
                    "write_block_size %" PRI_SIZELU, write_block_size);
    write_block_size_ = write_block_size;
  }

  inline void set_confidence_seq(enum ConfidenceLevel conf_level,
                                 uint32_t seq_number) {
    conf_seq_num_[conf_level] = seq_number;
  }

  inline uint32_t get_confidence_seq(enum ConfidenceLevel conf_level) {
    return conf_seq_num_[conf_level];
  }

 public:
  InnoEpSecondDouble before_read_ts;
  InnoEpSecondDouble enque_ts;
  InnoEpSecondDouble deque_ts;

 private:
  std::mutex mutex_;
  int32_t ref_count_;

  size_t write_block_size_;
  size_t max_leftover_size_;
  size_t leftover_size_;
  size_t tail_size_;
  size_t current_written_;
  uint32_t conf_seq_num_[INNO_CONFIDENCE_LEVEL_MAX]{0};

  bool is_first_chunck_;
  char buffer_[0];
};

class StageSignalJobPool {
 public:
  StageSignalJobPool(const char *name,
                     unsigned int unit_number,
                     size_t write_block_size,
                     size_t max_leftover_size,
                     unsigned int alignment = 32)
      : pool_(name,
              sizeof(StageSignalJob) + write_block_size + max_leftover_size,
              unit_number,
              alignment)
      , write_block_size_(write_block_size)
      , max_leftover_size_(max_leftover_size) {
  }

  ~StageSignalJobPool() {}

  StageSignalJob *alloc() {
    return new (pool_.alloc())
        StageSignalJob(write_block_size_, max_leftover_size_);
  }

  void free(StageSignalJob *o) {
    return pool_.free(o);
  }

 private:
  MemPool pool_;
  size_t write_block_size_;
  size_t max_leftover_size_;
};

}  // namespace innovusion
#endif  // SDK_STAGE_SIGNAL_JOB_H_
