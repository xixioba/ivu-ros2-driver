/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_STAGE_DELIVER2_H_
#define SDK_STAGE_DELIVER2_H_

#include <mutex>  // NOLINT
#include <string>

#include "utils/config.h"
#include "sdk/lidar.h"

namespace innovusion {
class InnoLidar;

class StageDeliver2Config: public Config {
 public:
  StageDeliver2Config() : Config() {
  }

  const char* get_type() const override {
    return "Lidar_StageDeliver2";
  }

  int set_key_value_(const std::string &key,
                             double value) override {
    return -1;
  }

  int set_key_value_(const std::string &key,
                     const std::string value) override {
    // no string attribute
    return -1;
  }

  BEGIN_CFG_MEMBER()
  END_CFG_MEMBER()
};

class StageDeliver2 {
  friend InnoLidar;

 public:
  static int process(void *job, void *ctx, bool prefer);

 public:
  explicit StageDeliver2(InnoLidar *l);
  ~StageDeliver2(void);

 public:
  void print_stats(void) const;
  void get_stats_string(char *buf, size_t buf_size) const;

 private:
  const char *get_name_() const;
  int process_job_(StageDeliver2Job *job,
                   bool prefer);

 private:
  InnoLidar *lidar_;
  std::mutex mutex_;
  InnoMean callback_mean_ms_;
  const LidarParams *params_;

  size_t stats_dropped_jobs_;
  size_t stats_delivered_jobs_;

  StageDeliver2Config config_base_;
  StageDeliver2Config config_;
};

}  // namespace innovusion

#endif  // SDK_STAGE_DELIVER2_H_
