/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_STAGE_HELP_H_
#define SDK_STAGE_HELP_H_

#include <mutex>  // NOLINT
#include <string>

// #include "sdk/stage_help_job.h"

namespace innovusion {
class InnoLidar;
class StageHelp;

enum HelperJobType {
  HELPER_JOB_TYPE_NONE = 0,
  HELPER_JOB_TYPE_REPORT_APD,
  HELPER_JOB_TYPE_MAX,
};

class StageHelpJob {
  friend StageHelp;

 public:
  StageHelpJob(enum HelperJobType type,
               const std::string &job_string,
               uint64_t ts)
      : type_(type)
      , ts_(ts)
      , job_string_(job_string) {
  }

  ~StageHelpJob() {
  }

 private:
  enum HelperJobType type_;
  uint64_t ts_;
  std::string job_string_;
};

class StageHelp {
  friend InnoLidar;

 public:
  static int process(void *job, void *ctx, bool prefer);

 public:
  explicit StageHelp(InnoLidar *l);
  ~StageHelp(void);

 private:
  const char *get_name_();
  int process_job_(StageHelpJob *job, bool prefer);
  void process_report_apd_(const std::string &cmd_stringm,
                           uint64_t ts);

 private:
  InnoLidar *lidar_;
  int command_fd_ {-1};
};

}  // namespace innovusion

#endif  // SDK_STAGE_HELP_H_

