/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "sdk/stage_help.h"

#include "sdk/lidar.h"
#include "sdk/lidar_communication.h"
#include "utils/log.h"

namespace innovusion {
StageHelp::StageHelp(InnoLidar *l) {
  lidar_ = l;
}

StageHelp::~StageHelp(void) {
  if (command_fd_ >= 0) {
    close(command_fd_);
    command_fd_ = -1;
  }
}

int StageHelp::process(void *in_job, void *ctx,
                       bool prefer) {
  StageHelp *s = reinterpret_cast<StageHelp *>(ctx);
  return s->process_job_(reinterpret_cast<StageHelpJob *>(in_job),
                         prefer);
}

int StageHelp::process_job_(StageHelpJob *job,
                            bool prefer) {
  switch (job->type_) {
    case HELPER_JOB_TYPE_REPORT_APD:
      process_report_apd_(job->job_string_, job->ts_);
      break;
    default:
      inno_log_panic("invalid type %d", job->type_);
  }
  delete job;
  return 0;
}

const char *StageHelp::get_name_() {
  return lidar_->get_name();
}

void StageHelp::process_report_apd_(const std::string &job_string,
                                    uint64_t ts) {
  if (command_fd_ < 0) {
    command_fd_ = lidar_->comm_->get_connection(0.2, 8 * 1024);
    inno_log_info("got connection to report_apd %d", command_fd_);
  }

  if (command_fd_ < 0) {
    inno_log_error("no connection to lidar, %s", job_string.c_str());
  } else {
    char reply[100];
    int reply_len;

    reply[0] = 0;
    reply_len = sizeof(reply);
    int r = lidar_->comm_->send_command_with_fd(command_fd_,
                                                reply, &reply_len,
                                                "report_apd_info%s",
                                                job_string.c_str());
    if (r < 0) {
      inno_log_error("no apd reply s=%s", job_string.c_str());
      close(command_fd_);
      command_fd_ = -1;
    } else {
      int value;
      uint64_t now = InnoUtils::get_time_ms(CLOCK_MONOTONIC_RAW);
      if (sscanf(reply, "%d", &value) == 1 && value > 0) {
        inno_log_trace("apd reply t=%" PRI_SIZEU
                       "/%" PRI_SIZEU " s=%s r=%s v=%d",
                       now, ts, job_string.c_str(), reply, value);
        lidar_->set_apd_report_interval(value);
      } else {
        inno_log_error("bad apd reply t=%" PRI_SIZEU
                       "/%" PRI_SIZEU " s=%s r=%s",
                       now, ts, job_string.c_str(), reply);
      }
    }
  }
}

}  // namespace innovusion
