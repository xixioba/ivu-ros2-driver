/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "sdk/stage_deliver2.h"

#include <math.h>

#include "utils/consumer_producer.h"
#include "sdk/lidar.h"
#include "sdk/stage_deliver_job.h"

namespace innovusion {

StageDeliver2::StageDeliver2(InnoLidar *l) {
  lidar_ = l;
  params_ = &lidar_->get_params();
  lidar_->add_config(&config_base_);
  config_.copy_from_src(&config_base_);

  stats_dropped_jobs_ = 0;
  stats_delivered_jobs_ = 0;
}

StageDeliver2::~StageDeliver2(void) {
  lidar_->remove_config(&config_base_);
}

void StageDeliver2::print_stats(void) const {
  char buf[1024];
  get_stats_string(buf, sizeof(buf));
  inno_log_info("%s", buf);
  return;
}

void StageDeliver2::get_stats_string(char *buf, size_t buf_size) const {
  int ret = snprintf(buf, buf_size, "StageDeliver2: dropped=%"
                     PRI_SIZELU "delivered=%" PRI_SIZELU,
                     stats_dropped_jobs_,
                     stats_delivered_jobs_);
  if (ret >= ssize_t(buf_size)) {
    buf[buf_size - 1] = 0;
    return;
  }

  if (stats_delivered_jobs_ == 0) {
    return;
  }

  size_t so_far = ret;
  ret = snprintf(buf + so_far, buf_size - so_far,
                 " callback=%.2f/%.2f/%.2f/%" PRI_SIZEU,
                 callback_mean_ms_.mean(),
                 callback_mean_ms_.std_dev(),
                 callback_mean_ms_.max(),
                 callback_mean_ms_.count());
  if (ret >= ssize_t(buf_size - so_far)) {
    buf[buf_size - 1] = 0;
    return;
  }
}

int StageDeliver2::process(void *in_job, void *ctx,
                           bool prefer) {
  StageDeliver2 *s = reinterpret_cast<StageDeliver2 *>(ctx);
#ifdef __APPLE__
  INNER_BEGIN_LOG(StageDeliver2_process, OS_LOG_CATEGORY_DYNAMIC_TRACING,
                  StageDeliver2_process);
#endif
  int ret =
      s->process_job_(reinterpret_cast<StageDeliver2Job *>(in_job), prefer);
#ifdef __APPLE__
  INNER_END_LOG(StageDeliver2_process);
#endif
  return ret;
}

int StageDeliver2::process_job_(StageDeliver2Job *job,
                                bool prefer) {
  inno_log_assert(job, "job");

  if (!prefer) {
    for (size_t i = 0; i < job->packet_cnt; i++) {
      lidar_->free_deliver_points_job(job->packets[i]);
    }
    lidar_->free_deliver2_job(job);

    stats_dropped_jobs_++;
    if (stats_dropped_jobs_ % 10 == 1) {
      inno_log_warning("drop data in deliver2 stage total_dropped=%"
                      PRI_SIZELU, stats_dropped_jobs_);
    }
    if (stats_dropped_jobs_ % 100 == 1) {
      print_stats();
      lidar_->cp_deliver2_->print_stats();
    }
    return 0;
  }

  config_.copy_from_src(&config_base_);

  for (size_t i = 0; i < job->packet_cnt; i++) {
    int cr = 0;
    uint64_t start = InnoUtils::get_time_ns(CLOCK_MONOTONIC_RAW);
    if (lidar_->data_packet_callback_) {
      cr = lidar_->do_data_callback(job->packets[i]);
      // cr = lidar_->data_packet_callback_(lidar_->handle_,
      //                                    lidar_->callback_context_,
      //                                    job->packets[i]);
    }
    callback_mean_ms_.add((InnoUtils::get_time_ns(CLOCK_MONOTONIC_RAW) -
                           start) / 1000000.0);
    inno_log_verify(cr == 0, "StageDeliver2 data_packet_callback return %d",
                    cr);
    lidar_->free_deliver_points_job(job->packets[i]);
  }

  stats_delivered_jobs_++;

  lidar_->free_deliver2_job(job);

  return 0;
}

const char *StageDeliver2::get_name_(void) const {
  return lidar_->get_name();
}

}  // namespace innovusion
