/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_SYSTEM_STATS_H_
#define SDK_SYSTEM_STATS_H_

#include <limits.h>

#include <mutex>  // NOLINT
#include <string>

#include "sdk_common/resource_stats.h"
#include "sdk_common/inno_lidar_packet.h"
#include "sdk/status_report.h"

namespace innovusion {
class InnoLidar;

class SystemStats : public ResourceStats {
 public:
  explicit SystemStats(InnoLidar *l);
  ~SystemStats() {
  }

 protected:
  void get_extra_info_(char *buf, size_t buf_size,
                       double time_diff);

 public:
  void get_last_info_buffer(char *buf, size_t buf_size);
  void get_sys_stats(InnoStatusCounters *counters);
  void init_config(const StatusReportConfig *config);

 private:
  void log_fault_(enum InnoLidarInFault fault_id, const char *fault_info);
  int detect_network_();
  int find_str_infile_(const char* filename, const char* str,
                       char* buf, int buf_len);

 public:
  static const uint32_t kNetStatIntervalMs = 1000;
  static const uint32_t kNetStatStartTimeS = 20;
  static const uint32_t kStatSysCpuIntervalMs = 2000;
  static const uint32_t kNProcsMax = 4;
  // hysteresis high/low limit for cpu usage percentage
  static const uint32_t kCpuUsageFaultSetThreshold = 95;
  static const uint32_t kCpuUsageFaultHealThreshold = 90;
  static const uint32_t kCpuUsageFaultDetectStartTimeS = 20;
  // When start to collect sys info, e.g.
  // If cpu usage of any single core is higher than 90,
  // we do a snapshot of current sysinfo like cpu_sys, cpu_usr...
  // So we can get the cpu usage in duration of cpu usage increase
  // from 90 to 95
  static const uint32_t kCpuUsageCollectSysInfoThreshold = 90;

 public:
  void get_cpu_usage(uint64_t (&cpu_usage)[kNProcsMax]) {
    std::unique_lock<std::mutex> lk(mutex_);
    for (uint32_t i = 0; i < kNProcsMax; i++) {
      cpu_usage[i] = cpu_usage_[i];
    }
  }

 private:
  std::mutex mutex_;
  InnoLidar *lidar_server_;
  uint64_t lastp_time_user_cpu_;
  uint64_t lastp_time_sys_cpu_;

  uint64_t cpu_HZ_;      // the number of clock ticks per second.
  uint64_t phys_pages_;  // the number of pages of physical memory.

  char pid_stat_filename_[PATH_MAX];
  char last_stats_buffer_[512];

  uint64_t stat_lastp_time_cpu_;
  int64_t stat_lastp_now_ms;

  uint64_t stat_netstat_last_time_;
  uint64_t stat_netstat_bytes_rx_last_v_;
  uint64_t stat_netstat_bytes_tx_last_v_;
  std::string stat_netstat_interface_name_;

  // for cpu percentage of system
  uint64_t stat_sys_cpu_last_time_;
  uint64_t stat_sys_cpu_ide_last_v_[kNProcsMax];
  uint64_t stat_sys_cpu_total_last_v_[kNProcsMax];
  uint32_t cpu_usage_[kNProcsMax];
};

}  // namespace innovusion

#endif  // SDK_SYSTEM_STATS_H_
