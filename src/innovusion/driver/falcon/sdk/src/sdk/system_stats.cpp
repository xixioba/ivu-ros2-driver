/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "sdk/system_stats.h"
#include "sdk/system_proc_structs.h"

#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#if !(defined(__APPLE__) || defined(__MINGW64__))
#include <sys/sysinfo.h>
#endif
#ifdef __MINGW64__
#include <windows.h>
#endif
#include <unistd.h>

#include "sdk/lidar.h"
#include "utils/consumer_producer.h"
#include "utils/log.h"

namespace innovusion {

SystemStats::SystemStats(InnoLidar *l)
    : ResourceStats(l) {
  lidar_server_ = l;

  lastp_time_user_cpu_ = 0;
  lastp_time_sys_cpu_ = 0;

  stat_lastp_time_cpu_ = 0;
  stat_lastp_now_ms = lidar_->get_monotonic_raw_time_ms();

  stat_netstat_last_time_ = 0;
  stat_netstat_bytes_rx_last_v_ = 0;
  stat_netstat_bytes_tx_last_v_ = 0;

  stat_sys_cpu_last_time_ = 0;
  memset(stat_sys_cpu_ide_last_v_, 0, sizeof(stat_sys_cpu_ide_last_v_));
  memset(stat_sys_cpu_total_last_v_, 0, sizeof(stat_sys_cpu_total_last_v_));

  pid_t pid = getpid();

#ifdef __MINGW64__
  MEMORYSTATUSEX status;
  status.dwLength = sizeof(status);
  GlobalMemoryStatusEx(&status);
  cpu_HZ_ = CLK_TCK;
  phys_pages_ =  status.ullTotalPhys/4096;
#else
  cpu_HZ_ = sysconf(_SC_CLK_TCK);
  phys_pages_ = sysconf(_SC_PHYS_PAGES);
#endif

  inno_log_verify(cpu_HZ_ != 0, "sysconf(_SC_CLK_TCK) return 0.");
  inno_log_verify(phys_pages_ != 0, "sysconf(_SC_PHYS_PAGES) return 0.");

  snprintf(pid_stat_filename_,
           sizeof(pid_stat_filename_),
           "/proc/%d/stat", pid);
  {
    std::unique_lock<std::mutex> lk(mutex_);
    last_stats_buffer_[0] = 0;
    for (uint32_t i = 0; i < kNProcsMax; i++) {
      cpu_usage_[i] = 0;
    }
  }
}

void SystemStats::get_extra_info_(char *buf, size_t buf_size,
                                  double time_diff) {
  ProcPidStat stat(lidar_->get_name(), pid_stat_filename_);
  if (!stat.is_valid) {
    return;
  }

  int ret = snprintf(
      buf, buf_size,
      "user_cpu=%.2fs, sys_cpu=%.2fs, user_cpu_p=%.2f%%, "
      "sys_cpu_p=%.2f%%, "
      "vsize=%" PRI_SIZEU ", rss=%" PRI_SIZEU ", "
      "mq=%d/%d, "
      "q=%d/%d/%d/%d/%d/%d/%d, "
      "q_max=%d/%d/%d/%d/%d/%d/%d, "
      "q_blocked=%" PRI_SIZELU "/%" PRI_SIZELU
      "/%" PRI_SIZELU "/%" PRI_SIZELU "/%"
      PRI_SIZELU "/%" PRI_SIZELU "/%" PRI_SIZELU ", "
      "q_dropped=%" PRI_SIZEU "/%" PRI_SIZEU
      "/%" PRI_SIZELU "/%" PRI_SIZELU "/%" PRI_SIZELU
      "/%" PRI_SIZELU "/%" PRI_SIZELU "/%" PRI_SIZELU "/%" PRI_SIZELU,
      stat.time_user_cpu * 1.0 / cpu_HZ_,
      stat.time_sys_cpu * 1.0 / cpu_HZ_,
      (stat.time_user_cpu - lastp_time_user_cpu_) * 100.0/cpu_HZ_/time_diff,
      (stat.time_sys_cpu - lastp_time_sys_cpu_) * 100.0/cpu_HZ_/time_diff,
      stat.vsize, stat.rss,
      lidar_server_->stage_read_->get_queue_len(),
      lidar_server_->stage_read_->get_max_queue_len(),
      lidar_server_->cp_read_->queue_length(),
      lidar_server_->cp_signal_->queue_length(),
      lidar_server_->cp_angle_->queue_length(),
      lidar_server_->cp_noise_filter_phase0_->queue_length(),
      lidar_server_->cp_noise_filter_phase1_->queue_length(),
      lidar_server_->cp_deliver_->queue_length(),
      lidar_server_->cp_help_->queue_length(),
      lidar_server_->cp_read_->max_queue_length(),
      lidar_server_->cp_signal_->max_queue_length(),
      lidar_server_->cp_angle_->max_queue_length(),
      lidar_server_->cp_noise_filter_phase0_->max_queue_length(),
      lidar_server_->cp_noise_filter_phase1_->max_queue_length(),
      lidar_server_->cp_deliver_->max_queue_length(),
      lidar_server_->cp_help_->max_queue_length(),
      lidar_server_->cp_read_->blocked_job_count(),
      lidar_server_->cp_signal_->blocked_job_count(),
      lidar_server_->cp_angle_->blocked_job_count(),
      lidar_server_->cp_noise_filter_phase0_->blocked_job_count(),
      lidar_server_->cp_noise_filter_phase1_->blocked_job_count(),
      lidar_server_->cp_deliver_->blocked_job_count(),
      lidar_server_->cp_help_->blocked_job_count(),
      lidar_server_->stat_adc_data_drop_[0].total,
      lidar_server_->stat_adc_data_drop_[1].total,
      lidar_server_->cp_read_->dropped_job_count(),
      lidar_server_->cp_signal_->dropped_job_count(),
      lidar_server_->cp_angle_->dropped_job_count(),
      lidar_server_->cp_noise_filter_phase0_->dropped_job_count(),
      lidar_server_->cp_noise_filter_phase1_->dropped_job_count(),
      lidar_server_->cp_deliver_->dropped_job_count(),
      lidar_server_->cp_help_->dropped_job_count());

  if (ret >= ssize_t(buf_size)) {
    inno_log_error("buffer too small %d", ret);
    buf[0] = 0;
    return;
  }

  {
    std::unique_lock<std::mutex> lk(mutex_);
    strncpy(last_stats_buffer_, buf, sizeof(last_stats_buffer_));
    last_stats_buffer_[sizeof(last_stats_buffer_) - 1] = 0;
  }

  lastp_time_user_cpu_ = stat.time_user_cpu;
  lastp_time_sys_cpu_ = stat.time_sys_cpu;
  return;
}

void SystemStats::get_last_info_buffer(char *buf, size_t buf_size) {
  std::unique_lock<std::mutex> lk(mutex_);
  inno_log_verify(buf && buf_size > 0, "buf_size=%" PRI_SIZELU "", buf_size);
  strncpy(buf, last_stats_buffer_, buf_size);
  buf[buf_size - 1] = 0;
  return;
}


void SystemStats::get_sys_stats(InnoStatusCounters *counters) {
  inno_log_verify(counters, "counters is NULL.");

  //
  constexpr uint16_t PERCENT = 100;
  constexpr uint16_t MS_PRE_SEC = 1000;

  int64_t now_ms = lidar_->get_monotonic_raw_time_ms();
#if (defined(__APPLE__) || defined(__MINGW64__))
  uint32_t info_uptime =
      std::chrono::steady_clock::now().time_since_epoch().count() / 1000000000;
#else
  struct sysinfo info;
  inno_log_verify(sysinfo(&info) == 0, "get sysinfo failed!");
#endif
  ProcPidStat stat(lidar_->get_name(), pid_stat_filename_);
  if (stat.is_valid) {
    // The time the process started after system boot.
    uint32_t start_time = static_cast<uint32_t>(stat.start_time / cpu_HZ_);

#if (defined(__APPLE__) || defined(__MINGW64__))
    counters->power_up_time_in_second = info_uptime;
    counters->process_up_time_in_second = info_uptime - start_time;
#else
    counters->power_up_time_in_second = info.uptime;
    counters->process_up_time_in_second = info.uptime - start_time;
#endif

    int64_t diff_ms = stat_lastp_time_cpu_ == 0
        ? counters->process_up_time_in_second * MS_PRE_SEC
        : (now_ms - stat_lastp_now_ms);
    if (diff_ms != 0) {
      counters->cpu_percentage = static_cast<uint16_t>(
          PERCENT * MS_PRE_SEC * (stat.time_cpu - stat_lastp_time_cpu_) /
          cpu_HZ_ / diff_ms);
    }

    stat_lastp_now_ms = now_ms;
    stat_lastp_time_cpu_ = stat.time_cpu;

    //
    counters->mem_percentage =
        static_cast<uint16_t>(PERCENT * stat.rss / phys_pages_);
  }
  uint64_t time_diff;
  if (lidar_server_->is_live_direct_memory_lidar_()) {
    time_diff = now_ms - stat_netstat_last_time_;
#if (defined(__APPLE__) || defined(__MINGW64__))
    if (info_uptime > kNetStatStartTimeS && time_diff >= kNetStatIntervalMs) {
#else
    if (info.uptime > kNetStatStartTimeS && time_diff >= kNetStatIntervalMs) {
#endif
      ProcNetDevStat netstat =
          ProcNetDevStat(lidar_->get_name(),
                        stat_netstat_interface_name_.c_str());
      if (netstat.is_valid) {
        if (stat_netstat_last_time_ > 0 && time_diff > 0) {
          counters->netstat_rx_speed_kBps = static_cast<uint16_t>(MS_PRE_SEC *
              (netstat.rx_bytes - stat_netstat_bytes_rx_last_v_)
              / time_diff / 1024);
          counters->netstat_tx_speed_kBps = static_cast<uint16_t>(MS_PRE_SEC *
              (netstat.tx_bytes - stat_netstat_bytes_tx_last_v_)
              / time_diff / 1024);
          counters->netstat_rx_drop = netstat.rx_drop;
          counters->netstat_tx_drop = netstat.tx_drop;
          counters->netstat_rx_err = netstat.rx_errs;
          counters->netstat_tx_err = netstat.tx_errs;
        }
        stat_netstat_bytes_rx_last_v_ = netstat.rx_bytes;
        stat_netstat_bytes_tx_last_v_ = netstat.tx_bytes;
      }
      stat_netstat_last_time_ = now_ms;
      detect_network_();
    }
  }

  time_diff = now_ms - stat_sys_cpu_last_time_;
#if (defined(__APPLE__) || defined(__MINGW64__))
  if (info_uptime > kCpuUsageFaultDetectStartTimeS &&
      time_diff > kStatSysCpuIntervalMs) {
#else
  if (info.uptime > kCpuUsageFaultDetectStartTimeS &&
      time_diff > kStatSysCpuIntervalMs) {
#endif
    ProcCpuStat cpu_stat = ProcCpuStat(lidar_->get_name());
    if (cpu_stat.is_valid) {
      uint16_t max_cpu = 0;
      int index = -1;
      for (uint32_t i = 0; i < kNProcsMax; ++i) {
        uint64_t total_cpu = cpu_stat.user[i] +
                             cpu_stat.nice[i] +
                             cpu_stat.system[i] +
                             cpu_stat.idle[i] +
                             cpu_stat.iowait[i] +
                             cpu_stat.irq[i] +
                             cpu_stat.softirq[i];

        if (stat_sys_cpu_last_time_ > 0) {
          // set meaningful value to counters
          // cpu_usage = 1 - (idle - idel_last) / (total - total_last)
          // total = user + nice + system + idle + iowait + irq + softirq
          counters->sys_cpu_percentage[i] = PERCENT - PERCENT *
              (cpu_stat.idle[i] - stat_sys_cpu_ide_last_v_[i]) /
              (total_cpu - stat_sys_cpu_total_last_v_[i]);
          if (counters->sys_cpu_percentage[i] > max_cpu) {
            max_cpu = counters->sys_cpu_percentage[i];
            index = i;
          }
        }
        stat_sys_cpu_last_time_ = now_ms;
        stat_sys_cpu_total_last_v_[i] = total_cpu;
        stat_sys_cpu_ide_last_v_[i] = cpu_stat.idle[i];
      }
      {
        std::unique_lock<std::mutex> lk(mutex_);
        for (uint32_t i = 0; i < kNProcsMax; i++) {
          cpu_usage_[i] = counters->sys_cpu_percentage[i];
        }
      }

      if (stat_sys_cpu_last_time_ > 0) {
        // print stage info
        if (max_cpu >= kCpuUsageFaultSetThreshold &&
            !lidar_server_->get_current_fault_status(
            INNO_LIDAR_IN_FAULT_CPULOAD_HIGH)) {
          lidar_server_->print_stats();
          if (stat.is_valid) {
            inno_log_info("vsize=%" PRI_SIZEU ", rss=%"
              PRI_SIZEU, stat.vsize, stat.rss);
          }
          inno_log_info(
              "mq=%d/%d "
              "q=%d/%d/%d/%d/%d/%d/%d, "
              "q_max=%d/%d/%d/%d/%d/%d/%d, "
              "q_blocked=%" PRI_SIZELU "/%"
              PRI_SIZELU "/%" PRI_SIZELU "/%"
              PRI_SIZELU "/%" PRI_SIZELU "/%"
              PRI_SIZELU "/%" PRI_SIZELU ", "
              "q_dropped=%" PRI_SIZEU "/%" PRI_SIZEU
              "/%" PRI_SIZELU "/%" PRI_SIZELU "/%" PRI_SIZELU
              "/%" PRI_SIZELU "/%" PRI_SIZELU "/%"
              PRI_SIZELU "/%" PRI_SIZELU,
              lidar_server_->stage_read_->get_queue_len(),
              lidar_server_->stage_read_->get_max_queue_len(),
              lidar_server_->cp_read_->queue_length(),
              lidar_server_->cp_signal_->queue_length(),
              lidar_server_->cp_angle_->queue_length(),
              lidar_server_->cp_noise_filter_phase0_->queue_length(),
              lidar_server_->cp_noise_filter_phase1_->queue_length(),
              lidar_server_->cp_deliver_->queue_length(),
              lidar_server_->cp_help_->queue_length(),
              lidar_server_->cp_read_->max_queue_length(),
              lidar_server_->cp_signal_->max_queue_length(),
              lidar_server_->cp_angle_->max_queue_length(),
              lidar_server_->cp_noise_filter_phase0_->max_queue_length(),
              lidar_server_->cp_noise_filter_phase1_->max_queue_length(),
              lidar_server_->cp_deliver_->max_queue_length(),
              lidar_server_->cp_help_->max_queue_length(),
              lidar_server_->cp_read_->blocked_job_count(),
              lidar_server_->cp_signal_->blocked_job_count(),
              lidar_server_->cp_angle_->blocked_job_count(),
              lidar_server_->cp_noise_filter_phase0_->blocked_job_count(),
              lidar_server_->cp_noise_filter_phase1_->blocked_job_count(),
              lidar_server_->cp_deliver_->blocked_job_count(),
              lidar_server_->cp_help_->blocked_job_count(),
              lidar_server_->stat_adc_data_drop_[0].total,
              lidar_server_->stat_adc_data_drop_[1].total,
              lidar_server_->cp_read_->dropped_job_count(),
              lidar_server_->cp_signal_->dropped_job_count(),
              lidar_server_->cp_angle_->dropped_job_count(),
              lidar_server_->cp_noise_filter_phase0_->dropped_job_count(),
              lidar_server_->cp_noise_filter_phase1_->dropped_job_count(),
              lidar_server_->cp_deliver_->dropped_job_count(),
              lidar_server_->cp_help_->dropped_job_count());
           log_fault_(INNO_LIDAR_IN_FAULT_CPULOAD_HIGH, "");
        }
        lidar_server_->set_raw_fault(INNO_LIDAR_IN_FAULT_CPULOAD_HIGH,
                                     max_cpu >= kCpuUsageFaultSetThreshold,
                                    "max_cpu: %d, index: %d, threshold: %u",
                                     max_cpu, index,
                                     kCpuUsageFaultSetThreshold);
        lidar_server_->heal_raw_fault(INNO_LIDAR_IN_FAULT_CPULOAD_HIGH,
                                      max_cpu <= kCpuUsageFaultHealThreshold,
                                      "INNO_LIDAR_IN_FAULT_CPULOAD_HIGH heals");
      }
    }
  }
}


void SystemStats::init_config(const StatusReportConfig *config) {
  stat_netstat_interface_name_ = config->interface_name;
}

void SystemStats::log_fault_(enum InnoLidarInFault fault_id,
                             const char *fault_info) {
  std::string cmd("/app/pointcloud/log_fault.sh " +
                  std::to_string(fault_id) + " \"" + fault_info + " \"");
//  std::thread t1([cmd]() {
  FILE *fp = popen(cmd.c_str(), "r");
  if (fp == nullptr) {
    inno_log_error("exec '%s' failed", cmd.c_str());
    return;
  }
  char line[300];
  std::string print_info;
  while (fgets(line, sizeof(line), fp) != nullptr) {
    print_info += line;
  }
  inno_log_error("%s", print_info.c_str());
  pclose(fp);
//  });
//  t1.detach();
}

int SystemStats::detect_network_() {
  int ret_op = -1;
  int ret_carr = -1;

  const char *operstate = "/sys/class/net/eth0/operstate";
  const char *carrier = "/sys/class/net/eth0/carrier";
  /*
   * Up-Ready to pass packets
   * Down-If admin status is down, then operational status should be down
   * Testing-In test mode, no operational packets can be passed
   * Unknown-Status can not be determined for some reason
   * Dormant-Interface is waiting for external actions
   * NotPresent-Some component is missing, typically hardware
   * LowerLayerDown-Down due to state of lower layer interface
   *
   * ref: https://www.kernel.org/doc/Documentation/ABI/testing/sysfs-class-net
   */
  const char *operstate_up = "up";
  // 1: physical link is up  0:physical link is down
  const char *carrier_1 = "1";
  char op_buffer[512] = {'\0'};
  ret_op = find_str_infile_(operstate, operstate_up,
                            op_buffer, sizeof(op_buffer));
  if (ret_op < 0) {
    return -1;
  }

  char carr_buffer[512] = {'\0'};
  ret_carr = find_str_infile_(carrier, carrier_1,
                              carr_buffer, sizeof(carr_buffer));
  if (ret_carr < 0) {
    return -1;
  }

  // avoid invocation of log_fault_ function every time;
  static bool s_fault_set = false;

  if (ret_op && ret_carr) {
    s_fault_set = false;
    lidar_server_->heal_raw_fault(INNO_LIDAR_IN_FAULT_NETWORK2, true,
                                 "INNO_LIDAR_IN_FAULT_NETWORK2 heals");
  } else {
    char info[2048]{0};
    snprintf(info, sizeof(info) - 1,
             "operstate info: %s carrier info: %s netwrok disconnect fault "
             "ret_op = %d, ret_carr = %d",
             op_buffer, carr_buffer, ret_op, ret_carr);

    lidar_server_->set_raw_fault(INNO_LIDAR_IN_FAULT_NETWORK2, true,
                                "%s", info);
    if (!s_fault_set) {
      s_fault_set = true;
      log_fault_(INNO_LIDAR_IN_FAULT_NETWORK2, info);
    }
  }
  return 0;
}


int SystemStats::find_str_infile_(const char* filename, const char* str,
                                  char* buf, int buf_len) {
  char    buffer[256] = {0};
  FILE    *fd = NULL;
  int     ret = -1;
  int     len = -1;
  fd = fopen(filename, "r");
  if (!fd) {
    inno_log_error("cannot open %s", filename);
    return -1;
  }
  len = fread(buffer, sizeof(char), sizeof(buffer), fd);
  if (len >= 0) {
    // can't find
    if (strstr(buffer, str) == NULL) {
      ret = 0;
      int r = snprintf(buf, buf_len,
                      "%s can't find the %s in %s.",
                       filename, str, buffer);
      if (r > buf_len) {
        buf = NULL;
      }
    } else {
      ret = 1;
    }
  } else {
    inno_log_error_errno("cannot read the %s", filename);
  }
  fclose(fd);
  return ret;
}

}  // namespace innovusion
