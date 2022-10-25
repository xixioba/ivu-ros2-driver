/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_SYSTEM_PROC_STRUCTS_H_
#define SDK_SYSTEM_PROC_STRUCTS_H_

#include <errno.h>
#include <fcntl.h>
#ifndef __MINGW64__
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#else
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

// if typedef doesn't exist (msvc, blah)
typedef intptr_t ssize_t;

ssize_t getline(char **lineptr, size_t *n, FILE *stream) {
  size_t pos;
  int c;

  if (lineptr == NULL || stream == NULL || n == NULL) {
    errno = EINVAL;
    return -1;
  }

  c = getc(stream);
  if (c == EOF) {
    return -1;
  }

  if (*lineptr == NULL) {
    *lineptr = malloc(128);
    if (*lineptr == NULL) {
      return -1;
    }
    *n = 128;
  }

  pos = 0;
  while (c != EOF) {
    if (pos + 1 >= *n) {
      size_t new_size = *n + (*n >> 2);
      if (new_size < 128) {
        new_size = 128;
      }
      char *new_ptr = realloc(*lineptr, new_size);
      if (new_ptr == NULL) {
        return -1;
      }
      *n = new_size;
      *lineptr = new_ptr;
    }

    ((unsigned char *)(*lineptr))[pos++] = c;
    if (c == '\n') {
      break;
    }
    c = getc(stream);
  }

  (*lineptr)[pos] = '\0';
  return pos;
}

#include <ws2tcpip.h>
#endif
#include <sys/stat.h>
#include <utils/inno_lidar_log.h>
#include "sdk/system_stats.h"

namespace innovusion {

#define checkfret(x, expect_ret) if (fret != (expect_ret)) { \
inno_log_warning_errno("%s read %s failed %d", lidar_name, x, fret);}
#define readone(x) do { \
fret = fscanf(input, "%ld ", x); checkfret(#x, 1) } while (0)
#define readunsigned(x) do { \
fret = fscanf(input, "%" PRI_SIZEU " ", x); checkfret(#x, 1) } while (0)
#define readstr(x) do { \
fret = fscanf(input, "%256s ", x); checkfret(#x, 1) } while (0)
#define readchar(x) do { \
fret = fscanf(input, "%c ", x); checkfret(#x, 1) } while (0)
#define skiponeline(line, len) do {                      \
(line) = NULL;                                        \
(len) = 0;                                            \
fret = getline(&(line), &(len), input);               \
if (line) { free(line); (line) = NULL; (len) = 0; }   \
} while (0)

// porc structures
//
// /proc/[PID]/stat
//
struct ProcPidStat {
  // file fields
  uint64_t pid;
  char tcomm[PATH_MAX];
  char state;

  uint64_t ppid;
  uint64_t pgid;
  uint64_t sid;
  uint64_t tty_nr;
  uint64_t tty_pgrp;

  uint64_t flags;
  uint64_t min_flt;
  uint64_t cmin_flt;
  uint64_t maj_flt;
  uint64_t cmaj_flt;
  uint64_t utime;
  uint64_t stimev;

  uint64_t cutime;
  uint64_t cstime;
  uint64_t priority;
  uint64_t nicev;
  uint64_t num_threads;
  uint64_t it_real_value;

  uint64_t start_time;

  uint64_t vsize;
  uint64_t rss;
  uint64_t rsslim;

  // calculated fields
  uint64_t time_user_cpu;
  uint64_t time_sys_cpu;
  uint64_t time_cpu;
  //
  bool is_valid;

  ProcPidStat(const char* lidar_name, const char *proc_filename) {
#ifndef __linux__
  is_valid = false;
  return;
#else
    FILE *input = fopen(proc_filename, "r");
    if (!input) {
      inno_log_error_errno("%s cannot open %s", lidar_name, proc_filename);
      is_valid = false;
      return;
    }

    int fret;
    readunsigned(&pid);
    readstr(tcomm);
    readchar(&state);
    readunsigned(&ppid);
    readunsigned(&pgid);
    readunsigned(&sid);
    readunsigned(&tty_nr);
    readunsigned(&tty_pgrp);
    readunsigned(&flags);
    readunsigned(&min_flt);
    readunsigned(&cmin_flt);
    readunsigned(&maj_flt);
    readunsigned(&cmaj_flt);
    readunsigned(&utime);
    readunsigned(&stimev);
    readunsigned(&cutime);
    readunsigned(&cstime);
    readunsigned(&priority);
    readunsigned(&nicev);
    readunsigned(&num_threads);
    readunsigned(&it_real_value);
    readunsigned(&start_time);
    readunsigned(&vsize);
    readunsigned(&rss);
    readunsigned(&rsslim);
    fclose(input);

    //
    time_user_cpu = cutime + utime;
    time_sys_cpu = cstime + stimev;
    time_cpu = time_user_cpu + time_sys_cpu;

    //
    is_valid = true;
#endif
  }
};

/*
 * file: /proc/net/dev
 * fields: 'cat /proc/net/dev' to check
 */
struct ProcNetDevStat{
  // file fields
#define MAX_INTERFACE_LEN 257
  char interface_[MAX_INTERFACE_LEN];
  uint64_t rx_bytes;
  uint64_t rx_packets;
  uint64_t rx_errs;
  uint64_t rx_drop;
  uint64_t rx_fifo;
  uint64_t rx_frame;
  uint64_t rx_compressed;
  uint64_t rx_mulitcast;

  uint64_t tx_bytes;
  uint64_t tx_packets;
  uint64_t tx_errs;
  uint64_t tx_drop;
  uint64_t tx_fifo;
  uint64_t tx_frame;
  uint64_t tx_compressed;
  uint64_t tx_mulitcast;

  bool is_valid = false;
  ProcNetDevStat(const char* lidar_name, const char *dev_name) {
    const char *file_name = "/proc/net/dev";
    static bool interface_exist = true;
    if (!interface_exist) {
      return;  // is_valid == false;
    }
    FILE *input = fopen(file_name, "r");
    if (!input) {
      inno_log_error_errno("%s cannot open %s", lidar_name, file_name);
      is_valid = false;
      return;
    }

    int fret;
    // read fields
    char *drop_line = NULL;
    size_t drop_len = 0;
    skiponeline(drop_line, drop_len);
    skiponeline(drop_line, drop_len);
    interface_exist = false;
    while (true) {
      readstr(interface_);
      if (fret == EOF) {
        break;
      }
      char dev_name_tmp[MAX_INTERFACE_LEN] = {0};
      snprintf(dev_name_tmp, sizeof(dev_name_tmp), "%s:", dev_name);
      if (strcmp(dev_name_tmp, interface_) != 0) {
        skiponeline(drop_line, drop_len);
        continue;
      }
      interface_exist = true;
      readunsigned(&rx_bytes);
      readunsigned(&rx_packets);
      readunsigned(&rx_errs);
      readunsigned(&rx_drop);
      readunsigned(&rx_fifo);
      readunsigned(&rx_frame);
      readunsigned(&rx_compressed);
      readunsigned(&rx_mulitcast);

      readunsigned(&tx_bytes);
      readunsigned(&tx_packets);
      readunsigned(&tx_errs);
      readunsigned(&tx_drop);
      readunsigned(&tx_fifo);
      readunsigned(&tx_frame);
      readunsigned(&tx_compressed);
      readunsigned(&tx_mulitcast);
      break;
    }
    fclose(input);
    if (fret == EOF) {
      is_valid = false;
      return;
    }

    // calculate fields

    is_valid = true;
  }
};

/*
 * 2nd-5th line of /proc/stat
 * example:
 *           id   user nice system idle iowait irq softirq
 * 1st line: cpu  1601138 0 281481 2783778 103 0 18686 0 0 0
 *           cpu0 375598 0 131276 647515 0 0 10201 0 0 0
 *           cpu1 367599 0 52654 747087 27 0 2851 0 0 0
 *           cpu2 405340 0 48718 716907 49 0 2708 0 0 0
 *           cpu3 452600 0 48832 672267 25 0 2924 0 0 0
 */

struct ProcCpuStat {
  // file fileds
  #define ID_LEN 257
  char name[ID_LEN];
  uint64_t user[SystemStats::kNProcsMax];
  uint64_t nice[SystemStats::kNProcsMax];
  uint64_t system[SystemStats::kNProcsMax];
  uint64_t idle[SystemStats::kNProcsMax];
  uint64_t iowait[SystemStats::kNProcsMax];
  uint64_t irq[SystemStats::kNProcsMax];
  uint64_t softirq[SystemStats::kNProcsMax];
  bool is_valid;

  explicit ProcCpuStat(const char *lidar_name) {
  #ifndef __linux__
    is_valid = false;
    return;
  #else
    const char *proc_filename = "/proc/stat";
    FILE *input = fopen(proc_filename, "r");
    if (!input) {
      inno_log_error_errno("%s cannot open %s", lidar_name, proc_filename);
      is_valid = false;
      return;
    }

    int fret;
    char *drop_line = NULL;
    size_t drop_len = 0;
    for (uint32_t i = 0; i < SystemStats::kNProcsMax; ++i) {
      skiponeline(drop_line, drop_len);
      readstr(name);
      readunsigned(&user[i]);
      readunsigned(&nice[i]);
      readunsigned(&system[i]);
      readunsigned(&idle[i]);
      readunsigned(&iowait[i]);
      readunsigned(&irq[i]);
      readunsigned(&softirq[i]);
    }

    fclose(input);
    is_valid = true;
#endif
  }
};
}  // namespace innovusion

#endif  // SDK_SYSTEM_PROC_STRUCTS_H_
