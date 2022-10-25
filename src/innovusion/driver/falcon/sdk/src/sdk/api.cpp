/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include <stdio.h>
#include <unistd.h>

#include <limits>

#include "sdk_client/lidar_client.h"
#include "sdk_common/lidar_base.h"
#include "sdk_common/inno_lidar_api.h"
#include "sdk_common/inno_lidar_other_api.h"
#include "sdk/lidar.h"
#include "utils/log.h"
#include "utils/utils.h"

int inno_lidar_open_live(const char *name,
                         const char *lidar_ip,
                         uint16_t port,
                         enum InnoLidarProtocol protocol,
                         uint16_t udp_port) {
  innovusion::InnoLidarBase *l = NULL;
  switch (protocol) {
    case INNO_LIDAR_PROTOCOL_RAW_TCP:
    case INNO_LIDAR_PROTOCOL_RAW_MEM:
      l = new innovusion::InnoLidar(name, lidar_ip, port,
                                    protocol == INNO_LIDAR_PROTOCOL_RAW_TCP);
      break;
    case INNO_LIDAR_PROTOCOL_PCS_UDP:
      l = new innovusion::InnoLidarClient(name, lidar_ip, port,
                                          false, udp_port);
      break;
    case INNO_LIDAR_PROTOCOL_PCS_TCP:
      l = new innovusion::InnoLidarClient(name, lidar_ip, port,
                                          true, udp_port);
      break;
    default:
      inno_log_panic("invalid protocol %d", protocol);
  }

  if (l) {
    int ret = innovusion::InnoLidarBase::add_lidar(l);
    return ret;
  } else {
    return -1;
  }
}

int inno_lidar_open_file(const char *name,
                         const char *filename,
                         bool raw_format,
                         int play_rate,
                         int rewind,
                         int64_t skip) {
  innovusion::InnoLidarBase *l = NULL;

  // check whether the file exists
  int fd = innovusion::InnoUtils::open_file(filename,
                                            O_RDONLY, 0);
  if (fd < 0) {
    return -1;
  } else {
    close(fd);
  }

  if (raw_format) {
    l = new innovusion::InnoLidar(name, filename, play_rate, rewind, skip);
  } else {
    l = new innovusion::InnoLidarClient(name, filename,
                                        play_rate, rewind, skip);
  }
  if (l) {
    int ret = innovusion::InnoLidarBase::add_lidar(l);
    return ret;
  } else {
    return -1;
  }
}
