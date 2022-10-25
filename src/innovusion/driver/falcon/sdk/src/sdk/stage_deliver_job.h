/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_STAGE_DELIVER_JOB_H_
#define SDK_STAGE_DELIVER_JOB_H_

#include "sdk_common/inno_lidar_packet.h"
#include "utils/log.h"
#include "utils/mem_pool_manager.h"

namespace innovusion {

typedef struct InnoDataPacket StageDeliverPointsJob;
typedef struct InnoDataPacket StageDeliverMessageJob;
typedef struct InnoStatusPacket StageDeliverStatusJob;

class StageDeliverPointsJobPool {
 public:
  StageDeliverPointsJobPool(const char *name,
                            unsigned int unit_number,
                            unsigned int block_number)
      : pool_(name,
              sizeof(InnoDataPacket) + sizeof(InnoBlock2) * block_number,
              unit_number,
              32)
      , payload_size_(block_number) {
  }

  InnoDataPacket *alloc() {
    return new (pool_.alloc())
        InnoDataPacket();
  }

  void free(InnoDataPacket *o) {
    return pool_.free(o);
  }

  size_t get_max_blocks_number(void) const {
    return payload_size_;
  }

 private:
  MemPool pool_;
  size_t payload_size_;
};

class StageDeliverMessageJobPool {
 public:
  StageDeliverMessageJobPool(const char *name,
                            unsigned int unit_number,
                            unsigned int message_size)
      : pool_(name,
              sizeof(InnoDataPacket) +
              sizeof(InnoMessage) + message_size,
              unit_number,
              32)
      , payload_size_(message_size) {
  }

  InnoDataPacket *alloc() {
    return new (pool_.alloc())
        InnoDataPacket();
  }

  void free(InnoDataPacket *o) {
    return pool_.free(o);
  }

  size_t get_max_message_size(void) const {
    return payload_size_;
  }

 private:
  MemPool pool_;
  size_t payload_size_;
};

class StageDeliverStatusJobPool {
 public:
  StageDeliverStatusJobPool(const char *name,
                            unsigned int unit_number)
      : pool_(name,
              sizeof(InnoStatusPacket),
              unit_number,
              32) {
  }

  InnoStatusPacket *alloc() {
    return new (pool_.alloc())
        InnoStatusPacket();
  }

  void free(InnoStatusPacket *o) {
    return pool_.free(o);
  }

 private:
  MemPool pool_;
};

class StageDeliver2Job {
 public:
  StageDeliver2Job() {
    packet_cnt = 0;
  }

 public:
  size_t packet_cnt;
  StageDeliverPointsJob *packets[0];
};

class StageDeliver2JobPool {
 public:
  StageDeliver2JobPool(const char *name,
                       unsigned int unit_number,
                       unsigned int packet_number)
      : pool_(name,
              sizeof(size_t) + sizeof(void *) * packet_number,
              unit_number,
              32) {
  }

  StageDeliver2Job *alloc() {
    return new (pool_.alloc())
        StageDeliver2Job();
  }

  void free(StageDeliver2Job *o) {
    return pool_.free(o);
  }

 private:
  MemPool pool_;
};

}  // namespace innovusion
#endif  // SDK_STAGE_DELIVER_JOB_H_
