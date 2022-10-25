/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_STAGE_ANGLE_JOB_H_
#define SDK_STAGE_ANGLE_JOB_H_

#include "sdk/scan_lines.h"
#include "utils/log.h"
#include "utils/mem_pool_manager.h"

namespace innovusion {

typedef ScanLines StageAngleJob;

class StageAngleJobPool {
 public:
  StageAngleJobPool(const char *name,
                    unsigned int unit_number,
                    unsigned int block_number =
                    ScanLine::kMaxBlocksBetweenP +
                    ScanLine::kMaxBlocksInScanLine * 2,
                    uint32_t encodes_pre_polygon = 1)
      : pool_(name,
              sizeof(StageAngleJob) +
              sizeof(RawBlock) * block_number,
              unit_number,
              64)
      , payload_size_(block_number)
      , encodes_pre_polygon_(encodes_pre_polygon) {
    inno_log_info("sizeof(StageAngleJob)=%" PRI_SIZELU " 0x%" PRI_SIZELX
                  "sizeof(RawBlock)=%" PRI_SIZELU " 0x%" PRI_SIZELX
                  "sizeof(RawChannelPoint)=%" PRI_SIZELU " 0x%" PRI_SIZELX,
                  sizeof(StageAngleJob), sizeof(StageAngleJob),
                  sizeof(RawBlock), sizeof(RawBlock),
                  sizeof(RawChannelPoint), sizeof(RawChannelPoint));
  }

  StageAngleJob *alloc() {
    return new (pool_.alloc())
        StageAngleJob(payload_size_, encodes_pre_polygon_);
  }

  void free(StageAngleJob *o) {
    return pool_.free(o);
  }

 private:
  MemPool pool_;
  size_t payload_size_;
  uint32_t encodes_pre_polygon_;
};

}  // namespace innovusion
#endif  // SDK_STAGE_ANGLE_JOB_H_
