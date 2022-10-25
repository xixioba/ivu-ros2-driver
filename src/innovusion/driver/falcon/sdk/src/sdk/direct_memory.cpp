/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "sdk/direct_memory.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <memory.h>
#include <string.h>
#ifndef __MINGW64__
#include <sys/ioctl.h>
#include <poll.h>
#endif
#include <fcntl.h>

#include <string>

#include "utils/log.h"

namespace innovusion {

DirectMemory * DirectMemory::get_instance() {
  static DirectMemory dm;
  return &dm;
}

DirectMemory::DirectMemory() : ps_reg_(PS_BASEADDR, PS_LENGTH) {
  for (size_t i = 0; i < kStreamFrameCount; i++) {
    datas_[i] = new MemMap(PACKET_START_ADDR + kPacketSize * i, kPacketSize);
    inno_log_verify(datas_[i], "memmap(%lu) failed", i);
  }
}

DirectMemory::~DirectMemory() {
  for (size_t i = 0; i < kStreamFrameCount; i++) {
    delete datas_[i];
    datas_[i] = NULL;
  }
}

int DirectMemory::read(void *buf,
                       int size,
                       int *queue_len,
                       FpgaRegsCollector* fpga_regs_collector) {
  inno_log_verify(buf, "buf is NULL");

  inno_log_verify(size == kPacketSize,
                  "direct_memory read size mismatch %d vs %d",
                  size, kPacketSize);

  unsigned int read_counter = ps_reg_.Read(DMA_READ_INDEX_REG);
  if (read_counter >= kStreamFrameCount) {
    read_counter = kStreamFrameCount  - 1;
  }

  unsigned int write_counter = ps_reg_.Read(DMA_WRITE_INDEX_REG);
  if (write_counter >= kStreamFrameCount) {
    write_counter = 0;
  }

  // check if data is ready
  // if there is data, return it, otherwise wait at most 100 ms
  unsigned int next_read_counter =
                    (read_counter + 1) & (kStreamFrameCount - 1);
  int wait_cnt = 0;
  while (next_read_counter == write_counter) {
    usleep(500);
    write_counter = ps_reg_.Read(DMA_WRITE_INDEX_REG);
    ++wait_cnt;
    if (fpga_regs_collector && wait_cnt % 64 == 0) {
      fpga_regs_collector->\
      collect(std::string("can not read: "),
              kInnoPLRegsReadTimeOut,
              sizeof(kInnoPLRegsReadTimeOut) / sizeof(struct RegInfo),
              kInnoPSRegsReadTimeOut,
              sizeof(kInnoPSRegsReadTimeOut) / sizeof(struct RegInfo));
    }
    if (wait_cnt > 200) {
      // timeout
      return 0;
    }
  }
  read_counter = next_read_counter;

  if (write_counter > next_read_counter) {
    *queue_len = write_counter - next_read_counter;
  } else {
    *queue_len = write_counter + kStreamFrameCount - next_read_counter;
  }

  ps_reg_.Write(DMA_READ_INDEX_REG, read_counter);

  // MemMap data(PACKET_START_ADDR + kPacketSize * read_counter, kPacketSize);
  datas_[read_counter]->Read(reinterpret_cast<char *>(buf), size);

  return size;
}

}  // namespace innovusion
