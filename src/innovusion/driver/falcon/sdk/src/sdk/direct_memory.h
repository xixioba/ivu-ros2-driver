/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_DIRECT_MEMORY_H_
#define SDK_DIRECT_MEMORY_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>

#ifndef __MINGW64__
#include <sys/mman.h>
#else
#include "sdk/mman-win32/mman.h"
#endif

#include "utils/inno_lidar_log.h"
#include "sdk/fpga_regs_collector.h"

#define PS_BASEADDR 0x43C0000000L
#define PS_HIGHADDR 0x43C0000FFFL
#define PS_LENGTH   ((int)(PS_HIGHADDR - PS_BASEADDR + 1))

#define DMA_READ_INDEX_REG    0x044
#define DMA_WRITE_INDEX_REG   0x048

#define PACKET_START_ADDR 0x1E000000

namespace innovusion {

class MemMap {
 public:
  MemMap(uint64_t base, size_t length) {
    mem_fd_ = -1;
    map_base_ = (volatile char *)MAP_FAILED;

    length += 4095;
    length &= ~4095;
    if (length < 4096) {
      length = 4096;
    }
    bool failed = false;
#ifndef __MINGW64__
    mem_fd_ = open("/dev/mem", O_RDWR | O_SYNC);
#else
    mem_fd_ = open("/dev/mem", O_RDWR);
#endif
    if (mem_fd_ < 0) {
      inno_log_error_errno("failed to open /dev/mem %s", "");
      failed = true;
    } else {
      map_base_ = reinterpret_cast<char *>(mmap(NULL,
                                                length,
                                                PROT_READ | PROT_WRITE,
                                                MAP_SHARED,
                                                mem_fd_,
                                                base));
      if (MAP_FAILED == map_base_) {
        inno_log_error_errno("memory map failed base: 0x%" PRI_SIZEX
                            " len=%" PRI_SIZELD,
                            base, length);
        failed = true;
      }
    }
    inno_log_verify(!failed, "memmap failed");

    base_ = base;
    length_ = length;
  }

  ~MemMap() {
    if (MAP_FAILED != map_base_) {
      munmap(map_base_void_, length_);
      map_base_ = (volatile char *)MAP_FAILED;
    }

    if (mem_fd_ >= 0) {
      close(mem_fd_);
      mem_fd_ = -1;
    }
  }

  uint32_t Read(int offset) const noexcept {
    if (MAP_FAILED == map_base_) {
      inno_log_verify(false, "cannot read");
      return 0xDEADBEEF;
    }

    offset &= ~3;   // align 4 bytes
    volatile char *virt_addr = map_base_ + offset;
    return *(reinterpret_cast<volatile uint32_t *>(virt_addr));
  }

  void Write(int offset, uint32_t value) const noexcept {
    if (MAP_FAILED == map_base_) {
      inno_log_verify(false, "cannot write");
      return;
    }

    offset &= ~3;   // align 4 bytes
    volatile char *virt_addr = map_base_ + offset;
    *(reinterpret_cast<volatile uint32_t *>(virt_addr)) = value;
  }

  // read a batch of data
  void Read(char *buf, int len) const noexcept {
    if (!buf || len <= 0 || MAP_FAILED == map_base_) {
      inno_log_verify(false, "cannot Read");
      return;
    }

    memcpy(buf, (const void*)map_base_, len);
  }

 private:
  int mem_fd_;
  uint64_t base_;
  size_t length_;
  union {
    void *map_base_void_;
    volatile char *map_base_;
  };
};

class DirectMemory {
 public:
  static DirectMemory *get_instance();

 public:
  DirectMemory(DirectMemory const&) = delete;
  void operator=(DirectMemory const&) = delete;

  /**
   * @brief      read dma data
   *             this interface is like TCP read function
   *             if there is no dma data after 1 sec, return 0
   * @param buf  store data to this buffer
   * @param size MUST BE 128K
   * @return     <=0: fail  128K: success
   */
  int read(void *buf,
           int size,
           int *queue_len,
           FpgaRegsCollector* fpga_regs_collector);

 public:
  static const int kTotalSize = 4 * 1024 * 1024;
  static const int kPacketSize = 64 * 1024;
  static const int kStreamFrameCount = kTotalSize / kPacketSize;

 private:
  DirectMemory();
  ~DirectMemory();

 private:
  MemMap ps_reg_;
  MemMap *datas_[kStreamFrameCount];
};

}  // namespace innovusion

#endif  // SDK_DIRECT_MEMORY_H_
