/*
 *  Copyright (C) 2021 Innovusion Inc.
 *
 *  License: BSD Software License Agreement
 *
 *  $Id$
 */

#include "sdk/reg.h"

#include <fcntl.h>

#ifndef __MINGW64__
#include <sys/mman.h>
#else
#include "sdk/mman-win32/mman.h"
int getpagesize() {
  return 4096;
}
#endif

#include <unistd.h>

#include <algorithm>

#include "utils/log.h"

namespace innovusion {
Reg *Reg::r_ = NULL;
std::mutex Reg::static_mutex_;
const uint16_t Reg::ch_base_addr_[kMaxChannelNum] = {
  kChaBaseAddr, kChbBaseAddr,
  kChcBaseAddr, kChdBaseAddr };

Reg::Reg() {
  mem_fd_ = -1;
  ps_map_base_ = (volatile char *)MAP_FAILED;
  pl_map_base_ = (volatile char *)MAP_FAILED;

#ifndef __MINGW64__
  mem_fd_ = open("/dev/mem", O_RDWR | O_SYNC);
#else
  mem_fd_ = open("/dev/mem", O_RDWR);
#endif
  if (mem_fd_ < 0) {
    inno_log_error_errno("failed to open /dev/mem %s", "");
  }

  ps_map_base_ = reinterpret_cast<char *>(mmap(NULL,
                                               std::max(getpagesize(),
                                                        PS_LENGTH),
                                               PROT_READ | PROT_WRITE,
                                               MAP_SHARED,
                                               mem_fd_,
                                               PS_BASEADDR));
  if (MAP_FAILED == ps_map_base_) {
    inno_log_error_errno("ps memory map failed %s", "");
  }

  pl_map_base_ = reinterpret_cast<char *>(mmap(NULL,
                                               std::max(getpagesize(),
                                                        PL_LENGTH),
                                               PROT_READ | PROT_WRITE,
                                               MAP_SHARED,
                                               mem_fd_,
                                               PL_BASEADDR));
  if (MAP_FAILED == pl_map_base_) {
    inno_log_error_errno("pl memory map failed %s", "");
  }
}

Reg::~Reg() {
  if (MAP_FAILED != ps_map_base_) {
    munmap(ps_map_base_void_,
           std::max(getpagesize(), PS_LENGTH));
    ps_map_base_ = (volatile char *)MAP_FAILED;
  }

  if (MAP_FAILED != pl_map_base_) {
    munmap(pl_map_base_void_,
           std::max(getpagesize(), PL_LENGTH));
    pl_map_base_ = (volatile char *)MAP_FAILED;
  }

  if (mem_fd_) {
    close(mem_fd_);
    mem_fd_ = -1;
  }
}

int Reg::ps_read_(uint16_t off, uint32_t *value) {
  if (is_ps_invalid_()) {
    return -2;
  }

  if (off >= PS_LENGTH) {
    inno_log_error("PS read error, offset 0x%X >= 0x%X", off, PS_LENGTH);
    return -3;
  }

  if (off & 3) {
    inno_log_error("PS read error, offset 0x%X must align 4", off);
    return -4;
  }

  volatile char *virt_addr = ps_map_base_ + off;
  *value = *(volatile uint32_t *)virt_addr;
  return 0;
}

int Reg::ps_write_(uint16_t off, uint32_t data) {
  if (is_ps_invalid_()) {
    return -2;
  }

  if (off >= PS_LENGTH) {
    inno_log_error("PS write error, offset 0x%X >= 0x%X", off, PS_LENGTH);
    return -3;
  }

  if (off & 3) {
    inno_log_error("PS write error, offset 0x%X must align 4", off);
    return -4;
  }

  volatile char *virt_addr = ps_map_base_ + off;
  *(volatile uint32_t *)virt_addr = data;
  return 0;
}

int Reg::pl_read_(uint16_t off, uint32_t *value) {
  if (is_pl_invalid_()) {
    return -2;
  }

  if (off >= PL_LENGTH) {
    inno_log_error("PL read error, offset 0x%X >= 0x%X", off, PL_LENGTH);
    return -3;
  }

  if (off & 3) {
    inno_log_error("PL read error, offset 0x%X must align 4", off);
    return -4;
  }

  volatile char *virt_addr = pl_map_base_ + off;
  *value = *(volatile uint32_t *)virt_addr;
  return 0;
}

int Reg::pl_write_(uint16_t off, uint32_t data) {
  if (is_pl_invalid_()) {
    return -2;
  }

  if (off >= PL_LENGTH) {
    inno_log_error("PL write error, offset 0x%X >= 0x%X", off, PL_LENGTH);
    return -3;
  }

  if (off & 3) {
    inno_log_error("PL write error, offset 0x%X must align 4", off);
    return -4;
  }

  volatile char *virt_addr = pl_map_base_ + off;
  *(volatile uint32_t *)virt_addr = data;
  return 0;
}

uint32_t Reg::read_error_reg_() {
  uint32_t e = 0;
  pl_read_(ERR_REG, &e);
  return e;
}

uint16_t Reg::get_chn_addr_(int chn, uint32_t off) {
  if (chn >= 0 && (uint32_t)chn < kMaxChannelNum) {
    return (ch_base_addr_[chn] + off);
  } else {
    inno_log_error("channel %d is not between [%d, %u)",
                   chn, 0, kMaxChannelNum);
    return (ch_base_addr_[0] + off);
  }
}

int Reg::pl_rd_mask_mod_(uint16_t offset, uint32_t val, uint32_t mask) {
  if (is_pl_invalid_()) {
    return -2;
  }

  uint32_t data;
  int ret = pl_read_(offset, &data);
  if (ret != 0) {
    return ret;
  }
  data &= (~mask);
  data |= val;
  return pl_write(offset, data);
}

}  // namespace innovusion
