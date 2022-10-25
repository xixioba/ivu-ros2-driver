/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef UTILS_SHARED_MEMORY_H_
#define UTILS_SHARED_MEMORY_H_

#ifndef __MINGW64__
#include <sys/ipc.h>
#include <sys/shm.h>
#endif

#include <sys/types.h>

#include "utils/log.h"

namespace innovusion {
class SharedMemory {
 public:
  SharedMemory(const char *path, int proj_id, size_t size) {
    shm_id_ = -1;
    shm_addr_ = NULL;
#ifndef __MINGW64__
    key_t key = ftok(path, proj_id);
    if (key == -1) {
      inno_log_warning_errno("ftok %d", key);
    } else {
      shm_id_ = shmget(key, size, 0666|IPC_CREAT);
      if (shm_id_ == -1) {
        inno_log_error_errno("shmget %d", shm_id_);
      } else {
        // shmat to attach to shared memory
        shm_addr_ = reinterpret_cast<volatile void *>(shmat(shm_id_, NULL, 0));
        if (shm_addr_ == NULL) {
          inno_log_error_errno("shmat %p", shm_addr_);
        }
      }
    }
#endif
  }

  ~SharedMemory() {
#ifndef __MINGW64__
    if (shm_addr_) {
      shmdt(shm_addr_detach_);
    }
#endif
  }

  inline bool is_valid() const {
    return shm_addr_ != NULL;
  }

  inline volatile void *get_memory() {
    return shm_addr_;
  }

  inline const volatile void *get_memory() const {
    return reinterpret_cast<const volatile void *>(shm_addr_);
  }

 private:
  int shm_id_;
  union {
    volatile void *shm_addr_;
    void *shm_addr_detach_;
  };
};

}  // namespace innovusion

#endif  // UTILS_SHARED_MEMORY_H_
