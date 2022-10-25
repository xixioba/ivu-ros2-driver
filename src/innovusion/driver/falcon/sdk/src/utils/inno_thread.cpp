/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "utils/inno_thread.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>

#if !(defined(_QNX_) || defined (__MINGW64__))
#include <sys/syscall.h>
#endif

#include <sys/types.h>

#include "utils/log.h"
#include "utils/utils.h"

namespace innovusion {
InnoThread::InnoThread(const char *name, int priority,
                       int worker_num,
                       void *(*func_main)(void *),
                       void *func_context,
                       size_t cpusetsize,
                       const cpu_set_t *cpuset)
    : priority_(priority)
    , worker_num_(worker_num)
    , func_main_(func_main)
    , func_context_(func_context)
    , cpusetsize_(cpusetsize)
    , cpuset_(cpuset)
    , shutdown_(true) {
  name_ = strdup(name);
  inno_log_verify(name_, "%s name", name);
  pthread_mutex_init(&mutex_, NULL);
  pthread_condattr_init(&condattr_);
#ifndef __APPLE__
  pthread_condattr_setclock(&condattr_, clockid_);
#endif
  pthread_cond_init(&cond_, &condattr_);
  threads_ = NULL;
  start_time_ = InnoUtils::get_time_ns(clockid_);
}

InnoThread::~InnoThread() {
  inno_log_verify(shutdown_, "%s shutdown = %d",
                  name_, shutdown_);
  pthread_mutex_destroy(&mutex_);
  pthread_cond_destroy(&cond_);
  free(name_);
  name_ = NULL;
}

void *InnoThread::inno_thread_func_(void *context) {
  InnoThread *cp = reinterpret_cast<InnoThread *>(context);
#if !(defined(_QNX_) || defined(__MINGW64__) || defined(__APPLE__))
  pid_t pid = 0;
  if (cp->cpuset_ && cp->cpusetsize_) {
    const pthread_t tid = pthread_self();
    pthread_setaffinity_np(tid, cp->cpusetsize_,
                           cp->cpuset_);
  }
  pid = syscall(SYS_gettid);
#else
  (void)cp->cpuset_;
  (void)cp->cpusetsize_;
  uint32_t pid = 0;
#endif

  inno_log_info("thread %s starts. pid=%d target_priority=%d",
                cp->name_, pid, cp->priority_);
  InnoUtils::set_self_thread_priority(cp->priority_);

  cp->func_main_(cp->func_context_);

  return NULL;
}

void InnoThread::start() {
  pthread_mutex_lock(&mutex_);
  shutdown_ = false;
  pthread_mutex_unlock(&mutex_);
  threads_ = reinterpret_cast<pthread_t *>\
             (malloc(worker_num_ *sizeof(pthread_t)));
  inno_log_verify(threads_, "alloc memory failed for threads_");
  for (int i = 0; i < worker_num_; i++) {
    pthread_create(&threads_[i], NULL, inno_thread_func_, this);
  }
}

void InnoThread::shutdown() {
  pthread_mutex_lock(&mutex_);
  inno_log_verify(!shutdown_, "%s shutdown = %d", name_, shutdown_);
  shutdown_ = true;
  pthread_mutex_unlock(&mutex_);
  pthread_cond_signal(&cond_);
  // join threads
  for (int i = 0; i < worker_num_; i++) {
    void *ret;
    pthread_join(threads_[i], &ret);
  }
  free(threads_);
  threads_ = NULL;
}

/**
 * Should ONLY be called by work thread itself.
 * @param useconds
 */
void InnoThread::timed_wait(uint32_t useconds) {
  timespec ts{};
  uint64_t cur_us = InnoUtils::get_time_us(clockid_);
  cur_us += useconds;
  InnoUtils::us_to_timespec(cur_us, &ts);
  pthread_cond_timedwait(&cond_, &mutex_, &ts);
}

}  // namespace innovusion
