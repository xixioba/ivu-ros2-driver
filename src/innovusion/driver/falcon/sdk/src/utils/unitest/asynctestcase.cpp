/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include <fcntl.h>
#include <pthread.h>
#include <stdarg.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <limits.h>

#ifndef __MINGW64__
#include <syscall.h>
#include <sys/uio.h>
#endif
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>

#include <algorithm>
#include <mutex>  // NOLINT
#include <unordered_map>
#include <vector>
#include <queue>
#include <set>
#include <string>
#include <random>
#include <iostream>

#include "gtest/gtest/googletest/include/gtest/gtest.h"
#include "utils/async_log.h"
#include "utils/utils.h"
#include "utils/consumer_producer.h"
#include "utils/mem_pool_manager.h"
#include "utils/log.h"

bool get4k = false;
char buffer[6000];
int warningcounter = 0;
uint32_t level_len = 0;
void asynctestcase_callbacktest(void *ctx,
                                  enum InnoLogLevel level,
                                  const char *header1,
                                  const char *header2,
                                  const char *msg) {
  int result = strcmp(buffer, msg);
  if (result == 0) {
    get4k = true;
  }
  if (level == INNO_LOG_LEVEL_ERROR) {
    warningcounter++;
  }

  return;
}

using std::string;
using std::random_device;
using std::default_random_engine;

string strRand(int length) {
  char tmp;
  string buffer;

  random_device rd;
  default_random_engine random(rd());

  for (int i = 0; i < length; i++) {
    tmp = random() % 36;
    if (tmp < 10) {
      tmp += '0';
    } else {
      tmp -= 10;
      tmp += 'A';
    }
    buffer += tmp;
  }
  return buffer;
}

namespace Unitesting {
  static int need_loop = 1;
class AsyncLogDemo {
 public:
  // AsyncLogDemo
  AsyncLogDemo() {
    log_p = &innovusion::InnoLog::get_instance();
    int i = 0;
    void* p = &i;
    need_loop = 0;
    if (!innovusion::InnoLog::get_instance().\
                                  get_asynclog_manager()) {
      innovusion::InnoLog::get_instance().\
          set_process_hook_for_asynclog(unitest_process);
      // Test log init (create async log)
      innovusion::InnoLog::get_instance().\
              set_logs(1, 2,
                      "./inno_pc_server.txt",
                      3,
                      2 * 1000UL,
                      "./inno_pc_server.error",
                      2,
                      1 * 1000UL,
                      asynctestcase_callbacktest,
                      p,
                      true);
     }
  }
  // AsyncLogDemo
  ~AsyncLogDemo() {
  }
  // proccess callback
  static int unitest_process(innovusion::logContextInfo *job_p, bool prefer) {
    level_len = job_p->level_len;
    sleep(need_loop);
    return 1;
  }
  innovusion::InnoLog * log_p = NULL;
};
class AsyncLogInitTest : public ::testing::Test{
 protected:
  // Remember that SetUp() is run immediately before a test starts.
  // This is a good place to record the start time.
  void SetUp() override { start_time_ = time(nullptr); }

  // TearDown() is invoked immediately after a test finishes.  Here we
  // check if the test was too slow.
  void TearDown() override {
    // Gets the time when the test finishes
  }

  // The UTC time (in seconds) when the test starts
  time_t start_time_;
  Unitesting::AsyncLogDemo * demoP = NULL;
};
//  CreateThreadLater
TEST_F(AsyncLogInitTest, CreateThreadLater) {
  EXPECT_EQ(false, (0 != innovusion::InnoLog::get_instance().\
                                  get_asynclog_manager()));
  demoP = new AsyncLogDemo();
}
// CreatedThread
TEST_F(AsyncLogInitTest, CreatedThread) {
  EXPECT_EQ(true, (0 != innovusion::InnoLog::get_instance().\
                                  get_asynclog_manager()));
}
// mempool test  (queues: 60, buffer num 90,bufsize 4096)
// 1
TEST_F(AsyncLogInitTest, BufferPoolTest) {
  int index = 0;
  bool shouldbepoolrange = true;
  bool shouldbemallocrange = true;
  demoP = new AsyncLogDemo();
  std::vector<innovusion::logContextInfo*> vector1(200, nullptr);
  need_loop = 0;
  while (true) {
    innovusion::logContextInfo* p = demoP->log_p->get_asynclog_manager()->
                                alloc_buffer(4096);
    if (index < 90) {
      if (demoP->log_p->get_asynclog_manager()->\
                        get_mempool()->is_manager_of(p) != true) {
        shouldbepoolrange = false;
      }
    } else {
      if (demoP->log_p->get_asynclog_manager()->\
                        get_mempool()->is_manager_of(p) == true) {
        shouldbemallocrange = false;
      }
      if (index >= 100) {
        break;
      }
    }
    vector1[index] = p;
    index++;
  }

  for (const auto& value : vector1) {
     demoP->log_p->get_asynclog_manager()->
                                free_buffer(value);
  }

  EXPECT_EQ(true, (shouldbepoolrange));
  EXPECT_EQ(true, (shouldbemallocrange));
}

// mempool test  [0...4096] pool (4096,..) malloc
TEST_F(AsyncLogInitTest, BufferPoolMaxlenTest) {
  bool shouldbepoolrange = true;
  bool shouldbemallocrange = true;

  demoP = new AsyncLogDemo();
  std::vector<innovusion::logContextInfo*> vector1(5, nullptr);
  need_loop = 0;
  innovusion::logContextInfo* p = demoP->log_p->get_asynclog_manager()->
                              alloc_buffer(4096);
  if (demoP->log_p->get_asynclog_manager()->\
                    get_mempool()->is_manager_of(p) != true) {
    shouldbepoolrange = false;
  }
  EXPECT_EQ(true, (shouldbepoolrange));
  demoP->log_p->get_asynclog_manager()->
                            free_buffer(p);

  p = demoP->log_p->get_asynclog_manager()->
                              alloc_buffer(4097);

  if (!p || demoP->log_p->get_asynclog_manager()->\
                    get_mempool()->is_manager_of(p) == true) {
    shouldbemallocrange = false;
  }
  EXPECT_EQ(true, (shouldbemallocrange));
  demoP->log_p->get_asynclog_manager()->
                            free_buffer(p);
  EXPECT_EQ(true, (shouldbemallocrange));
}

// max len send
// mempool test  [0...4096] pool (4096,..) malloc
TEST_F(AsyncLogInitTest, Maxdatalength) {
  need_loop = 0;
  string tt = strRand(5000);
  snprintf(buffer, sizeof(buffer),
                        "%s", tt.c_str());
  inno_log_info("%s", tt.c_str());
  sleep(2);
  EXPECT_EQ(true, get4k);
  get4k = false;
}

// level length check
TEST_F(AsyncLogInitTest, LevelLenghtCheck) {
const char *inno_log_header_g[] = {
  "[FATAL]",
  "[CRITI]",  // critical
  "[ERROR]",
  "[ TEMP]",
  "[ WARN]",
  "[DEBUG]",
  "[ INFO]",
  "[TRACE]",
  "[DETAL]",  // detail
  "[     ]",
};
  need_loop = 0;
  string tt = strRand(5000);
  snprintf(buffer, sizeof(buffer),
                        "%s", tt.c_str());
  inno_log_info("%s", tt.c_str());
  sleep(2);
  EXPECT_EQ(true, \
    (level_len == strlen(inno_log_header_g[INNO_LOG_LEVEL_INFO])));
  level_len = 0;
  inno_log_error("%s", tt.c_str());
  sleep(2);
  EXPECT_EQ(true, \
    (level_len == strlen(inno_log_header_g[INNO_LOG_LEVEL_ERROR])));
}

// max len send
// mempool test  [0...4096] pool (4096,..) malloc
TEST_F(AsyncLogInitTest, ERRORWaring) {
  need_loop = 1;
  int index = 0;
  warningcounter  = 0;
  // do flush and wait
  while (index < 20) {
    inno_log_info("before warning%d", index++);
  }
  index = 0;
  while (index < 40) {
    inno_log_error("enqueue job%d", index++);
  }
  index = 0;
  while (index < 60) {
    inno_log_info("after inno_log_warning%d", index++);
  }
  innovusion::InnoLog::get_instance().\
      set_logs_callback(NULL, NULL);
  EXPECT_TRUE((warningcounter == 40));
  warningcounter  = 0;
}

// mempool test  [0...4096] pool (4096,..) malloc
TEST_F(AsyncLogInitTest, Dataflow) {
  need_loop = 1;
  int index = 0;
  // do flush and wait
  while (index < 200) {
    inno_log_info("enqueue job%d", index++);
  }
  const time_t starttime = time(nullptr);
  innovusion::InnoLog::get_instance().\
      set_logs_callback(NULL, NULL);
  const time_t end_time = time(nullptr);
  EXPECT_TRUE(end_time - starttime >= need_loop*50)\
          << "The cost is" << (end_time - starttime);
}
}  //  namespace Unitesting
