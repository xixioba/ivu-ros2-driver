/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_PARAMS_H_
#define SDK_PARAMS_H_

#include <map>
#include <string>

namespace innovusion {
#include "sdk/iv_params_code_gen.gen_h"
class LidarParams {
 public:
  LidarParams()
    : initialized(false)
    , version(0) {
    set_default();
  }
  explicit LidarParams(const char *filename) {
    read(filename);
  }
  std::string to_string();
  int read(const char *filename);
  int parse(const char *yaml);
  bool copy_from_src(LidarParams *src_params);
  bool is_invalid();
  void set_default();
  void set_default_for_missing();
  bool is_initialized() const {
    return initialized;
  }
  void inc_version() {
    version++;
  }
  uint64_t get_version() const {
    return version;
  }

 public:
  IvParams iv_params;

 private:
  bool initialized;
  uint64_t version;
};
}  // namespace innovusion

#endif  // SDK_PARAMS_H_
