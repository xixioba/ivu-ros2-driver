/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>

#include <limits>
#include <map>
#include "sdk_common/inno_lidar_api.h"
#include "sdk_common/inno_lidar_other_api.h"
#include "sdk_common/lidar_base.h"
#include "utils/log.h"
#include "utils/utils.h"

#include "sdk_common/version_gen.gen_cc"

using innovusion::InnoLidarBase;

static const char *inno_api_build_time_g = __TIME__ " " __DATE__;

const char *inno_api_version(void) {
  return inno_api_version_g;
}

const char *inno_api_build_tag(void) {
  return inno_api_build_tag_g;
}

const char *inno_api_build_time(void) {
  return inno_api_build_time_g;
}

void inno_lidar_setup_sig_handler() {
  innovusion::InnoLog::setup_sig_handler();
}

/**
 * @brief : inno_lidar_set_logs
 */
void inno_lidar_set_logs(int out_fd, int error_fd,
                         const char *rotate_file_base_file,
                         uint32_t rotate_file_number,
                         uint64_t rotate_file_size_limit,
                         InnoLogCallback log_callback,
                         void *ctx,
                         const char *rotate_file_base_file_err,
                         uint32_t rotate_file_number_err,
                         uint64_t rotate_file_size_limit_err,
                         uint64_t flags) {
  bool use_async_log = (flags & 1);
  innovusion::InnoLog::get_instance().set_logs(out_fd, error_fd,
                                               rotate_file_base_file,
                                               rotate_file_number,
                                               rotate_file_size_limit,
                                               rotate_file_base_file_err,
                                               rotate_file_number_err,
                                               rotate_file_size_limit_err,
                                               log_callback,
                                               ctx,
                                               use_async_log);

  inno_log_with_level(INNO_LOG_LEVEL_INFO,
                      "LIDAR SDK version is %s",
                      inno_api_version());
  inno_log_with_level(INNO_LOG_LEVEL_INFO,
                      "LIDAR SDK build tag is %s",
                      inno_api_build_tag());
  inno_log_with_level(INNO_LOG_LEVEL_INFO,
                      "LIDAR SDK build time is %s",
                      inno_api_build_time());
}

/**
 * @brief : inno_lidar_set_log_level
 * @param  log_level : InnoLogLevel
 */
void inno_lidar_set_log_level(enum InnoLogLevel log_level) {
  inno_log_with_level(INNO_LOG_LEVEL_INFO,
                      "log level change from %d to %d",
                      inno_log_level_g, log_level);
  inno_log_level_g = log_level;
}

/**
 * @brief : inno_lidar_log_callback
 * @param  log_callback : InnoLogCallback
 * @param  ctx : void *
 */
void inno_lidar_log_callback(InnoLogCallback log_callback,
                              void *ctx) {
  innovusion::InnoLog::get_instance().\
                        set_logs_callback(log_callback, ctx);
}

int inno_lidar_set_mode(int handle,
                        enum InnoLidarMode mode,
                        enum InnoLidarMode *mode_before_change,
                        enum InnoLidarStatus *status_before_change) {
  inno_log_verify(mode_before_change, "NULL pointer");
  InnoLidarBase *l = InnoLidarBase::find_lidar(handle);
  if (l) {
    return l->set_mode(mode, mode_before_change, status_before_change);
  } else {
    return 1;
  }
}

int inno_lidar_get_mode_status(int handle,
                               enum InnoLidarMode *mode,
                               enum InnoLidarMode *pre_mode,
                               enum InnoLidarStatus *status,
                               uint64_t *in_transition_mode_ms) {
  inno_log_verify(mode, "NULL pointer");
  inno_log_verify(pre_mode, "NULL pointer");
  inno_log_verify(status, "NULL pointer");
  inno_log_verify(in_transition_mode_ms, "NULL pointer");
  InnoLidarBase *l = InnoLidarBase::find_lidar(handle);
  if (l) {
    return l->get_mode_status(mode, pre_mode, status,
                              in_transition_mode_ms);
  } else {
    return 1;
  }
}

int inno_lidar_get_attribute_string(int handle,
                                    const char *attribute,
                                    char *buf, size_t buf_size) {
  inno_log_verify(attribute, "NULL pointer");
  inno_log_verify(buf, "NULL pointer");
  inno_log_verify(buf_size, "buf_size");
  InnoLidarBase *l = InnoLidarBase::find_lidar(handle);
  if (l) {
    double value;
    int ret = l->get_attribute(attribute, &value);
    if (ret == 0) {
      size_t cnt = snprintf(buf, buf_size, "%lf", value);
      if (cnt >= buf_size) {
        inno_log_error("buffer overflow");
        buf[0] = 0;
        ret = -1;
      }
    } else {
      ret = l->get_attribute_string(attribute, buf, buf_size);
    }
    return ret;
  } else {
    return 1;
  }
}

int inno_lidar_set_attribute_string(int handle,
                                    const char *attribute,
                                    const char *buf) {
  inno_log_verify(attribute, "NULL pointer");
  inno_log_verify(buf, "NULL pointer");
  InnoLidarBase *l = InnoLidarBase::find_lidar(handle);
  if (l) {
    return l->set_attribute_string(attribute, buf);
  } else {
    return 1;
  }
}

int inno_lidar_read_ps_reg(int handle, uint16_t off, uint32_t *value) {
  inno_log_verify(value, "NULL pointer");
  InnoLidarBase *l = InnoLidarBase::find_lidar(handle);
  if (l) {
    return l->read_ps_reg(off, value);
  } else {
    return 1;
  }
}

int inno_lidar_read_pl_reg(int handle, uint16_t off, uint32_t *value) {
  inno_log_verify(value, "NULL pointer");
  InnoLidarBase *l = InnoLidarBase::find_lidar(handle);
  if (l) {
    return l->read_pl_reg(off, value);
  } else {
    return 1;
  }
}

int inno_lidar_write_ps_reg(int handle, uint16_t off, uint32_t value) {
  InnoLidarBase *l = InnoLidarBase::find_lidar(handle);
  if (l) {
    return l->write_ps_reg(off, value);
  } else {
    return 1;
  }
}

int inno_lidar_write_pl_reg(int handle, uint16_t off, uint32_t value) {
  InnoLidarBase *l = InnoLidarBase::find_lidar(handle);
  if (l) {
    return l->write_pl_reg(off, value);
  } else {
    return 1;
  }
}

int inno_lidar_set_callbacks(int handle,
                             InnoMessageCallback message_callback,
                             InnoDataPacketCallback data_callback,
                             InnoStatusPacketCallback status_callback,
                             InnoHosttimeCallback get_host_time,
                             void *callback_context) {
  InnoLidarBase *l = InnoLidarBase::find_lidar(handle);
  if (l) {
    l->set_callbacks(message_callback, data_callback, status_callback,
                     get_host_time, callback_context);
    return 0;
  } else {
    return 1;
  }
}

int inno_lidar_set_recorder_callback(int handle,
                                     enum InnoRecorderCallbackType type,
                                     InnoRecorderCallback callback,
                                     void *ctx) {
  InnoLidarBase *l = InnoLidarBase::find_lidar(handle);
  if (l) {
    return l->set_recorder_callback(type, callback, ctx);
  } else {
    return 1;
  }
}

int inno_lidar_set_parameters(int handle,
                              const char *lidar_model,
                              const char *yaml_filename) {
  inno_log_verify(lidar_model, "NULL pointer");
  InnoLidarBase *l = InnoLidarBase::find_lidar(handle);
  if (l) {
    return l->set_params_file(lidar_model, yaml_filename);
  } else {
    return 1;
  }
}

int inno_lidar_set_config_name_value(int handle,
                                     const char *name,
                                     const char *value) {
  inno_log_verify(name, "NULL pointer");
  inno_log_verify(value, "NULL pointer");
  InnoLidarBase *l = InnoLidarBase::find_lidar(handle);
  if (l) {
    return l->set_config_name_value(name, value);
  } else {
    return 1;
  }
}

int inno_lidar_set_reflectance_mode(int handle, enum InnoReflectanceMode mode) {
  InnoLidarBase *l = InnoLidarBase::find_lidar(handle);
  if (l) {
    return l->set_reflectance_mode(mode);
  } else {
    return 1;
  }
}

int inno_lidar_set_return_mode(int handle,
                               InnoMultipleReturnMode ret_mode) {
  InnoLidarBase *l = InnoLidarBase::find_lidar(handle);
  if (l) {
    return l->set_return_mode(ret_mode);
  } else {
    return 1;
  }
}

int inno_lidar_set_roi(int handle, double horz_angle, double vert_angle) {
  InnoLidarBase *l = InnoLidarBase::find_lidar(handle);
  if (l) {
    return l->set_roi(horz_angle, vert_angle);
  } else {
    return 1;
  }
}

int inno_lidar_get_roi(int handle, double *horz_angle, double *vert_angle) {
  InnoLidarBase *l = InnoLidarBase::find_lidar(handle);
  if (l) {
    return l->get_roi(horz_angle, vert_angle);
  } else {
    return 1;
  }
}

int inno_lidar_set_motion_compensation(int handle,
                                       double velocity[3],
                                       double angular_velocity[3]) {
  InnoLidarBase *l = InnoLidarBase::find_lidar(handle);
  if (l) {
    return l->set_motion_compensation(velocity, angular_velocity);
  } else {
    return -1;
  }
}

int inno_lidar_thread_setaffinity_np(int handle,
                                     size_t cpusetsize,
                                     const cpu_set_t *cpuset,
                                     int exclude_callback_thread) {
  inno_log_verify(cpuset, "NULL pointer");
  InnoLidarBase *l = InnoLidarBase::find_lidar(handle);
  if (l) {
    return l->thread_setaffinity_np(cpusetsize, cpuset,
                                    exclude_callback_thread);
  } else {
    return 1;
  }
}

int inno_lidar_get_fw_state(int handle,
                            InnoLidarState *state,
                            int *error_code) {
  inno_log_verify(state, "NULL pointer");
  InnoLidarBase *l = InnoLidarBase::find_lidar(handle);
  if (l) {
    return l->get_fw_state(state, error_code);
  } else {
    return 1;
  }
}

int inno_lidar_get_fw_version(int handle, char *buffer, int buffer_len) {
  inno_log_verify(buffer, "NULL pointer");
  InnoLidarBase *l = InnoLidarBase::find_lidar(handle);
  if (l) {
    return l->get_fw_version(buffer, buffer_len);
  } else {
    return -1;
  }
}

int inno_lidar_get_sn(int handle, char *buffer, int buffer_len) {
  inno_log_verify(buffer, "NULL pointer");
  InnoLidarBase *l = InnoLidarBase::find_lidar(handle);
  if (l) {
    return l->get_sn(buffer, buffer_len);
  } else {
    return -1;
  }
}

int inno_lidar_get_model(int handle, char *buffer, int buffer_len) {
  inno_log_verify(buffer, "NULL pointer");
  InnoLidarBase *l = InnoLidarBase::find_lidar(handle);
  if (l) {
    return l->get_model(buffer, buffer_len);
  } else {
    return -1;
  }
}

int inno_lidar_start(int handle) {
  InnoLidarBase *l = InnoLidarBase::find_lidar(handle);
  if (l) {
    l->start();
    return 0;
  } else {
    return 1;
  }
}

int inno_lidar_stop(int handle) {
  InnoLidarBase *l = InnoLidarBase::find_lidar(handle);
  if (l) {
    l->stop();
    return 0;
  } else {
    return 1;
  }
}

int inno_lidar_stop_all() {
  return InnoLidarBase::stop_lidar_all();
}

int inno_lidar_close(int handle) {
  return InnoLidarBase::remove_lidar(handle);
}

int inno_lidar_close_all() {
  return InnoLidarBase::remove_lidar_all();
}

namespace innovusion {

/**********************
 * static members
 **********************/

std::mutex InnoLidarBase::static_mutex_s;
std::map<int, InnoLidarBase*> InnoLidarBase::lidars_s;
int InnoLidarBase::max_handle_s = 0;
uint32_t InnoLidarBase::open_count_s = 0;

/**********************
 * static methods
 **********************/
int InnoLidarBase::add_lidar(InnoLidarBase *l) {
  std::unique_lock<std::mutex> lk(static_mutex_s);
  inno_log_verify(l, "lidar cannot be NULL");
  int handle = l->handle_;
  lidars_s[handle] = l;
  open_count_s++;
  inno_log_info("add lidar %d (total=%u total_active=%u)",
                handle, open_count_s, (uint32_t)lidars_s.size());
  return handle;
}

int InnoLidarBase::remove_lidar(int handle) {
  std::map<int, InnoLidarBase*>::iterator it;
  std::unique_lock<std::mutex> lk(static_mutex_s);
  it = lidars_s.find(handle);
  if (it != lidars_s.end()) {
    InnoLidarBase *l = it->second;
    lidars_s.erase(it);
    lk.unlock();
    State s = l->get_state_();
    inno_log_verify(s == STATE_INIT,
                    "%s stop lidar %d bad state = %d",
                    l->name_, handle, s);
    inno_log_info("remove lidar %d", handle);
    delete(l);
    return 0;
  } else {
    inno_log_error("Cannot find lidar, invalid handle %d. "
                   "Possible double-close?",
                   handle);
    return 1;
  }
}

int InnoLidarBase::remove_lidar_all() {
  std::unique_lock<std::mutex> lk(static_mutex_s);
  std::map<int, InnoLidarBase*>::iterator it;
  int delete_size = 0;
  while (true) {
    it = lidars_s.begin();
    if (it == lidars_s.end()) {
      break;
    }
    InnoLidarBase *l = it->second;
    State s = l->get_state_();
    inno_log_verify(s == STATE_INIT,
                    "%s stop lidar %d bad state = %d",
                    l->name_, it->first, s);
    inno_log_info("remove lidar %d", it->first);
    delete(l);
    lidars_s.erase(it->first);
    delete_size++;
  }
  inno_log_info("remove %d lidars", delete_size);
  return delete_size;
}

int InnoLidarBase::stop_lidar_all() {
  std::unique_lock<std::mutex> lk(static_mutex_s);
  std::map<int, InnoLidarBase*>::iterator it;
  int stop_size = 0;
  for (auto& it : lidars_s) {
    InnoLidarBase *l = it.second;
    State s = l->get_state_();
    if (s == STATE_INIT) {
      inno_log_info("lidar %d is not started", it.first);
    } else {
      inno_log_info("stop lidar %d", it.first);
      l->stop();
      stop_size++;
    }
  }
  inno_log_info("stop %d lidars", stop_size);
  return stop_size;
}

InnoLidarBase *InnoLidarBase::find_lidar(int handle) {
  std::map<int, InnoLidarBase*>::iterator it;
  std::unique_lock<std::mutex> lk(static_mutex_s);
  it = lidars_s.find(handle);
  if (it != lidars_s.end()) {
    InnoLidarBase *l = it->second;
    return l;
  } else {
    return NULL;
  }
}

double InnoLidarBase::get_host_time_default(void *) {
  uint64_t now = InnoUtils::get_time_ns(CLOCK_REALTIME);
  return now / 1.0e9;
}

}  // namespace innovusion
