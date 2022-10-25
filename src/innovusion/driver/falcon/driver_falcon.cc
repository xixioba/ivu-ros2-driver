#include "driver_falcon.h"

#include <iostream>
#include <mutex>

#include "sdk_common/inno_lidar_api.h"
#include "sdk_common/inno_lidar_other_api.h"
namespace innovusion {

bool DriverFalcon::init_() {
  inno_lidar_set_log_level((enum InnoLogLevel)inno_log_level);
  enum InnoLidarProtocol protocol_;
  if (data_filename != "") {
    // setup read from file
    handle_ = inno_lidar_open_file(lidar_name.c_str(), data_filename.c_str(),
                                   !processed, file_speed, file_rewind,
                                   file_skip * 1000000UL);
    protocol_ =
        processed ? INNO_LIDAR_PROTOCOL_PCS_FILE : INNO_LIDAR_PROTOCOL_RAW_FILE;
  } else {
    // setup read from live
    uint16_t udp_port = 0;
    if (lidar_udp_port > 0) {
      if (!processed) {
        inno_log_warning(
            "raw_udp not supported, "
            "force to use pcs_udp");
      }
      protocol_ = INNO_LIDAR_PROTOCOL_PCS_UDP;
      udp_port = lidar_udp_port;
    } else if (processed) {
      protocol_ = INNO_LIDAR_PROTOCOL_PCS_TCP;
    } else {
      if (lidar_ip == "local") {
        lidar_ip = "127.0.0.1";
        protocol_ = INNO_LIDAR_PROTOCOL_RAW_MEM;
      } else {
        protocol_ = INNO_LIDAR_PROTOCOL_RAW_TCP;
      }
    }
    handle_ = inno_lidar_open_live(lidar_name.c_str(), lidar_ip.c_str(),
                                   lidar_port, protocol_, udp_port);
  }
  if (handle_ > 0) {
    int ret = 0;
    ret = inno_lidar_set_reflectance_mode(handle_,
                                          (InnoReflectanceMode)reflectance);
    if (ret != 0) {
      inno_log_warning("set_reflectance ", reflectance, " return ", ret);
    }

    ret = inno_lidar_set_return_mode(handle_,
                                     (InnoMultipleReturnMode)multireturn);
    if (ret != 0) {
      inno_log_warning("set_return_mode ", multireturn, " return ", ret);
    }

    if (set_falcon_eye) {
      ret = inno_lidar_set_roi(handle_, roi_center_h, roi_center_v);
      if (ret != 0) {
        inno_log_warning("set_roi h", roi_center_h, " v", roi_center_v,
                         "return ", ret);
      }
    }

    ret = inno_lidar_set_parameters(handle_, "", yaml_filename.c_str());
    inno_log_verify(ret == 0, "set_parameters %d", ret);

    ret =
        inno_lidar_set_callbacks(handle_, message_callback_s_, data_callback_s_,
                                 status_callback_s_, NULL, this);
    inno_log_verify(ret == 0, "set_callbacks %d", ret);
    return true;
  }
  return false;
};

bool DriverFalcon::start() {
  // no blocking mode
  if (handle_ <= 0) init_();
  inno_lidar_start(handle_);
  return true;
};

bool DriverFalcon::pause() {
  // no blocking mode
  if (handle_ > 0) {
    inno_lidar_stop(handle_);
  }
  return true;
};

bool DriverFalcon::stop() {
  if (handle_ > 0) {
    inno_lidar_stop(handle_);
    inno_lidar_close(handle_);
  }
  is_running_ = 0;
  handle_ = 0;
  return true;
};

int DriverFalcon::get_lidar(const std::string &cmd, std::string *result) {
  char buffer[1024 * 64];
  int buffer_len = sizeof(buffer);
  int ret = -1;
  if (cmd == "ip") {
    *result += "127.0.0.1";
    return 0;
  }
  if (cmd == "fw_version") {
    ret = inno_lidar_get_fw_version(handle_, buffer, buffer_len);
  } else if (cmd == "sn") {
    ret = inno_lidar_get_sn(handle_, buffer, buffer_len);
  } else if (cmd == "model") {
    ret = inno_lidar_get_model(handle_, buffer, buffer_len);
  } else if (cmd == "mode_status" || cmd == "mode") {
    enum InnoLidarMode mode;
    enum InnoLidarMode pre_mode;
    enum InnoLidarStatus status;
    uint64_t in_transition_mode_ms;
    ret = inno_lidar_get_mode_status(handle_, &mode, &pre_mode, &status,
                                     &in_transition_mode_ms);
    if (ret == 0) {
      snprintf(buffer, sizeof(buffer), "%d,%d,%d,%lu", static_cast<int>(mode),
               static_cast<int>(pre_mode), static_cast<int>(status),
               in_transition_mode_ms);
    } else {
      inno_log_warning("cannot get_mode_status %d", ret);
    }
  } else if (cmd == "roi") {
    double h_roi, v_roi;
    ret = inno_lidar_get_roi(handle_, &h_roi, &v_roi);
    if (ret == 0) {
      snprintf(buffer, sizeof(buffer), "%f,%f", h_roi, v_roi);
    } else {
      inno_log_warning("cannot get_roi %d", ret);
    }
  } else {
    char buffer[8192];
    ret = inno_lidar_get_attribute_string(handle_, cmd.c_str(), buffer,
                                          sizeof(buffer));
    if (ret == 0) {
      *result += buffer;
      return 0;
    } else {
      inno_log_warning("cannot get attribute %s", cmd.c_str());
      return -1;
    }
  }
  if (ret == 0) {
    *result += buffer;
    return 0;
  } else {
    inno_log_warning("get %s failed", cmd.c_str());
    return -1;
  }
}

int DriverFalcon::set_lidar(const std::string &key, const std::string &value) {
  int val;
  double h, v;
  int ret;
  if (key == "return_mode" || key == "multiple_return" ||
      key == "multireturn") {
    if (sscanf(value.c_str(), "%d", &val) != 1) {
      inno_log_warning("Invalid query %s %s", key.c_str(), value.c_str());
      return -1;
    } else {
      ret = inno_lidar_set_return_mode(handle_, (InnoMultipleReturnMode)val);
      return ret;
    }
  } else if (key == "reflectance_mode" || key == "reflectance") {
    if (sscanf(value.c_str(), "%d", &val) != 1) {
      inno_log_warning("Invalid query %s %s", key.c_str(), value.c_str());
      return -1;
    } else {
      ret = inno_lidar_set_reflectance_mode(handle_, (InnoReflectanceMode)val);
      return ret;
    }
  } else if (key == "roi") {
    if (sscanf(value.c_str(), "%lf,%lf", &h, &v) != 2) {
      inno_log_warning("Invalid query %s %s", key.c_str(), value.c_str());
      return -1;
    } else {
      int ret = inno_lidar_set_roi(handle_, h, v);
      return ret;
    }
  } else if (key == "mode") {
    if (sscanf(value.c_str(), "%d", &val) != 1) {
      inno_log_warning("Invalid query %s %s", key.c_str(), value.c_str());
      return -1;
    } else {
      enum InnoLidarMode pre_mode;
      enum InnoLidarStatus status;
      int ret =
          inno_lidar_set_mode(handle_, InnoLidarMode(val), &pre_mode, &status);
      if (ret == 0) {
        inno_log_info("set_mode to %d, pre: %d status: %d", val,
                      static_cast<int>(pre_mode), static_cast<int>(status));
      } else {
        inno_log_warning("set_mode to %d failed, ret=%d", val, ret);
      }
      return ret;
    }
  } else {
    ret = inno_lidar_set_attribute_string(handle_, key.c_str(), value.c_str());
    if (ret != 0) {
      inno_log_warning("set_attribute %s to %s failed, ret=%d", key.c_str(),
                       value.c_str(), ret);
    }
    return ret;
  }
}

int DriverFalcon::set_config_name_value(const std::string &key,
                                        const std::string &value) {
  return inno_lidar_set_config_name_value(handle_, key.c_str(), value.c_str());
}

}  // namespace innovusion