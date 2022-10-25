#pragma once

#include <regex>

#include "../driver_factory.h"
#include "sdk_common/converter/cframe_converter.h"

namespace innovusion {

class __attribute__((visibility("default"))) DriverFalcon
    : public DriverFactory {
 public:
  DriverFalcon(InnoCframeCallBack data_callback, void *callback_context,
               InnoStatusCallBack status_callback = nullptr)
      : DriverFactory(data_callback, callback_context, status_callback) {
    cframe_converter_ = new (::innovusion::CframeConverter);
  };

  ~DriverFalcon() {
    stop();  // make sure that handle_ has been closed
    if (cframe_converter_) {
      delete cframe_converter_;
      cframe_converter_ = NULL;
    }
  };

  // callback group
  void message_callback_(uint32_t from_remote, enum InnoMessageLevel level,
                         enum InnoMessageCode code, const char *msg) {
    if (level <= (enum InnoMessageLevel)inno_log_level)
      inno_log_warning("level=", level, " code=", code, " msg=", msg);
    if (code == INNO_MESSAGE_CODE_CANNOT_READ &&
        (std::regex_match(std::string(msg), std::regex("^end of file.*")) ||
         data_filename != "")) {
      std::unique_lock<std::mutex> lk(mtx);
      is_running_ = -2;  // file error, stop and exit
      cv.notify_all();
    } else if ((level <= INNO_MESSAGE_LEVEL_CRITICAL &&
                code != INNO_MESSAGE_CODE_LIB_VERSION_MISMATCH) ||
               (code == INNO_MESSAGE_CODE_CANNOT_READ)) {
      std::unique_lock<std::mutex> lk(mtx);
      is_running_ = -1;  // live error, restart every second
      cv.notify_all();
    } else {
      std::unique_lock<std::mutex> lk(mtx);
      is_running_ = 1;
      cv.notify_all();
    }
  };

  int data_callback_(int handle_, void *ctx, const InnoDataPacket *pkt) {
    inno_cframe_header *cframe = cframe_converter_->add_data_packet(pkt, 0);
    if (cframe != NULL) {
      cframe_callback_(handle_, cframe_callback_ctx_, (void *)cframe);
    }
    return 0;
  };

  int status_callback_(const InnoStatusPacket *pkt) { return 0; };

  // static callback warpper
  static void message_callback_s_(int handle_, void *ctx, uint32_t from_remote,
                                  enum InnoMessageLevel level,
                                  enum InnoMessageCode code,
                                  const char *error_message) {
    (reinterpret_cast<DriverFalcon *>(ctx))
        ->message_callback_(from_remote, level, code, error_message);
  }

  static int data_callback_s_(int handle_, void *ctx,
                              const InnoDataPacket *pkt) {
    return (reinterpret_cast<DriverFalcon *>(ctx))
        ->data_callback_(handle_, ctx, pkt);
  }

  static int status_callback_s_(int handle_, void *ctx,
                                const InnoStatusPacket *pkt) {
    return (reinterpret_cast<DriverFalcon *>(ctx))->status_callback_(pkt);
  }

  // lidar configuration
  bool init_();
  bool start() override;
  bool pause() override;
  bool stop() override;
  // optional
  int get_lidar(const std::string &cmd, std::string *result) override;
  int set_lidar(const std::string &key, const std::string &value) override;
  int set_config_name_value(const std::string &key,
                            const std::string &value) override;

 private:
  ::innovusion::CframeConverter *cframe_converter_;
  char sn_[InnoStatusPacket::kSnSize];
};

}  // namespace innovusion