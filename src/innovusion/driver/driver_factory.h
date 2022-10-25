#pragma once

#include <chrono>
#include <condition_variable>
#include <iostream>
#include <thread>

#include "httplib.h"

namespace innovusion {

typedef int (*InnoCframeCallBack)(int handle_, void *context, void *cframe);
typedef int (*InnoStatusCallBack)(int handle_, void *context,
                                  std::string status);

class DriverFactory {
 public:
  DriverFactory(InnoCframeCallBack cframe_callback, void *callback_context,
                InnoStatusCallBack status_callback)
      : cframe_callback_(cframe_callback),
        cframe_callback_ctx_(callback_context),
        status_callback_(status_callback) {
    sub_thread_status = std::thread(&DriverFactory::StatusPollThread, this);
  };
  ~DriverFactory() {
    {
      std::unique_lock<std::mutex> lk(mtx);
      is_running_ = -3;  // system err, exit
      cv.notify_all();
    }
    if (sub_thread_status.joinable()) sub_thread_status.join();
  };

  // lidar configuration
  virtual bool start() = 0;  // init and start receive data
  virtual bool pause() = 0;  // pause receive data
  virtual bool stop() = 0;   // release resouce
  // optional
  virtual int get_lidar(const std::string &cmd, std::string *result) = 0;
  virtual int set_lidar(const std::string &key, const std::string &value) = 0;
  virtual int set_config_name_value(const std::string &key,
                                    const std::string &value) = 0;

  void StatusPollThread() {
    std::shared_ptr<httplib::Client> cli = nullptr;
    while (true) {
      {
        std::unique_lock<std::mutex> lk(mtx);
        cv.wait(lk, [&]() { return is_running_ != 0; });
      }
      if (is_running_ > 0 && status_callback_ != nullptr) {
        if (cli != nullptr && !cli->is_valid()) {
          cli.reset();
          cli = nullptr;
        }
        if (cli == nullptr) {
          std::string url = "http://" + lidar_ip + ":" + std::to_string(8088);
          cli = std::make_shared<httplib::Client>(url);
          cli->set_connection_timeout(0, 300000);  // 300 milliseconds
          cli->set_read_timeout(1, 0);             // 1 seconds
          cli->set_write_timeout(1, 0);            // 1 seconds
        }
        if (cli != nullptr && cli->is_valid()) {
          static int scope = 0;
          if (scope++ >= 50) {
            scope = 0;
            if (auto res = cli->Get("/get-updated-lidar-status")) {
              if (res->status == 200) {
                status_callback_(handle_, cframe_callback_ctx_, res->body);
              }
            }
          } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
            if (auto res = cli->Get("/get-gyroscope-xyz")) {
              if (res->status == 200) {
                status_callback_(handle_, cframe_callback_ctx_, res->body);
              }
            }
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
      } else if (is_running_ == -1) {  // live err
        sleep(1);  // need check again if need exit now, no need for next loop
        if (is_running_ == -1)  // restart
        {
          std::cerr << "live err, will restart in 1s\n";
          stop();
          start();
          sleep(1);
        }
      } else if (is_running_ == -2) {  // file err, exit
        std::cerr << "file/processed error, exit\n";
        raise(SIGINT);
        break;
      } else if (is_running_ == -3) {  // system err, exit
        std::cerr << "StatusPollThread get thread join signal\n";
        break;
      }
    }
  };

  // configuration, need set before call Init()
  std::string lidar_name{"falcon"};
  std::string frame_id{"innovusion"};
  uint32_t lidar_id{0};
  std::string lidar_ip{"172.168.1.10"};
  uint32_t lidar_port{8010};         // jaguar:8001/10001, falcon:8010
  std::string lidar_model{"rev_i"};  // rev_h[c], rev_i[k]...
  uint32_t reflectance{1};           // 0,1,2,3
  uint32_t multireturn{1};           // only for falcon now

  // fix frame time err
  // 0: nothing, <0: for use host time, >0: fix time shifting
  int32_t time_fix_err_ms{0};

  // InnoLogLevel, falcon is contray to jaguar
  uint32_t inno_log_level{2};

  // replay file
  std::string data_filename{""};
  std::string yaml_filename{""};
  uint32_t file_speed{10000};
  int32_t file_rewind{0};
  int32_t file_skip{0};

  // falcon
  int32_t lidar_udp_port{0};
  uint32_t processed{0};  // 0:raw 1:inno_pc
  bool set_falcon_eye{false};
  int32_t roi_center_h{0};
  int32_t roi_center_v{0};

  // status
  int is_running_{0};  //-1: live err, 0: default, 1: ok
                       //-2: file err, -3: system err
  std::mutex mtx;
  std::condition_variable cv;

 protected:
  int handle_{-1};  // -1:no init, 0:not running, 1:running
  InnoCframeCallBack cframe_callback_ = nullptr;
  void *cframe_callback_ctx_;
  InnoStatusCallBack status_callback_ = nullptr;
  std::thread sub_thread_status;

 private:
};
}  // namespace innovusion