/**
 *  Copyright (C) 2022 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include <ctype.h>
#include <sys/types.h>

#ifndef __MINGW64__
#include <arpa/inet.h>
#include <netinet/in.h>
#else
#include <winsock2.h>
#endif

#include <algorithm>
#include <functional>
#include <mutex>  //NOLINT
#include <queue>
#include <string>
#include <thread>  //NOLINT

#include "src/sdk_client/lidar_client_communication.h"
#include "src/sdk_common/inno_lidar_packet_utils.h"
#include "src/utils/log.h"
#include "src/utils/mem_pool_manager.h"
#include "src/utils/utils.h"

/**
 * @brief get cali data by udp, parser data to csv
 * 1. parser command line(args: lidar-ip, lidar-port, cali-data-port,
 * frame-number, output-filename)
 * 2. check lidar status
 * 3. network init, create socket
 * 4. check frame number, read cali data and combine one frame of data
 * 5. write file
 * 6. check frame number is end
 */

#define ERROR_EXIT 1

using namespace innovusion;  // NOLINT

DEFINE_INNO_COMPACT_STRUCT(Buffer) {
  size_t len;
  char address[0];
};

DEFINE_INNO_COMPACT_STRUCT(RecordPoint) {
  uint16_t frame_id;
  float x;
  float y;
  float z;
  uint8_t channel;
  uint8_t facet;
  int16_t poly_angle;
  int16_t galvo_angle;
  uint16_t ref_intensity;
  uint32_t radius;
  uint32_t intensity;
  float reflectance;
};

class Record {
 public:
  explicit Record(const std::string& filename)
      : fd_(NULL), filename_(filename) {
    fd_ = fopen(filename_.c_str(), "w+");
    inno_log_verify(fd_, "open file fail");
  }

  virtual void write_packet(InnoCaliDataPacket* packet) {
    inno_log_verify(packet, "packet is nullptr");

    for (int i = 0; i < packet->item_count; i++) {
      write_point(packet->frame_id, (packet->points + i));
    }
  }

  virtual void write_point(uint16_t frame_id, InnoCaliDataPoint* point) {
    inno_log_verify(point, "point is nullptr");

    InnoXyzrD xyz;

    InnoBlockAngles angles;

    angles.h_angle = point->h_angle;
    angles.v_angle = point->v_angle;

    InnoDataPacketUtils::get_xyzr_meter(angles, point->radius, point->channel,
                                        &xyz);

    RecordPoint record_point;
    record_point.frame_id = frame_id;
    record_point.x = xyz.x;
    record_point.y = xyz.y;
    record_point.z = xyz.z;

    record_point.channel = point->channel;
    record_point.facet = point->channel;
    record_point.poly_angle = point->poly_angle;
    record_point.galvo_angle = point->galvo_angle;
    record_point.ref_intensity = point->ref_intensity;
    record_point.radius = point->radius;
    record_point.intensity = point->intensity;
    record_point.reflectance = point->reflectance;
    write_point(&record_point);
  }

  virtual void write_point(RecordPoint* point) = 0;

  virtual ~Record() {
    if (fd_) {
      fclose(fd_);
      fd_ = NULL;
    }
  }

  virtual void write_header() {}
  virtual void write_end() {}

 protected:
  FILE* fd_;
  std::string filename_;
};

class CSVRecord : public Record {
 public:
  explicit CSVRecord(const std::string& filename) : Record(filename) {}

  void write_point(RecordPoint* point) override {
    inno_log_verify(point, "point is nullptr");

    static const char* csv_format =
        "\n%u,%.3f,%.3f,%.3f,%u,%u,%u,%u,%u,%u,%u,%.3f";

    static const int kMaxCsvBufferSize = 1024;
    char buffer[kMaxCsvBufferSize];

    int csv_len =
        snprintf(buffer, sizeof(buffer), csv_format, point->frame_id, point->x,
                 point->y, point->z, point->channel, point->facet,
                 point->poly_angle, point->galvo_angle, point->ref_intensity,
                 point->radius, point->intensity, point->reflectance);

    inno_log_verify(csv_len > 0, "snprintf fail");

    std::size_t n = fwrite(buffer, 1, csv_len, fd_);

    inno_log_verify(n == static_cast<std::size_t>(csv_len),
                    "fwrite return %" PRI_SIZELU, n);
  }

  void write_header() override {
#if (defined(__APPLE__) || defined(__MINGW64__))
    if (fd_->_file < 0) return;
#else
#ifndef _QNX_
    if (fd_->_fileno < 0) return;
#endif
#endif

    static const char csv_header[] =
        "frame_id,x,y,z,channel,facet,poly_angle,galvo_angle,ref_intensity,"
        "radius,"
        "intensity,reflectance";

    std::size_t n = fwrite(csv_header, 1, sizeof(csv_header) - 1, fd_);

    inno_log_verify(n == sizeof(csv_header) - 1, "fwrite return %" PRI_SIZELU,
                    n);
  }
};

class PCDRecord : public Record {
 public:
  explicit PCDRecord(const std::string& filename)
      : Record(filename), point_count_(0) {}

  void write_point(RecordPoint* point) override {
    inno_log_verify(point, "point is nullptr");

    int n = fwrite(point, 1, sizeof(RecordPoint), fd_);
    inno_log_verify(n >= 0, "cannot write file");

    point_count_++;
  }

  void write_header() override {
    memset(header_buffer_, ' ', sizeof(header_buffer_));
    header_buffer_[kHeaderMaxSize - 1] = '\n';
    int n = fwrite(header_buffer_, 1, sizeof(header_buffer_), fd_);
    inno_log_verify(n >= 0, "cannot write file");
  }

  void write_end() override {
    const char pcd_header[] =
        "# .PCD v.7 - Point Cloud Data file format\n"
        "FIELDS frame_id x y z channel facet poly_angle galvo_angle "
        "ref_intensity radius intensity reflectance\n"
        "SIZE 2 4 4 4 1 1 2 2 2 4 4 4\n"
        "TYPE U F F F U U I I U U U F\n"
        "COUNT 1 1 1 1 1 1 1 1 1 1 1 1\n"
        "WIDTH %u\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        "POINTS %u\n"
        "DATA binary";
    fseek(fd_, 0, SEEK_SET);
    int header_len = snprintf(header_buffer_, sizeof(header_buffer_),
                              pcd_header, point_count_, point_count_);
    int n = fwrite(header_buffer_, 1, header_len, fd_);
    inno_log_verify(n >= 0, "cannot write file");
  }

 protected:
  static const int kHeaderMaxSize = 340;
  char header_buffer_[kHeaderMaxSize];
  uint32_t point_count_;
};

class CaliDataReader {
 public:
  explicit CaliDataReader(const std::string& filename, const std::string& ip,
                          uint16_t lidar_port, uint16_t udp_port,
                          uint32_t frame_number)
      : recorder_(NULL),
        fd_(-1),
        ip_(ip),
        port_(udp_port),
        frame_number_(frame_number),
        cur_frame_number_(-1),
        cur_frame_id_(-1),
        cur_sub_frame_id_(-1),
        has_last_sub_frame_(false),
        is_done_(false),
        comm_(ip.c_str(), lidar_port, 0.5),
        record_thread_(NULL),
        mem_pool_(NULL) {
#ifdef __MINGW64__
    if (WSAStartup(MAKEWORD(2, 2), &data_) != 0) {
      exit(ERROR_EXIT);
    }
#endif
    std::size_t idx = filename.find_last_of(".");

    inno_log_verify(idx != std::string::npos, "invalid file type");
    std::string file_type = filename.substr(idx + 1);

    if (file_type == "csv")
      recorder_ = new CSVRecord(filename);
    else if (file_type == "pcd")
      recorder_ = new PCDRecord(filename);
    else
      inno_log_error("invalid file type(%s)", file_type.c_str());
    recorder_->write_header();

    inno_log_verify(recorder_, "recorder is nullptr");

    int max_jobs = std::max(frame_number, 10U) * 50;
    mem_pool_ = new MemPool("buffer mem pool", sizeof(Buffer) + kMaxReadSize,
                            max_jobs, 32);
    inno_log_verify(mem_pool_, "mem_pool is nullptr");
  }

  ~CaliDataReader() {
    if (recorder_) {
      recorder_->write_end();
      delete recorder_;
      recorder_ = NULL;
    }

    while (!buffer_queue_.empty()) {
      Buffer* buffer = buffer_queue_.front();
      buffer_queue_.pop();
      mem_pool_->free(buffer);
    }

    if (fd_ != -1) {
      InnoUtils::close_fd(fd_);
      fd_ = -1;
    }

    if (is_done()) {
      int ret = comm_.set_attribute_string("send_cali_data", "0");
      inno_log_verify(ret == 0, "set send cali data command fail");

      char result[10];
      ret = comm_.get_attribute("send_cali_data", result, sizeof(result));
      inno_log_verify(ret == 0, "get send cali data command fail");

      if (result[0] != '0') {
        inno_log_error("close pcs send cali data fial");
        exit(ERROR_EXIT);
      }
    }

    if (record_thread_) {
      delete record_thread_;
      record_thread_ = NULL;
    }

    if (mem_pool_) {
      delete mem_pool_;
      mem_pool_ = NULL;
    }
#ifdef __MINGW64__
    WSACleanup();
#endif
  }

  inline bool is_done() const { return is_done_; }

  void start() {
    if (is_done()) {
      inno_log_error("CaliDataReader is done");
      return;
    }

    start_read_before();
    record_thread_ = new std::thread(
        std::bind(&CaliDataReader::process_cali_data_packet, this));
    inno_log_verify(record_thread_, "record_thread is nullptr");
    read_cali_data();
    record_thread_->join();
  }

 private:
  Record* recorder_;

  int fd_;
  std::string ip_;
  uint16_t port_;

  uint32_t frame_number_;
  uint32_t cur_frame_number_;
  int cur_frame_id_;
  int cur_sub_frame_id_;
  bool has_last_sub_frame_;

  bool is_done_;

  LidarClientCommunication comm_;

  std::thread* record_thread_;
  std::mutex mutex_;
  std::queue<Buffer*> buffer_queue_;

  MemPool* mem_pool_;

  static const int kMaxReadSize = 65535;
#ifdef __MINGW64__
  WSADATA data_;
#endif

  void start_read_before() {
    if (!InnoUtils::check_ip_valid(ip_.c_str())) {
      inno_log_error("invalid ip address(%s)", ip_.c_str());
      exit(ERROR_EXIT);
    }

    // set pcs send cali data
    int ret = comm_.set_attribute_string("send_cali_data", "1");
    inno_log_verify(ret == 0, "set send cali data command fail");

    char result[10];
    ret = comm_.get_attribute("send_cali_data", result, sizeof(result));
    inno_log_verify(ret == 0, "get send cali data command fail");

    if (result[0] != '1') {
      inno_log_error("open pcs send cali data fial");
      exit(ERROR_EXIT);
    }

    // bind socket
    std::vector<InnoUdpOpt> opts;

    int32_t ports[3];  // data message status
    char ip[64];
    char my_ip[64];
    ret = comm_.get_server_udp_ports_ip(&ports[0], &ports[1], &ports[2], ip,
                                        sizeof(ip), my_ip, sizeof(my_ip));

    inno_log_verify(ret == 0, "cannot get server udp ports %d", ret);
    inno_log_info("read udps: data:%d message:%d status:%d ip=%s my_ip=%s",
                  ports[0], ports[1], ports[2], ip, my_ip);

    if (ip[0] == 0 || ip[0] == '0') {
      inno_log_error("LIDAR needs to be configured as multicast or broadcast");
      exit(ERROR_EXIT);
    }

    struct ip_mreq mreq_;
    if (my_ip[0] != 0) {
      mreq_.imr_multiaddr.s_addr = inet_addr(ip);
      mreq_.imr_interface.s_addr = inet_addr(my_ip);
      inno_log_verify(mreq_.imr_multiaddr.s_addr != INADDR_NONE,
                      "bad ip m-addr %s", ip);
      inno_log_verify(mreq_.imr_interface.s_addr != INADDR_NONE,
                      "bad ip i-addr %s", my_ip);
      inno_log_info("use multicast addr %s on interface %s", ip, my_ip);
      // set opt
      opts.emplace_back(IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq_, sizeof(mreq_),
                        "IP_ADD_MEMBERSHIP");
    }

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 500000;
    opts.emplace_back(SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv), "SO_RCVTIMEO");

    inno_log_info("start bind socket port(%u)", port_);
    fd_ = InnoUdpHelper::bind(port_, opts);
    inno_log_verify(fd_ >= 0, "socket fd < 0");
  }

  void read_cali_data() {
    // var
    struct sockaddr_in cliaddr;
    socklen_t len = sizeof(cliaddr);
    int n;

    while (!is_done()) {
      Buffer* buffer = reinterpret_cast<Buffer*>(mem_pool_->alloc());

      // read cail data
#ifndef __MINGW64__
      while (-1 ==
                 (n = recvfrom(fd_, buffer->address, kMaxReadSize, MSG_WAITALL,
                               (struct sockaddr*)&cliaddr, &len)) &&
             errno == EINTR) {
      }
#else
      while (-1 == (n = recvfrom(fd_, buffer->address, kMaxReadSize, 0,
                                 (struct sockaddr*)&cliaddr, &len)) &&
             errno == EINTR) {
      }
#endif

      if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          // EAGAIN means timeout
          inno_log_info("%s", errno == EAGAIN ? "EAGAIN" : "EWOULDBLOCK");
        } else {
          inno_log_error_errno("recv port=%d, n=%d, fd=%d", port_, n, fd_);
        }
        mem_pool_->free(buffer);
      } else {
        buffer->len = n;
        std::unique_lock<std::mutex> lk(mutex_);
        buffer_queue_.push(buffer);
      }
    }
  }

  bool check_packet_valid(char* buffer, int len) {
    if (len <= 0) {
      return false;
    }

    if (static_cast<std::size_t>(len) < sizeof(InnoCaliDataPacket)) {
      inno_log_warning("cali data packet len too small");
      return false;
    }

    InnoCaliDataPacket* packet = reinterpret_cast<InnoCaliDataPacket*>(buffer);

    if (sizeof(InnoCaliDataPacket) +
            packet->item_count * sizeof(InnoCaliDataPoint) !=
        static_cast<std::size_t>(len)) {
      static uint32_t cnt = 0;
      if (cnt++ == 10) exit(ERROR_EXIT);
      inno_log_warning("cali data packet len fail");
      return false;
    }

    uint32_t crc = InnoUtils::crc32_start();
    crc = innovusion::InnoUtils::crc32_do(
        crc, packet->points, packet->item_count * sizeof(InnoCaliDataPoint));

    if (packet->checksum != crc) {
      inno_log_warning("cali data packet checksum fail");
      return false;
    }

    return true;
  }

  void process_cali_data_packet() {
    while (!is_done()) {
      Buffer* buffer = NULL;
      {
        std::unique_lock<std::mutex> lk(mutex_);
        if (buffer_queue_.size() > 0) {
          buffer = buffer_queue_.front();
          buffer_queue_.pop();
        }
      }
      if (buffer) {
        parser_cali_data_packet(buffer->address, buffer->len);
        mem_pool_->free(buffer);
      } else {
        usleep(2000);
      }
    }
  }

  void parser_cali_data_packet(char* buffer, int len) {
    if (is_done()) return;

    if (!check_packet_valid(buffer, len)) {
      return;
    }

    InnoCaliDataPacket* packet = reinterpret_cast<InnoCaliDataPacket*>(buffer);

    if (packet->frame_id != cur_frame_id_) {
      if (cur_frame_id_ != -1 && cur_frame_id_ + 1 != packet->frame_id)
        inno_log_error("frame_id discontinuous, now is %d, is not %d",
                       packet->frame_id, cur_frame_id_ + 1);
      if (cur_frame_id_ != -1 && !has_last_sub_frame_)
        inno_log_error("frame_id %d have not last_sub_frame", cur_frame_id_);

      cur_frame_number_++;
      cur_frame_id_ = packet->frame_id;
      cur_sub_frame_id_ = packet->sub_id;
      has_last_sub_frame_ = false;

      if (cur_frame_number_ > frame_number_) {
        is_done_ = true;
        return;
      }
    } else {
      cur_sub_frame_id_++;
    }

    if (cur_sub_frame_id_ != packet->sub_id) {
      inno_log_error("sub_id discontinuous, now is %d, is not %d",
                     packet->sub_id, cur_sub_frame_id_);
    }

    if (packet->is_last_sub_frame) has_last_sub_frame_ = true;

    // the data of the first frame may be incomplete, so the data starts from
    // the second frame
    if (cur_frame_number_ >= 1) {
      inno_log_info(
          "cali data pakcet(frame_id = %d, sub_id = %d, item_count = %d)",
          packet->frame_id, packet->sub_id, packet->item_count);
      if (recorder_) {
        recorder_->write_packet(packet);
      }
    }
  }
};

/***********************
 * usage()
 ***********************/
void usage(const char* arg0) {
  inno_fprintf(2,
               "Usage:\n"
               "   %s\n"
               "\t{[--lidar-ip <LIDAR_IP>]\n"
               "\t\t[--lidar-port <LIDAR_PORT>]\n"
               "\t\t[--lidar-cali-data-port <LIDAR_CALI_DATA_UDP_PORT>]}\n"
               "\t[--frame-number <NUMBER_OF_FRAME>]\n"
               "\t[--output-filename <OUTPUT_FILENAME.csv|pcd>]\n"
               "\t[--help]\n",
               arg0);
  inno_fprintf(
      2,
      "\n"
      "Examples:\n"
      " --record cali data from live LIDAR to test.csv (LIDAR "
      "is configured to multicast or broadcast).\n"
      "   %s --output-filename test.csv\n\n"
      " --record cali data from live LIDAR to test.pcd (LIDAR "
      "is configured to multicast or broadcast).\n"
      "   %s --output-filename test.pcd\n\n"
      " --record 100 frames cali data from live LIDAR to test.csv (LIDAR "
      "is configured to multicast or broadcast).\n"
      "   %s --frame-number 100 --output-filename test.csv\n\n",
      arg0, arg0, arg0);
  return;
}

int main(int argc, char** argv) {
  if (argc < 2) {
    usage(argv[0]);
    exit(ERROR_EXIT);
  }

  std::string lidar_ip = "172.168.1.10";
  uint16_t lidar_port = 8010;
  uint16_t lidar_cali_data_port = 8012;
  uint32_t frame_number = 20;
  std::string output_filenane = "";

  /* getopt_long stores the option index here. */
  int c;
  struct option long_options[] = {
      /* These options set a flag. */
      {"lidar-ip", required_argument, 0, 'i'},
      {"lidar_port", required_argument, 0, 'p'},
      {"lidar-cali-data-port", required_argument, 0, 'd'},
      {"frame-number", required_argument, 0, 'n'},
      {"output-filename", required_argument, 0, 'o'},
      {"help", no_argument, NULL, 'h'},
      {0, 0, 0, 0}};

  const char* optstring = "i:p:n:o:h";

  while (1) {
    int option_index = 0;
    c = getopt_long(argc, argv, optstring, long_options, &option_index);

    /* Detect the end of the options. */
    if (c == -1) {
      break;
    }

    switch (c) {
      case 0:
        /* If this option set a flag, do nothing else now. */
        if (long_options[option_index].flag != 0) {
          break;
        }
        inno_log_verify(optarg == NULL, "option %s with arg %s",
                        long_options[option_index].name, optarg);
        break;

      case 'i':
        lidar_ip = optarg;
        break;

      case 'p':
        lidar_port = std::stoul(optarg);
        break;

      case 'd':
        lidar_cali_data_port = std::stoul(optarg);
        break;

      case 'n':
        frame_number = std::stoul(optarg);
        break;

      case 'o':
        output_filenane = optarg;
        break;

      case 'h':
        usage(argv[0]);
        exit(0);
        break;

      case '?':
        abort();

      default:
        inno_log_error("unknown options %c\n", c);
        usage(argv[0]);
        exit(ERROR_EXIT);
    }
  }

  CaliDataReader reader(output_filenane, lidar_ip, lidar_port,
                        lidar_cali_data_port, frame_number);

  reader.start();

  return 0;
}
