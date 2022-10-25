#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "driver_falcon.h"
#include "point_types.h"
#include "rclcpp/rclcpp.hpp"

typedef innovusion_pointcloud::PointXYZIT iVuPoint;
typedef pcl::PointCloud<iVuPoint> iVuPointCloud;

class InnovusionComponent : public rclcpp::Node {
 public:
  InnovusionComponent()
      : Node("ivu_pub",
             rclcpp::NodeOptions()
                 .allow_undeclared_parameters(true)
                 .automatically_declare_parameters_from_overrides(true)
                 .use_intra_process_comms(true)) {
    using std::string;

    driver_.reset(new innovusion::DriverFalcon(data_callback_s_, this));

    this->get_parameter_or<string>("lidar_name", driver_->lidar_name, "falcon");
    this->get_parameter_or<string>("frame_id", driver_->frame_id, "innovusion");
    this->get_parameter_or<uint32_t>("lidar_id", driver_->lidar_id, 0);
    this->get_parameter_or<string>("lidar_ip", driver_->lidar_ip,
                                   "172.168.1.10");
    this->get_parameter_or<uint32_t>("lidar_port", driver_->lidar_port, 8010);
    this->get_parameter_or<string>("lidar_model", driver_->lidar_model,
                                   "rev_i");
    this->get_parameter_or<uint32_t>("reflectance", driver_->reflectance, 1);
    this->get_parameter_or<uint32_t>("multireturn", driver_->multireturn, 1);
    this->get_parameter_or<string>("data_filename", driver_->data_filename, "");
    this->get_parameter_or<string>("yaml_filename", driver_->yaml_filename, "");
    this->get_parameter_or<uint32_t>("file_speed", driver_->file_speed, 10000);
    this->get_parameter_or<int32_t>("file_rewind", driver_->file_rewind, 0);
    this->get_parameter_or<int32_t>("file_skip", driver_->file_skip, 0);
    this->get_parameter_or<int32_t>("lidar_udp_port", driver_->lidar_udp_port,
                                    0);
    this->get_parameter_or<uint32_t>("processed", driver_->processed, 0);
    this->get_parameter_or<bool>("set_falcon_eye", driver_->set_falcon_eye,
                                 false);
    this->get_parameter_or<int32_t>("roi_center_h", driver_->roi_center_h, 0);
    this->get_parameter_or<int32_t>("roi_center_v", driver_->roi_center_v, 0);
    this->get_parameter_or<uint32_t>("inno_log_level", driver_->inno_log_level,
                                     2);
    RCLCPP_INFO(this->get_logger(),
                "\n\tlidar_name: %s, frame_id: %s, lidar_id: %d\n"
                "\tlidar_ip: %s, lidar_port: %d, lidar_model: %s\n"
                "\treflectance: %d, multireturn: %d\n"
                "\tdata_filename: %s\n"
                "\tyaml_filename: %s\n"
                "\tfile_speed: %d, file_rewind: %d,file_skip: %d\n"
                "\tlidar_udp_port: %d, processed %d\n"
                "\tset_falcon_eye: %d, roi_center_h: %d,roi_center_v: %d\n"
                "\tinno_log_level: %d",
                driver_->lidar_name.c_str(), driver_->frame_id.c_str(),
                driver_->lidar_id, driver_->lidar_ip.c_str(),
                driver_->lidar_port, driver_->lidar_model.c_str(),
                driver_->reflectance, driver_->multireturn,
                driver_->data_filename.c_str(), driver_->yaml_filename.c_str(),
                driver_->file_speed, driver_->file_rewind, driver_->file_skip,
                driver_->lidar_udp_port, driver_->processed,
                driver_->set_falcon_eye, driver_->roi_center_h,
                driver_->roi_center_v, driver_->inno_log_level);

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ivu_points",
                                                                 qos);
    driver_->start();  // do not forget to turn on lidar stream
  }

 private:
  // static callback wrapper, called by the driver_
  static int data_callback_s_(int lidar_handle, void *ctx, void *frame) {
    return (reinterpret_cast<InnovusionComponent *>(ctx))
        ->data_callback_(frame);
  }

  int data_callback_(void *cframe) {
    inno_cframe_header *frame = (inno_cframe_header *)cframe;
    // only use INNO_CFRAME_CPOINT
    if (frame && frame->type == INNO_CFRAME_CPOINT) {
      auto start = std::chrono::high_resolution_clock::now();
      // get every point from frame
      pcl::PointCloud<iVuPoint>::Ptr cloud_ptr(new pcl::PointCloud<iVuPoint>);
      double ts_start_s = frame->ts_us_start / 1e6;
      for (unsigned int i = 0; i < frame->item_number; i++) {
        inno_cpoint *p = &frame->cpoints[i];

        double radius = p->radius / 100.0;
        float x = radius * sin(p->v_angle * cpoint_angle_unit_c);
        float t = radius * cos(p->v_angle * cpoint_angle_unit_c);
        float y = t * sin(p->h_angle * cpoint_angle_unit_c);
        float z = t * cos(p->h_angle * cpoint_angle_unit_c);

        iVuPoint pt;
        pt.x = x;
        pt.y = y;
        pt.z = z;
        pt.intensity = static_cast<uint>(p->ref);
        pt.timestamp = p->ts_100us / 1e4 + ts_start_s;
        cloud_ptr->points.push_back(pt);
      }
      cloud_ptr->height = 1;
      cloud_ptr->width = frame->item_number;

      // create msg
      pc2_msg_ = std::make_unique<sensor_msgs::msg::PointCloud2>();
      pcl::toROSMsg(*cloud_ptr, *pc2_msg_);
      pc2_msg_.get()->header.frame_id = driver_->frame_id;
      pc2_msg_.get()->header.stamp = now();
      auto end = std::chrono::high_resolution_clock::now();
      auto delay_s =
          std::chrono::duration_cast<std::chrono::nanoseconds>(end - start)
              .count() *
          1e-9;
      pub_->publish(std::move(pc2_msg_));
      iVuPoint &pt = cloud_ptr->points[0];
      RCLCPP_INFO(this->get_logger(),
                  "send frame num: %d, process: %lfs, pt[0].ts: %f",
                  frame->item_number, delay_s, pt.timestamp);

      return frame->item_number;
    }
    return 0;
  }

 private:
  std::shared_ptr<innovusion::DriverFactory> driver_ = nullptr;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  std::unique_ptr<sensor_msgs::msg::PointCloud2> pc2_msg_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InnovusionComponent>());
  rclcpp::shutdown();
  return 0;
}