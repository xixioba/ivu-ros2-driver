#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <memory>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "point_types.h"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

typedef innovusion_pointcloud::PointXYZIT iVuPoint;
typedef pcl::PointCloud<iVuPoint> iVuPointCloud;

class InnovusionComponent : public rclcpp::Node {
 public:
  InnovusionComponent()
      : Node("ivu_sub",
             rclcpp::NodeOptions()
                 .allow_undeclared_parameters(true)
                 .automatically_declare_parameters_from_overrides(true)
                 .use_intra_process_comms(true)) {
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "ivu_points", qos,
        std::bind(&InnovusionComponent::data_callback_, this,
                  std::placeholders::_1));
  }

 private:
  void data_callback_(
      const std::shared_ptr<sensor_msgs::msg::PointCloud2> msg) {
    builtin_interfaces::msg::Time now = this->get_clock()->now();
    auto delay_s = (double)(now.sec - msg->header.stamp.sec +
                            (now.nanosec - msg->header.stamp.nanosec) * 1e-9);
    pcl::PointCloud<iVuPoint>::Ptr cloud_ptr(new pcl::PointCloud<iVuPoint>);
    pcl::fromROSMsg(*msg, *cloud_ptr);
    iVuPoint &pt = cloud_ptr->points[0];
    RCLCPP_INFO(this->get_logger(),
                "get frame num: %d, delay: %f, pt[0].ts: %f", msg->width,
                delay_s, pt.timestamp);
  }

 private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InnovusionComponent>());
  rclcpp::shutdown();
  return 0;
}