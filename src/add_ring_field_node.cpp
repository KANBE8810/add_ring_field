#include <chrono>
#include <iostream>
#include <memory>

#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

struct PointXYZRI
{
  PCL_ADD_POINT4D;
  float intensity;
  int ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRI,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring))

void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<PointXYZRI>::Ptr cloud_xyzri(new pcl::PointCloud<PointXYZRI>);
  pcl::fromROSMsg(*msg, *cloud_xyzri);

  // Add extra field "ring" to PointCloud2
  for (int i = 0; i < cloud_xyzri->points.size(); ++i) {
    cloud_xyzri->points[i].ring = 0;
  }
  std::cout << "pcd " << cloud_xyzri->width << " " << cloud_xyzri->height << std::endl;
  cloud_xyzri->is_dense = true;
  sensor_msgs::msg::PointCloud2 cloud_ros;
  pcl::toROSMsg(*cloud_xyzri, cloud_ros);

  cloud_ros.header = msg->header;
  // Do something with cloud_ros here

  static rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr s_pub;
  s_pub->publish(cloud_ros);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("add_ring_to_pointcloud2");
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub = node->create_subscription<sensor_msgs::msg::PointCloud2>("input_pointcloud", 10, callback);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

