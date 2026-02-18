// Copyright 2024 Josh Esplin
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef UMRR_ROS2_DRIVER__RADAR_CLUSTER_VIZ_NODE_HPP_
#define UMRR_ROS2_DRIVER__RADAR_CLUSTER_VIZ_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <umrr_ros2_driver/visibility_control.hpp>

#include <string>
#include <vector>

namespace smartmicro
{
namespace drivers
{
namespace radar
{

class UMRR_ROS2_DRIVER_PUBLIC RadarClusterVizNode : public ::rclcpp::Node
{
public:
  explicit RadarClusterVizNode(const rclcpp::NodeOptions & node_options);

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void declare_parameters();

  std::vector<int> dbscan_cluster(
    const std::vector<float> & px,
    const std::vector<float> & py,
    float eps, int min_pts);

  struct Color { float r, g, b; };
  static Color color_from_id(int cluster_id);

  int prev_marker_count_{0};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace radar
}  // namespace drivers
}  // namespace smartmicro

#endif  // UMRR_ROS2_DRIVER__RADAR_CLUSTER_VIZ_NODE_HPP_
