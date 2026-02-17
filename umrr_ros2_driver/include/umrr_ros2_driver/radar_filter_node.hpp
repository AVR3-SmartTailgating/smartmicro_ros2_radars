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

#ifndef UMRR_ROS2_DRIVER__RADAR_FILTER_NODE_HPP_
#define UMRR_ROS2_DRIVER__RADAR_FILTER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <umrr_ros2_driver/visibility_control.hpp>

#include <deque>
#include <string>
#include <vector>

#include "umrr_ros2_msgs/msg/can_target_header.hpp"

namespace smartmicro
{
namespace drivers
{
namespace radar
{

class UMRR_ROS2_DRIVER_PUBLIC RadarFilterNode : public ::rclcpp::Node
{
public:
  explicit RadarFilterNode(const rclcpp::NodeOptions & node_options);

private:
  void targets_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void header_callback(const umrr_ros2_msgs::msg::CanTargetHeader::SharedPtr msg);

  void declare_filter_parameters();

  void apply_persistence_filter(
    const sensor_msgs::msg::PointCloud2 & cloud_msg,
    std::vector<bool> & keep);
  void apply_cluster_filter(
    const sensor_msgs::msg::PointCloud2 & cloud_msg,
    std::vector<bool> & keep);

  struct FilterParams
  {
    bool enable_range_gate{true};
    bool enable_azimuth_gate{true};
    bool enable_elevation_gate{true};
    bool enable_snr_filter{true};
    bool enable_rcs_filter{true};
    bool enable_speed_filter{true};
    bool enable_persistence_filter{false};
    bool enable_cluster_filter{false};

    double range_min{8.0};
    double range_max{65.0};
    double azimuth_max{0.105};
    double elevation_min{-0.087};
    double elevation_max{0.087};
    double z_min{-0.5};
    double z_max{3.0};
    double snr_min{12.0};
    double rcs_min{5.0};
    double speed_min{0.5};

    int persistence_n{3};
    int persistence_m{5};
    double persistence_dist{2.0};

    double cluster_eps{2.0};
    int cluster_min_points{2};
  };

  FilterParams params_;

  struct Point2D
  {
    float x;
    float y;
  };
  std::deque<std::vector<Point2D>> scan_history_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr targets_sub_;
  rclcpp::Subscription<umrr_ros2_msgs::msg::CanTargetHeader>::SharedPtr header_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr targets_pub_;
  rclcpp::Publisher<umrr_ros2_msgs::msg::CanTargetHeader>::SharedPtr header_pub_;

  umrr_ros2_msgs::msg::CanTargetHeader::SharedPtr latest_header_;
};

}  // namespace radar
}  // namespace drivers
}  // namespace smartmicro

#endif  // UMRR_ROS2_DRIVER__RADAR_FILTER_NODE_HPP_
