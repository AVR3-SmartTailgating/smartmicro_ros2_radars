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

#include "umrr_ros2_driver/radar_cluster_viz_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

namespace smartmicro
{
namespace drivers
{
namespace radar
{

RadarClusterVizNode::RadarClusterVizNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node{"radar_cluster_viz", node_options}
{
  declare_parameters();

  const auto input_topic = this->get_parameter("input_topic").as_string();
  const auto output_topic = this->get_parameter("output_topic").as_string();
  const auto qos_depth = static_cast<size_t>(this->get_parameter("qos_depth").as_int());

  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic, qos_depth,
    std::bind(&RadarClusterVizNode::pointcloud_callback, this, std::placeholders::_1));

  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    output_topic, qos_depth);

  RCLCPP_INFO(this->get_logger(), "Radar cluster viz node initialized.");
  RCLCPP_INFO(this->get_logger(), "  Input:  %s", input_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "  Output: %s", output_topic.c_str());
}

void RadarClusterVizNode::declare_parameters()
{
  this->declare_parameter<std::string>("input_topic", "smart_radar/can_targets_0/filtered");
  this->declare_parameter<std::string>("output_topic", "smart_radar/cluster_markers");
  this->declare_parameter<int>("qos_depth", 10);
  this->declare_parameter<double>("cluster_eps", 2.0);
  this->declare_parameter<int>("cluster_min_points", 2);
  this->declare_parameter<double>("marker_alpha", 0.4);
  this->declare_parameter<double>("marker_z_height", 1.0);
}

void RadarClusterVizNode::pointcloud_callback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  const auto cluster_eps = this->get_parameter("cluster_eps").as_double();
  const auto cluster_min_points = static_cast<int>(this->get_parameter("cluster_min_points").as_int());
  const auto marker_alpha = this->get_parameter("marker_alpha").as_double();
  const auto marker_z_height = this->get_parameter("marker_z_height").as_double();

  const auto num_points = msg->width * msg->height;

  // Handle empty cloud — clean up stale markers
  if (num_points == 0) {
    visualization_msgs::msg::MarkerArray cleanup;
    for (int i = 1; i <= prev_marker_count_; ++i) {
      visualization_msgs::msg::Marker del;
      del.header = msg->header;
      del.ns = "radar_clusters";
      del.id = i;
      del.action = visualization_msgs::msg::Marker::DELETE;
      cleanup.markers.push_back(del);
    }
    if (!cleanup.markers.empty()) {
      marker_pub_->publish(cleanup);
    }
    prev_marker_count_ = 0;
    return;
  }

  // Find field offsets
  const auto point_step = msg->point_step;
  const auto & data = msg->data;

  std::size_t off_x = 0, off_y = 0, off_z = 0;
  for (const auto & field : msg->fields) {
    if (field.name == "x") { off_x = field.offset; }
    else if (field.name == "y") { off_y = field.offset; }
    else if (field.name == "z") { off_z = field.offset; }
  }

  auto read_float = [&data, point_step](std::size_t idx, std::size_t offset) -> float {
    float val;
    std::memcpy(&val, &data[idx * point_step + offset], sizeof(float));
    return val;
  };

  // Extract positions
  std::vector<float> px(num_points), py(num_points), pz(num_points);
  for (std::size_t i = 0; i < num_points; ++i) {
    px[i] = read_float(i, off_x);
    py[i] = read_float(i, off_y);
    pz[i] = read_float(i, off_z);
  }

  // Run DBSCAN on (x, y)
  auto labels = dbscan_cluster(
    px, py, static_cast<float>(cluster_eps), cluster_min_points);

  // Find max cluster id
  int max_cluster = 0;
  for (auto l : labels) {
    max_cluster = std::max(max_cluster, l);
  }

  // Build bounding box marker per cluster
  visualization_msgs::msg::MarkerArray marker_array;

  for (int cid = 1; cid <= max_cluster; ++cid) {
    float x_min = std::numeric_limits<float>::max();
    float x_max = std::numeric_limits<float>::lowest();
    float y_min = std::numeric_limits<float>::max();
    float y_max = std::numeric_limits<float>::lowest();
    float z_min = std::numeric_limits<float>::max();
    float z_max = std::numeric_limits<float>::lowest();
    int count = 0;

    for (std::size_t i = 0; i < num_points; ++i) {
      if (labels[i] == cid) {
        x_min = std::min(x_min, px[i]);
        x_max = std::max(x_max, px[i]);
        y_min = std::min(y_min, py[i]);
        y_max = std::max(y_max, py[i]);
        z_min = std::min(z_min, pz[i]);
        z_max = std::max(z_max, pz[i]);
        ++count;
      }
    }

    if (count == 0) { continue; }

    // Enforce minimum bounding box size so single-point clusters are visible
    constexpr float min_dim = 0.5f;
    if (x_max - x_min < min_dim) { x_min -= min_dim / 2; x_max += min_dim / 2; }
    if (y_max - y_min < min_dim) { y_min -= min_dim / 2; y_max += min_dim / 2; }

    const float z_center = (z_min + z_max) / 2.0f;

    visualization_msgs::msg::Marker marker;
    marker.header = msg->header;
    marker.ns = "radar_clusters";
    marker.id = cid;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = static_cast<double>(x_min + x_max) / 2.0;
    marker.pose.position.y = static_cast<double>(y_min + y_max) / 2.0;
    marker.pose.position.z = static_cast<double>(z_center);
    marker.pose.orientation.w = 1.0;

    marker.scale.x = std::max(static_cast<double>(x_max - x_min), 0.3);
    marker.scale.y = std::max(static_cast<double>(y_max - y_min), 0.3);
    marker.scale.z = marker_z_height;

    auto color = color_from_id(cid);
    marker.color.r = color.r;
    marker.color.g = color.g;
    marker.color.b = color.b;
    marker.color.a = static_cast<float>(marker_alpha);

    // Auto-expire after 0.5s if no updates
    marker.lifetime = rclcpp::Duration(0, 500000000);

    marker_array.markers.push_back(marker);
  }

  // Delete stale markers from previous frame
  for (int i = max_cluster + 1; i <= prev_marker_count_; ++i) {
    visualization_msgs::msg::Marker del;
    del.header = msg->header;
    del.ns = "radar_clusters";
    del.id = i;
    del.action = visualization_msgs::msg::Marker::DELETE;
    marker_array.markers.push_back(del);
  }
  prev_marker_count_ = max_cluster;

  marker_pub_->publish(marker_array);
}

std::vector<int> RadarClusterVizNode::dbscan_cluster(
  const std::vector<float> & px,
  const std::vector<float> & py,
  float eps, int min_pts)
{
  const std::size_t n = px.size();
  const float eps_sq = eps * eps;

  std::vector<int> labels(n, 0);  // 0=unvisited, -1=noise, >0=cluster
  int cluster_id = 0;

  auto region_query = [&](std::size_t idx) -> std::vector<std::size_t> {
    std::vector<std::size_t> neighbors;
    for (std::size_t j = 0; j < n; ++j) {
      const float dx = px[idx] - px[j];
      const float dy = py[idx] - py[j];
      if (dx * dx + dy * dy <= eps_sq) {
        neighbors.push_back(j);
      }
    }
    return neighbors;
  };

  for (std::size_t i = 0; i < n; ++i) {
    if (labels[i] != 0) { continue; }

    auto neighbors = region_query(i);
    if (static_cast<int>(neighbors.size()) < min_pts) {
      labels[i] = -1;
      continue;
    }

    ++cluster_id;
    labels[i] = cluster_id;

    std::vector<std::size_t> seed_set(neighbors.begin(), neighbors.end());
    for (std::size_t si = 0; si < seed_set.size(); ++si) {
      std::size_t q = seed_set[si];
      if (labels[q] == -1) { labels[q] = cluster_id; }
      if (labels[q] != 0) { continue; }
      labels[q] = cluster_id;

      auto q_neighbors = region_query(q);
      if (static_cast<int>(q_neighbors.size()) >= min_pts) {
        for (auto nn : q_neighbors) {
          seed_set.push_back(nn);
        }
      }
    }
  }

  return labels;
}

RadarClusterVizNode::Color RadarClusterVizNode::color_from_id(int cluster_id)
{
  // Golden ratio conjugate for uniform hue distribution
  constexpr float golden = 0.618033988749895f;
  float h = std::fmod(cluster_id * golden, 1.0f);
  float s = 0.85f;
  float v = 0.95f;

  // HSV to RGB
  int hi = static_cast<int>(h * 6.0f);
  float f = h * 6.0f - static_cast<float>(hi);
  float p = v * (1.0f - s);
  float q = v * (1.0f - f * s);
  float t = v * (1.0f - (1.0f - f) * s);

  switch (hi % 6) {
    case 0: return {v, t, p};
    case 1: return {q, v, p};
    case 2: return {p, v, t};
    case 3: return {p, q, v};
    case 4: return {t, p, v};
    default: return {v, p, q};
  }
}

}  // namespace radar
}  // namespace drivers
}  // namespace smartmicro

RCLCPP_COMPONENTS_REGISTER_NODE(smartmicro::drivers::radar::RadarClusterVizNode)
