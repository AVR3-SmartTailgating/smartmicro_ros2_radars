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

#include "umrr_ros2_driver/radar_filter_node.hpp"

#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>
#include <limits>
#include <string>
#include <tuple>
#include <vector>

namespace
{
constexpr bool float_eq(const float a, const float b) noexcept
{
  const auto maximum = std::max(std::fabs(a), std::fabs(b));
  return std::fabs(a - b) <= maximum * std::numeric_limits<float>::epsilon();
}

struct RadarPoint
{
  float x{};
  float y{};
  float z{};
  float radial_speed{};
  float power{};
  float rcs{};
  float noise{};
  float snr{};
  float azimuth_angle{};
  float elevation_angle{};
  float range{};

  constexpr friend bool operator==(const RadarPoint & p1, const RadarPoint & p2) noexcept
  {
    return float_eq(p1.x, p2.x) && float_eq(p1.y, p2.y) && float_eq(p1.z, p2.z) &&
           float_eq(p1.radial_speed, p2.radial_speed) && float_eq(p1.power, p2.power) &&
           float_eq(p1.rcs, p2.rcs) && float_eq(p1.noise, p2.noise) && float_eq(p1.snr, p2.snr) &&
           float_eq(p1.azimuth_angle, p2.azimuth_angle) &&
           float_eq(p1.elevation_angle, p2.elevation_angle) && float_eq(p1.range, p2.range);
  }
};

LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(radial_speed);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(power);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(rcs);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(noise);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(snr);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(azimuth_angle);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(elevation_angle);
LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(range);

using Generators = std::tuple<
  point_cloud_msg_wrapper::field_x_generator, point_cloud_msg_wrapper::field_y_generator,
  point_cloud_msg_wrapper::field_z_generator, field_radial_speed_generator, field_power_generator,
  field_rcs_generator, field_noise_generator, field_snr_generator, field_azimuth_angle_generator,
  field_elevation_angle_generator, field_range_generator>;

using RadarCloudModifier =
  point_cloud_msg_wrapper::PointCloud2Modifier<RadarPoint, Generators>;

}  // namespace

namespace smartmicro
{
namespace drivers
{
namespace radar
{

RadarFilterNode::RadarFilterNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node{"radar_filter_node", node_options}
{
  declare_filter_parameters();

  const auto targets_input = this->get_parameter("input_targets_topic").as_string();
  const auto header_input = this->get_parameter("input_header_topic").as_string();
  const auto targets_output = this->get_parameter("output_targets_topic").as_string();
  const auto header_output = this->get_parameter("output_header_topic").as_string();
  const auto qos_depth = this->get_parameter("qos_depth").as_int();

  targets_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    targets_input, qos_depth,
    std::bind(&RadarFilterNode::targets_callback, this, std::placeholders::_1));

  header_sub_ = this->create_subscription<umrr_ros2_msgs::msg::CanTargetHeader>(
    header_input, qos_depth,
    std::bind(&RadarFilterNode::header_callback, this, std::placeholders::_1));

  targets_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    targets_output, qos_depth);

  header_pub_ = this->create_publisher<umrr_ros2_msgs::msg::CanTargetHeader>(
    header_output, qos_depth);

  RCLCPP_INFO(this->get_logger(), "Radar filter node initialized.");
  RCLCPP_INFO(this->get_logger(), "  Input:  %s, %s",
    targets_input.c_str(), header_input.c_str());
  RCLCPP_INFO(this->get_logger(), "  Output: %s, %s",
    targets_output.c_str(), header_output.c_str());
  RCLCPP_INFO(this->get_logger(), "  Filters: range=%s azimuth=%s elevation=%s snr=%s "
    "rcs=%s speed=%s persistence=%s cluster=%s",
    params_.enable_range_gate ? "ON" : "OFF",
    params_.enable_azimuth_gate ? "ON" : "OFF",
    params_.enable_elevation_gate ? "ON" : "OFF",
    params_.enable_snr_filter ? "ON" : "OFF",
    params_.enable_rcs_filter ? "ON" : "OFF",
    params_.enable_speed_filter ? "ON" : "OFF",
    params_.enable_persistence_filter ? "ON" : "OFF",
    params_.enable_cluster_filter ? "ON" : "OFF");
}

void RadarFilterNode::declare_filter_parameters()
{
  // Topic configuration
  this->declare_parameter<std::string>("input_targets_topic", "smart_radar/can_targets_0");
  this->declare_parameter<std::string>("input_header_topic", "smart_radar/can_targetheader_0");
  this->declare_parameter<std::string>(
    "output_targets_topic", "smart_radar/can_targets_0/filtered");
  this->declare_parameter<std::string>(
    "output_header_topic", "smart_radar/can_targetheader_0/filtered");
  this->declare_parameter<int>("qos_depth", 10);

  // Filter enable flags
  params_.enable_range_gate = this->declare_parameter<bool>("enable_range_gate", true);
  params_.enable_azimuth_gate = this->declare_parameter<bool>("enable_azimuth_gate", true);
  params_.enable_elevation_gate = this->declare_parameter<bool>("enable_elevation_gate", true);
  params_.enable_snr_filter = this->declare_parameter<bool>("enable_snr_filter", true);
  params_.enable_rcs_filter = this->declare_parameter<bool>("enable_rcs_filter", true);
  params_.enable_speed_filter = this->declare_parameter<bool>("enable_speed_filter", true);
  params_.enable_persistence_filter =
    this->declare_parameter<bool>("enable_persistence_filter", false);
  params_.enable_cluster_filter = this->declare_parameter<bool>("enable_cluster_filter", false);

  // Filter thresholds
  params_.range_min = this->declare_parameter<double>("range_min", 8.0);
  params_.range_max = this->declare_parameter<double>("range_max", 65.0);
  params_.azimuth_max = this->declare_parameter<double>("azimuth_max", 0.105);
  params_.elevation_min = this->declare_parameter<double>("elevation_min", -0.087);
  params_.elevation_max = this->declare_parameter<double>("elevation_max", 0.087);
  params_.z_min = this->declare_parameter<double>("z_min", -0.5);
  params_.z_max = this->declare_parameter<double>("z_max", 3.0);
  params_.snr_min = this->declare_parameter<double>("snr_min", 12.0);
  params_.rcs_min = this->declare_parameter<double>("rcs_min", 5.0);
  params_.speed_min = this->declare_parameter<double>("speed_min", 0.5);

  // Temporal persistence
  params_.persistence_n = this->declare_parameter<int>("persistence_n", 3);
  params_.persistence_m = this->declare_parameter<int>("persistence_m", 5);
  params_.persistence_dist = this->declare_parameter<double>("persistence_dist", 2.0);

  // Spatial clustering (DBSCAN)
  params_.cluster_eps = this->declare_parameter<double>("cluster_eps", 2.0);
  params_.cluster_min_points = this->declare_parameter<int>("cluster_min_points", 2);
}

void RadarFilterNode::header_callback(
  const umrr_ros2_msgs::msg::CanTargetHeader::SharedPtr msg)
{
  latest_header_ = msg;
}

void RadarFilterNode::targets_callback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Read current parameter values to support runtime changes
  params_.enable_range_gate = this->get_parameter("enable_range_gate").as_bool();
  params_.enable_azimuth_gate = this->get_parameter("enable_azimuth_gate").as_bool();
  params_.enable_elevation_gate = this->get_parameter("enable_elevation_gate").as_bool();
  params_.enable_snr_filter = this->get_parameter("enable_snr_filter").as_bool();
  params_.enable_rcs_filter = this->get_parameter("enable_rcs_filter").as_bool();
  params_.enable_speed_filter = this->get_parameter("enable_speed_filter").as_bool();
  params_.enable_persistence_filter = this->get_parameter("enable_persistence_filter").as_bool();
  params_.enable_cluster_filter = this->get_parameter("enable_cluster_filter").as_bool();
  params_.range_min = this->get_parameter("range_min").as_double();
  params_.range_max = this->get_parameter("range_max").as_double();
  params_.azimuth_max = this->get_parameter("azimuth_max").as_double();
  params_.elevation_min = this->get_parameter("elevation_min").as_double();
  params_.elevation_max = this->get_parameter("elevation_max").as_double();
  params_.z_min = this->get_parameter("z_min").as_double();
  params_.z_max = this->get_parameter("z_max").as_double();
  params_.snr_min = this->get_parameter("snr_min").as_double();
  params_.rcs_min = this->get_parameter("rcs_min").as_double();
  params_.speed_min = this->get_parameter("speed_min").as_double();
  params_.persistence_n = static_cast<int>(this->get_parameter("persistence_n").as_int());
  params_.persistence_m = static_cast<int>(this->get_parameter("persistence_m").as_int());
  params_.persistence_dist = this->get_parameter("persistence_dist").as_double();
  params_.cluster_eps = this->get_parameter("cluster_eps").as_double();
  params_.cluster_min_points = static_cast<int>(this->get_parameter("cluster_min_points").as_int());

  // Attempt to create typed view of the incoming PointCloud2
  try {
    const auto num_points = msg->width * msg->height;
    if (num_points == 0) {
      targets_pub_->publish(*msg);
      if (latest_header_) {
        auto header_out = *latest_header_;
        header_out.number_of_targets = 0;
        header_pub_->publish(header_out);
      }
      return;
    }

    // Read points from the incoming cloud using byte offsets
    const auto point_step = msg->point_step;
    const auto & data = msg->data;

    // Build field offset map
    std::size_t off_x = 0, off_y = 0, off_z = 0;
    std::size_t off_radial_speed = 0, off_power = 0, off_rcs = 0;
    std::size_t off_noise = 0, off_snr = 0;
    std::size_t off_azimuth = 0, off_elevation = 0, off_range = 0;

    for (const auto & field : msg->fields) {
      if (field.name == "x") { off_x = field.offset; }
      else if (field.name == "y") { off_y = field.offset; }
      else if (field.name == "z") { off_z = field.offset; }
      else if (field.name == "radial_speed") { off_radial_speed = field.offset; }
      else if (field.name == "power") { off_power = field.offset; }
      else if (field.name == "rcs") { off_rcs = field.offset; }
      else if (field.name == "noise") { off_noise = field.offset; }
      else if (field.name == "snr") { off_snr = field.offset; }
      else if (field.name == "azimuth_angle") { off_azimuth = field.offset; }
      else if (field.name == "elevation_angle") { off_elevation = field.offset; }
      else if (field.name == "range") { off_range = field.offset; }
    }

    // Helper to read a float from the point cloud data buffer
    auto read_float = [&data, point_step](std::size_t point_idx, std::size_t offset) -> float {
      float val;
      std::memcpy(&val, &data[point_idx * point_step + offset], sizeof(float));
      return val;
    };

    // Initialize keep-mask
    std::vector<bool> keep(num_points, true);

    // Apply stateless per-point filters
    for (std::size_t i = 0; i < num_points; ++i) {
      // Range gate
      if (params_.enable_range_gate && keep[i]) {
        const float r = read_float(i, off_range);
        if (r < static_cast<float>(params_.range_min) ||
            r > static_cast<float>(params_.range_max)) {
          keep[i] = false;
        }
      }

      // Azimuth gate
      if (params_.enable_azimuth_gate && keep[i]) {
        const float az = read_float(i, off_azimuth);
        if (std::fabs(az) > static_cast<float>(params_.azimuth_max)) {
          keep[i] = false;
        }
      }

      // Elevation/height gate
      if (params_.enable_elevation_gate && keep[i]) {
        const float el = read_float(i, off_elevation);
        const float z = read_float(i, off_z);
        if (el < static_cast<float>(params_.elevation_min) ||
            el > static_cast<float>(params_.elevation_max) ||
            z < static_cast<float>(params_.z_min) ||
            z > static_cast<float>(params_.z_max)) {
          keep[i] = false;
        }
      }

      // SNR threshold
      if (params_.enable_snr_filter && keep[i]) {
        const float s = read_float(i, off_snr);
        if (s < static_cast<float>(params_.snr_min)) {
          keep[i] = false;
        }
      }

      // RCS filter
      if (params_.enable_rcs_filter && keep[i]) {
        const float r = read_float(i, off_rcs);
        if (r < static_cast<float>(params_.rcs_min)) {
          keep[i] = false;
        }
      }

      // Radial speed filter (reject near-zero = static objects)
      if (params_.enable_speed_filter && keep[i]) {
        const float spd = read_float(i, off_radial_speed);
        if (std::fabs(spd) < static_cast<float>(params_.speed_min)) {
          keep[i] = false;
        }
      }
    }

    // Temporal persistence filter
    if (params_.enable_persistence_filter) {
      apply_persistence_filter(*msg, keep);
    }

    // Spatial clustering filter (DBSCAN)
    if (params_.enable_cluster_filter) {
      apply_cluster_filter(*msg, keep);
    }

    // Build output PointCloud2 using RadarCloudModifier
    sensor_msgs::msg::PointCloud2 out_msg;
    RadarCloudModifier modifier{out_msg, msg->header.frame_id};
    out_msg.header = msg->header;

    std::size_t count = 0;
    for (std::size_t i = 0; i < num_points; ++i) {
      if (keep[i]) {
        RadarPoint pt;
        pt.x = read_float(i, off_x);
        pt.y = read_float(i, off_y);
        pt.z = read_float(i, off_z);
        pt.radial_speed = read_float(i, off_radial_speed);
        pt.power = read_float(i, off_power);
        pt.rcs = read_float(i, off_rcs);
        pt.noise = read_float(i, off_noise);
        pt.snr = read_float(i, off_snr);
        pt.azimuth_angle = read_float(i, off_azimuth);
        pt.elevation_angle = read_float(i, off_elevation);
        pt.range = read_float(i, off_range);
        modifier.push_back(
          {pt.x, pt.y, pt.z, pt.radial_speed, pt.power, pt.rcs,
           pt.noise, pt.snr, pt.azimuth_angle, pt.elevation_angle, pt.range});
        ++count;
      }
    }

    targets_pub_->publish(out_msg);

    // Publish updated header
    if (latest_header_) {
      auto header_out = *latest_header_;
      header_out.number_of_targets = static_cast<uint8_t>(std::min(count, std::size_t{255}));
      header_pub_->publish(header_out);
    }

  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to process PointCloud2: %s", e.what());
  }
}

void RadarFilterNode::apply_persistence_filter(
  const sensor_msgs::msg::PointCloud2 & cloud_msg,
  std::vector<bool> & keep)
{
  const auto point_step = cloud_msg.point_step;
  const auto & data = cloud_msg.data;
  const auto num_points = cloud_msg.width * cloud_msg.height;

  // Find x and y field offsets
  std::size_t off_x = 0, off_y = 0;
  for (const auto & field : cloud_msg.fields) {
    if (field.name == "x") { off_x = field.offset; }
    else if (field.name == "y") { off_y = field.offset; }
  }

  auto read_float = [&data, point_step](std::size_t idx, std::size_t offset) -> float {
    float val;
    std::memcpy(&val, &data[idx * point_step + offset], sizeof(float));
    return val;
  };

  // Collect current scan's (x,y) for all points that passed stateless filters
  std::vector<Point2D> current_points;
  std::vector<std::size_t> current_indices;
  for (std::size_t i = 0; i < num_points; ++i) {
    if (keep[i]) {
      current_points.push_back({read_float(i, off_x), read_float(i, off_y)});
      current_indices.push_back(i);
    }
  }

  const float dist_sq_thresh =
    static_cast<float>(params_.persistence_dist * params_.persistence_dist);
  const int required_historical = params_.persistence_n - 1;  // current scan counts as 1

  if (required_historical > 0) {
    // For each current point, count historical scans with a nearby neighbor
    for (std::size_t ci = 0; ci < current_points.size(); ++ci) {
      int hit_count = 0;
      const auto & cp = current_points[ci];

      for (const auto & past_scan : scan_history_) {
        bool found = false;
        for (const auto & pp : past_scan) {
          const float dx = cp.x - pp.x;
          const float dy = cp.y - pp.y;
          if (dx * dx + dy * dy <= dist_sq_thresh) {
            found = true;
            break;
          }
        }
        if (found) {
          ++hit_count;
        }
        if (hit_count >= required_historical) {
          break;
        }
      }

      if (hit_count < required_historical) {
        keep[current_indices[ci]] = false;
      }
    }
  }

  // Add current scan to history (points that passed stateless filters,
  // before persistence rejection, to preserve the association pool)
  scan_history_.push_back(std::move(current_points));
  while (static_cast<int>(scan_history_.size()) > params_.persistence_m - 1) {
    scan_history_.pop_front();
  }
}

void RadarFilterNode::apply_cluster_filter(
  const sensor_msgs::msg::PointCloud2 & cloud_msg,
  std::vector<bool> & keep)
{
  const auto point_step = cloud_msg.point_step;
  const auto & data = cloud_msg.data;
  const auto num_points = cloud_msg.width * cloud_msg.height;

  // Find x and y field offsets
  std::size_t off_x = 0, off_y = 0;
  for (const auto & field : cloud_msg.fields) {
    if (field.name == "x") { off_x = field.offset; }
    else if (field.name == "y") { off_y = field.offset; }
  }

  auto read_float = [&data, point_step](std::size_t idx, std::size_t offset) -> float {
    float val;
    std::memcpy(&val, &data[idx * point_step + offset], sizeof(float));
    return val;
  };

  // Collect surviving points
  std::vector<std::size_t> indices;
  for (std::size_t i = 0; i < num_points; ++i) {
    if (keep[i]) {
      indices.push_back(i);
    }
  }

  if (indices.empty()) {
    return;
  }

  const std::size_t n = indices.size();
  const float eps_sq = static_cast<float>(params_.cluster_eps * params_.cluster_eps);
  const int min_pts = params_.cluster_min_points;

  // DBSCAN labels: -1 = noise, 0 = unvisited, >0 = cluster id
  std::vector<int> labels(n, 0);
  int cluster_id = 0;

  // Precompute positions
  std::vector<float> px(n), py(n);
  for (std::size_t j = 0; j < n; ++j) {
    px[j] = read_float(indices[j], off_x);
    py[j] = read_float(indices[j], off_y);
  }

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
    if (labels[i] != 0) {
      continue;
    }

    auto neighbors = region_query(i);
    if (static_cast<int>(neighbors.size()) < min_pts) {
      labels[i] = -1;  // noise
      continue;
    }

    ++cluster_id;
    labels[i] = cluster_id;

    // Expand cluster
    std::vector<std::size_t> seed_set(neighbors.begin(), neighbors.end());
    for (std::size_t si = 0; si < seed_set.size(); ++si) {
      std::size_t q = seed_set[si];
      if (labels[q] == -1) {
        labels[q] = cluster_id;  // border point
      }
      if (labels[q] != 0) {
        continue;
      }
      labels[q] = cluster_id;

      auto q_neighbors = region_query(q);
      if (static_cast<int>(q_neighbors.size()) >= min_pts) {
        for (auto nn : q_neighbors) {
          seed_set.push_back(nn);
        }
      }
    }
  }

  // Reject noise points (label == -1)
  for (std::size_t j = 0; j < n; ++j) {
    if (labels[j] == -1) {
      keep[indices[j]] = false;
    }
  }
}

}  // namespace radar
}  // namespace drivers
}  // namespace smartmicro

RCLCPP_COMPONENTS_REGISTER_NODE(smartmicro::drivers::radar::RadarFilterNode)
