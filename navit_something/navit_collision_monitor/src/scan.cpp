// Copyright (c) 2022 Samsung R&D Institute Russia
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

#include "navit_collision_monitor/scan.hpp"
#include "navit_collision_monitor/utils.hpp"
#include <cmath>
#include <functional>
#include <utility>

namespace navit_collision_monitor {

Scan::Scan(ros::NodeHandle &node, const std::string &source_name,
           const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
           const std::string &base_frame_id, const std::string &global_frame_id,
           const ros::Duration &transform_tolerance,
           const ros::Duration &source_timeout,
           const bool base_shift_correction)
    : Source(node, source_name, tf_buffer, base_frame_id, global_frame_id,
             transform_tolerance, source_timeout, base_shift_correction),
      data_(nullptr) {
  ROS_INFO("[%s]: Creating Scan", source_name_.c_str());
}

Scan::~Scan() {
  ROS_INFO("[%s]: Destroying Scan", source_name_.c_str());
  data_sub_.shutdown();
}

void Scan::configure() {
  std::string source_topic;
  // Laser scanner has no own parameters
  getCommonParameters(source_topic);

  data_sub_ = node_.subscribe<sensor_msgs::LaserScan>(
      source_topic, 1,
      boost::bind(&Scan::dataCallback, this, boost::placeholders::_1));
}

void Scan::getData(const ros::Time &curr_time, std::vector<Point> &data) const {
  // Ignore data from the source if it is not being published yet or
  // not being published for a long time
  auto sensor_data = data_;
  if (sensor_data == nullptr) {
    return;
  }
  if (!sourceValid(sensor_data->header.stamp, curr_time)) {
    return;
  }

  tf2::Transform tf_transform;
  if (base_shift_correction_) {
    // Obtaining the transform to get data from source frame and time where it
    // was received to the base frame and current time
    if (!collision_monitor_utils::getTransform(
            sensor_data->header.frame_id, sensor_data->header.stamp, base_frame_id_,
            curr_time, global_frame_id_, transform_tolerance_, tf_buffer_,
            tf_transform)) {
      return;
    }
  } else {
    // Obtaining the transform to get data from source frame to base frame
    // without time shift considered. Less accurate but much more faster option
    // not dependent on state estimation frames.
    if (!collision_monitor_utils::getTransform(
            sensor_data->header.frame_id, base_frame_id_, transform_tolerance_,
            tf_buffer_, tf_transform)) {
      return;
    }
  }

  // Calculate poses and refill data array
  float angle = sensor_data->angle_min;
  for (size_t i = 0; i < sensor_data->ranges.size(); i++) {
    if (sensor_data->ranges[i] >= sensor_data->range_min &&
        sensor_data->ranges[i] <= sensor_data->range_max) {
      // Transform point coordinates from source frame -> to base frame
      tf2::Vector3 p_v3_s(sensor_data->ranges[i] * std::cos(angle),
                          sensor_data->ranges[i] * std::sin(angle), 0.0);
      tf2::Vector3 p_v3_b = tf_transform * p_v3_s;

      // Refill data array
      data.push_back({p_v3_b.x(), p_v3_b.y()});
    }
    angle += sensor_data->angle_increment;
  }
}

void Scan::dataCallback(sensor_msgs::LaserScan::ConstPtr msg) { data_ = std::move(msg); }

} // namespace navit_collision_monitor
