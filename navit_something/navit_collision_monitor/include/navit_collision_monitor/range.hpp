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

#ifndef COLLISION_MONITOR__RANGE_HPP_
#define COLLISION_MONITOR__RANGE_HPP_

#include "navit_collision_monitor/source.hpp"
#include <memory>
#include <sensor_msgs/Range.h>
#include <string>
#include <vector>

namespace navit_collision_monitor {

/**
 * @brief Implementation for IR/ultrasound range sensor source
 */
class Range : public Source {
public:
  /**
   * @brief Range constructor
   * @param node Collision Monitor node pointer
   * @param source_name Name of data source
   * @param tf_buffer Shared pointer to a TF buffer
   * @param base_frame_id Robot base frame ID. The output data will be
   * transformed into this frame.
   * @param global_frame_id Global frame ID for correct transform calculation
   * @param transform_tolerance Transform tolerance
   * @param source_timeout Maximum time interval in which data is considered
   * valid
   * @param base_shift_correction Whether to correct source data towards to base
   * frame movement, considering the difference between current time and latest
   * source time
   */
  Range(ros::NodeHandle &node, const std::string &source_name,
        const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
        const std::string &base_frame_id, const std::string &global_frame_id,
        const ros::Duration &transform_tolerance,
        const ros::Duration &source_timeout, const bool base_shift_correction);
  /**
   * @brief Range destructor
   */
  ~Range();

  /**
   * @brief Data source configuration routine. Obtains ROS-parameters
   * and creates range sensor subscriber.
   */
  void configure();

  /**
   * @brief Adds latest data from range sensor to the data array.
   * @param curr_time Current node time for data interpolation
   * @param data Array where the data from source to be added.
   * Added data is transformed to base_frame_id_ coordinate system at curr_time.
   */
  void getData(const ros::Time &curr_time, std::vector<Point> &data) const;

protected:
  /**
   * @brief Getting sensor-specific ROS-parameters
   * @param source_topic Output name of source subscription topic
   */
  void getParameters(std::string &source_topic);

  /**
   * @brief Range sensor data callback
   * @param msg Shared pointer to Range sensor message
   */
  void dataCallback(sensor_msgs::Range::ConstPtr msg);

  // ----- Variables -----

  /// @brief Range sensor data subscriber
  ros::Subscriber data_sub_;

  /// @brief Angle increment (in rad) between two obstacle points at the range
  /// arc
  double obstacles_angle_;

  /// @brief Latest data obtained from range sensor
  sensor_msgs::Range::ConstPtr data_;
}; // class Range

} // namespace navit_collision_monitor

#endif // COLLISION_MONITOR__RANGE_HPP_
