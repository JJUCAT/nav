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

#ifndef COLLISION_MONITOR__SCAN_HPP_
#define COLLISION_MONITOR__SCAN_HPP_

#include "navit_collision_monitor/source.hpp"
#include <memory>
#include <sensor_msgs/LaserScan.h>
#include <string>
#include <vector>

namespace navit_collision_monitor {

/**
 * @brief Implementation for laser scanner source
 */
class Scan : public Source {
public:
  /**
   * @brief Scan constructor
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
  Scan(ros::NodeHandle &node, const std::string &source_name,
       const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
       const std::string &base_frame_id, const std::string &global_frame_id,
       const ros::Duration &transform_tolerance,
       const ros::Duration &source_timeout, const bool base_shift_correction);
  /**
   * @brief Scan destructor
   */
  ~Scan();

  /**
   * @brief Data source configuration routine. Obtains ROS-parameters
   * and creates laser scanner subscriber.
   */
  void configure();

  /**
   * @brief Adds latest data from laser scanner to the data array.
   * @param curr_time Current node time for data interpolation
   * @param data Array where the data from source to be added.
   * Added data is transformed to base_frame_id_ coordinate system at curr_time.
   */
  void getData(const ros::Time &curr_time, std::vector<Point> &data) const;

protected:
  /**
   * @brief Laser scanner data callback
   * @param msg Shared pointer to LaserScan message
   */
  void dataCallback(sensor_msgs::LaserScan::ConstPtr msg);

  // ----- Variables -----

  /// @brief Laser scanner data subscriber
  ros::Subscriber data_sub_;

  /// @brief Latest data obtained from laser scanner
  sensor_msgs::LaserScan::ConstPtr data_;
}; // class Scan

} // namespace navit_collision_monitor

#endif // COLLISION_MONITOR__SCAN_HPP_
