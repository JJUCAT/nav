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

#include "navit_collision_monitor/source.hpp"

#include <exception>
#include <geometry_msgs/TransformStamped.h>

namespace navit_collision_monitor {

Source::Source(ros::NodeHandle &node, const std::string &source_name,
               const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
               const std::string &base_frame_id,
               const std::string &global_frame_id,
               const ros::Duration &transform_tolerance,
               const ros::Duration &source_timeout,
               const bool base_shift_correction)
    : node_(node), source_name_(source_name), tf_buffer_(tf_buffer),
      base_frame_id_(base_frame_id), global_frame_id_(global_frame_id),
      transform_tolerance_(transform_tolerance),
      source_timeout_(source_timeout),
      base_shift_correction_(base_shift_correction) {}

Source::~Source() {}

void Source::getCommonParameters(std::string &source_topic) {
  node_.param(source_name_ + "/topic", source_topic, std::string("scan"));
}

bool Source::sourceValid(const ros::Time &source_time,
                         const ros::Time &curr_time) const {
  // Source is considered as not valid, if latest received data timestamp is
  // earlier than current time by source_timeout_ interval
  const ros::Duration dt = curr_time - source_time;
  if (dt > source_timeout_) {
    ROS_WARN("[%s]: Latest source and current collision monitor node "
             "timestamps differ on %f seconds. "
             "Ignoring the source.",
             source_name_.c_str(), dt.toSec());
    return false;
  }

  return true;
}

} // namespace navit_collision_monitor
