// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Francisco Martin Rico
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

#include <string>
#include <memory>
#include <limits>

#include "behaviortree_cpp_v3/decorator_node.h"
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

#include "navit_bt_nodes/plugins/action/truncate_path_action.hpp"

namespace navit_bt_nodes
{

TruncatePath::TruncatePath(const std::string& name, const BT::NodeConfiguration& conf)
  : BT::ActionNodeBase(name, conf), distance_(0.1)
{
  ROS_INFO("TruncatePath construction.");
  getInput("distance", distance_);
}

inline BT::NodeStatus TruncatePath::tick()
{
  // ROS_INFO("TruncatePath ticking.");

  setStatus(BT::NodeStatus::RUNNING);

  nav_msgs::Path input_path;

  getInput("input_path", input_path);

  if (input_path.poses.empty())
  {
    setOutput("output_path", input_path);
    return BT::NodeStatus::SUCCESS;
  }

  geometry_msgs::PoseStamped final_pose = input_path.poses.back();
  // ROS_INFO("frame_id: [%s]", input_path.header.frame_id.c_str());

  double distance_to_goal = hypot(input_path.poses.back().pose.position.x - final_pose.pose.position.x,
                                  input_path.poses.back().pose.position.y - final_pose.pose.position.y);

  while (distance_to_goal < distance_ && input_path.poses.size() > 2)
  {
    input_path.poses.pop_back();
    distance_to_goal = hypot(input_path.poses.back().pose.position.x - final_pose.pose.position.x,
                             input_path.poses.back().pose.position.y - final_pose.pose.position.y);
  }

  double dx = final_pose.pose.position.x - input_path.poses.back().pose.position.x;
  double dy = final_pose.pose.position.y - input_path.poses.back().pose.position.y;

  double final_angle = atan2(dy, dx);

  if (std::isnan(final_angle) || std::isinf(final_angle))
  {
    ROS_WARN("Final angle is not valid while truncating path. Setting to 0.0");
    final_angle = 0.0;
  }

  input_path.poses.back().pose.orientation = tf::createQuaternionMsgFromYaw(final_angle);

  setOutput("output_path", input_path);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace navit_bt_nodes

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navit_bt_nodes::TruncatePath>("TruncatePath");
}
