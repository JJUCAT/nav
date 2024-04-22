// Copyright (c) 2021 Samsung Research America
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

#include <nav_msgs/Path.h>
#include <nav_msgs/Path.h>

#include "navit_bt_nodes/plugins/action/remove_passed_goals_action.hpp"

namespace navit_bt_nodes
{

RemovePassedGoals::RemovePassedGoals(const std::string& name, const BT::NodeConfiguration& conf)
  : BT::ActionNodeBase(name, conf), viapoint_achieved_radius_(0.2)
{
  ROS_INFO("RemovePassedGoals construction.");
  getInput("radius", viapoint_achieved_radius_);
}

inline BT::NodeStatus RemovePassedGoals::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  nav_msgs::Path input_path;
  Goals input_goals;
  Goals output_goals;

  getInput("input_path", input_path);
  getInput("input_goals", input_goals);

  output_goals = input_goals;

  if (input_path.poses.empty() && input_goals.empty())
  {
    setOutput("output_path", input_path);
    return BT::NodeStatus::SUCCESS;
  }

  geometry_msgs::PoseStamped final_pose = input_path.poses.back();
  geometry_msgs::PoseStamped start_pose = input_path.poses.front();

  double distance_to_goal = hypot(start_pose.pose.position.x - final_pose.pose.position.x,
                                  start_pose.pose.position.y - final_pose.pose.position.y);

  if (distance_to_goal < viapoint_achieved_radius_ && output_goals.size() >= 1)
  {
    ROS_WARN("A passed viapoints removed");
    output_goals.erase(output_goals.begin());
  }

  setOutput("output_goals", output_goals);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace navit_bt_nodes

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navit_bt_nodes::RemovePassedGoals>("RemovePassedGoals");
}
