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

#ifndef NAVIT_BT_NODES__PLUGINS__ACTION__REMOVE_PASSED_GOALS_ACTION_HPP_
#define NAVIT_BT_NODES__PLUGINS__ACTION__REMOVE_PASSED_GOALS_ACTION_HPP_

#include <vector>
#include <memory>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include "behaviortree_cpp_v3/action_node.h"

namespace navit_bt_nodes
{

class RemovePassedGoals : public BT::ActionNodeBase
{
public:
  typedef std::vector<geometry_msgs::PoseStamped> Goals;

  RemovePassedGoals(const std::string& xml_tag_name, const BT::NodeConfiguration& conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<Goals>("input_goals", "Original goals to remove viapoints from"),
      BT::OutputPort<Goals>("output_goals", "Goals with passed viapoints removed(truncated goals)"),
      BT::InputPort<double>("radius", 0.2, "radius to goal for it to be considered for removal"),
      BT::InputPort<nav_msgs::Path>("input_path", "Original Global Planner Path"),
    };
  }

private:
  void halt() override
  {
  }
  BT::NodeStatus tick() override;

  double viapoint_achieved_radius_;
};

}  // namespace navit_bt_nodes

#endif  // NAVIT_BT_NODES__PLUGINS__ACTION__REMOVE_PASSED_GOALS_ACTION_HPP_
