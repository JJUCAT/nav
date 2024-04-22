// Copyright (c) 2018 Intel Corporation
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

#ifndef NAVIT_BT_NODES__PLUGINS__ACTION__COMPUTE_PATH_TO_POSE_ACTION_HPP_
#define NAVIT_BT_NODES__PLUGINS__ACTION__COMPUTE_PATH_TO_POSE_ACTION_HPP_

#include <string>

#include <navit_msgs/ComputePathAction.h>
#include <nav_msgs/Path.h>
#include "navit_bt_nodes/bt_action_node.h"

namespace navit_bt_nodes
{

class ComputePathToPoseAction : public BT::RosActionNode<navit_msgs::ComputePathAction>
{
public:
  ComputePathToPoseAction(ros::NodeHandle& handle, const std::string& name, const BT::NodeConfiguration& conf)
    : RosActionNode<navit_msgs::ComputePathAction>(handle, name, conf)
  {
  }

  bool on_first_tick() override
  {
    ROS_INFO("ComputePath ticking.");
    getInput("goal", goal_.goal);
    getInput("planner_id", goal_.planner_plugin);
    if (getInput("start", goal_.start))
    {
      goal_.use_start = true;
    }
    return true;
  }

  BT::NodeStatus onResult(const ResultType& res) override
  {
    setOutput("path", res.path);
    std::string error_code = std::to_string(res.error_code);
    setOutput("error_code", error_code);
    if(error_code.compare("207") == 0 || error_code.compare("208") == 0)
    {
      return BT::NodeStatus::FAILURE;
    }

    if (first_time_)
    {
      first_time_ = false;
    }
    else
    {
      config().blackboard->set("path_updated", true);
    }
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::OutputPort<nav_msgs::Path>("path", "Path created by ComputePathToPose node"),
        BT::OutputPort<std::string>("error_code","result error code"),
        BT::InputPort<geometry_msgs::PoseStamped>("goal", "Destination to plan to"),
        BT::InputPort<geometry_msgs::PoseStamped>("start", "Start pose of the path if overriding current robot pose"),
        BT::InputPort<std::string>("planner_id", ""),
    });
  }

private:
  bool first_time_{ true };
};

}  // namespace navit_bt_nodes

#endif  // NAVIT_BT_NODES__PLUGINS__ACTION__COMPUTE_PATH_TO_POSE_ACTION_HPP_
