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

#ifndef NAVIT_BT_NODES__PLUGINS__ACTION__FOLLOW_PATH_ACTION_HPP_
#define NAVIT_BT_NODES__PLUGINS__ACTION__FOLLOW_PATH_ACTION_HPP_

#include <string>

#include <navit_msgs/FollowPathAction.h>
#include "navit_bt_nodes/bt_action_node.h"
#include "navit_msgs/FollowPathFeedback.h"
#include "navit_msgs/FollowPathGoal.h"

namespace navit_bt_nodes
{

class FollowPathAction : public BT::RosActionNode<navit_msgs::FollowPathAction>
{
public:
  FollowPathAction(ros::NodeHandle& handle, const std::string& name, const BT::NodeConfiguration& conf)
    : RosActionNode<navit_msgs::FollowPathAction>(handle, name, conf)
  {
    config().blackboard->set("path_updated", false);
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::InputPort<nav_msgs::Path>("path", "Path to follow"),
        BT::InputPort<std::string>("controller_id", ""),
        BT::OutputPort<bool>("goal_updated", ""),
        BT::OutputPort<float>("completion_percentage", ""),
    });
  }

  bool on_first_tick() override
  {
    getInput("path", goal_.path);
    getInput("controller_id", goal_.controller_plugin);
    return true;
  }

  BT::NodeStatus onResult(const ResultType& res) override
  {
    setOutput("goal_updated", false);
    return BT::NodeStatus::SUCCESS;
  }

  bool on_every_tick()
  {
    // Check if the goal has been updated
    if (config().blackboard->get<bool>("path_updated"))
    {
      // Reset the flag in the blackboard
      config().blackboard->set("path_updated", false);

      // Grab the new goal and set the flag so that we send the new goal to
      // the action server on the next loop iteration
      getInput("path", goal_.path);
      goal_updated_ = true;
      navit_msgs::FollowPathFeedback feedback;
      float completion_percentage = feedback.completion_percentage;
      setOutput("completion_percentage", completion_percentage);
    }
    return true;
  }
};

}  // namespace navit_bt_nodes

#endif  // NAVIT_BT_NODES__PLUGINS__ACTION__FOLLOW_PATH_ACTION_HPP_
