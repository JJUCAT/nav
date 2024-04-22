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

#ifndef NAVIT_BT_NODES__PLUGINS__ACTION__MOVE_THROUGH_POSE_ACTION_HPP_
#define NAVIT_BT_NODES__PLUGINS__ACTION__MOVE_THROUGH_POSE_ACTION_HPP_

#include <string>

#include <navit_msgs/BackToReferenceLineAction.h>
#include <nav_msgs/Path.h>
#include "navit_bt_nodes/bt_action_node.h"

namespace navit_bt_nodes
{

class BackToReferenceLineAction : public BT::RosActionNode<navit_msgs::BackToReferenceLineAction>
{
public:
  BackToReferenceLineAction(ros::NodeHandle& handle, const std::string& name, const BT::NodeConfiguration& conf)
    : RosActionNode<navit_msgs::BackToReferenceLineAction>(handle, name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<nav_msgs::Path>("reference_line", "The path robot wannt go back"),
                                BT::InputPort<std::string>("bt_name", "bt name") });
  }

  bool on_first_tick() override
  {
    ROS_INFO("Back to reference line ticking.");

    getInput("reference_line", goal_.reference_line);
    getInput("bt_name", goal_.bt_name);

    return true;
  }

  BT::NodeStatus onResult(const ResultType& res) override
  {
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace navit_bt_nodes

#endif
