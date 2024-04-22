// Copyright (c) 2018 Samsung Research America
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

#ifndef NAVIT_BT_NODES__PLUGINS__ACTION__WAIT_ACTION_HPP_
#define NAVIT_BT_NODES__PLUGINS__ACTION__WAIT_ACTION_HPP_

#include <string>

#include <navit_msgs/BackUpAction.h>
#include "navit_bt_nodes/bt_action_node.h"

namespace navit_bt_nodes
{

class BackUpAction : public BT::RosActionNode<navit_msgs::BackUpAction>
{
public:
  BackUpAction(ros::NodeHandle& handle, const std::string& name, const BT::NodeConfiguration& conf)
    : RosActionNode<navit_msgs::BackUpAction>(handle, name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<double>("backup_dist", 0.25, "Distance to backup"),
                                BT::InputPort<double>("backup_speed", 0.05, "Speed at which to backup") });
  }

  bool on_first_tick() override
  {
    double dist;
    getInput("backup_dist", dist);
    double speed;
    getInput("backup_speed", speed);

    // Populate the input message
    goal_.backup_distance = dist;
    goal_.speed = speed;
    return true;
  }

  BT::NodeStatus onResult(const ResultType& res) override
  {
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace navit_bt_nodes

#endif  // NAVIT_BT_NODES__PLUGINS__ACTION__WAIT_ACTION_HPP_
