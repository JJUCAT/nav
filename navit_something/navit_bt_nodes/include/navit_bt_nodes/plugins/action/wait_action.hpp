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

#include <navit_msgs/WaitAction.h>
#include "navit_bt_nodes/bt_action_node.h"

namespace navit_bt_nodes
{

class WaitAction : public BT::RosActionNode<navit_msgs::WaitAction>
{
public:
  WaitAction(ros::NodeHandle& handle, const std::string& name, const BT::NodeConfiguration& conf)
    : RosActionNode<navit_msgs::WaitAction>(handle, name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<int>("wait_duration", 1, "Wait time") });
  }

  bool on_first_tick() override
  {
    int duration;
    getInput("wait_duration", duration);
    if (duration <= 0)
    {
      ROS_WARN(
          "Wait duration is negative or zero "
          "(%i). Setting to positive.",
          duration);
      duration *= -1;
    }

    goal_.time.data = ros::Duration(duration);  //.time.data.toSec()
    return true;
  }

  BT::NodeStatus onResult(const ResultType& res) override
  {
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace navit_bt_nodes

#endif  // NAVIT_BT_NODES__PLUGINS__ACTION__WAIT_ACTION_HPP_
