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

#include <navit_msgs/RotateAction.h>
#include "navit_bt_nodes/bt_action_node.h"

namespace navit_bt_nodes
{

class RotateAction : public BT::RosActionNode<navit_msgs::RotateAction>
{
public:
  RotateAction(ros::NodeHandle& handle, const std::string& name, const BT::NodeConfiguration& conf)
    : RosActionNode<navit_msgs::RotateAction>(handle, name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
        { BT::InputPort<double>("rotate_rad", 0.1, "Rotate value in rad"),
          BT::InputPort<double>("rotate_speed", 0.1, "Rotate speed in rad/s"),
          BT::InputPort<double>("is_rotate_clockwise", true, "Rotate direction defalt is clockwise") });
  }

  bool on_first_tick() override
  {
    double rotate_rad = 0.0, rotate_speed = 0.0;
    bool rotate_clockwise = true;

    getInput("rotate_rad", rotate_rad);
    getInput("rotate_speed", rotate_speed);
    getInput("is_rotate_clockwise", rotate_clockwise);
    // Populate the input message
    goal_.rotate_rad = rotate_rad;
    goal_.rotate_speed = rotate_speed;
    goal_.is_rotate_clockwise = rotate_clockwise;
    return true;
  }

  BT::NodeStatus onResult(const ResultType& res) override
  {
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace navit_bt_nodes

#endif  // NAVIT_BT_NODES__PLUGINS__ACTION__WAIT_ACTION_HPP_
