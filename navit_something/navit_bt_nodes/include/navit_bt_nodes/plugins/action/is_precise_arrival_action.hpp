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

#ifndef NAVIT_BT_NODES__PLUGINS__ACTION__IS_PRECISE_ARRIVAL_ACTION_HPP_
#define NAVIT_BT_NODES__PLUGINS__ACTION__IS_PRECISE_ARRIVAL_ACTION_HPP_

#include <cstdint>
#include <string>
#include <ros/ros.h>

#include <navit_msgs/NavigateToPoseAction.h>
#include "navit_bt_nodes/bt_action_node.h"

namespace navit_bt_nodes
{

class IsPreciseArrivalAction : public BT::ActionNodeBase
{
public:

  IsPreciseArrivalAction(const std::string& name, const BT::NodeConfiguration& conf) : BT::ActionNodeBase(name, conf) {}
  static BT::PortsList providedPorts()
  {
    return {};
  }

private:

    void halt() override {}
    BT::NodeStatus tick() override {
      uint16_t arrival_mode = config().blackboard->get<uint16_t>("arrival_mode");
      std::cout << "arrival_mode: " << arrival_mode << std::endl;
      if (arrival_mode == 0) {
        // Precise arrival
        return BT::NodeStatus::SUCCESS;
      }
      else {
        return BT::NodeStatus::FAILURE;
      }
    };
};

}

#endif  // NAVIT_BT_NODES__PLUGINS__ACTION__PATH_TO_POINT_ACTION_HPP_

