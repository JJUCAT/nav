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

#include "navit_bt_nodes/plugins/action/is_precise_arrival_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

// BT_REGISTER_NODES(factory)
// {
//   BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config) {
//     static ros::NodeHandle nh;
//     config.blackboard->template get<ros::NodeHandle>("node", nh);
//     return std::make_unique<navit_bt_nodes::IsPreciseArrivalAction>(nh, name, config);
//   };

//   factory.registerBuilder<navit_bt_nodes::IsPreciseArrivalAction>("IsPreciseArrivalAction", builder);
// }

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navit_bt_nodes::IsPreciseArrivalAction>("IsPreciseArrivalAction");
}