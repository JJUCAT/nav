// Copyright (c) 2019 Samsung Research America
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

#ifndef NAVIT_BT_NODES__PLUGINS__ACTION__CLEAR_COSTMAP_SERVICE_HPP_
#define NAVIT_BT_NODES__PLUGINS__ACTION__CLEAR_COSTMAP_SERVICE_HPP_

#include <std_srvs/Empty.h>
#include "navit_bt_nodes/bt_service_node.h"

namespace navit_bt_nodes
{

class ClearEntireCostmapService : public BT::RosServiceNode<std_srvs::Empty>
{
public:
  ClearEntireCostmapService(ros::NodeHandle& handle, const std::string& node_name, const BT::NodeConfiguration& conf)
    : RosServiceNode(handle, node_name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  bool prepareRequest() override
  {
    return true;
  }

  BT::NodeStatus onResponse(const ResponseType& rep) override
  {
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace navit_bt_nodes

#endif  // NAVIT_BT_NODES__PLUGINS__ACTION__CLEAR_COSTMAP_SERVICE_HPP_
