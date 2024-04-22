// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Francisco Martin Rico
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

#ifndef NAVIT_BT_NODES__PLUGINS__ACTION__TRUNCATE_PATH_ACTION_HPP_
#define NAVIT_BT_NODES__PLUGINS__ACTION__TRUNCATE_PATH_ACTION_HPP_

#include <memory>
#include <string>

#include <nav_msgs/Path.h>
#include <nav_msgs/Path.h>
#include "behaviortree_cpp_v3/action_node.h"

namespace navit_bt_nodes
{

/**
 * @brief A BT::ActionNodeBase to shorten path by some distance
 */
class TruncatePath : public BT::ActionNodeBase
{
public:
  /**
   * @brief A navit_bt_nodes::TruncatePath constructor
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  TruncatePath(const std::string& xml_tag_name, const BT::NodeConfiguration& conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::Path>("input_path", "Original Path"),
      BT::OutputPort<nav_msgs::Path>("output_path", "Path truncated to a certain distance"),
      BT::InputPort<double>("distance", 0.05, "distance"),
    };
  }

private:
  /**
   * @brief The other (optional) override required by a BT action.
   */
  void halt() override
  {
  }

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  double distance_;
};

}  // namespace navit_bt_nodes

#endif  // NAVIT_BT_NODES__PLUGINS__ACTION__TRUNCATE_PATH_ACTION_HPP_
