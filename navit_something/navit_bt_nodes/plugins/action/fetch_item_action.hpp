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

#ifndef WINNIE_BT_NODES__PLUGINS__ACTION__FETCH_ITEM_ACTION_HPP_
#define WINNIE_BT_NODES__PLUGINS__ACTION__FETCH_ITEM_ACTION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"

namespace navit_bt_nodes
{

/**
 * @brief A BT::ActionNodeBase to shorten path by some distance
 */
class FetchItemAction : public BT::ActionNodeBase
{
public:
  /**
   * @brief A winnie_bt_nodes::TruncatePath constructor
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  FetchItemAction(const std::string& xml_tag_name, const BT::NodeConfiguration& conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<std::string>>("in_items", "get string item from vector string."),
      BT::OutputPort<std::string>("out_item", "fetch string item in order."),
    };
  }

private:
  /**
   * @brief The other (optional) override required by a BT action.
   */
  void halt() override
  {
    current_index_ = 0;
  }

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  int current_index_;
  std::vector<std::string> items_;
};

}  // namespace navit_bt_nodes

#endif  // WINNIE_BT_NODES__PLUGINS__ACTION__FETCH_ITEM_ACTION_HPP_
