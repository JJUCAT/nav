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

#include "fetch_item_action.hpp"

namespace navit_bt_nodes
{

FetchItemAction::FetchItemAction(const std::string& name, const BT::NodeConfiguration& conf)
  : BT::ActionNodeBase(name, conf)
{
  current_index_ = 0;
}

inline BT::NodeStatus FetchItemAction::tick()
{
  if (!getInput("in_items", items_))
  {
    return BT::NodeStatus::FAILURE;
  }

  if (current_index_ >= items_.size())
  {
    return BT::NodeStatus::FAILURE;
  }

  std::string item = items_[current_index_++];
  printf("  **********  FetchItemAction %s **********", item.c_str());
  setOutput("out_item", item);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace navit_bt_nodes

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navit_bt_nodes::FetchItemAction>("FetchItemAction");
}
