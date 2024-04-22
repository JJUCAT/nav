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

#include <string>

#include "ros/ros.h"
#include "navit_bt_nodes/plugins/decorator/j110/ignore.hpp"
#include <navit_bt_nodes/bt_exception.hpp>

namespace navit_bt_nodes
{

IgnoreDecorator::IgnoreDecorator(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::DecoratorNode(name, conf), count_(0) {

}

BT::NodeStatus IgnoreDecorator::tick() {
  count_ ++;
  size_t ignore_count;
  LoadArg(ignore_count);

  if (count_ <= ignore_count) {
    ROS_INFO("[BT][%s] ignore count %lu, current count %lu",
      node_name_, ignore_count, count_);    
    return BT::NodeStatus::SUCCESS;
  }

  const BT::NodeStatus child_state = child_node_->executeTick();
  return child_state;
}

void IgnoreDecorator::LoadArg(size_t& ignore_count) {
  if (!getInput<size_t>("ignore_count", ignore_count)) {
    std::string msg("missing arg [ignore_count]");
    ROS_ERROR("[BT][%s] %s", node_name_, msg.c_str());
    throw(BT_Exception(msg));
  }
}

}  // namespace navit_bt_nodes

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navit_bt_nodes::IgnoreDecorator>("IgnoreDecorator");
}
