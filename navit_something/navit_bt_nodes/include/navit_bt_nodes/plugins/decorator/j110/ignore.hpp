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

#ifndef NAVIT_BT_NODES__PLUGINS__DECORATOR__IGNORE_HPP_
#define NAVIT_BT_NODES__PLUGINS__DECORATOR__IGNORE_HPP_


#include <string>

#include "behaviortree_cpp_v3/decorator_node.h"

namespace navit_bt_nodes
{

/**
 * @brief IgnoreDecorator
 * 忽略前 ignore_count 个 tick，不会 tick 子节点直接返回成功
 * 否则 tick 子节点，返回子节点状态
 * 创建节点后，每次 tick 该节点 count_ 都会累计，halt() 不会重置 count_
 */
class IgnoreDecorator : public BT::DecoratorNode
{
 public:
  /**
   * @brief A constructor for navit_bt_nodes::IgnoreDecorator
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IgnoreDecorator(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<size_t>("ignore_count", 0, "count to ignore.")
    };
  }

 private:

  /**
    * @brief  加载参数
    * @param  ignore_count  要忽略的次数
    */
  void LoadArg(size_t& ignore_count);

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

 private:
  const char* node_name_ = "ignore_decorator";
  size_t count_;

};

}  // namespace navit_bt_nodes

#endif  // NAVIT_BT_NODES__PLUGINS__DECORATOR__IGNORE_HPP_
