#ifndef SELECT_START_ON_PLAN_ACTION_HPP_
#define SELECT_START_ON_PLAN_ACTION_HPP_

#include "navit_bt_nodes/bt_action_node.h"
#include <nav_msgs/Path.h>

namespace navit_bt_nodes
{

class SelectStartOnPlanAction : public BT::ActionNodeBase
{
 public:

    SelectStartOnPlanAction(const std::string& xml_tag_name, const BT::NodeConfiguration& conf)
      : BT::ActionNodeBase(xml_tag_name, conf)
    {

    }

    static BT::PortsList providedPorts()
    {
      return {
        BT::InputPort<nav_msgs::Path>("ref_plan", "ref_plan"), // 参考路径
        BT::InputPort<std::vector<uint32_t>>("collised_indexes", "collised_indexes"), // 参考路径上的碰撞点
        BT::InputPort<double>("jump", "jump"), // 跳过的距离
        BT::OutputPort<size_t>("start", "start"), // 选点起点下标
      };
    }

 private:

    /**
     * @brief  加载参数
     * @param  ref_plan  参考路径
     * @param  collised_indexes  参考路径上的碰撞点
     * @param  jump  选点步长
     */
    void LoadArg(nav_msgs::Path& ref_plan, std::vector<uint32_t>& collised_indexes, double& jump);

    void halt() override;

    BT::NodeStatus tick() override;

 private:

  const char* node_name_ = "select_start_on_plan_action";

}; // class SelectStartOnPlanAction

}  // namespace navit_bt_nodes

#endif  // SELECT_START_ON_PLAN_ACTION_HPP_
