#ifndef MERGE_PLANS_ACTION_HPP_
#define MERGE_PLANS_ACTION_HPP_

#include "navit_bt_nodes/bt_action_node.h"
#include <nav_msgs/Path.h>

namespace navit_bt_nodes
{

class MergePlansAction : public BT::ActionNodeBase
{
 public:

    MergePlansAction(const std::string& xml_tag_name, const BT::NodeConfiguration& conf)
      : BT::ActionNodeBase(xml_tag_name, conf)
    {

    }

    static BT::PortsList providedPorts()
    {
      return {
        BT::InputPort<nav_msgs::Path>("plan0", "plan0"),
        BT::InputPort<nav_msgs::Path>("plan1", "plan1"),
        BT::InputPort<size_t>("knot", "knot"),
        BT::OutputPort<nav_msgs::Path>("merged_plan", "merged_plan"),
      };
    }

 private:

    /**
     * @brief  加载参数
     * @param  plan0  路径0
     * @param  plan1  路径1
     * @param  knot  路径1 上的连接点
     */
    void LoadArg(nav_msgs::Path& plan0, nav_msgs::Path& plan1, size_t& knot);

    void halt() override;

    BT::NodeStatus tick() override;

 private:

  const char* node_name_ = "merge_plans_action";

}; // class MergePlansAction

}  // namespace navit_bt_nodes

#endif  // MERGE_PLANS_ACTION_HPP_
