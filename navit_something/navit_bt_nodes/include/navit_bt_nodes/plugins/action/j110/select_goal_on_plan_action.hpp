#ifndef SELECT_GOAL_ON_PLAN_ACTION_HPP_
#define SELECT_GOAL_ON_PLAN_ACTION_HPP_

#include "geometry_msgs/PoseStamped.h"
#include "navit_bt_nodes/bt_action_node.h"
#include <nav_msgs/Path.h>
#include <limits>

namespace navit_bt_nodes
{

class SelectGoalOnPlanAction : public BT::ActionNodeBase
{
 public:

    SelectGoalOnPlanAction(const std::string& xml_tag_name, const BT::NodeConfiguration& conf)
      : BT::ActionNodeBase(xml_tag_name, conf)
    {

    }

    static BT::PortsList providedPorts()
    {
      return {
        BT::InputPort<nav_msgs::Path>("ref_plan", "ref_plan"), // 参考路径
        BT::InputPort<double>("distance", "distance"), // 尝试的距离
        BT::InputPort<double>("jump", "jump"), // 选点步长
        BT::InputPort<double>("timeout", "timeout"), // 超时
        BT::OutputPort<geometry_msgs::PoseStamped>("goal", "goal"), // 目标点
        BT::OutputPort<size_t>("goal_index", "goal_index"), // 目标点下标
        BT::BidirectionalPort<size_t>("start", "start index to select"), // 点选的起点下标 
      };
    }

 private:

    /**
     * @brief  加载参数
     * @param  ref_plan  参考路径
     * @param  distance  尝试的距离
     * @param  jump  选点步长
     * @param  timeout  超时
     * @param  start  选点起点
     */
    void LoadArg(nav_msgs::Path& ref_plan, double& distance, double& jump, double& timeout, size_t& start);

    /**
     * @brief  判断是否是新参考路径
     * @param  ref_plan  参考路径
     * @return true 
     * @return false 
     */
    bool IsNewPlan(const nav_msgs::Path& ref_plan);

    /**
     * @brief  是否超时
     * @param  timeout  时间上限
     * @return true 
     * @return false TaskPlanEditorAction
     */
    bool IsTimeout(const double timeout);

    /**
     * @brief  选点
     * @param  ref_plan  参考路径
     * @param  distance  尝试的距离
     * @param  jump  选点步长
     * @param  start  选点起点
     * @return size_t 目标点下标
     */
    size_t Select(const nav_msgs::Path& ref_plan, const double distance, const double jump, const size_t start);

    void halt() override;

    BT::NodeStatus tick() override;

 private:

  const char* node_name_ = "select_goal_on_plan_action";
  size_t last_plan_seq_ = std::numeric_limits<size_t>::max(); // 上次参考路径的序号，用于区分新旧参考路径
  double distance_ = 0; // 记录当前选点的距离
  ros::Time start_time_; // 超时限制

}; // class SelectGoalOnPlanAction

}  // namespace navit_bt_nodes

#endif  // SELECT_GOAL_ON_PLAN_ACTION_HPP_
