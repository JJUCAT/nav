#ifndef TASK_PLAN_EDITOR_ACTION_HPP_
#define TASK_PLAN_EDITOR_ACTION_HPP_

#include "geometry_msgs/PoseStamped.h"
#include "navit_bt_nodes/bt_action_node.h"
#include <string>
#include <google/protobuf/util/json_util.h>
#include <navit_bt_nodes/bt_exception.hpp>
#include <navit_common/log.h>
#include <tf/tf.h>
#include "nav_msgs/Path.h"

namespace navit_bt_nodes {

class TaskPlanEditorAction : public BT::ActionNodeBase
{
 public:

  TaskPlanEditorAction(const std::string& xml_tag_name, const BT::NodeConfiguration& conf)
    : BT::ActionNodeBase(xml_tag_name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::Path>("task_plan", "task_plan"), // 任务路径
      BT::InputPort<std::vector<size_t>>("recorder", "recorder"), // 记录任务路径执行到哪里了
      BT::InputPort<double>("jump", "jump"), // 在 recorder 位置上跳过一段距离
      BT::OutputPort<nav_msgs::Path>("ref_plan", "ref_plan"), // 输出的参考路径
    };
  }

 private:

  void LoadArg(nav_msgs::Path& task_plan, std::vector<size_t>& recorder, double& jump)
  {
    if (!getInput<nav_msgs::Path>("task_plan", task_plan)) {
      std::string msg("missing arg [task_plan]");
      ROS_ERROR("[BT][%s] %s", node_name_, msg.c_str());
      throw(BT_Exception(msg));
    }

    if (!getInput<std::vector<size_t>>("recorder", recorder)) {
      std::string msg("missing arg [recorder]");
      ROS_ERROR("[BT][%s] %s", node_name_, msg.c_str());
      throw(BT_Exception(msg));
    }

    if (!getInput<double>("jump", jump)) {
      std::string msg("missing arg [jump]");
      ROS_ERROR("[BT][%s] %s", node_name_, msg.c_str());
      throw(BT_Exception(msg));
    }
  }

  void halt() override
  {
  }

  BT::NodeStatus tick() override
  {
    nav_msgs::Path task_plan;
    std::vector<size_t> recorder;
    double jump;
    LoadArg(task_plan, recorder, jump);
    ROS_INFO("[BT][%s] task plan size %lu, recorder %lu, jump %f",
      node_name_, task_plan.poses.size(), recorder.size(), jump);

    size_t start = 0u;
    if (!recorder.empty()) {
      ROS_INFO("[BT][%s] use latest record <%lu>", node_name_, recorder.back());
      start = recorder.back();
    } else {
      ROS_WARN("[BT][%s] recorder is empty, robot has not reach task plan yet.", node_name_);
    }

    double d = 0.f;
    auto& plan = task_plan.poses;
    size_t s = plan.size()-1;
    for (size_t i = start, j = i; i < plan.size(); j = i, i ++) {
      d += std::hypot(plan.at(i).pose.position.y - plan.at(j).pose.position.y,
                      plan.at(i).pose.position.x - plan.at(j).pose.position.x);
      if (d >= jump) { s = i; break; }
    }

    nav_msgs::Path ref_plan;
    ref_plan.header = task_plan.header;
    ref_plan.header.seq = plan_seq_++;
    ref_plan.poses.insert(ref_plan.poses.end(), task_plan.poses.begin()+s, task_plan.poses.end());
    ROS_INFO("[BT][%s] s %lu, ref plan size %lu", node_name_, s, ref_plan.poses.size());

    setOutput("ref_plan", ref_plan);
    return BT::NodeStatus::SUCCESS;
  };

 private:
    const char* node_name_ = "task_plan_editor_action";
    size_t plan_seq_ = 0;

}; // class TaskPlanEditorAction

}  // namespace navit_bt_nodes

#endif  // TASK_PLAN_EDITOR_ACTION_HPP_
