#ifndef TASK_PLAN_REPORTER_ACTION_HPP_
#define TASK_PLAN_REPORTER_ACTION_HPP_

#include "geometry_msgs/PoseStamped.h"
#include "navit_bt_nodes/bt_action_node.h"
#include <string>
#include <google/protobuf/util/json_util.h>
#include <navit_bt_nodes/bt_exception.hpp>
#include <navit_common/log.h>
#include <tf/tf.h>
#include "nav_msgs/Path.h"

namespace navit_bt_nodes {

class TaskPlanReporterAction : public BT::ActionNodeBase
{
 public:

  TaskPlanReporterAction(const std::string& xml_tag_name, const BT::NodeConfiguration& conf)
    : BT::ActionNodeBase(xml_tag_name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::Path>("task_plan", "task_plan"), // 任务路径
      BT::InputPort<std::vector<size_t>>("recorder", "recorder"), // 记录任务路径执行到哪里了
      BT::OutputPort<double>("completion_percentage", "completion_percentage"), // 任务进度
      BT::OutputPort<double>("clean_percentage", "clean_percentage"), // 清洁进度
    };
  }

 private:

  void LoadArg(nav_msgs::Path& task_plan, std::vector<size_t>& recorder)
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
  }

  void halt() override
  {
  }

  BT::NodeStatus tick() override
  {
    nav_msgs::Path task_plan;
    std::vector<size_t> recorder;
    LoadArg(task_plan, recorder);
    // ROS_INFO("[BT][%s] task plan size %lu, recorder %lu",
    //   node_name_, task_plan.poses.size(), recorder.size());

    double completion_percentage = 0.0, clean_percentage = 0.0;
    if (!recorder.empty()) {
      completion_percentage = (recorder.back()+1) * 100.f / task_plan.poses.size();
      clean_percentage = (recorder.size()) * 100.f / task_plan.poses.size();
    }
    // ROS_INFO("[BT][%s] completion_percentage %.3f, clean_percentage %.3f",
    //   node_name_, completion_percentage, clean_percentage);

    setOutput("completion_percentage", completion_percentage);
    setOutput("clean_percentage", clean_percentage);
    return BT::NodeStatus::SUCCESS;
  };

 private:
    const char* node_name_ = "task_plan_reporter_action";

}; // class TaskPlanReporterAction

}  // namespace navit_bt_nodes

#endif  // TASK_PLAN_REPORTER_ACTION_HPP_
