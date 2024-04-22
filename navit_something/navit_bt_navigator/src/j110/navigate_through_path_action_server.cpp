#include "navit_bt_navigator/j110/navigate_through_path_action_server.hpp"
#include <ros/package.h>

namespace navit_bt_navigator
{
void NavigateThroughPathActionServer::executeCB(const GoalTypeConstPtr& goal)
{
  double server_timeout_;
  std::vector<size_t> recorder_; // 记录机器走过的任务路径点
  bool on_taskplan_{false}; // 机器是否抵达任务路径上

  auto blackboard = BT::Blackboard::create();
  blackboard->set<ros::NodeHandle>("node", nh_);
  blackboard->set<double>("server_timeout", server_timeout_);
  blackboard->set<nav_msgs::Path>("task_plan", goal->path);
  blackboard->set<std::vector<size_t>>("recorder", recorder_);
  blackboard->set<bool>("on_taskplan", on_taskplan_);

  // feedback date
  blackboard->set<unsigned int>("status_code", feedback_.status_code);
  blackboard->set<std::string>("status_msg", feedback_.status_msg);
  blackboard->set<double>("distance_to_goal", feedback_.distance_to_goal);
  blackboard->set<double>("current_vel", feedback_.current_vel);
  blackboard->set<double>("completion_percentage", feedback_.completion_percentage);
  blackboard->set<unsigned short>("num_of_recoveries", feedback_.num_of_recoveries);

  NavitBtRunner navit_bt_runner("navigate_through_path");
  std::string xml_path = ros::package::getPath("j110_bringup") +
    "/behavior_trees/navigate_through_path_action.xml";
  ROS_INFO("[NavigateThroughPath BT] loading xml %s.", xml_path.c_str());
  
  std::ifstream f{xml_path, std::ios::binary};
  std::stringstream ss; ss << f.rdbuf();
  std::string xml_content = ss.str();

  // if (!navit_bt_runner.init(default_xml_string, "", blackboard))
  if (!navit_bt_runner.init(xml_content, "", blackboard))
  {
    result_.error_code = 999;
    result_.error_msg = "Aborting for bt_tree init error!";
    as_.setAborted(result_, result_.error_msg);
    return;
  }
  ROS_INFO("[NavigateThroughPath BT] create tree succeed.");

  ros::Rate r(100);
  while (ros::ok())
  {
    auto node_status = navit_bt_runner.tickRoot();
    if (node_status == BT::NodeStatus::FAILURE)
    {
      result_.error_code = 999;
      result_.error_msg = "Aborting on the goal because the BT node_status is FAILURE";
      as_.setAborted(result_, result_.error_msg);
      return;
    }

    if (node_status == BT::NodeStatus::SUCCESS)
    {
      result_.error_code = 0;
      result_.error_msg = "Docked successfully.";

      as_.setSucceeded(result_, "Goal path is finished.");
      return;
    }

    if (as_.isPreemptRequested())
    {
      as_.setPreempted();
      return;
    }

    blackboard->get("on_taskplan", on_taskplan_);
    // ROS_WARN("[NTP] ----- on task plan %d ----- ", on_taskplan_);
    feedback_.on_taskplan = on_taskplan_;

    //TODO(czk): feedback
    feedback_.status_code =
    blackboard->get("status_code", feedback_.status_code);
    blackboard->get("status_msg", feedback_.status_msg);
    blackboard->get("distance_to_goal", feedback_.distance_to_goal);
    blackboard->get("current_vel", feedback_.current_vel);
    blackboard->get("completion_percentage", feedback_.completion_percentage);
    blackboard->get("num_of_recoveries", feedback_.num_of_recoveries);

    as_.publishFeedback(feedback_);
    r.sleep();
  }
}
};  // namespace navit_bt_navigator

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "navigate_through_path_action_server");
  std::string xml_string;
  ros::NodeHandle nh;
  navit_bt_navigator::NavigateThroughPathActionServer navigate_through_path_action_server(nh, xml_string);
  ros::spin();
  return 0;
}
