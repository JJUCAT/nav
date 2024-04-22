#include "navit_bt_navigator/j110/navigate_to_pose_action_server.hpp"
#include <ros/package.h>

namespace navit_bt_navigator
{

void NavigateToPoseActionServer::executeCB(const GoalTypeConstPtr& goal)
{
  double server_timeout_;
  double bt_loop_duration_;

  auto blackboard = BT::Blackboard::create();
  blackboard->set<ros::NodeHandle>("node", nh_);
  blackboard->set<double>("server_timeout", server_timeout_);
  blackboard->set<std::string>("error_code", std::to_string(result_.error_code));
  blackboard->set<geometry_msgs::PoseStamped>("start", goal->start);
  blackboard->set<geometry_msgs::PoseStamped>("goal", goal->goal);

  blackboard->set<bool>("use_start", goal->use_start);
  blackboard->set<uint16_t>("arrival_mode", goal->arrival_mode);
  blackboard->set<float>("xy_goal_tolerance", goal->xy_goal_tolerance);
  blackboard->set<float>("yaw_goal_tolerance", goal->yaw_goal_tolerance);

  // feedback message
  blackboard->set<unsigned int>("status_code", feedback_.status_code);
  blackboard->set<std::string>("status_msg", feedback_.status_msg);
  blackboard->set<float>("distance_to_goal", feedback_.distance_to_goal);
  blackboard->set<float>("current_vel", feedback_.current_vel);
  blackboard->set<float>("completion_percentage", feedback_.completion_percentage);

  NavitBtRunner navit_bt_runner("navigator_to_pose");

  std::string xml_path = ros::package::getPath("j110_bringup") +
    "/behavior_trees/A2BMove.xml";
  ROS_INFO("[NavigateToPath BT] loading xml %s.", xml_path.c_str());
  
  std::ifstream f{xml_path, std::ios::binary};
  std::stringstream ss; ss << f.rdbuf();
  std::string xml_content = ss.str();

  if (!navit_bt_runner.init(xml_content, "", blackboard))
  {
    result_.error_code = 999;
    result_.error_msg = "Aborting for bt_tree init error!";
    as_.setAborted(result_, result_.error_msg);
    return;
  }

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
      result_.error_msg = "Navigate to pose successfully.";
      as_.setSucceeded(result_, "Goal reached.");
      return;
    }

    if (as_.isPreemptRequested())
    {
      as_.setPreempted();
      return;
    }

    //TODO(czk): feedback
    feedback_.status_code =
    blackboard->get("status_code", feedback_.status_code);
    blackboard->get("status_msg", feedback_.status_msg);
    blackboard->get("distance_to_goal", feedback_.distance_to_goal);
    blackboard->get("current_vel", feedback_.current_vel);
    blackboard->get("completion_percentage", feedback_.completion_percentage);
    as_.publishFeedback(feedback_);
    r.sleep();
  }
}
};  // namespace navit_bt_navigator

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "navigate_to_pose_action_server");
  std::string xml_string;
  ros::NodeHandle nh;
  navit_bt_navigator::NavigateToPoseActionServer navigate_to_pose_action_server(nh, xml_string);
  ros::spin();
  return 0;
}
