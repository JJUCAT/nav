#include "navit_bt_navigator/j110/auto_dock_server.hpp"

namespace navit_bt_navigator
{

void AutoDockActionServer::executeCB(const GoalTypeConstPtr& goal)
{
  std::string xml_string;
  if (goal->bt_name.empty())
  {
    ROS_INFO("Goal bt name is empty.");
    xml_string = default_xml_string_.empty() ? default_xml_string : default_xml_string_;
  }
  else
  {
      std::ifstream fin(goal->bt_name);
      if (!fin.good())
      { 
        result_.error_code = 999; 
        result_.error_msg = "Aborting for bt_name file is not exists.";
        as_.setAborted(result_, result_.error_msg);
        return;
      }
      xml_string = std::string((std::istreambuf_iterator<char>(fin)), std::istreambuf_iterator<char>());
  }

  double server_timeout_;
  double bt_loop_duration_;

  auto blackboard = BT::Blackboard::create();
  blackboard->set<ros::NodeHandle>("node", nh_);

  blackboard->set<double>("server_timeout", server_timeout_);
  // blackboard->set<double>("bt_loop_duration", bt_loop_duration_);

  blackboard->set<geometry_msgs::PoseStamped>("expected_dock_pose", goal->expected_dock_pose);
  blackboard->set<std::string>("controller_plugin_name", goal->controller_plugin_name);
  blackboard->set<std::string>("perception_plugin_name", goal->perception_plugin_name);
  blackboard->set<std::string>("filter_plugin_name", goal->filter_plugin_name);
  blackboard->set<float>("percepted_pose_offset", goal->percepted_pose_offset);
  blackboard->set<bool>("use_forward_nav_point", goal->use_forward_nav_point);
  blackboard->set<geometry_msgs::PoseStamped>("forward_nav_point", goal->forward_nav_point);
  blackboard->set<bool>("rotate_in_place", goal->rotate_in_place);

  NavitBtRunner navit_bt_runner("auto_dock_server");

  if (!navit_bt_runner.init(xml_string, "", blackboard))
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
      result_.docked = true;
      result_.error_code = 0;
      result_.error_msg = "Docked successfully.";

      as_.setSucceeded(result_, "Goal reached.");
      return;
    }

    if (as_.isPreemptRequested())
    {
      as_.setPreempted();
      return;
    }
    //TODO(czk): feedback
    as_.publishFeedback(feedback_);

    r.sleep();
  }
}
};  // namespace navit_bt_navigator

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "bt_navigator_auto_dock_server");
  std::string xml_string;
  ros::NodeHandle nh;
  navit_bt_navigator::AutoDockActionServer auto_docke_server(nh, xml_string);
  ros::spin();
  return 0;
}
