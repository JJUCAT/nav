//
// Created by fan on 23-1-30.
//

#include "navit_bt_navigator/move_to_pose_action_server.h"
#include <fstream>

namespace navit_bt_navigator
{

static const std::string default_xml_string = R"(
<?xml version="1.0"?>
<root main_tree_to_execute="MainTree">
    <!-- ////////// -->
    <BehaviorTree ID="MainTree">
        <Sequence>
            <Action ID="GetStationPose" server_name="/get_map_station_pose" server_timeout="1.0" station="{goal_id}"
                    station_pose="{updated_goal}"/>
            <Control ID="PipelineSequence" name="NavigateWithReplanning">
                <Decorator ID="RateController" hz="1.0">
                    <ForceSuccess>
                        <Action ID="ComputePathToPose" goal="{updated_goal}" path="{path}" planner_id="topoint"
                                server_name="/compute_path" server_timeout="1.0" start="false"/>
                    </ForceSuccess>
                </Decorator>
                <Action ID="FollowPath" controller_id="avoidance" path="{path}" server_name="/follow_path"
                        server_timeout="1.0"/>
            </Control>
        </Sequence>
    </BehaviorTree>
    <!-- ////////// -->
    <TreeNodesModel>
        <Action ID="ComputePathToPose">
            <input_port name="goal">Destination to plan to</input_port>
            <output_port name="path">Path created by ComputePathToPose node</output_port>
            <input_port name="planner_id"/>
            <input_port default="/compute_path" name="server_name">remapped action server name.</input_port>
            <input_port name="server_timeout">...</input_port>
            <input_port name="start">Start pose of the path if overriding current robot pose</input_port>
        </Action>
        <Action ID="FollowPath">
            <input_port name="controller_id"/>
            <input_port name="path">Path to follow</input_port>
            <input_port default="/follow_path" name="server_name">remapped action server name</input_port>
            <input_port name="server_timeout">...</input_port>
        </Action>
        <Action ID="GetStationPose">
            <input_port name="server_name" default="/get_map_station_pose">remapped action server name</input_port>
            <input_port name="server_timeout">...</input_port>
            <input_port name="station">station name or id.</input_port>
            <output_port name="station_pose">The pose of the station.</output_port>
        </Action>
        <Action ID="GetRoutePath">
            <input_port name="server_name" default="/get_map_route_path">remapped action server name</input_port>
            <input_port name="server_timeout">...</input_port>
            <input_port name="route">The route such as LM1, LM2, LM3, LM9.</input_port>
            <output_port name="route_path">The path of the route.</output_port>
        </Action>
        <Decorator ID="GoalUpdater">
            <input_port name="input_goal">Original goal in</input_port>
            <output_port name="output_goal">Output goal set by subscription</output_port>
        </Decorator>
        <Control ID="PipelineSequence"/>
        <Decorator ID="RateController">
            <input_port name="hz">Rate</input_port>
        </Decorator>
        <Action ID="TruncatePath">
            <input_port name="distance">Distance before goal to truncate</input_port>
            <input_port name="input_path">Path to truncate</input_port>
            <output_port name="output_path">Truncated path to utilize</output_port>
        </Action>
    </TreeNodesModel>
    <!-- ////////// -->
</root>

)";

// # MoveToPose.action
// uint64 pose #终点编号
// string bt_name #行为树名 optional, 含有默认行为树
// ---
// int16 error_code #错误码
// string error_msg #错误信息
// ---
// int16 num_of_revoveries #重试次数
// float32 distance_remaining  #剩余距离
// std_msgs/Duration time_elapsed   #已用时间
// std_msgs/Duration time_remaining  #剩余时间

void MoveToPoseActionServer::executeCB(const GoalTypeConstPtr& goal)
{
  std::string xml_string;
  if (goal->bt_name.empty())
  {
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
  blackboard->set<double>("bt_loop_duration", bt_loop_duration_);
  blackboard->set<uint64_t>("goal_id", goal->pose);

  NavitBtRunner navit_bt_runner("move_to_pose");
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
      as_.setSucceeded(result_, "Goal reached.");
      return;
    }

    if (as_.isPreemptRequested())
    {
      as_.setPreempted();
      return;
    }

    // int16 num_of_revoveries #重试次数
    // float32 distance_remaining  #剩余距离
    // std_msgs/Duration time_elapsed   #已用时间
    // std_msgs/Duration time_remaining  #剩余时间
    blackboard->get("num_of_revoveries", feedback_.num_of_revoveries);
    blackboard->get("distance_remaining", feedback_.distance_remaining);
    blackboard->get("time_elapsed", feedback_.time_elapsed);
    blackboard->get("time_remaining", feedback_.time_remaining);
    as_.publishFeedback(feedback_);

    r.sleep();
  }
}
};  // namespace navit_bt_navigator

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "move_to_pose");

  std::string xml_string;
  if (argc >= 2)
  {
    std::string fname(argv[1]);
    ROS_INFO("start move_to_pose_action_server with xml file %s", fname.c_str());

    std::ifstream fin(fname);
    if (!fin.good())
    {
      return -1;
    }
    xml_string = std::string((std::istreambuf_iterator<char>(fin)), std::istreambuf_iterator<char>());
  }

  ros::NodeHandle nh;
  navit_bt_navigator::MoveToPoseActionServer move_to_pose_as(nh, xml_string);
  ros::spin();

  return 0;
}
