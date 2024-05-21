#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/PointStamped.h"
#include "navit_utils/GetSelection.h"
#include "navit_msgs/CoveragePathAction.h"
#include <actionlib/client/simple_action_client.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "boustrophedon_explorator_planner_test");

  ros::NodeHandle nh("~");

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>();
  tf2_ros::TransformListener tf_listener(*tf_buffer);

  ros::ServiceClient get_edge_client = nh.serviceClient<navit_utils::GetSelection>("/get_selection");

  // 创建 Action 客户端
  typedef actionlib::SimpleActionClient<navit_msgs::CoveragePathAction> CoveragePathActionClient;
  CoveragePathActionClient coverage_path_action_client(nh, "/coverage_path", true);

  nav_msgs::Path coverage_path;
  ros::Subscriber sub = nh.subscribe<geometry_msgs::PointStamped>(
      "/clicked_point", 1, [&](const geometry_msgs::PointStamped::ConstPtr& msg) {
        nav_msgs::Path edge_path;

        // 发起服务请求
        navit_utils::GetSelection srv;
        if (get_edge_client.call(srv))
        {
          if (!srv.response.selection.empty())
          {
            auto& polygons = srv.response.selection.front().polygons;
            for (const auto& polygon : polygons)
            {
              for (const auto& point : polygon.polygon.points)
              {
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = point.x;
                pose.pose.position.y = point.y;
                pose.pose.position.z = point.z;
                edge_path.poses.push_back(pose);
              }
            }
          }
        }

        geometry_msgs::Pose start_pose;
        start_pose.position.x = msg->point.x;
        start_pose.position.y = msg->point.y;
        start_pose.orientation.w = 1;

        // 等待 Action Server 启动
        ROS_INFO("waitForServer...");
        coverage_path_action_client.waitForServer();

        // 创建 Action 请求
        navit_msgs::CoveragePathGoal goal;

        // 设置 Action 请求的字段 (start, edge_path, planner_plugin)
        goal.start.header.frame_id = "map";
        goal.start.pose = start_pose;
        goal.edge_path = edge_path;
        goal.planner_plugin = "boustrophedon";

        // 发起 Action 请求
        ROS_INFO("sendGoal...");
        coverage_path_action_client.sendGoal(goal);

        // 等待 Action 响应
        ROS_INFO("waitForResult...");
        coverage_path_action_client.waitForResult();

        // 处理 Action 响应
        if (coverage_path_action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          ROS_INFO("Action SUCCEEDED.");
          navit_msgs::CoveragePathResultConstPtr result = coverage_path_action_client.getResult();

          // 处理 Action 响应中的字段 (path_found, coverage_path, planning_used_time)
          coverage_path = result->coverage_path;
        }
        else
        {
          ROS_ERROR("Action FAILED.");
        }
      });

  // display
  nav_msgs::Path display_path;
  display_path.header.frame_id = "map";
  ros::Publisher publisher = nh.advertise<nav_msgs::Path>("/out_coverage_path", 1);
  auto timer = nh.createTimer(ros::Duration(0.01), [&](const ros::TimerEvent& event) {
    static int index = 0;
    if (display_path.poses.size() < (index + 1))
    {
      if (coverage_path.poses.size() < (index + 1))
      {
        display_path.poses.clear();
        index = 0;
      }
      else
      {
        display_path.poses.push_back(coverage_path.poses.at(index));
      }
    }
    publisher.publish(display_path);
    ++index;
  });

  timer.start();

  ros::spin();
  return 0;
}
