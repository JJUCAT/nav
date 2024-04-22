#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <navit_msgs/NavigateComputeCoveragePathAction.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "coverage_planner_client");

  actionlib::SimpleActionClient<navit_msgs::NavigateComputeCoveragePathAction> ac("/navigate_compute_coverage_path", true);

  ROS_INFO("Waiting for action server to start.");

  ac.waitForServer();

  ROS_INFO("Action server started, sending goal.");

  navit_msgs::NavigateComputeCoveragePathGoal goal;

    geometry_msgs::Polygon coverage_area;
    geometry_msgs::Point32 point;

    point.x = -1.0;
    point.y = -1.0;
    coverage_area.points.push_back(point);

    point.x = 30.0;
    point.y = 0.0;
    coverage_area.points.push_back(point);

    point.x = 30.0;
    point.y = 10.0;
    coverage_area.points.push_back(point);

    point.x = -10.0;
    point.y = 30.0;
    coverage_area.points.push_back(point);

    point.x = -18.0;
    point.y = 20.0;
    coverage_area.points.push_back(point);

    point.x = -11.0;
    point.y = 10.0;
    coverage_area.points.push_back(point);

    // point.x = -10.0;
    // point.y = 30.0;
    // coverage_area.points.push_back(point);


    goal.coverage_area = coverage_area;
    // // 设置禁行区域（holes）


    // 设置插件名称（plugin_name）

    geometry_msgs::Pose start_pose;
    start_pose.position.x = 0.0;
    start_pose.position.y = 0.0;
    start_pose.position.z = 0.0;
    start_pose.orientation.w = 1.0;

    goal.start = start_pose;

    geometry_msgs::Pose end_pose;
    end_pose.position.x = 0.0;
    end_pose.position.y = 0.0;
    end_pose.position.z = 0.0;
    end_pose.orientation.w = 1.0;

    goal.end = end_pose;

    ac.sendGoal(goal);

    bool finishedBeforeTimeout = ac.waitForResult(ros::Duration(30.0));

    if (finishedBeforeTimeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
    {
        ROS_INFO("Action did not finish before the time out.");
    }

    return 0;
}
