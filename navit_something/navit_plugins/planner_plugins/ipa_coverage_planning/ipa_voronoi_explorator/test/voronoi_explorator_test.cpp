#include <tf2_ros/transform_listener.h>
#include "ipa_voronoi_explorator/voronoi_explorator_ros.h"
#include "geometry_msgs/PointStamped.h"
#include "navit_utils/GetSelection.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "voronoi_explorator_test");

  ros::NodeHandle nh("~");

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>();
  tf2_ros::TransformListener tf_listener(*tf_buffer);

  ros::ServiceClient client = nh.serviceClient<navit_utils::GetSelection>("/get_selection");
  auto costmap = std::make_shared<navit_costmap::Costmap2DROS>("ccpp_costmap", *tf_buffer, nh);

  ipa_voronoi_explorator::VoronoiExploratorRos planner;
  planner.initialize(nh.getNamespace(), costmap);

  nav_msgs::Path coverage_path;

  ros::Subscriber sub = nh.subscribe<geometry_msgs::PointStamped>(
      "/clicked_point", 1, [&](const geometry_msgs::PointStamped::ConstPtr& msg) {
        nav_msgs::Path edge_path;

        // 发起服务请求
        navit_utils::GetSelection srv;
        if (client.call(srv))
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

        coverage_path.poses.clear();
        planner.makePlan(edge_path, start_pose, coverage_path);
      });

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
