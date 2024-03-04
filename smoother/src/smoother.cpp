#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include "smoother/smoother_cost_function.hpp"
#include "tf/tf.h"
#include <memory>
#include <nav_msgs/Path.h>
#include <limits>
#include <smoother/smoother.hpp>

nav_msgs::Path genRandomPath()
{
  unsigned int now = clock();
  srand(static_cast<unsigned int>(now));

  nav_msgs::Path path;
  path.header.frame_id = "map";
  path.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped pose;
  pose.header = path.header;
  for (int i = 0; i < 50; i++) {
    int rand0 = rand();
    srand(static_cast<unsigned int>(rand0));
    pose.pose.position.x = i * 0.1;
    pose.pose.position.y = static_cast<double>(rand0) / RAND_MAX * 1.0 - 0.5;
    path.poses.push_back(pose);
  }
  return path;
}

nav_msgs::Path Smooth(const std::shared_ptr<smoother::Smoother>& smoother,
                      const nav_msgs::Path& unsmooth_path)
{
  std::vector<Eigen::Vector2d> splan;
  for (auto p : unsmooth_path.poses) {
    Eigen::Vector2d p2d(p.pose.position.x, p.pose.position.y);
    splan.push_back(p2d);
  }

  nav_msgs::Path smooth_path;  
  smooth_path.header = unsmooth_path.header;

  smoother::SmootherParams params;
  // params.distance_weight = 10;
  if (!smoother->smooth(splan, params)) {
    ROS_ERROR("can't smooth plan");
  } else {
    for (auto p2d : splan) {
      geometry_msgs::PoseStamped p;
      p.header = smooth_path.header;
      p.pose.position.x = p2d(0);
      p.pose.position.y = p2d(1);
      p.pose.position.z = 0;
      p.pose.orientation.w = 1;
      smooth_path.poses.push_back(p);
    }
  }

  return smooth_path;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "smoother");
  ros::NodeHandle n("~");
  ros::Publisher plan_pub = n.advertise<nav_msgs::Path>("unsmoother_plan", 1);
  ros::Publisher splan_pub = n.advertise<nav_msgs::Path>("smoother_plan", 1);

  auto smoother = std::make_shared<smoother::Smoother>();
  smoother->initialize();

  double hz = 0.5f;
  ros::Rate r(hz);
  while(ros::ok()) {
    nav_msgs::Path unsmooth_path = genRandomPath();
    plan_pub.publish(unsmooth_path);

    nav_msgs::Path smooth_path = Smooth(smoother, unsmooth_path);
    splan_pub.publish(smooth_path);
    r.sleep();
    ros::spinOnce();
  }
  return 0;
}