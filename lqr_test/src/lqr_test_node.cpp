#include "ros/init.h"
#include "ros/publisher.h"
#include <lqr_test/height_tracking.hpp>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "lqr_test");
  ros::NodeHandle n("~");
  auto ht = lqr_test::Height_tracker(&n);
  ros::Rate r(5);
  double dt = r.expectedCycleTime().toSec();
  ROS_INFO("lqr test loop ..., dt:%f", dt);
  int sleep = 5*3;
  while(ros::ok()) {
    if (sleep--) break;
    r.sleep();
    ros::spinOnce();
  }

  while(ros::ok()) {
    if (ht.Run(dt, 0.01)) break;
    r.sleep();
    ros::spinOnce();
  }
  return 0;
}

