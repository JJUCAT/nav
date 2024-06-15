#include "ros/init.h"
#include "ros/publisher.h"
#include <lqr_test/height_tracking.hpp>

#include <nav_msgs/Path.h>
#include <ros/ros.h>


int main(int argc, char** argv) {
  ros::init(argc, argv, "lqr_test");
  ros::NodeHandle n("~");

  ros::Publisher height_pub = n.advertise<nav_msgs::Path>("height", 1);

  auto ht = lqr_test::Height_tracker();
  ros::Rate r(5);
  while(ros::ok()) {




    r.sleep();
    ros::spinOnce();
  }
  return 0;
}

