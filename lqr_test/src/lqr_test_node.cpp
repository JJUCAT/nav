#include "ros/init.h"
#include "ros/publisher.h"
#include <lqr_test/height_tracking.hpp>
#include <lqr_test/diff_tracking.hpp>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "lqr_test");
  std::string mode = argv[1];
  ROS_INFO("lqr test mode : %s", mode.c_str());

  ros::NodeHandle n("~");
  auto ht = lqr_test::Height_tracker(&n);
  auto ct = lqr_test::Diff_tracker(&n);
  ros::Rate r(10);
  double dt = r.expectedCycleTime().toSec();
  ROS_INFO("lqr test loop ..., dt:%f", dt);
  int sleep = 5*3;
  while(ros::ok()) {
    if (sleep--) break;
    r.sleep();
    ros::spinOnce();
  }

  while(ros::ok()) {
    if (mode == "height") {
      if (ht.Run(dt, 0.01)) break;
    } else if (mode == "diff") {
      if (ct.Run(dt, 0.01)) break;
    }
    r.sleep();
    ros::spinOnce();
  }
  return 0;
}

