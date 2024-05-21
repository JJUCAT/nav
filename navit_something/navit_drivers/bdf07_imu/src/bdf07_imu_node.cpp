#include "bdf07_imu/bdf07_imu.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "bdf07_imu_node");
  ros::NodeHandle nh("~");

  ROS_INFO("start node bdf07_imu!");
  bdf07_imu::Bdf07Imu app(nh);

  ros::spin();
  return 0;
}