//
// Created by fan on 23-11-9.
//

#ifndef BDF07_IMU_BDF07_IMU_H
#define BDF07_IMU_BDF07_IMU_H

#include <ros/ros.h>
#include <thread>

namespace bdf07_imu
{

class Bdf07Imu
{
public:
  Bdf07Imu(ros::NodeHandle& nh);
  ~Bdf07Imu();

private:
  void run();

  ros::NodeHandle nh_;

  bool stop_ = false;
  std::thread thread_;
};

}  // namespace bdf07_imu

#endif  // BDF07_IMU_BDF07_IMU_H
