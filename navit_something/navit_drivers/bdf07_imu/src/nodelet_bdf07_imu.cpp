//
// Created by fan on 23-11-9.
//

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>
#include "bdf07_imu/bdf07_imu.h"

namespace nodelet_bdf07_imu
{

class Bdf07Imu : public nodelet::Nodelet
{
public:
  void onInit() override
  {
    ros_handle_ = getPrivateNodeHandle();
    app_map_mng_ = std::make_unique<bdf07_imu::Bdf07Imu>(ros_handle_);
  }

private:
  ros::NodeHandle ros_handle_;
  std::unique_ptr<bdf07_imu::Bdf07Imu> app_map_mng_;
};

}  // namespace nodelet_app_map_mng
PLUGINLIB_EXPORT_CLASS(nodelet_bdf07_imu::Bdf07Imu, nodelet::Nodelet)