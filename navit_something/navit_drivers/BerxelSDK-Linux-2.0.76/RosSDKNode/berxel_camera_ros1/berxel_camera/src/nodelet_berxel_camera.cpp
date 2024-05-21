//
// Created by fan on 23-6-27.
//

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <pluginlib/class_list_macros.hpp>

#include "BerxelHawkCamera.h"

namespace nodelet_berxel_camera
{

class BerxelCamera : public nodelet::Nodelet
{
public:
  void onInit() override
  {
    nh_ = getPrivateNodeHandle();

    berxel_camera_ = std::make_unique<BerxelHawkCamera>(nh_);
    int32_t ret = berxel_camera_->initBerxelCamera();
    if (ret != 0)
    {
      ROS_ERROR("Init Berxel Camera Failed");
    }
  }

private:
  ros::NodeHandle nh_;
  std::unique_ptr<BerxelHawkCamera> berxel_camera_;
};

}  // namespace nodelet_berxel_camera
PLUGINLIB_EXPORT_CLASS(nodelet_berxel_camera::BerxelCamera, nodelet::Nodelet)
