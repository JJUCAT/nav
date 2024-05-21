//
// Created by fan on 23-6-27.
//

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <pluginlib/class_list_macros.hpp>

#include "sunny_tof_camera.h"

namespace nodelet_sunny_tof_camera {

class SunnyTofCamera : public nodelet::Nodelet {
   public:
    void onInit() override {
        nh_               = getPrivateNodeHandle();
        sunny_tof_camera_ = std::make_unique<sunny_tof_camera::SunnyTofCamera>(nh_);
        sunny_tof_camera_->Init();
    }

    ~SunnyTofCamera() override { sunny_tof_camera_->Stop(); }

   private:
    ros::NodeHandle nh_;
    std::unique_ptr<sunny_tof_camera::SunnyTofCamera> sunny_tof_camera_;
};

}  // namespace nodelet_sunny_tof_camera
PLUGINLIB_EXPORT_CLASS(nodelet_sunny_tof_camera::SunnyTofCamera, nodelet::Nodelet)
