//
// Created by fan on 23-6-27.
//

#ifndef TOF_DEV_SDK_DEMO_SUNNY_TOF_CAMERA_H
#define TOF_DEV_SDK_DEMO_SUNNY_TOF_CAMERA_H

#include <ros/ros.h>

#include <thread>

namespace sunny_tof_camera {

class SunnyTofCamera {
   public:
    explicit SunnyTofCamera(ros::NodeHandle &nh);
    ~SunnyTofCamera();

    int Init();

    void Stop();

   private:
    ros::NodeHandle nh_;
    std::thread thread_;
};

}  // namespace sunny_tof_camera

#endif  // TOF_DEV_SDK_DEMO_SUNNY_TOF_CAMERA_H
