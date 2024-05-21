//
// Created by fan on 23-6-27.
//

#include <ros/ros.h>

#include "sunny_tof_camera.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "sunny_tof_camera_node");

    ros::NodeHandle nh("~");

    auto sunny_tof_camera = std::make_unique<sunny_tof_camera::SunnyTofCamera>(nh);
    sunny_tof_camera->Init();

    ros::spin();

    sunny_tof_camera->Stop();
    return 0;
}