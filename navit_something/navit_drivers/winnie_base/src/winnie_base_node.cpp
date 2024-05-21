//
// Created by fan on 23-5-30.
//

#include "winnie_base/winnie_base.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "winnie_base_node");

    ros::NodeHandle ros_handle("~");
    std::string device_url = "udpm:239.255.76.67:7667";
    ros_handle.param("device_url", device_url, device_url);

    auto sdk_handle = std::make_shared<winnie_sdk::Handle>(device_url);
    if (!sdk_handle->Init()) {
        ROS_ERROR("sdk handle init error! device url is:%s", device_url.c_str());
        return -1;
    }

    auto winnie_base = std::make_unique<winnie_base::WinnieBase>(&ros_handle, sdk_handle.get());

    ros::spin();

    return 0;
}