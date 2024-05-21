//
// Created by fan on 22-12-2.
//

#ifndef WINNIE_BASE_DRIVER_BASE_H
#define WINNIE_BASE_DRIVER_BASE_H

#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <winnie_sdk/winnie_sdk.h>

namespace winnie_base {

class DeviceBase;

class WinnieBase {
   public:
    explicit WinnieBase(ros::NodeHandle *ros_handle, winnie_sdk::Handle *sdk_handle);
    ~WinnieBase();

   private:
    ros::Timer timer_;
    std::thread thread_;
    ros::NodeHandle *ros_handle_;
    winnie_sdk::Handle *sdk_handle_;
    pluginlib::ClassLoader<DeviceBase> device_loader_;
    std::map<std::string, boost::shared_ptr<DeviceBase>> devices_;
};

}  // namespace winnie_base

#endif  // WINNIE_BASE_DRIVER_BASE_H
