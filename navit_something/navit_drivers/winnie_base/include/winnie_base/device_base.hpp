//
// Created by fan on 23-7-3.
//

#ifndef WINNIE_BASE_DEVICE_BASE_HPP
#define WINNIE_BASE_DEVICE_BASE_HPP

#include <ros/ros.h>
#include <winnie_sdk/winnie_sdk.h>

namespace winnie_base {

class DeviceBase {
   public:
    virtual ~DeviceBase() = default;

    bool Init(ros::NodeHandle *ros_handle, winnie_sdk::Handle *sdk_handle, const std::string &device_name) {
        ros_handle_  = ros_handle;
        sdk_handle_  = sdk_handle;
        device_name_ = device_name;
        return OnInit();
    }

    virtual void SpinOnce() {}

   protected:
    virtual bool OnInit() { return true; }

   protected:
    ros::NodeHandle *ros_handle_;
    winnie_sdk::Handle *sdk_handle_;
    std::string device_name_;
};

}  // namespace winnie_base

#endif  // WINNIE_BASE_DEVICE_BASE_HPP
