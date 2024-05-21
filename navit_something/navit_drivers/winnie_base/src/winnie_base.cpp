//
// Created by fan on 23-6-25.
//

#include "winnie_base/winnie_base.h"

#include "winnie_base/device_base.hpp"

namespace winnie_base {

WinnieBase::WinnieBase(ros::NodeHandle *ros_handle, winnie_sdk::Handle *sdk_handle)
    : ros_handle_(ros_handle), sdk_handle_(sdk_handle), device_loader_("winnie_base", "winnie_base::DeviceBase") {
    if (ros_handle->hasParam("plugins")) {
        XmlRpc::XmlRpcValue my_list;
        ros_handle->getParam("plugins", my_list);
        for (int32_t i = 0; i < my_list.size(); ++i) {
            auto device_name = static_cast<std::string>(my_list[i]["name"]);
            auto device_type = static_cast<std::string>(my_list[i]["type"]);
            ROS_INFO("Load device:\"%s\" with type:\"%s\"", device_name.c_str(), device_type.c_str());

            auto device = device_loader_.createInstance(device_type);
            device->Init(ros_handle_, sdk_handle_, device_name);
            devices_[device_name] = device;
        }
    } else {
        ROS_WARN("Param plugins not found!");
    }

    /// heartbeat
    timer_ = ros_handle->createTimer(ros::Duration(1.0), [this](const ros::TimerEvent &event) {
        for (auto &[device_name, device] : devices_) {
            device->SpinOnce();
        }
    });

    thread_ = std::thread([this]() {
        while (ros::ok()) {
            sdk_handle_->Spin();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
}

WinnieBase::~WinnieBase() {
    timer_.stop();
    if (thread_.joinable()) {
        thread_.join();
    }
}

}  // namespace winnie_base