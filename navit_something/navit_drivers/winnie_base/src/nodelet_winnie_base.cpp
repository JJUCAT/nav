#include <nodelet/nodelet.h>

#include <pluginlib/class_list_macros.hpp>

#include "winnie_base/winnie_base.h"

namespace nodelet_winnie_base {

class WinnieBase : public nodelet::Nodelet {
   public:
    void onInit() override {
        ros_handle_            = getPrivateNodeHandle();
        std::string device_url = "udpm:239.255.76.67:7667";
        ros_handle_.param("device_url", device_url, device_url);

        sdk_handle_ = std::make_shared<winnie_sdk::Handle>(device_url);
        if (!sdk_handle_->Init()) {
            ROS_ERROR("sdk handle init error! device url is:%s", device_url.c_str());
            return;
        }

        winnie_base_ = std::make_unique<winnie_base::WinnieBase>(&ros_handle_, sdk_handle_.get());
    }

   private:
    ros::NodeHandle ros_handle_;
    std::shared_ptr<winnie_sdk::Handle> sdk_handle_;
    std::unique_ptr<winnie_base::WinnieBase> winnie_base_;
};

}  // namespace nodelet_winnie_base
PLUGINLIB_EXPORT_CLASS(nodelet_winnie_base::WinnieBase, nodelet::Nodelet)
