//
// Created by fan on 23-7-3.
//

#ifndef WINNIE_BASE_ROS_HELPER_HPP
#define WINNIE_BASE_ROS_HELPER_HPP

#include <ros/ros.h>

namespace winnie_base {

/// 未考虑相同的消息类型，不同的Topic.
template <typename T>
class RosPublisherHelper {
   public:
    void Init(ros::NodeHandle* handle, const std::string& topic, uint32_t queue_size = 1, bool latch = false) {
        publisher_ = handle->advertise<T>(topic, queue_size, latch);
    }

    void publish(T& msg) { publisher_.publish(msg); }
    void publish(const boost::shared_ptr<T>& msg) const { publisher_.publish(msg); }

   private:
    ros::Publisher publisher_;
};

template <typename T>
class RosSubscriberHelper {
   public:
    using MsgTypePtr   = typename T::ConstPtr;
    using CallBackType = std::function<void(const MsgTypePtr&)>;

    void Init(ros::NodeHandle* handle, const std::string& topic, uint32_t queue_size = 1) {
        subscriber_ = handle->subscribe<T>(topic, queue_size, &RosSubscriberHelper::callback, this);
    }

    void register_topic_callback(const CallBackType& callback) { callback_ = callback; }

   private:
    void callback(const MsgTypePtr& msg) {
        if (callback_)
            callback_(msg);
    }
    CallBackType callback_;
    ros::Subscriber subscriber_;
};

template <typename T>
class RosServiceServerHelper {
   public:
    using TRequest     = typename T::RequestType;
    using TResponse    = typename T::ResponseType;
    using CallBackType = std::function<bool(TRequest&, TResponse&)>;

    void Init(ros::NodeHandle* handle, const std::string& service_name) {
        service_server_ = handle->advertiseService(service_name, &RosServiceServerHelper::callback, this);
    }

    void register_service_callback(const CallBackType& callback) { callback_ = callback; }

   private:
    bool callback(TRequest& req, TResponse& res) { return callback_ && callback_(req, res); }
    CallBackType callback_;
    ros::ServiceServer service_server_;
};
}  // namespace winnie_base
#endif  // WINNIE_BASE_ROS_HELPER_HPP
