#ifndef INTEGRATED_NAVIGATION_WHEEL_SUBSCRIBER_HPP
#define INTEGRATED_NAVIGATION_WHEEL_SUBSCRIBER_HPP

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "integrated_navigation/sensor_data/wheel_data.h"

namespace integrated_navigation {
    class WheelSubscriber {
    public:
        WheelSubscriber(ros::NodeHandle& nh, std::string topic_name,
                        size_t buff_size, const double wheel_scale_factor);
        WheelSubscriber() = default;
        void ParseData(std::deque<WheelDataPtr> &wheel_data_buffer);

    private:
        void msg_callback(const nav_msgs::OdometryConstPtr &odom_msg_ptr);

    private:
        ros::Subscriber subscriber_;
        const double wheel_scale_factor_;
        std::deque<WheelDataPtr> new_wheel_data_;
        std::mutex buff_mutex_;
    };
}
#endif //INTEGRATED_NAVIGATION_WHEEL_SUBSCRIBER_HPP