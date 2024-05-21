#ifndef CATKIN_INTEGRATED_NAVIGATION_IMU_SUBSCRIBER_HPP
#define CATKIN_INTEGRATED_NAVIGATION_IMU_SUBSCRIBER_HPP

#include <ros/ros.h>
#include "sensor_msgs/Imu.h"

#include <deque>
#include <mutex>
#include <thread>

#include "integrated_navigation/sensor_data/imu_data.h"

namespace integrated_navigation {
    class IMUSubscriber {
    public:
        IMUSubscriber(ros::NodeHandle &nh,std::string topic_name,size_t buff_size);
        IMUSubscriber() = default;
        void ParseData(std::deque<ImuDataPtr>& deque_imu_data);

    private:
        void msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;
        std::deque<ImuDataPtr> new_imu_data_;

        std::mutex buff_mutex_;
    };
}
#endif //CATKIN_INTEGRATED_NAVIGATION_IMU_SUBSCRIBER_HPP
