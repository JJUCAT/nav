#ifndef INTEGRATED_NAVIGATION_SYSTEM_CONTROL_PUBLISHER_HPP
#define INTEGRATED_NAVIGATION_SYSTEM_CONTROL_PUBLISHER_HPP

#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>

#include "integrated_navigation/control/system_message_defination.h"
#include <navit_msgs/SystemErrorStatus.h>

namespace integrated_navigation {
    class SystemControlPublisher {
    public:
        SystemControlPublisher(ros::NodeHandle& nh, std::string topic_name, std::string frame_id, int buff_size);
        SystemControlPublisher() = default;

        void Publish(const RoverMessagePtr rover_ptr, const BaseMessagePtr base_ptr, const bool imu_status, const bool gnss_status);

    private:
        void PublishData(const RoverMessagePtr rover_ptr, const BaseMessagePtr base_ptr, const bool imu_status, const bool gnss_status);

        std::string frame_id_;
        ros::Publisher publisher_;
    };
}
#endif //INTEGRATED_NAVIGATION_SYSTEM_CONTROL_PUBLISHER_HPP