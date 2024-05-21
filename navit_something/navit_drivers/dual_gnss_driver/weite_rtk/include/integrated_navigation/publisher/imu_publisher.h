#ifndef INTEGRATED_NAVIGATION_IMU_PUBLISHER_HPP
#define INTEGRATED_NAVIGATION_IMU_PUBLISHER_HPP

#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "integrated_navigation/filtering/filter_defination.h"
#include "integrated_navigation/utils/utils.h"

namespace integrated_navigation {
    class ImuPublisher {
    public:
        ImuPublisher(ros::NodeHandle& nh, std::string topic_name, int buff_size);
        ImuPublisher() = default;

        void Publish(const State state);

    private:
        void PublishData(const State state);

        ros::Publisher publisher_;
    };
}
#endif //INTEGRATED_NAVIGATION_IMU_PUBLISHER_HPP