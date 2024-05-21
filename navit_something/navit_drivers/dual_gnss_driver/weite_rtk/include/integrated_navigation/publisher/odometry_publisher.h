#ifndef INTEGRATED_NAVIGATION_ODOMETRY_PUBLISHER_HPP
#define INTEGRATED_NAVIGATION_ODOMETRY_PUBLISHER_HPP

#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "integrated_navigation/filtering/filter_defination.h"
#include "integrated_navigation/utils/utils.h"

namespace integrated_navigation {
    class OdometryPublisher {
    public:
        OdometryPublisher(ros::NodeHandle& nh,
                          std::string topic_name,
                          std::string base_frame_id,
                          std::string child_frame_id,
                          int buff_size);
        OdometryPublisher() = default;

        void Publish(const State state);
        void Publish(const State state, const double time);

    private:
        void PublishData(const State state);
        void PublishData(const State state, const double time);

        ros::Publisher publisher_;
        std::string base_frame_id_;
        std::string child_frame_id_;
    };
}
#endif //INTEGRATED_NAVIGATION_ODOMETRY_PUBLISHER_HPP