#ifndef CATKIN_INTEGRATED_NAVIGATION_POSE_NAVSATFIX_PUBLISHER_HPP
#define CATKIN_INTEGRATED_NAVIGATION_POSE_NAVSATFIX_PUBLISHER_HPP

#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include "integrated_navigation/filtering/filter_defination.h"
#include "integrated_navigation/utils/utils.h"

namespace integrated_navigation {
    class NavSatFixPublisher {
    public:
        NavSatFixPublisher(ros::NodeHandle& nh, std::string topic_name, std::string frame_id, int buff_size);
        NavSatFixPublisher() = default;

        void Publish(const State state);

    private:
        void PublishData(const State state);

        std::string frame_id_;
        ros::Publisher publisher_;
    };
}
#endif //CATKIN_INTEGRATED_NAVIGATION_POSE_NAVSATFIX_PUBLISHER_HPP