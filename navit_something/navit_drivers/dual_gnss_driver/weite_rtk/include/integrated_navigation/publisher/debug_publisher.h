#ifndef CATKIN_INTEGRATED_NAVIGATION_DEBUG_DATA_PUBLISHER_HPP
#define CATKIN_INTEGRATED_NAVIGATION_DEBUG_DATA_PUBLISHER_HPP

#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>

#include "integrated_navigation/filtering/filter_defination.h"
#include "integrated_navigation/utils/utils.h"
#include <navit_msgs/Debug.h>

namespace integrated_navigation {
    class DebugPublisher {
    public:
        DebugPublisher(ros::NodeHandle& nh, std::string topic_name, int buff_size);
        DebugPublisher() = default;

        void Publish(const DebugState dstate);

    private:
        void PublishData(const DebugState dstate);

        ros::Publisher publisher_;
    };
}
#endif //CATKIN_INTEGRATED_NAVIGATION_DEBUG_DATA_PUBLISHER_HPP