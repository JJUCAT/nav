#ifndef CATKIN_INTEGRATED_NAVIGATION_POSI_VEL_QUAT_STATUS_PUBLISHER_HPP
#define CATKIN_INTEGRATED_NAVIGATION_POSI_VEL_QUAT_STATUS_PUBLISHER_HPP

#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>

#include <navit_msgs/InsInfo.h>
#include "integrated_navigation/filtering/filter_defination.h"
#include "integrated_navigation/utils/utils.h"

namespace integrated_navigation {
    class InsInfoPublisher {
    public:
        InsInfoPublisher(ros::NodeHandle& nh, std::string topic_name, std::string frame_id, int buff_size);
        InsInfoPublisher() = default;

        void Publish(const State state);

    private:
        void PublishData(const State state);

        std::string frame_id_;
        ros::Publisher publisher_;
    };
}
#endif //CATKIN_INTEGRATED_NAVIGATION_POSI_VEL_QUAT_STATUS_PUBLISHER_HPP