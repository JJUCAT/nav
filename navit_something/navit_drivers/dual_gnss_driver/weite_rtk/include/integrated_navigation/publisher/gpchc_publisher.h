#ifndef CATKIN_INTEGRATED_NAVIGATION_GPCHC_PUBLISHER_HPP
#define CATKIN_INTEGRATED_NAVIGATION_GPCHC_PUBLISHER_HPP

#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>

#include "integrated_navigation/filtering/filter_defination.h"
#include "integrated_navigation/utils/utils.h"
#include "navit_msgs/Gpchc.h"

namespace integrated_navigation {
    class GpchcPublisher {
    public:
        GpchcPublisher(ros::NodeHandle& nh, std::string topic_name, std::string health_topic_name, std::string frame_id);
        GpchcPublisher() = default;

        void Publish(const navit_msgs::GpchcPtr gpchc_msg);
        void PublishHealth(const navit_msgs::GpchcPtr gpchc_msg);

    private:
        void PublishData(const navit_msgs::GpchcPtr gpchc_msg);
        void PublishHealthData(const navit_msgs::GpchcPtr gpchc_msg);

        std::string frame_id_;
        ros::Publisher publisher_;
        ros::Publisher health_publisher_;
    };
}
#endif //CATKIN_INTEGRATED_NAVIGATION_GPCHC_PUBLISHER_HPP