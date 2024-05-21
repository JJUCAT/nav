#include "integrated_navigation/publisher/debug_publisher.h"

namespace integrated_navigation {
    DebugPublisher::DebugPublisher(ros::NodeHandle &nh, std::string topic_name, int buff_size) {
        publisher_ = nh.advertise<navit_msgs::Debug>(topic_name, buff_size);
    }

    void DebugPublisher::Publish(const DebugState dstate) { PublishData(dstate); }

    void DebugPublisher::PublishData(const DebugState dstate) {
        navit_msgs::DebugPtr debug(new navit_msgs::Debug());
        ros::Time ros_time(dstate.timestamp);
        debug->header.stamp = ros_time;
        debug->lat = dstate.init_lla.x();
        debug->lon = dstate.init_lla.y();
        debug->height = dstate.init_lla.z();
        debug->initialise = dstate.initialised;

        publisher_.publish(debug);
    }
} // namespace integrated_navigation