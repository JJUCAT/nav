#include "integrated_navigation/publisher/navsatfix_health_publisher.h"

namespace integrated_navigation {
    NavSatFixHealthPublisher::NavSatFixHealthPublisher(ros::NodeHandle &nh, std::string topic_name, int buff_size) {
        health_publisher_ = nh.advertise<std_msgs::Bool>(topic_name, buff_size, true);
    }

    void NavSatFixHealthPublisher::Publish(const State state) { PublishData(state); }

    void NavSatFixHealthPublisher::PublishData(const State state) {
        std_msgs::BoolPtr fix(new std_msgs::Bool());
        if (state.filter_status == 0) {
            fix->data = false;
        } else {
            if (state.gnss_quality == 4 || state.gnss_quality == 17 || state.gnss_quality == 3 || state.gnss_quality == 16) {
                fix->data = true;
            } else {
                fix->data = false;
            }
        }

        health_publisher_.publish(fix);
    }
}