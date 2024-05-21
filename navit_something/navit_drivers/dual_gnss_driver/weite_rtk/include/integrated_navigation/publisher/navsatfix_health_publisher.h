#ifndef SRC_NAVSATFIX_HEALTH_PUBLISHER_H
#define SRC_NAVSATFIX_HEALTH_PUBLISHER_H

#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "integrated_navigation/filtering/filter_defination.h"
#include "integrated_navigation/utils/utils.h"

namespace integrated_navigation {
    class NavSatFixHealthPublisher {
    public:
        NavSatFixHealthPublisher(ros::NodeHandle& nh, std::string topic_name, int buff_size);
        NavSatFixHealthPublisher() = default;

        void Publish(const State state);

    private:
        void PublishData(const State state);

        std::string frame_id_;
        ros::Publisher health_publisher_;
    };
}
#endif //SRC_NAVSATFIX_HEALTH_PUBLISHER_H