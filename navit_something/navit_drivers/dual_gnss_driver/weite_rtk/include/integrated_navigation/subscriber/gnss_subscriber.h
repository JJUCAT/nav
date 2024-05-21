#ifndef SRC_GNSS_SUBSCRIBER_HPP
#define SRC_GNSS_SUBSCRIBER_HPP

#include <navit_msgs/NovatelPosVelHeading.h>
#include <ros/ros.h>

#include <deque>
#include <mutex>
#include <thread>

#include "integrated_navigation/sensor_data/gnss_data.h"

namespace integrated_navigation {
    class GNSSSubscriber {
    public:
        GNSSSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);
        GNSSSubscriber() = default;
        void ParseData(std::deque<GnssPositionDataPtr> &deque_gnss_data);

    private:
        void msg_callback(const navit_msgs::NovatelPosVelHeadingConstPtr &novatel_msg_ptr);

    private:
        ros::Subscriber subscriber_;
        std::deque<GnssPositionDataPtr> new_gnss_data_;

        std::mutex buff_mutex_;
    };
} // namespace integrated_navigation
#endif // SRC_GNSS_SUBSCRIBER_HPP