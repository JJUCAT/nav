#ifndef SRC_GNSS_PUBLISHER_HPP
#define SRC_GNSS_PUBLISHER_HPP

#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "integrated_navigation/sensor_data/gnss_data.hpp"

namespace integrated_navigation {
    class GNSSPublisher {
    public:
        GNSSPublisher(
                ros::NodeHandle& nh,
                std::string topic_name,
                std::string frame_id,
                size_t buff_size
        );
        GNSSPublisher() = default;

        void Publish(const IMUData &imu_data, double time);

        void Publish(const IMUData &imu_data);

        bool HasSubscribers(void);

    private:
        void PublishData(const IMUData &imu_data, ros::Time time);

        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        std::string frame_id_;

        sensor_msgs::NavSatFix gnss_;
    };
}
#endif //SRC_GNSS_PUBLISHER_HPP