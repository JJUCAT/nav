#ifndef SRC_KY102N_IMU_H
#define SRC_KY102N_IMU_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "integrated_navigation/driver/io_common/io_common.h"
#include "integrated_navigation/driver/imu/ky102n_protocal.h"
#include "integrated_navigation/models/complementary_filter/complementary_filter.h"

namespace integrated_navigation {
    class ky102n_imu_driver {
    public:
        ky102n_imu_driver(ros::NodeHandle &nh);
        ~ky102n_imu_driver();

        void ky102n_data_callback(uint8_t *data, const uint16_t crc);

    private:
        uint8_t ky102n_decode(uint8_t *data, const uint16_t crc);
        void ky102n_published();
        double deg2rad(double deg);
        double g2mss(double g);

        std::string frame_id_;
        ros::Publisher publisher_;

        Imu102nStruct cache_imu_data102n_;
        std::shared_ptr<ComplementaryFilter> complementary_filter_;
    };
}
#endif //SRC_KY102N_IMU_H