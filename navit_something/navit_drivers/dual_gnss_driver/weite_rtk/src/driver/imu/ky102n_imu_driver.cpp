#include "integrated_navigation/driver/imu/ky102n_imu_driver.h"

namespace integrated_navigation {
    ky102n_imu_driver::ky102n_imu_driver(ros::NodeHandle &nh) {
        // Load yaml file parameters.
        // imu configuration.
        std::string imu_topic = "ky102n_a0/imu_data";
        nh.getParam("/imu_config/frame", frame_id_);
        nh.getParam("/imu_config/topic", imu_topic);
        LOG(INFO) << "imu frame and topic = " << frame_id_ << ", " << imu_topic<<std::endl;

        complementary_filter_ = std::make_shared<ComplementaryFilter>(nh);
        publisher_ = nh.advertise<sensor_msgs::Imu>(imu_topic, 1000);
    }

    ky102n_imu_driver::~ky102n_imu_driver() { }

    void ky102n_imu_driver::ky102n_data_callback(uint8_t *data, const uint16_t crc){
        ky102n_decode(data, crc);
        ky102n_published();
    }

    void ky102n_imu_driver::ky102n_published() {
        // set timestamps
        sensor_msgs::ImuPtr ky102n(new sensor_msgs::Imu());

        ky102n->header.frame_id = frame_id_;
        ky102n->header.stamp = ros::Time::now();

        // temperature
        ky102n->orientation_covariance[0] = cache_imu_data102n_.temp.temperature;

        // set angular velocity:
        geometry_msgs::Vector3 angular_velocity;
        angular_velocity.x = deg2rad(cache_imu_data102n_.Gx.fGx);
        angular_velocity.y = deg2rad(cache_imu_data102n_.Gy.fGy);
        angular_velocity.z = deg2rad(cache_imu_data102n_.Gz.fGz);
        ky102n->angular_velocity = angular_velocity;
        //std::cout  << std::setprecision(14) << ky102n->header.stamp.toSec() << std::endl;

        // set linear acceleration:
        geometry_msgs::Vector3 linear_acceleration;
        linear_acceleration.x = g2mss(cache_imu_data102n_.Ax.fAx);
        linear_acceleration.y = g2mss(cache_imu_data102n_.Ay.fAy);
        linear_acceleration.z = g2mss(cache_imu_data102n_.Az.fAz);
        ky102n->linear_acceleration = linear_acceleration;

        // set quaternion:
        complementary_filter_->ImuDataProcess(linear_acceleration, angular_velocity, ky102n->header.stamp.toSec());
        // Get the orientation:
        double q0, q1, q2, q3;
        complementary_filter_->getOrientation(q0, q1, q2, q3);
        tf::quaternionTFToMsg(tf::Quaternion(q1, q2, q3, q0), ky102n->orientation);

        publisher_.publish(ky102n);
    }

    uint8_t ky102n_imu_driver::ky102n_decode(uint8_t *data, const uint16_t crc) {
        uint8_t *p = data;

        if(ChkCrcValueEx(p, 0x1c, 0xffff) != crc){
            printf("imu crc error.\r\n");
            return -1;
        }

        cache_imu_data102n_.Gx.int_gx = 0;
        cache_imu_data102n_.Gy.int_gy = 0;
        cache_imu_data102n_.Gz.int_gz = 0;
        cache_imu_data102n_.Gx.int_gx = data[3]  << 24  | data[2]  << 16 | data[1]  << 8  | data[0] << 0;
        cache_imu_data102n_.Gy.int_gy = data[7]  << 24  | data[6]  << 16 | data[5]  << 8  | data[4] << 0;
        cache_imu_data102n_.Gz.int_gz = data[11] << 24  | data[10] << 16 | data[9]  << 8  | data[8] << 0;
//        printf(" Gx = %f, Gy = %f, Gz = %f \r\n",cache_imu_data102n_.Gx.fGx, cache_imu_data102n_.Gy.fGy, cache_imu_data102n_.Gz.fGz);

        cache_imu_data102n_.Ax.int_ax = 0;
        cache_imu_data102n_.Ay.int_ay = 0;
        cache_imu_data102n_.Az.int_az = 0;
        cache_imu_data102n_.Ax.int_ax = data[15] << 24  | data[14] << 16 | data[13] << 8  | data[12] << 0;
        cache_imu_data102n_.Ay.int_ay = data[19] << 24  | data[18] << 16 | data[17] << 8  | data[16] << 0;
        cache_imu_data102n_.Az.int_az = data[23] << 24  | data[22] << 16 | data[21] << 8  | data[20] << 0;
//        printf(" Ax = %f, Ay = %f, Az = %f \r\n",cache_imu_data102n_.Ax.fAx, cache_imu_data102n_.Ay.fAy, cache_imu_data102n_.Az.fAz);

        cache_imu_data102n_.temp.int_temperature = 0;
        cache_imu_data102n_.temp.int_temperature = data[27] << 24  | data[26] << 16 | data[25] << 8  | data[24] << 0;
//        printf(" temperature = %f \r\n",cache_imu_data102n_.temp.temperature);

        return 1;
    }

    double ky102n_imu_driver::deg2rad(double deg) {
        return deg * kDegreeToRadian;
    }

    double ky102n_imu_driver::g2mss(double g) {
        return g * 9.78775;
    }
}