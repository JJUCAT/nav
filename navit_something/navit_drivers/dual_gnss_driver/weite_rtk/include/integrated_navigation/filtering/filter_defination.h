#ifndef SRC_FILTER_DEFINATION_HPP
#define SRC_FILTER_DEFINATION_HPP

#include <memory>
#include <Eigen/Core>
#include <deque>
#include <mutex>
#include "integrated_navigation/sensor_data/imu_data.h"
#include "integrated_navigation/sensor_data/gnss_data.h"
#include "integrated_navigation/sensor_data/wheel_data.h"
#include "integrated_navigation/sensor_data/velocity_data.h"

namespace integrated_navigation {
    struct State {
        double timestamp;                     // filter timestamp.

        // output
        Eigen::Vector3d lla;                  // WGS84 position, difference between imu and antenna fusion center.
        Eigen::Vector3d position;             // position in E, N, U, difference between imu and antenna fusion center. if imu, equal to G_p_I.
        Eigen::Vector3d velocity;             // velocity in E, N, U, difference between imu and antenna fusion center. if imu, equal to G_v_I.

        // filter
        Eigen::Vector3d G_p_I;                // The original point of the IMU frame in the Global frame.
        Eigen::Vector3d G_v_I;                // The velocity original point of the IMU frame in the Global frame.
        Eigen::Matrix3d G_R_I;                // The rotation from the IMU frame to the Global frame.
        Eigen::Vector3d acc_bias;             // The bias of the acceleration sensor.
        Eigen::Vector3d gyro_bias;            // The bias of the gyroscope sensor.
        Eigen::Matrix<double, 15, 15> cov;    // Filter Covariance.

        Eigen::Matrix3d residual;

        // The imu data.
        ImuDataPtr imu_data_ptr;

        // filter state.
        unsigned int filter_status;

        // gnss info, from gpgga and bestpos.
        unsigned int gps_week_num;
        double gps_seconds;
        unsigned int gnss_quality;
        unsigned int num_satellites1;
        unsigned int num_satellites2;
        double age;
        bool steady_state;
        int steady_status;
        double lock_heading;
    };

    struct DebugState{
        double timestamp;

        bool initialised = false;
        Eigen::Vector3d init_lla = Eigen::Vector3d::Zero();
    };
}  // integrated_navigation
#endif //SRC_FILTER_DEFINATION_HPP