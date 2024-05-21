#ifndef CATKIN_INTEGRATED_NAVIGATION_IMU_DATA_HPP
#define CATKIN_INTEGRATED_NAVIGATION_IMU_DATA_HPP

#include <deque>
#include <cmath>
#include <Eigen/Dense>

namespace integrated_navigation {
    struct ImuData {
        double timestamp;                     // timestamp in second.

        Eigen::Vector3d acc;                  // Acceleration in m/s^2
        Eigen::Vector3d gyro;                 // Angular velocity in radian/s.
    };
    using ImuDataPtr = std::shared_ptr<ImuData>;
}
#endif //CATKIN_INTEGRATED_NAVIGATION_IMU_DATA_HPP