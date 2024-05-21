#ifndef INTEGRATED_NAVIGATION_WHEEL_DATA_HPP
#define INTEGRATED_NAVIGATION_WHEEL_DATA_HPP

#include <deque>
#include <cmath>
#include <Eigen/Dense>

namespace integrated_navigation {

    struct WheelData {
    public:
        double time = 0.0;
        Eigen::Vector3d base_velocity = Eigen::Vector3d::Zero();
    };
    using WheelDataPtr = std::shared_ptr<WheelData>;
}
#endif //INTEGRATED_NAVIGATION_WHEEL_DATA_HPP