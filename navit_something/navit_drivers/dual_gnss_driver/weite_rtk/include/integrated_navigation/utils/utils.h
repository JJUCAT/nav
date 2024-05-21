#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <deque>
#include <iomanip>

#include "integrated_navigation/log/logging.h"
#include "integrated_navigation/library/GeographicLib/LocalCartesian.hpp"

namespace integrated_navigation {

    constexpr double kDegreeToRadian = M_PI / 180.;
    constexpr double kRadianToDegree = 180. / M_PI;

    void ConvertLLAToENU(const Eigen::Vector3d& init_lla,
                                const Eigen::Vector3d& point_lla,
                                Eigen::Vector3d* point_enu,
                                std::vector<double>& M);

    void ConvertENUToLLA(const Eigen::Vector3d& init_lla,
                                const Eigen::Vector3d& point_enu,
                                Eigen::Vector3d* point_lla);

    void PositionAndMatrixFromGeoLib(const Eigen::Vector3d& init_lla,
                                                      const Eigen::Vector3d& lla,
                                                      Eigen::Vector3d* position,
                                                      Eigen::Matrix3d* M_ecef2enu);

    // skew matrix.
    Eigen::Matrix3d hat(const Eigen::Vector3d& v);
}  // namespace integrated_navigation