#ifndef CATKIN_INTEGRATED_NAVIGATION_WHEEL_CORRECTOR_H
#define CATKIN_INTEGRATED_NAVIGATION_WHEEL_CORRECTOR_H

#include "integrated_navigation/models/error_state_kalman_filter/filter_corrector_common.h"

namespace integrated_navigation {
    class WheelCorrector : public FilterCorrectCommon {
    public:
        WheelCorrector(const Eigen::Vector3d I_p_Wheel);
        ~WheelCorrector() = default;
        void CorrectByWheelVelocity(const Eigen::Vector3d wheel_velocity, State *state);

    private:
        void CorrectErrorEstimationWheelVelocity(const Eigen::Vector3d wheel_velocity, State *state, Eigen::Matrix<double, 3, 15> *H,
                                                 Eigen::Matrix<double, 3, 1> *residual, Eigen::Matrix<double, 3, 3> *V);

        void ComputeJacobianAndResidual(const Eigen::Vector3d wheel_velocity, const State &state,
                                        Eigen::Matrix<double, 3, 15> *jacobian, Eigen::Matrix<double, 3, 1> *residual);

        void VelocityJacobianAndResidual(const Eigen::Vector3d &G_v_I,
                                         const Eigen::Matrix3d &G_R_I,
                                         const Eigen::Vector3d &wheel_velocity,
                                         const Eigen::Vector3d &gyro_unbias,
                                         Eigen::Vector3d *residual_velocity,
                                         Eigen::Matrix<double, 3, 15> *jacobian_velocity);

        Eigen::Vector3d I_p_Wheel_;
    };
}
#endif //CATKIN_INTEGRATED_NAVIGATION_WHEEL_CORRECTOR_H