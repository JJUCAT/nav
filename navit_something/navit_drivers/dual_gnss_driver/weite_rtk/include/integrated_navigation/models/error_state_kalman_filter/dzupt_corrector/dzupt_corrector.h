#ifndef CATKIN_INTEGRATED_NAVIGATION_DZUPT_CORRECTOR_H
#define CATKIN_INTEGRATED_NAVIGATION_DZUPT_CORRECTOR_H

#include "integrated_navigation/models/error_state_kalman_filter/filter_corrector_common.h"

namespace integrated_navigation {
    class DZUPT: public FilterCorrectCommon {
    public:
        DZUPT(const Eigen::Vector3d I_p_Wheel, const int confined_dim, const double vel_cov_coeff);
        ~DZUPT() = default;
        void CorrectByConfinedVelocity(const Eigen::Vector3d confined_velocity, State *state);

    private:
        void CorrectErrorEstimationConfinedVelocity(const Eigen::Vector3d confined_velocity, State *state, Eigen::Matrix<double, 3, 15> *H,
                                                 Eigen::Matrix<double, 3, 1> *residual, Eigen::Matrix<double, 3, 3> *V);

        void ComputeJacobianAndResidual(const Eigen::Vector3d confined_velocity, const State &state,
                                        Eigen::Matrix<double, 3, 15> *jacobian, Eigen::Matrix<double, 3, 1> *residual);

        void VelocityJacobianAndResidual(const Eigen::Vector3d &G_v_I,
                                         const Eigen::Matrix3d &G_R_I,
                                         const Eigen::Vector3d &confined_velocity,
                                         const Eigen::Vector3d &gyro_unbias,
                                         Eigen::Vector3d *residual_velocity,
                                         Eigen::Matrix<double, 3, 15> *jacobian_velocity);

        Eigen::Vector3d I_p_Wheel_;
        int confined_dim_;
        double vel_cov_coeff_;
    };
}
#endif //CATKIN_INTEGRATED_NAVIGATION_DZUPT_CORRECTOR_H