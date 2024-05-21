#ifndef SRC_GNSS_CORRECTOR_HPP
#define SRC_GNSS_CORRECTOR_HPP

#include "integrated_navigation/models/error_state_kalman_filter/filter_corrector_common.h"

namespace integrated_navigation {
    class GNSSCorrector : public  FilterCorrectCommon{
    public:
        GNSSCorrector(const Eigen::Vector3d I_p_Gps);

        void CorrectByPosiVelHeading(const Eigen::Vector3d& init_lla, const GnssPositionDataPtr gnss_data_ptr, State* state);
        void CorrectByPosi(const Eigen::Vector3d& init_lla, const GnssPositionDataPtr gnss_data_ptr, State* state);
        void CorrectByVel(const Eigen::Vector3d &init_lla, const GnssPositionDataPtr gnss_data_ptr, State *state);
        void CorrectByHeading(const GnssPositionDataPtr gnss_data_ptr, State *state);
        void CorrectByZeroVelocityZ(const bool is_zero, State *state);

    private:

        void CorrectErrorEstimationPosiVelHeading(const Eigen::Vector3d& init_lla, const GnssPositionDataPtr gnss_data_ptr, State* state,
                                                  Eigen::Matrix<double, 7, 15> *H, Eigen::Matrix<double, 7, 1> *residual, Eigen::Matrix<double, 7, 7> *V);
        void CorrectErrorEstimationPosi(const Eigen::Vector3d& init_lla, const GnssPositionDataPtr gnss_data_ptr, State* state,
                                        Eigen::Matrix<double, 3, 15> *H, Eigen::Matrix<double, 3, 1> *residual, Eigen::Matrix<double, 3, 3> *V);
        void CorrectErrorEstimationVel(const Eigen::Vector3d& init_lla, const GnssPositionDataPtr gnss_data_ptr, State* state,
                                        Eigen::Matrix<double, 3, 15> *H, Eigen::Matrix<double, 3, 1> *residual, Eigen::Matrix<double, 3, 3> *V);
        void CorrectErrorEstimationHeading(const GnssPositionDataPtr gnss_data_ptr, State* state,
                                       Eigen::Matrix<double, 1, 15> *H, double *residual, double *V);
        void ComputeJacobianAndResidual(const Eigen::Vector3d &init_lla,
                                        const GnssPositionDataPtr gps_data,
                                        State *state,
                                        Eigen::Matrix<double, 7, 15> *jacobian,
                                        Eigen::Matrix<double, 7, 1> *residual);
        void ComputeJacobianAndResidual(const Eigen::Vector3d &init_lla,
                                        const GnssPositionDataPtr gps_data,
                                        const State &state,
                                        Eigen::Matrix<double, 3, 15> *jacobian,
                                        Eigen::Matrix<double, 3, 1> *residual);
        void ComputeJacobianAndResidual(const Eigen::Vector3d &init_lla,
                                        const State &state,
                                        const GnssPositionDataPtr gps_data,
                                        Eigen::Matrix<double, 3, 15> *jacobian,
                                        Eigen::Matrix<double, 3, 1> *residual);
        void ComputeJacobianAndResidual(const State &state,
                                        const GnssPositionDataPtr gps_data,
                                        Eigen::Matrix<double, 1, 15> *jacobian,
                                        double *residual);
        void PositionJacobianAndResidual(const Eigen::Vector3d &init_lla,
                                         const Eigen::Vector3d &lla,
                                         const Eigen::Vector3d &G_p_I,
                                         const Eigen::Matrix3d &G_R_I,
                                         Eigen::Vector3d *residual_position,
                                         Eigen::Matrix<double, 3, 15> *jacobian_position,
                                         Eigen::Matrix3d *M_ecef2enu);
        void VelocityJacobianAndResidual(const Eigen::Vector3d &G_v_I,
                                         const Eigen::Matrix3d &G_R_I,
                                         const Eigen::Matrix<double, 3, 3> &M_ecef2enu,
                                         const Eigen::Vector3d &velocity,
                                         const Eigen::Vector3d &gyro_unbias,
                                         Eigen::Vector3d *residual_velocity,
                                         Eigen::Matrix<double, 3, 15> *jacobian_velocity);
        void HeadingJacobianAndResidual(const Eigen::Matrix3d &G_R_I,
                                        const double &heading_radius,
                                        double *residual_heading,
                                        Eigen::Matrix<double, 1, 15> *jacobian_heading);
        void CalculatedJacobHeading(const Eigen::Matrix3d &C_nb, Eigen::Matrix<double, 1, 3> &jacob);

        const Eigen::Vector3d I_p_Gps_;
        const double k_heading_cov_;
    };
} // namespace integrated_navigation
#endif //SRC_GNSS_CORRECTOR_HPP