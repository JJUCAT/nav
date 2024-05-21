#ifndef SRC_FILTER_PREDICTOR_HPP
#define SRC_FILTER_PREDICTOR_HPP

#include "integrated_navigation/filtering/filter_defination.h"
#include "integrated_navigation/utils/utils.h"

namespace integrated_navigation {
    class FilterPredictor {
    public:
        FilterPredictor(const Eigen::Matrix3d acc_noise, const Eigen::Matrix3d gyro_noise,
                        const Eigen::Matrix3d acc_bias_noise, const Eigen::Matrix3d gyro_bias_noise,const Eigen::Vector3d gravity);

        void Predict(const ImuDataPtr last_imu, const ImuDataPtr cur_imu, State *state);

    private:
        void UpdateDynamicMatrix(const Eigen::Matrix3d &G_R_I,const Eigen::Vector3d &acc_unbias,
                                 const double &delta_t, const double &delta_t2,
                                 Eigen::Matrix<double, 15, 15> *Fx);

        const Eigen::Matrix3d acc_noise_;
        const Eigen::Matrix3d gyro_noise_;
        const Eigen::Matrix3d acc_bias_noise_;
        const Eigen::Matrix3d gyro_bias_noise_;

        Eigen::Vector3d gravity_;
    };
} // namespace integrated_navigation
#endif //SRC_FILTER_PREDICTOR_HPP