#ifndef SRC_FILTER_CHECKER_H
#define SRC_FILTER_CHECKER_H

#include "integrated_navigation/filtering/filter_defination.h"
#include "integrated_navigation/utils/utils.h"

namespace integrated_navigation {
    class FilterChecker {
    public:
        FilterChecker(const Eigen::Vector3d I_p_Gps, const double gravity);
        ~FilterChecker();

        void AddImuData(const ImuDataPtr imu_data_ptr);
        bool CheckState(const ImuDataPtr imu_data_ptr);
        bool UpdateBiases(const ImuDataPtr imu_data_ptr, State *state, Eigen::Vector3d *position);
        bool ZeroCheckGyroBiasUpdate(State *state, Eigen::Vector3d *position);

    private:
        bool ZeroStatusCheck();
        bool ZeroCheckByAcc();
        bool ZeroCheckByGyro();

        const Eigen::Vector3d I_p_Gps_;
        const double gravity_;
        bool reset_;
        const double kAngularVelocityThreshold;
        const double kAccelerationThreshold;
        const double kDeltaAngularVelocityThreshold;
        double wx_prev_, wy_prev_, wz_prev_;
        double wx_bias_, wy_bias_, wz_bias_;
        double bias_alpha_;
    };
} // namespace integrated_navigation
#endif //SRC_FILTER_CHECKER_H