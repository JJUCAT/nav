#ifndef SRC_FILTER_INITIALIZER_HPP
#define SRC_FILTER_INITIALIZER_HPP

#include "integrated_navigation/filtering/filter_defination.h"
#include "integrated_navigation/utils/utils.h"

namespace integrated_navigation {
    class FilterInitializer {
    public:
        FilterInitializer(const int kImuDataBufferLength, const double kAccStdLimit);

        void AddImuData(const ImuDataPtr imu_data_ptr);
        void ClearImuBuffer();
        bool AddNovatelMeasurements(const Eigen::Vector3d init_lla, const GnssPositionDataPtr gnss_data_ptr, State *state);

    private:
        bool ComputeG_R_IFromImuData(const GnssPositionDataPtr gnss_data_ptr, Eigen::Matrix3d *G_R_I);

        const unsigned int kImuDataBufferLength_;
        const double kAccStdLimit_;

        std::deque <ImuDataPtr> imu_buffer_;
    };
} // namespace integrated_navigation
#endif //SRC_FILTER_INITIALIZER_HPP