#ifndef FILTER_CORRECTOR_COMMON_H
#define FILTER_CORRECTOR_COMMON_H

#include "integrated_navigation/filtering/filter_defination.h"
#include "integrated_navigation/utils/utils.h"

namespace integrated_navigation {
    class FilterCorrectCommon{
        public:
        void UpdateFilterStateEstimation(const Eigen::MatrixXd &V, const Eigen::MatrixXd &H, const Eigen::VectorXd &residual, State *state);
        void UpdateFilterStateEstimation(const double &V, const Eigen::Matrix<double, 1, 15> &H, const double &residual, State *state);
        void AddDeltaToState(const Eigen::Matrix<double, 15, 1> &delta_x, State *state);
        void BiasLimitArrange(Eigen::Vector3d *bias, const double limited_value);
    };
}  // namespace integrated_navigation
#endif