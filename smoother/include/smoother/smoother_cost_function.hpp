// Copyright (c) 2020, Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#ifndef SMAC_PLANNER__SMOOTHER_COST_FUNCTION_HPP_
#define SMAC_PLANNER__SMOOTHER_COST_FUNCTION_HPP_

#include <cmath>
#include <vector>
#include <iostream>
#include <unordered_map>
#include <memory>
#include <queue>
#include <utility>

#include "ceres/ceres.h"
#include "Eigen/Core"

#define EPSILON 0.0001

namespace smoother
{


struct SmootherParams
{
  double max_time = 3.0;
  double max_curvature = 1.0;
  double smooth_weight = 1000;
  double curvature_weight = 30.0;
  double distance_weight = 2000;
};


/**
 * @struct smac_planner::UnconstrainedSmootherCostFunction
 * @brief Cost function for path smoothing with multiple terms
 * including curvature, smoothness, collision, and avoid obstacles.
 */
class UnconstrainedSmootherCostFunction : public ceres::FirstOrderFunction
{
public:
  /**
   * @brief A constructor for smac_planner::UnconstrainedSmootherCostFunction
   * @param original_path Original unsmoothed path to smooth
   * @param costmap A costmap to get values for collision and obstacle avoidance
   */
  UnconstrainedSmootherCostFunction(
    std::vector<Eigen::Vector2d> * original_path,
    const SmootherParams & params,
    const bool use_new_curvature_jacobian)
  : _original_path(original_path),
    _num_params(2 * original_path->size()),
    _params(params),
    use_new_curvature_jacobian_(use_new_curvature_jacobian)
  {
  }

  /**
   * @struct CurvatureComputations
   * @brief Cache common computations between the curvature terms to minimize recomputations\
   * 缓存曲率项之间的公共计算，以最小化重新计算
   */
  struct CurvatureComputations
  {
    /**
     * @brief A constructor for smac_planner::CurvatureComputations
     */
    CurvatureComputations()
    {
      valid = true;
    }

    bool valid;
    /**
     * @brief Check if result is valid for penalty
     * @return is valid (non-nan, non-inf, and turning angle > max)
     */
    bool isValid()
    {
      return valid;
    }

    Eigen::Vector2d delta_xi{0.0, 0.0};
    Eigen::Vector2d delta_xi_p{0.0, 0.0};
    double delta_xi_norm{0};
    double delta_xi_p_norm{0};
    double delta_phi_i{0};
    double turning_rad{0};
    double ki_minus_kmax{0};
  };

  /**
   * @brief Smoother cost function evaluation，代价函数
   * @param parameters X,Y pairs of points
   * @param cost total cost of path
   * @param gradient of path at each X,Y pair from cost function derived analytically
   * @return if successful in computing values
   */
  virtual bool Evaluate(
    const double * parameters,
    double * cost,
    double * gradient) const
  {
    Eigen::Vector2d xi;
    Eigen::Vector2d xi_p1;
    Eigen::Vector2d xi_m1;
    unsigned int x_index, y_index;
    cost[0] = 0.0;
    double cost_raw = 0.0;
    double grad_x_raw = 0.0;
    double grad_y_raw = 0.0;
    unsigned int mx, my;
    bool valid_coords = true;
    double costmap_cost = 0.0;

    // cache some computations between the residual and jacobian
    CurvatureComputations curvature_params;

    for (int i = 0; i != NumParameters() / 2; i++) {
      x_index = 2 * i;
      y_index = 2 * i + 1;
      gradient[x_index] = 0.0;
      gradient[y_index] = 0.0;
      if (i < 1 || i >= NumParameters() / 2 - 1) { // 跳过第一个路径点后最后一个路径点
        continue;
      }

      xi = Eigen::Vector2d(parameters[x_index], parameters[y_index]);
      xi_p1 = Eigen::Vector2d(parameters[x_index + 2], parameters[y_index + 2]); // 后一个点
      xi_m1 = Eigen::Vector2d(parameters[x_index - 2], parameters[y_index - 2]); // 前一个点

      // compute cost
      // addSmoothingResidual(_params.smooth_weight, xi, xi_p1, xi_m1, cost_raw); // 路径平滑度残差
      addCurvatureResidual(_params.curvature_weight, xi, xi_p1, xi_m1, curvature_params, cost_raw); // 曲率残差
      // addDistanceResidual(_params.distance_weight, xi, _original_path->at(i), cost_raw); // 与原路径点距离残差

      if (gradient != NULL) {
        // compute gradient
        gradient[x_index] = 0.0;
        gradient[y_index] = 0.0;
        // addSmoothingJacobian(_params.smooth_weight, xi, xi_p1, xi_m1, grad_x_raw, grad_y_raw);
        if (!use_new_curvature_jacobian_) {
          addCurvatureJacobian(_params.curvature_weight, xi, xi_p1, xi_m1, curvature_params, grad_x_raw, grad_y_raw);
        } else {
          addCurvatureJacobianNew(_params.curvature_weight, xi, xi_p1, xi_m1, curvature_params, grad_x_raw, grad_y_raw);
        }          
        // addDistanceJacobian(_params.distance_weight, xi, _original_path->at(i), grad_x_raw, grad_y_raw);

        gradient[x_index] = grad_x_raw;
        gradient[y_index] = grad_y_raw;
      }
    }

    cost[0] = cost_raw;

    return true;
  }

  /**
   * @brief Get number of parameter blocks
   * @return Number of parameters in cost function
   */
  virtual int NumParameters() const {return _num_params;}

protected:
  /**
   * @brief Cost function term for smooth paths      // 平滑度的成本函数
   * @param weight Weight to apply to function
   * @param pt Point Xi for evaluation
   * @param pt Point Xi+1 for calculating Xi's cost
   * @param pt Point Xi-1 for calculating Xi's cost
   * @param r Residual (cost) of term
   */
  inline void addSmoothingResidual(
    const double & weight,
    const Eigen::Vector2d & pt,
    const Eigen::Vector2d & pt_p,
    const Eigen::Vector2d & pt_m,
    double & r) const
  {
    // a·b，点乘表示 a 在 b 上面投影长度乘以 b 的长度
    // 点乘表示两个向量在方向上的相似度，越大约相近
    r += weight * (
      pt_p.dot(pt_p) -
      4 * pt_p.dot(pt) +
      2 * pt_p.dot(pt_m) +
      4 * pt.dot(pt) -
      4 * pt.dot(pt_m) +
      pt_m.dot(pt_m));    // objective function value
  }

  /**
   * @brief Cost function derivative term for smooth paths
   * @param weight Weight to apply to function
   * @param pt Point Xi for evaluation
   * @param pt Point Xi+1 for calculating Xi's cost
   * @param pt Point Xi-1 for calculating Xi's cost
   * @param j0 Gradient of X term
   * @param j1 Gradient of Y term
   */
  inline void addSmoothingJacobian(
    const double & weight,
    const Eigen::Vector2d & pt,
    const Eigen::Vector2d & pt_p,
    const Eigen::Vector2d & pt_m,
    double & j0,
    double & j1) const
  {
    j0 += weight *
      (-4 * pt_m[0] + 8 * pt[0] - 4 * pt_p[0]);   // xi x component of partial-derivative
    j1 += weight *
      (-4 * pt_m[1] + 8 * pt[1] - 4 * pt_p[1]);   // xi y component of partial-derivative
  }

  /**
   * @brief Cost function term for maximum curved paths
   * @param weight Weight to apply to function
   * @param pt Point Xi for evaluation
   * @param pt Point Xi+1 for calculating Xi's cost
   * @param pt Point Xi-1 for calculating Xi's cost
   * @param curvature_params A struct to cache computations for the jacobian to use
   * @param r Residual (cost) of term
   */
  inline void addCurvatureResidual(
    const double & weight,
    const Eigen::Vector2d & pt,
    const Eigen::Vector2d & pt_p,
    const Eigen::Vector2d & pt_m,
    CurvatureComputations & curvature_params,
    double & r) const
  {
    curvature_params.valid = true;
    curvature_params.delta_xi = Eigen::Vector2d(pt[0] - pt_m[0], pt[1] - pt_m[1]);
    curvature_params.delta_xi_p = Eigen::Vector2d(pt_p[0] - pt[0], pt_p[1] - pt[1]);
    curvature_params.delta_xi_norm = curvature_params.delta_xi.norm();
    curvature_params.delta_xi_p_norm = curvature_params.delta_xi_p.norm();

    if (curvature_params.delta_xi_norm < EPSILON || curvature_params.delta_xi_p_norm < EPSILON ||
      std::isnan(curvature_params.delta_xi_p_norm) || std::isnan(curvature_params.delta_xi_norm) ||
      std::isinf(curvature_params.delta_xi_p_norm) || std::isinf(curvature_params.delta_xi_norm))
    {
      // ensure we have non-nan values returned
      curvature_params.valid = false;
      return;
    }

    const double & delta_xi_by_xi_p =
      curvature_params.delta_xi_norm * curvature_params.delta_xi_p_norm;
    double projection =
      curvature_params.delta_xi.dot(curvature_params.delta_xi_p) / delta_xi_by_xi_p;
    if (fabs(1 - projection) < EPSILON || fabs(projection + 1) < EPSILON) {
      projection = 1.0;
    }

    curvature_params.delta_phi_i = std::acos(projection);
    curvature_params.turning_rad = curvature_params.delta_phi_i / curvature_params.delta_xi_norm;

    curvature_params.ki_minus_kmax = curvature_params.turning_rad - _params.max_curvature;

    if (curvature_params.ki_minus_kmax <= EPSILON) {
      // Quadratic penalty need not apply
      curvature_params.valid = false;
      return;
    }

    r += weight *
      curvature_params.ki_minus_kmax * curvature_params.ki_minus_kmax;  // objective function value
  }

  /**
   * @brief Cost function derivative term for maximum curvature paths
   * @param weight Weight to apply to function
   * @param pt Point Xi for evaluation
   * @param pt Point Xi+1 for calculating Xi's cost
   * @param pt Point Xi-1 for calculating Xi's cost
   * @param curvature_params A struct with cached values to speed up Jacobian computation
   * @param j0 Gradient of X term
   * @param j1 Gradient of Y term
   */
  inline void addCurvatureJacobian(
    const double & weight,
    const Eigen::Vector2d & pt,
    const Eigen::Vector2d & pt_p,
    const Eigen::Vector2d & /*pt_m*/,
    CurvatureComputations & curvature_params,
    double & j0,
    double & j1) const
  {
    if (!curvature_params.isValid()) {
      return;
    }

    const double & partial_delta_phi_i_wrt_cost_delta_phi_i =
      -1 / std::sqrt(1 - std::pow(std::cos(curvature_params.delta_phi_i), 2));
    // const Eigen::Vector2d ones = Eigen::Vector2d(1.0, 1.0);
    auto neg_pt_plus = -1 * pt_p;
    Eigen::Vector2d p1 = normalizedOrthogonalComplement(
      pt, neg_pt_plus, curvature_params.delta_xi_norm, curvature_params.delta_xi_p_norm);
    Eigen::Vector2d p2 = normalizedOrthogonalComplement(
      neg_pt_plus, pt, curvature_params.delta_xi_p_norm, curvature_params.delta_xi_norm);

    const double & u = 2 * curvature_params.ki_minus_kmax;
    const double & common_prefix =
      (1 / curvature_params.delta_xi_norm) * partial_delta_phi_i_wrt_cost_delta_phi_i;
    const double & common_suffix = curvature_params.delta_phi_i /
      (curvature_params.delta_xi_norm * curvature_params.delta_xi_norm);

    const Eigen::Vector2d & d_delta_xi_d_xi = curvature_params.delta_xi /
      curvature_params.delta_xi_norm;

    const Eigen::Vector2d jacobian = u *
      (common_prefix * (-p1 - p2) - (common_suffix * d_delta_xi_d_xi));
    const Eigen::Vector2d jacobian_im1 = u *
      (common_prefix * p2 + (common_suffix * d_delta_xi_d_xi));
    const Eigen::Vector2d jacobian_ip1 = u * (common_prefix * p1);

    // Old formulation we may require again.
    // j0 += weight *
    //   (jacobian_im1[0] + 2 * jacobian[0] + jacobian_ip1[0]);
    // j1 += weight *
    //   (jacobian_im1[1] + 2 * jacobian[1] + jacobian_ip1[1]);

    j0 += weight * jacobian[0];  // xi x component of partial-derivative
    j1 += weight * jacobian[1];  // xi x component of partial-derivative
  }

  // 新方法计算曲率梯度
  inline void addCurvatureJacobianNew(
    const double & weight,
    const Eigen::Vector2d & pt,
    const Eigen::Vector2d & pt_p,
    const Eigen::Vector2d & /*pt_m*/,
    CurvatureComputations & curvature_params,
    double & j0,
    double & j1) const
  {
    if (!curvature_params.isValid()) {
      return;
    }

    const double & partial_delta_phi_i_wrt_cost_delta_phi_i =
      -1 / std::sqrt(1 - std::pow(std::cos(curvature_params.delta_phi_i), 2));

    // h(x) 函数的求导
    Eigen::Vector2d p3 = derivation_hx(curvature_params.delta_xi,
      curvature_params.delta_xi_p, curvature_params.delta_xi_norm, curvature_params.delta_xi_p_norm);

    const double & u = 2 * curvature_params.ki_minus_kmax;
    const double & common_prefix =
      (1 / curvature_params.delta_xi_norm) * partial_delta_phi_i_wrt_cost_delta_phi_i;
    const double & common_suffix = curvature_params.delta_phi_i /
      (curvature_params.delta_xi_norm * curvature_params.delta_xi_norm);

    const Eigen::Vector2d & d_delta_xi_d_xi = curvature_params.delta_xi /
      curvature_params.delta_xi_norm;

    const Eigen::Vector2d jacobian = u *
      ((common_prefix * p3) - (common_suffix * d_delta_xi_d_xi));
    j0 += weight * jacobian[0];  // xi x component of partial-derivative
    j1 += weight * jacobian[1];  // xi x component of partial-derivative
  }

  /**
   * @brief Cost function derivative term for steering away changes in pose
   * @param weight Weight to apply to function
   * @param xi Point Xi for evaluation
   * @param xi_original original point Xi for evaluation
   * @param r Residual (cost) of term
   */
  inline void addDistanceResidual(
    const double & weight,
    const Eigen::Vector2d & xi,
    const Eigen::Vector2d & xi_original,
    double & r) const
  {
    // 相当于 r += weight * (欧氏距离)
    r += weight * (xi - xi_original).dot(xi - xi_original);  // objective function value
  }

  /**
   * @brief Cost function derivative term for steering away changes in pose
   * @param weight Weight to apply to function
   * @param xi Point Xi for evaluation
   * @param xi_original original point Xi for evaluation
   * @param j0 Gradient of X term
   * @param j1 Gradient of Y term
   */
  inline void addDistanceJacobian(
    const double & weight,
    const Eigen::Vector2d & xi,
    const Eigen::Vector2d & xi_original,
    double & j0,
    double & j1) const
  {
    j0 += weight * 2 * (xi[0] - xi_original[0]);  // xi y component of partial-derivative
    j1 += weight * 2 * (xi[1] - xi_original[1]);  // xi y component of partial-derivative
  }

  /**
   * @brief Computing the normalized orthogonal component of 2 vectors
   * @param a Vector
   * @param b Vector
   * @param norm a Vector's norm
   * @param norm b Vector's norm
   * @return Normalized vector of orthogonal components
   */
  inline Eigen::Vector2d normalizedOrthogonalComplement(
    const Eigen::Vector2d & a,
    const Eigen::Vector2d & b,
    const double & a_norm,
    const double & b_norm) const
  {
    return (a - (a.dot(b) * b / b.squaredNorm())) / (a_norm * b_norm);
  }

  /**
   * @brief  h(x) = a.dot(b)/(a_norm*b_norm), x=[x_pt, y_pt]
   * @param  a  向量 u
   * @param  b  向量 v
   * @param  a_norm  向量 u 的模
   * @param  b_norm  向量 v 的模
   * @return Eigen::Vector2d 
   */
  inline Eigen::Vector2d derivation_hx(
    const Eigen::Vector2d & a,
    const Eigen::Vector2d & b,
    const double & a_norm,
    const double & b_norm) const
  {
    Eigen::Vector2d prefix = (b-a)/(a_norm*b_norm);
    Eigen::Vector2d suffix = (a.dot(b)*(-(a*a.squaredNorm()+b*b.squaredNorm())))/(a_norm*b_norm);
    return prefix + suffix;
  }

  std::vector<Eigen::Vector2d> * _original_path{nullptr};
  int _num_params;
  SmootherParams _params;
  bool use_new_curvature_jacobian_{false};
};

}  // namespace smac_planner

#endif  // SMAC_PLANNER__SMOOTHER_COST_FUNCTION_HPP_
