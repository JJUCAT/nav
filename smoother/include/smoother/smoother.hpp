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

#ifndef SMAC_PLANNER__SMOOTHER_HPP_
#define SMAC_PLANNER__SMOOTHER_HPP_

#include <cmath>
#include <vector>
#include <iostream>
#include <memory>
#include <queue>
#include <utility>
#include <smoother/smoother_cost_function.hpp>

#include "ceres/ceres.h"
#include "Eigen/Core"

namespace smoother
{


/**
 * @class smac_planner::Smoother
 * @brief A Conjugate Gradient 2D path smoother implementation
 */
class Smoother
{
public:
  /**
   * @brief A constructor for smac_planner::Smoother
   */
  Smoother() {}

  /**
   * @brief A destructor for smac_planner::Smoother
   */
  ~Smoother() {}

  /**
   * @brief Initialization of the smoother
   * @param params OptimizerParam struct
   */
  void initialize()
  {
    _debug = true;

    // General Params

    // 2 most valid options: STEEPEST_DESCENT, NONLINEAR_CONJUGATE_GRADIENT
    _options.line_search_direction_type = ceres::NONLINEAR_CONJUGATE_GRADIENT;
    _options.line_search_type = ceres::WOLFE;
    _options.nonlinear_conjugate_gradient_type = ceres::POLAK_RIBIERE;
    _options.line_search_interpolation_type = ceres::CUBIC;

    _options.max_num_iterations = 100000;
    _options.max_solver_time_in_seconds = 3.0;

    _options.function_tolerance = 1e-6;
    _options.gradient_tolerance = 1e-10;
    _options.parameter_tolerance = 1e-8;

    _options.min_line_search_step_size = 1e-9;
    _options.max_num_line_search_step_size_iterations = 20;
    _options.line_search_sufficient_function_decrease = 1e-4;
    _options.max_line_search_step_contraction = 1e-3;
    _options.min_line_search_step_contraction = 0.6;
    _options.max_num_line_search_direction_restarts = 20;
    _options.line_search_sufficient_curvature_decrease = 0.9;
    _options.max_line_search_step_expansion = 10;

    if (_debug) {
      _options.minimizer_progress_to_stdout = true;
    } else {
      _options.logging_type = ceres::SILENT;
    }
  }

  /**
   * @brief Smoother method
   * @param path Reference to path
   * @param costmap Pointer to minimal costmap
   * @param smoother parameters weights
   * @return If smoothing was successful
   */
  bool smooth(
    std::vector<Eigen::Vector2d> & path,
    const SmootherParams & params)
  {
    _options.max_solver_time_in_seconds = params.max_time;

#ifdef _MSC_VER
    std::vector<double> parameters_vec(path.size() * 2);
    double * parameters = parameters_vec.data();
#else
    double parameters[path.size() * 2];  // NOLINT
#endif
    for (unsigned int i = 0; i != path.size(); i++) {
      parameters[2 * i] = path[i][0];
      parameters[2 * i + 1] = path[i][1];
    }

    ceres::GradientProblemSolver::Summary summary;
    ceres::GradientProblem problem(new UnconstrainedSmootherCostFunction(&path, params));
    ceres::Solve(_options, problem, parameters, &summary);

    if (_debug) {
      std::cout << summary.FullReport() << '\n';
    }

    if (!summary.IsSolutionUsable() || summary.initial_cost - summary.final_cost <= 0.0) {
      return false;
    }

    for (unsigned int i = 0; i != path.size(); i++) {
      path[i][0] = parameters[2 * i];
      path[i][1] = parameters[2 * i + 1];
    }

    return true;
  }

private:
  bool _debug;
  ceres::GradientProblemSolver::Options _options;
};

}  // namespace smac_planner

#endif  // SMAC_PLANNER__SMOOTHER_HPP_
