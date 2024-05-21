/*********************************************************************
 *
 *  Software License Agreement
 *
 *  Copyright (c) 2020, Christoph Rösmann, All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *  Authors: Christoph Rösmann
 *********************************************************************/

#ifndef STAGE_INEQUALITY_SE2_H_
#define STAGE_INEQUALITY_SE2_H_

#include <corbo-core/reference_trajectory.h>
#include <corbo-optimal-control/functions/stage_functions.h>

#include <cmath>
#include <memory>

namespace dock_fg100_controller {

/**
 * @brief Stage inequality constraint for obstacle avoidance and control
 * deviation limits
 *
 * This class defines the inequality constraint for obstacle avoidance and
 * control
 * deviation limits (in case limits are active).
 *
 * The current obstacle association strategy is borrowed from the
 * teb_local_planner.
 * It takes the robot footprint model and the geometric obstacle shapes into
 * account.
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class StageInequalitySE2 : public corbo::StageInequalityConstraint {
public:
  using Ptr = std::shared_ptr<StageInequalitySE2>;

  // implements interface method
  corbo::StageInequalityConstraint::Ptr getInstance() const override {
    return std::make_shared<StageInequalitySE2>();
  }

  // implements interface method
  bool hasNonIntegralTerms(int k) const override { return true; }
  // implements interface method
  bool hasIntegralTerms(int k) const override { return false; }

  // implements interface method
  int getNonIntegralControlDeviationTermDimension(int k) const override;

  // implements interface method
  void computeNonIntegralControlDeviationTerm(
      int k, const Eigen::Ref<const Eigen::VectorXd> &u_k,
      const Eigen::Ref<const Eigen::VectorXd> &u_prev, double dt,
      Eigen::Ref<Eigen::VectorXd> cost) const override;

  //! Set to true to enable dynamic obstacle (constant-velocity prediction)
  void setControlDeviationBounds(const Eigen::VectorXd &du_lb,
                                 const Eigen::VectorXd &du_ub);

protected:
  double _current_dt = 0.1;
  int _num_du_lb_finite = 0;
  int _num_du_ub_finite = 0;

  Eigen::VectorXd _du_lb;
  Eigen::VectorXd _du_ub;
};
}

#endif // STAGE_INEQUALITY_SE2_H_
