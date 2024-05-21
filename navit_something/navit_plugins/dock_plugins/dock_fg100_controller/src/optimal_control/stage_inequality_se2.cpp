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

#include <dock_fg100_controller/optimal_control/full_discretization_grid_base_se2.h>
#include <dock_fg100_controller/optimal_control/stage_inequality_se2.h>
#include <dock_fg100_controller/utils/pose_se2.h>

#include <cmath>
#include <memory>

namespace dock_fg100_controller {

int StageInequalitySE2::getNonIntegralControlDeviationTermDimension(
    int k) const {
  return _num_du_lb_finite + _num_du_ub_finite;
}

void StageInequalitySE2::computeNonIntegralControlDeviationTerm(
    int k, const Eigen::Ref<const Eigen::VectorXd> &u_k,
    const Eigen::Ref<const Eigen::VectorXd> &u_prev, double dt_prev,
    Eigen::Ref<Eigen::VectorXd> cost) const {
  assert(cost.size() == _num_du_lb_finite + _num_du_ub_finite);
  if (cost.size() == 0)
    return; // TODO(roesmann): necessary?
  if (k == 0 && dt_prev == 0) {
    cost.setZero(); // this is fine as dt_prev for k==0 is not subject to
                    // optimization
    return;
  }

  assert(_num_du_lb_finite == 0 || _du_lb.size() == u_k.size());
  assert(_num_du_ub_finite == 0 || _du_ub.size() == u_k.size());
  assert(dt_prev != 0);

  int idx_lb = 0;
  int idx_ub = 0;
  for (int i = 0; i < u_k.size(); ++i) {
    if (_du_lb[i] > -corbo::CORBO_INF_DBL) {
      cost[idx_lb] = _du_lb[i] - (u_k[i] - u_prev[i]) / dt_prev;
      ++idx_lb;
    }
    if (_du_ub[i] < corbo::CORBO_INF_DBL) {
      cost[_num_du_lb_finite + idx_ub] =
          (u_k[i] - u_prev[i]) / dt_prev - _du_ub[i];
      ++idx_ub;
    }
  }
}

void StageInequalitySE2::setControlDeviationBounds(
    const Eigen::VectorXd &du_lb, const Eigen::VectorXd &du_ub) {
  _du_lb = du_lb;
  _du_ub = du_ub;
}
}
