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

#include <fg100_mpc_local_planner/optimal_control/final_state_conditions_se2.h>

#include <fg100_mpc_local_planner/utils/math_utils.h>

#include <cmath>

namespace fg100_mpc_local_planner {

void QuadraticFinalStateCostSE2::computeNonIntegralStateTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k,
                                                             Eigen::Ref<Eigen::VectorXd> cost) const
{
    assert(cost.size() == getNonIntegralStateTermDimension(k));

    Eigen::VectorXd xd = x_k - _x_ref->getReferenceCached(k);
    xd[2]              = normalize_theta(xd[2]);
    if (_lsq_form)
    {
        if (_diagonal_mode)
            cost.noalias() = _Qf_diag_sqrt * xd;
        else
            cost.noalias() = _Qf_sqrt * xd;
    }
    else
    {
        if (_diagonal_mode)
            cost.noalias() = xd.transpose() * _Qf_diag * xd;
        else
            cost.noalias() = xd.transpose() * _Qf * xd;
    }
}

void TerminalBallSE2::computeNonIntegralStateTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, Eigen::Ref<Eigen::VectorXd> cost) const
{
    assert(cost.size() == getNonIntegralStateTermDimension(k));

    Eigen::VectorXd xd = x_k - _x_ref->getReferenceCached(k);
    xd[2]              = normalize_theta(xd[2]);
    if (_diagonal_mode)
        cost[0] = xd.transpose() * _S_diag * xd - _gamma;
    else
        cost[0] = xd.transpose() * _S * xd - _gamma;
}

}  // namespace mpc_local_planner
