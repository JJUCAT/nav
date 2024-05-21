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

#ifndef SYSTEMS_SIMPLE_CAR_DELAY_H_
#define SYSTEMS_SIMPLE_CAR_DELAY_H_

#include <fg100_mpc_local_planner/systems/base_robot_se2.h>
#include <fg100_mpc_local_planner/systems/simple_car.h>

#include <cmath>

namespace fg100_mpc_local_planner {

/**
 * @brief Simple car model
 *
 * This class implements the dynamics for a simple car model
 * in which the rear wheels are actuated.
 * The front wheels are the steering wheels (for wheelbase > 0).
 * The state vector [x, y, theta] is defined at the center of the rear axle.
 * See [1,2] for a mathematical description and a figure.
 *
 * [1] S. M. LaValle, Planning Algorithms, Cambridge University Press, 2006.
 *     (Chapter 13, http://planning.cs.uiuc.edu/)
 * [2] A. De Luca et al., Feedback Control of a Nonholonomic Car-like Robot,
 *     in Robot Motion Planning and Control (Ed. J.-P. Laumond), Springer, 1998.
 *     (https://homepages.laas.fr/jpl/promotion/chap4.pdf)
 *
 * @see SimpleCarFrontWheelDrivingModel KinematicBicycleModelVelocityInput
 *      BaseRobotSE2 RobotDynamicsInterface corbo::SystemDynamicsInterface
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class SimpleCarDelayModel : public SimpleCarModel
{
 public:
    //! Default constructor
    SimpleCarDelayModel() = default;

    //! Constructs model with given wheelbase
    SimpleCarDelayModel(double wheelbase, double T_1, double T_2) : _wheelbase(wheelbase) , _T_1(T_1), _T_2(T_2)  {}

    // implements interface method
    SystemDynamicsInterface::Ptr getInstance() const override { return std::make_shared<SimpleCarDelayModel>(); }

    // implements interface method

    int getStateDimension() const override { return 5; }

    // implements interface method
    void dynamics(const Eigen::Ref<const StateVector>& x, const Eigen::Ref<const ControlVector>& u, Eigen::Ref<StateVector> f) const override
    {
        assert(x.size() == getStateDimension());
        assert(u.size() == getInputDimension());
        assert(x.size() == f.size() && "SimpleCarDelayModel::dynamics(): x and f are not of the same size, do not forget to pre-allocate f.");
        //x[x,y,theta,angle_actual,v_actual] (pose), u[v, angle] (cmd_vel), f[vx,vy,v_theta,v_steer_angle,v_car]
        f[0] = x[4] * std::cos(x[2]);
        f[1] = x[4] * std::sin(x[2]);
        f[2] = x[4] * std::tan(x[3]) / _wheelbase;
        f[3] = (u[1] - x[3]) / _T_1;
        f[4] = (u[0] - x[4]) / _T_2;
    }


 protected:
    double _wheelbase = 1.0;
    double _T_1 = 0.8;
    double _T_2 = 0.2;
};
}  // namespace mpc_local_planner

#endif  // SYSTEMS_SIMPLE_CAR_DELAY_H_
