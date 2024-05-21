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

#include <dock_fg100_controller/controller.h>

#include <corbo-optimal-control/functions/hybrid_cost.h>
#include <corbo-optimal-control/functions/minimum_time.h>
#include <corbo-optimal-control/functions/quadratic_control_cost.h>
#include <corbo-optimal-control/structured_ocp/discretization_grids/finite_differences_variable_grid.h>
#include <corbo-optimal-control/structured_ocp/structured_optimal_control_problem.h>
#include <corbo-optimization/hyper_graph/hyper_graph_optimization_problem_edge_based.h>
#include <corbo-optimization/solver/nlp_solver_ipopt.h>

#include <dock_fg100_controller/optimal_control/fd_collocation_se2.h>
#include <dock_fg100_controller/optimal_control/final_state_conditions_se2.h>
#include <dock_fg100_controller/optimal_control/finite_differences_variable_grid_se2.h>
#include <dock_fg100_controller/optimal_control/quadratic_cost_se2.h>
#include <dock_fg100_controller/optimal_control/stage_inequality_se2.h>
#include <dock_fg100_controller/systems/simple_car.h>
#include <dock_fg100_controller/systems/unicycle_robot.h>
#include <dock_fg100_controller/utils/time_series_se2.h>

#include <navit_msgs/OptimalControlResult.h>
#include <navit_msgs/StateFeedback.h>

#include <corbo-communication/utilities.h>
#include <corbo-core/console.h>

#include <tf2/utils.h>

#include <memory>
#include <mutex>

namespace dock_fg100_controller {

bool Controller::configure(ros::NodeHandle &nh) {
  _dynamics = configureRobotDynamics(nh);
  if (!_dynamics)
    return false; // we may need state and control dimensions to check other
                  // parameters

  _grid = configureGrid(nh);
  _solver = configureSolver(nh);

  _structured_ocp = configureOcp(nh);
  _ocp = _structured_ocp; // copy pointer also to parent member

  int outer_ocp_iterations = 1;
  nh.param("controller/outer_ocp_iterations", outer_ocp_iterations,
           outer_ocp_iterations);
  setNumOcpIterations(outer_ocp_iterations);

  // custom feedback:
  nh.param("controller/prefer_x_feedback", _prefer_x_feedback,
           _prefer_x_feedback);

  std::string state_feedback_topic_name = "state_feedback";
  ros::NodeHandle global_nh;
  nh.param("controller/state_feedback_topic_name", state_feedback_topic_name,
           state_feedback_topic_name);
  _x_feedback_sub = global_nh.subscribe(
      state_feedback_topic_name, 1, &Controller::stateFeedbackCallback, this);

  // result publisher:
  _ocp_result_pub =
      nh.advertise<navit_msgs::OptimalControlResult>("ocp_result", 100);
  nh.param("controller/publish_ocp_results", _publish_ocp_results,
           _publish_ocp_results);
  nh.param("controller/print_cpu_time", _print_cpu_time, _print_cpu_time);

  setAutoUpdatePreviousControl(
      false); // we want to update the previous control value manually

  if (_ocp->initialize())
    ROS_INFO("OCP initialized.");
  else {
    ROS_ERROR("OCP initialization failed");
    return false;
  }
  return _grid && _dynamics && _solver && _structured_ocp;
}

bool Controller::step(const PoseSE2 &start, const PoseSE2 &goal,
                      const geometry_msgs::Twist &vel, double dt, ros::Time t,
                      corbo::TimeSeries::Ptr u_seq,
                      corbo::TimeSeries::Ptr x_seq) {
  std::vector<geometry_msgs::PoseStamped> initial_plan(2);
  start.toPoseMsg(initial_plan.front().pose);
  goal.toPoseMsg(initial_plan.back().pose);
  return step(initial_plan, vel, dt, t, u_seq, x_seq);
}

bool Controller::step(
    const std::vector<geometry_msgs::PoseStamped> &initial_plan,
    const geometry_msgs::Twist &vel, double dt, ros::Time t,
    corbo::TimeSeries::Ptr u_seq, corbo::TimeSeries::Ptr x_seq) {
  if (!_dynamics || !_grid || !_structured_ocp) {
    ROS_ERROR("Controller must be configured before invoking step().");
    return false;
  }
  if (initial_plan.size() < 2) {
    ROS_ERROR(
        "Controller::step(): initial plan must contain at least two poses.");
    return false;
  }

  PoseSE2 start(initial_plan.front().pose);
  PoseSE2 goal(initial_plan.back().pose);

  Eigen::VectorXd xf(_dynamics->getStateDimension());
  _dynamics->getSteadyStateFromPoseSE2(goal, xf);

  // retrieve or estimate current state
  Eigen::VectorXd x(_dynamics->getStateDimension());
  // check for new measurements
  bool new_x = false;
  {
    std::lock_guard<std::mutex> lock(_x_feedback_mutex);
    new_x = _recent_x_feedback.size() > 0 &&
            (t - _recent_x_time).toSec() < 2.0 * dt;
    if (new_x)
      x = _recent_x_feedback;
  }
  if (!new_x && (!_x_ts || _x_ts->isEmpty() ||
                 !_x_ts->getValuesInterpolate(
                     dt, x))) // predict with previous state sequence
  {
    _dynamics->getSteadyStateFromPoseSE2(
        start, x); // otherwise initialize steady state
  }
  if (!new_x || !_prefer_x_feedback) {
    // Merge state feedback with odometry feedback if desired.
    // Note, some models like unicycle overwrite the full state by odom feedback
    // unless _prefer_x_measurement is set to true.
    _dynamics->mergeStateFeedbackAndOdomFeedback(start, vel, x);
  }

  if (_grid->isEmpty()) {
    // generate custom initialization based on initial_plan
    // check if the goal is behind the start pose (w.r.t. start orientation)
    bool backward =
        (goal.position() - start.position()).dot(start.orientationUnitVec()) <
        0;
    generateInitialStateTrajectory(x, xf, initial_plan, backward);
  }
  corbo::Time time(t.toSec());
  _x_seq_init.setTimeFromStart(time);

  corbo::StaticReference xref(
      xf); // currently, we only support point-to-point transitions in ros
  corbo::ZeroReference uref(_dynamics->getInputDimension());

  _ocp_successful =
      PredictiveController::step(x, xref, uref, corbo::Duration(dt), time,
                                 u_seq, x_seq, nullptr, nullptr, &_x_seq_init);
  // publish results if desired
  if (_publish_ocp_results)
    publishOptimalControlResult(); // TODO(roesmann): we could also pass time t
                                   // from above
  ROS_INFO_STREAM_COND(_print_cpu_time,
                       "Cpu time: " << _statistics.step_time.toSec() * 1000.0
                                    << " ms.");
  ++_ocp_seq;
  _last_goal = goal;
  return _ocp_successful;
}

void Controller::stateFeedbackCallback(
    const navit_msgs::StateFeedback::ConstPtr &msg) {
  if (!_dynamics)
    return;

  if ((int)msg->state.size() != _dynamics->getStateDimension()) {
    ROS_ERROR_STREAM("stateFeedbackCallback(): state feedback dimension does "
                     "not match robot state dimension: "
                     << msg->state.size()
                     << " != " << _dynamics->getStateDimension());
    return;
  }

  std::lock_guard<std::mutex> lock(_x_feedback_mutex);
  _recent_x_time = msg->header.stamp;
  _recent_x_feedback = Eigen::Map<const Eigen::VectorXd>(
      msg->state.data(), (int)msg->state.size());
}

void Controller::publishOptimalControlResult() {
  if (!_dynamics)
    return;
  navit_msgs::OptimalControlResult msg;
  msg.header.stamp = ros::Time::now();
  msg.header.seq = static_cast<unsigned int>(_ocp_seq);
  msg.dim_states = _dynamics->getStateDimension();
  msg.dim_controls = _dynamics->getInputDimension();
  msg.optimal_solution_found = _ocp_successful;
  msg.cpu_time = _statistics.step_time.toSec();

  if (_x_ts && !_x_ts->isEmpty()) {
    msg.time_states = _x_ts->getTime();
    msg.states = _x_ts->getValues();
  }

  if (_u_ts && !_u_ts->isEmpty()) {
    msg.time_controls = _u_ts->getTime();
    msg.controls = _u_ts->getValues();
  }

  _ocp_result_pub.publish(msg);
}

void Controller::reset() { PredictiveController::reset(); }

corbo::DiscretizationGridInterface::Ptr
Controller::configureGrid(const ros::NodeHandle &nh) {
  if (!_dynamics)
    return {};

  FiniteDifferencesGridSE2::Ptr grid;

  grid = std::make_shared<FiniteDifferencesGridSE2>();

  // common grid parameters
  int grid_size_ref = 60;
  nh.param("grid/grid_size_ref", grid_size_ref, grid_size_ref);
  ROS_DEBUG("MPC controller grid size ref is %d", grid_size_ref);
  grid->setNRef(grid_size_ref);

  double dt_ref = 0.3;
  nh.param("grid/dt_ref", dt_ref, dt_ref);
  grid->setDtRef(dt_ref);

  std::vector<bool> xf_fixed = {true, true, true};
  if (_robot_type == "fg_robot")
    std::vector<bool> xf_fixed = {true, true, true, true, true};

  nh.param("grid/xf_fixed", xf_fixed, xf_fixed);
  if ((int)xf_fixed.size() != _dynamics->getStateDimension()) {
    ROS_ERROR_STREAM(
        "Array size of `xf_fixed` does not match robot state dimension(): "
        << xf_fixed.size() << " != " << _dynamics->getStateDimension());
    return {};
  }
  Eigen::Matrix<bool, -1, 1> xf_fixed_eigen(
      xf_fixed.size()); // we cannot use Eigen::Map as vector<bool> does not
                        // provide raw access
  for (int i = 0; i < (int)xf_fixed.size(); ++i)
    xf_fixed_eigen[i] = xf_fixed[i];
  grid->setXfFixed(xf_fixed_eigen);

  bool warm_start = true;
  nh.param("grid/warm_start", warm_start, warm_start);
  grid->setWarmStart(warm_start);

  std::string collocation_method = "forward_differences";
  nh.param("grid/collocation_method", collocation_method, collocation_method);

  if (collocation_method == "forward_differences") {
    grid->setFiniteDifferencesCollocationMethod(
        std::make_shared<ForwardDiffCollocationSE2>());
  } else if (collocation_method == "midpoint_differences") {
    grid->setFiniteDifferencesCollocationMethod(
        std::make_shared<MidpointDiffCollocationSE2>());
  } else if (collocation_method == "crank_nicolson_differences") {
    grid->setFiniteDifferencesCollocationMethod(
        std::make_shared<CrankNicolsonDiffCollocationSE2>());
  } else {
    ROS_ERROR_STREAM("Unknown collocation method '"
                     << collocation_method
                     << "' specified. Falling back to default...");
  }

  std::string cost_integration_method = "left_sum";
  nh.param("grid/cost_integration_method", cost_integration_method,
           cost_integration_method);

  if (cost_integration_method == "left_sum") {
    grid->setCostIntegrationRule(
        FullDiscretizationGridBaseSE2::CostIntegrationRule::LeftSum);
  } else if (cost_integration_method == "trapezoidal_rule") {
    grid->setCostIntegrationRule(
        FullDiscretizationGridBaseSE2::CostIntegrationRule::TrapezoidalRule);
  } else {
    ROS_ERROR_STREAM("Unknown cost integration method '"
                     << cost_integration_method
                     << "' specified. Falling back to default...");
  }

  return std::move(grid);
}

RobotDynamicsInterface::Ptr
Controller::configureRobotDynamics(const ros::NodeHandle &nh) {
  _robot_type = "simple_car";
  nh.param("robot/type", _robot_type, _robot_type);
  if (_robot_type == "diff")
      return std::make_shared<UnicycleModel>();

  if (_robot_type == "simple_car") {
    double wheelbase = 0.4;
    nh.param("robot/simple_car/wheelbase", wheelbase, wheelbase);
    return std::make_shared<SimpleCarModel>(wheelbase);
  } else if (_robot_type == "fg_robot") {
    double wheelbase = 2.0;
    double tau_steer = 0.5;
    double tau_v = 0.1;
    nh.param("robot/fg_robot/wheelbase", wheelbase, wheelbase);
    nh.param("robot/fg_robot/tau_steer", tau_steer, tau_steer);
    nh.param("robot/fg_robot/tau_v", tau_v, tau_v);

    return std::make_shared<FgRobotModel>(wheelbase, tau_steer, tau_v);
  } else {
    ROS_ERROR_STREAM("Unknown robot type '" << _robot_type << "' specified.");
  }

  return {};
}

corbo::NlpSolverInterface::Ptr
Controller::configureSolver(const ros::NodeHandle &nh) {
  corbo::SolverIpopt::Ptr solver = std::make_shared<corbo::SolverIpopt>();
  solver->initialize(); // requried for setting parameters afterward

  int iterations = 100;
  nh.param("solver/ipopt/iterations", iterations, iterations);
  solver->setIterations(iterations);

  double max_cpu_time = -1.0;
  nh.param("solver/ipopt/max_cpu_time", max_cpu_time, max_cpu_time);
  solver->setMaxCpuTime(max_cpu_time);

  // now check for additional ipopt options
  std::map<std::string, double> numeric_options;
  nh.param("solver/ipopt/ipopt_numeric_options", numeric_options,
           numeric_options);
  for (const auto &item : numeric_options) {
    if (!solver->setIpoptOptionNumeric(item.first, item.second))
      ROS_WARN_STREAM("Ipopt option " << item.first << " could not be set.");
  }

  std::map<std::string, std::string> string_options;
  nh.param("solver/ipopt/ipopt_string_options", string_options, string_options);
  for (const auto &item : string_options) {
    if (!solver->setIpoptOptionString(item.first, item.second))
      ROS_WARN_STREAM("Ipopt option " << item.first << " could not be set.");
  }

  std::map<std::string, int> integer_options;
  nh.param("solver/ipopt/ipopt_integer_options", integer_options,
           integer_options);
  for (const auto &item : integer_options) {
    if (!solver->setIpoptOptionInt(item.first, item.second))
      ROS_WARN_STREAM("Ipopt option " << item.first << " could not be set.");
  }

  return std::move(solver);
}

corbo::StructuredOptimalControlProblem::Ptr
Controller::configureOcp(const ros::NodeHandle &nh) {
  corbo::BaseHyperGraphOptimizationProblem::Ptr hg =
      std::make_shared<corbo::HyperGraphOptimizationProblemEdgeBased>();

  corbo::StructuredOptimalControlProblem::Ptr ocp =
      std::make_shared<corbo::StructuredOptimalControlProblem>(_grid, _dynamics,
                                                               hg, _solver);

  const int x_dim = _dynamics->getStateDimension();
  const int u_dim = _dynamics->getInputDimension();

  if (_robot_type == "diff")
  {
	double max_vel_x = 0.4;
    nh.param("robot/diff/max_vel_x", max_vel_x, max_vel_x);
    double max_vel_x_backwards = 0.2;
    nh.param("robot/diff/max_vel_x_backwards", max_vel_x_backwards, max_vel_x_backwards);
    if (max_vel_x_backwards < 0)
    {
        ROS_WARN("max_vel_x_backwards must be >= 0");
        max_vel_x_backwards *= -1;
    }
    double max_vel_theta = 0.3;
    nh.param("robot/diff/max_vel_theta", max_vel_theta, max_vel_theta);

    ocp->setControlBounds(Eigen::Vector2d(-max_vel_x_backwards, -max_vel_theta), Eigen::Vector2d(max_vel_x, max_vel_theta));
  
  } 
  else if (_robot_type == "simple_car") {
    double max_vel_x = 0.2;
    nh.param("robot/simple_car/max_vel_x", max_vel_x, max_vel_x);
    double max_vel_x_backwards = 0.2;
    nh.param("robot/simple_car/max_vel_x_backwards", max_vel_x_backwards,
             max_vel_x_backwards);
    if (max_vel_x_backwards < 0) {
      ROS_WARN("max_vel_x_backwards must be >= 0");
      max_vel_x_backwards *= -1;
    }
    double max_steering_angle = 0.5;
    nh.param("robot/simple_car/max_steering_angle", max_steering_angle,
             max_steering_angle);

    ocp->setControlBounds(
        Eigen::Vector2d(-max_vel_x_backwards, -max_steering_angle),
        Eigen::Vector2d(max_vel_x, max_steering_angle));
  } else if (_robot_type == "fg_robot") {
    double max_vel_x = 0.2;
    nh.param("robot/fg_robot/max_vel_x", max_vel_x, max_vel_x);
    double max_vel_x_backwards = 0.2;
    nh.param("robot/fg_robot/max_vel_x_backwards", max_vel_x_backwards,
             max_vel_x_backwards);
    if (max_vel_x_backwards < 0) {
      ROS_WARN("max_vel_x_backwards must be >= 0");
      max_vel_x_backwards *= -1;
    }
    double max_steering_angle = 0.5;
    nh.param("robot/fg_robot/max_steering_angle", max_steering_angle,
             max_steering_angle);

    ocp->setControlBounds(
        Eigen::Vector2d(-max_vel_x_backwards, -max_steering_angle),
        Eigen::Vector2d(max_vel_x, max_steering_angle));
  } else {
    ROS_ERROR_STREAM("Cannot configure OCP for unknown robot type "
                     << _robot_type << ".");
    return {};
  }

  std::string objective_type = "quadratic_form";
  nh.param("planning/objective/type", objective_type, objective_type);
  bool lsq_solver = _solver->isLsqSolver();

  if (objective_type == "quadratic_form") {
    std::vector<double> state_weights = {10.0, 100.0, 100.0};
    nh.param("planning/objective/quadratic_form/state_weights", state_weights,
             state_weights);
    Eigen::MatrixXd Q;
    if (state_weights.size() == x_dim) {
      Q = Eigen::Map<Eigen::VectorXd>(state_weights.data(), x_dim).asDiagonal();
    } else if (state_weights.size() == x_dim * x_dim) {
      Q = Eigen::Map<Eigen::MatrixXd>(state_weights.data(), x_dim,
                                      x_dim); // Eigens default is column major
    } else {
      ROS_ERROR_STREAM("State weights dimension invalid. Must be either "
                       << x_dim << " x 1 or " << x_dim << " x " << x_dim
                       << ".");
      return {};
    }
    std::vector<double> control_weights = {1.0, 1.0};
    nh.param("planning/objective/quadratic_form/control_weights",
             control_weights, control_weights);
    Eigen::MatrixXd R;
    if (control_weights.size() == u_dim) {
      R = Eigen::Map<Eigen::VectorXd>(control_weights.data(), u_dim)
              .asDiagonal();
    } else if (control_weights.size() == u_dim * u_dim) {
      R = Eigen::Map<Eigen::MatrixXd>(control_weights.data(), u_dim,
                                      u_dim); // Eigens default is column major
    } else {
      ROS_ERROR_STREAM("Control weights dimension invalid. Must be either "
                       << u_dim << " x 1 or " << u_dim << " x " << u_dim
                       << ".");
      return {};
    }
    bool integral_form = false;
    nh.param("planning/objective/quadratic_form/integral_form", integral_form,
             integral_form);
    bool hybrid_cost_minimum_time = false;
    nh.param("planning/objective/quadratic_form/hybrid_cost_minimum_time",
             hybrid_cost_minimum_time, hybrid_cost_minimum_time);

    bool q_zero = Q.isZero();
    bool r_zero = R.isZero();
    if (!q_zero && !r_zero) {
      PRINT_ERROR_COND(hybrid_cost_minimum_time,
                       "Hybrid minimum time and quadratic form cost is "
                       "currently only supported for non-zero control weights "
                       "only. Falling "
                       "back to quadratic form.");
      ocp->setStageCost(std::make_shared<QuadraticFormCostSE2>(
          Q, R, integral_form, lsq_solver));
    } else if (!q_zero && r_zero) {
      PRINT_ERROR_COND(hybrid_cost_minimum_time,
                       "Hybrid minimum time and quadratic form cost is "
                       "currently only supported for non-zero control weights "
                       "only. Falling "
                       "back to only quadratic state cost.");
      ocp->setStageCost(std::make_shared<QuadraticStateCostSE2>(
          Q, integral_form, lsq_solver));
    } else if (q_zero && !r_zero) {
      if (hybrid_cost_minimum_time) {
        ocp->setStageCost(std::make_shared<corbo::MinTimeQuadraticControls>(
            R, integral_form, lsq_solver));
      } else {
        ocp->setStageCost(std::make_shared<corbo::QuadraticControlCost>(
            R, integral_form, lsq_solver));
      }
    }
  } else {
    ROS_ERROR_STREAM("Unknown objective type '"
                     << objective_type
                     << "' specified ('planning/objective/type').");
    return {};
  }

  std::string terminal_cost = "quadratic";
  nh.param("planning/terminal_cost/type", terminal_cost, terminal_cost);

  if (terminal_cost == "none") {
    ocp->setFinalStageCost({});
  } else if (terminal_cost == "quadratic") {
    std::vector<double> state_weights = {10.0, 100.0, 100.0};
    nh.param("planning/terminal_cost/quadratic/final_state_weights",
             state_weights, state_weights);
    Eigen::MatrixXd Qf;
    if (state_weights.size() == x_dim) {
      Qf =
          Eigen::Map<Eigen::VectorXd>(state_weights.data(), x_dim).asDiagonal();
    } else if (state_weights.size() == x_dim * x_dim) {
      Qf = Eigen::Map<Eigen::MatrixXd>(state_weights.data(), x_dim,
                                       x_dim); // Eigens default is column major
    } else {
      ROS_ERROR_STREAM("Final state weights dimension invalid. Must be either "
                       << x_dim << " x 1 or " << x_dim << " x " << x_dim
                       << ".");
      return {};
    }
    ocp->setFinalStageCost(
        std::make_shared<QuadraticFinalStateCostSE2>(Qf, lsq_solver));
  } else {
    ROS_ERROR_STREAM("Unknown terminal_cost type '"
                     << terminal_cost
                     << "' specified ('planning/terminal_cost/type').");
    return {};
  }

  std::string terminal_constraint = "none";
  nh.param("planning/terminal_constraint/type", terminal_constraint,
           terminal_constraint);

  if (terminal_constraint == "none") {
    ocp->setFinalStageConstraint({});
  } else if (terminal_constraint == "l2_ball") {
    std::vector<double> weight_matrix;
    nh.param("planning/terminal_constraint/l2_ball/weight_matrix",
             weight_matrix, weight_matrix);
    Eigen::MatrixXd S;
    if (weight_matrix.size() == x_dim) {
      S = Eigen::Map<Eigen::VectorXd>(weight_matrix.data(), x_dim).asDiagonal();
    } else if (weight_matrix.size() == x_dim * x_dim) {
      S = Eigen::Map<Eigen::MatrixXd>(weight_matrix.data(), x_dim,
                                      x_dim); // Eigens default is column major
    } else {
      ROS_ERROR_STREAM(
          "l2-ball weight_matrix dimensions invalid. Must be either "
          << x_dim << " x 1 or " << x_dim << " x " << x_dim << ".");
      return {};
    }
    double radius = 1.0;
    nh.param("planning/terminal_constraint/l2_ball/radius", radius, radius);
    ocp->setFinalStageConstraint(std::make_shared<TerminalBallSE2>(S, radius));
  } else {
    ROS_ERROR_STREAM("Unknown terminal_constraint type '"
                     << terminal_constraint
                     << "' specified ('planning/terminal_constraint/type').");
    return {};
  }

  _inequality_constraint = std::make_shared<StageInequalitySE2>();

  // configure control deviation bounds
  if (_robot_type == "diff")
    {
        double acc_lim_x = 0.0;
        nh.param("robot/diff/acc_lim_x", acc_lim_x, acc_lim_x);
        double dec_lim_x = 0.0;
        nh.param("robot/diff/dec_lim_x", dec_lim_x, dec_lim_x);
        if (dec_lim_x < 0)
        {
            ROS_WARN("dec_lim_x must be >= 0");
            dec_lim_x *= -1;
        }
        double acc_lim_theta = 0.0;
        nh.param("robot/diff/acc_lim_theta", acc_lim_theta, acc_lim_theta);

        if (acc_lim_x <= 0) acc_lim_x = corbo::CORBO_INF_DBL;
        if (dec_lim_x <= 0) dec_lim_x = corbo::CORBO_INF_DBL;
        if (acc_lim_theta <= 0) acc_lim_theta = corbo::CORBO_INF_DBL;
        Eigen::Vector2d ud_lb(-dec_lim_x, -acc_lim_theta);
        Eigen::Vector2d ud_ub(acc_lim_x, acc_lim_theta);
        _inequality_constraint->setControlDeviationBounds(ud_lb, ud_ub);
  }
  else if (_robot_type == "simple_car") {
    double acc_lim_x = 0.5;
    nh.param("robot/simple_car/acc_lim_x", acc_lim_x, acc_lim_x);
    double dec_lim_x = 0.5;
    nh.param("robot/simple_car/dec_lim_x", dec_lim_x, dec_lim_x);
    if (dec_lim_x < 0) {
      ROS_WARN("dec_lim_x must be >= 0");
      dec_lim_x *= -1;
    }
    double max_steering_rate = 0.2;
    nh.param("robot/simple_car/max_steering_rate", max_steering_rate,
             max_steering_rate);

    if (acc_lim_x <= 0)
      acc_lim_x = corbo::CORBO_INF_DBL;
    if (dec_lim_x <= 0)
      dec_lim_x = corbo::CORBO_INF_DBL;
    if (max_steering_rate <= 0)
      max_steering_rate = corbo::CORBO_INF_DBL;
    Eigen::Vector2d ud_lb(-dec_lim_x, -max_steering_rate);
    Eigen::Vector2d ud_ub(acc_lim_x, max_steering_rate);
    _inequality_constraint->setControlDeviationBounds(ud_lb, ud_ub);
  } else if (_robot_type == "fg_robot") {
    double acc_lim_x = 0.5;
    nh.param("robot/fg_robot/acc_lim_x", acc_lim_x, acc_lim_x);
    double dec_lim_x = 0.5;
    nh.param("robot/fg_robot/dec_lim_x", dec_lim_x, dec_lim_x);
    if (dec_lim_x < 0) {
      ROS_WARN("dec_lim_x must be >= 0");
      dec_lim_x *= -1;
    }
    double max_steering_rate = 0.2;
    nh.param("robot/fg_robot/max_steering_rate", max_steering_rate,
             max_steering_rate);

    if (acc_lim_x <= 0)
      acc_lim_x = corbo::CORBO_INF_DBL;
    if (dec_lim_x <= 0)
      dec_lim_x = corbo::CORBO_INF_DBL;
    if (max_steering_rate <= 0)
      max_steering_rate = corbo::CORBO_INF_DBL;
    Eigen::Vector2d ud_lb(-dec_lim_x, -max_steering_rate);
    Eigen::Vector2d ud_ub(acc_lim_x, max_steering_rate);
    _inequality_constraint->setControlDeviationBounds(ud_lb, ud_ub);

  } else {
    ROS_ERROR_STREAM(
        "Cannot configure control deviation bounds for unknown robot type "
        << _robot_type << ".");
    return {};
  }

  ocp->setStageInequalityConstraint(_inequality_constraint);

  return ocp;
}

bool Controller::generateInitialStateTrajectory(
    const Eigen::VectorXd &x0, const Eigen::VectorXd &xf,
    const std::vector<geometry_msgs::PoseStamped> &initial_plan,
    bool backward) {
  if (initial_plan.size() < 2 || !_dynamics)
    return false;

  TimeSeriesSE2::Ptr ts = std::make_shared<TimeSeriesSE2>();

  int n_init = (int)initial_plan.size();
  int n_ref = _grid->getInitialN();
  if (n_ref < 2) {
    ROS_ERROR("Controller::generateInitialStateTrajectory(): grid not properly "
              "initialized");
    return false;
  }
  ts->add(0.0, x0);

  double dt_ref = _grid->getInitialDt();
  double tf_ref = (double)(n_ref - 1) * dt_ref;

  Eigen::VectorXd x(_dynamics->getStateDimension());

  // we initialize by assuming equally distributed poses
  double dt_init = tf_ref / double(n_init - 1);

  double t = dt_init;
  for (int i = 1; i < n_init - 1; ++i) {
    // get yaw from the orientation of the distance vector between pose_{i+1}
    // and pose_{i}
    double yaw;
    if (_initial_plan_estimate_orientation) {
      double dx =
          initial_plan[i + 1].pose.position.x - initial_plan[i].pose.position.x;
      double dy =
          initial_plan[i + 1].pose.position.y - initial_plan[i].pose.position.y;
      yaw = std::atan2(dy, dx);
      if (backward)
        normalize_theta(yaw + M_PI);
    } else {
      yaw = tf2::getYaw(initial_plan[i].pose.orientation);
    }
    PoseSE2 intermediate_pose(initial_plan[i].pose.position.x,
                              initial_plan[i].pose.position.y, yaw);
    _dynamics->getSteadyStateFromPoseSE2(intermediate_pose, x);
    ts->add(t, x);
    t += dt_init;
  }

  ts->add(tf_ref, xf);

  _x_seq_init.setTrajectory(ts, corbo::TimeSeries::Interpolation::Linear);
  return true;
}
}
