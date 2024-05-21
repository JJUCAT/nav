/*********************************************************************
*  Copyright (c) 2017 - for information on the respective copyright
*  owner see the NOTICE file and/or the repository
*
*      https://github.com/hbanzhaf/steering_functions.git
*
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*  You may obtain a copy of the License at
*
*      http://www.apache.org/licenses/LICENSE-2.0
*
*  Unless required by applicable law or agreed to in writing, software
*  distributed under the License is distributed on an "AS IS" BASIS,
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
*  implied. See the License for the specific language governing
*  permissions and limitations under the License.
***********************************************************************/

#include <iostream>

//#include <Eigen/Dense>

#include "steering_functions/dubins_state_space/dubins_state_space.hpp"
#include "steering_functions/hc_cc_state_space/cc00_dubins_state_space.hpp"
#include "steering_functions/hc_cc_state_space/cc00_reeds_shepp_state_space.hpp"
#include "steering_functions/hc_cc_state_space/cc0pm_dubins_state_space.hpp"
#include "steering_functions/hc_cc_state_space/cc_dubins_state_space.hpp"
#include "steering_functions/hc_cc_state_space/ccpm0_dubins_state_space.hpp"
#include "steering_functions/hc_cc_state_space/ccpmpm_dubins_state_space.hpp"
#include "steering_functions/hc_cc_state_space/hc00_reeds_shepp_state_space.hpp"
#include "steering_functions/hc_cc_state_space/hc0pm_reeds_shepp_state_space.hpp"
#include "steering_functions/hc_cc_state_space/hc_reeds_shepp_state_space.hpp"
#include "steering_functions/hc_cc_state_space/hcpm0_reeds_shepp_state_space.hpp"
#include "steering_functions/hc_cc_state_space/hcpmpm_reeds_shepp_state_space.hpp"
#include "steering_functions/reeds_shepp_state_space/reeds_shepp_state_space.hpp"
#include "steering_functions/steering_functions.hpp"

#define FRAME_ID "/world"
#define DISCRETIZATION 0.1               // [m]
#define VISUALIZATION_DURATION 2         // [s]
#define ANIMATE false                    // [-]
#define OPERATING_REGION_X 20.0          // [m]
#define OPERATING_REGION_Y 20.0          // [m]
#define OPERATING_REGION_THETA 2 * M_PI  // [rad]
#define random(lower, upper) (rand() * (upper - lower) / RAND_MAX + lower)

using namespace std;

class PathClass
{
public:
  string path_type_;
  double discretization_;
  State state_start_;
  State state_goal_;
  double kappa_max_;
  double sigma_max_;
  vector<State> path_;

  // filter parameters
  Motion_Noise motion_noise_;
  Measurement_Noise measurement_noise_;
  Controller controller_;


  // constructor
  PathClass(const string& path_type, const State& state_start, const State& state_goal,
            const double kappa_max, const double sigma_max)
    : path_type_(path_type)
    , discretization_(DISCRETIZATION)
    , state_start_(state_start)
    , state_goal_(state_goal)
    , kappa_max_(kappa_max)
    , sigma_max_(sigma_max)
  {

    // path
    if (path_type_ == "CC_Dubins")
    {
      CC_Dubins_State_Space state_space(kappa_max_, sigma_max_, discretization_, true);
      state_space.set_filter_parameters(motion_noise_, measurement_noise_, controller_);
      path_ = state_space.get_path(state_start_, state_goal_);
    }
    else if (path_type_ == "CC00_Dubins")
    {
      CC00_Dubins_State_Space state_space(kappa_max_, sigma_max_, discretization_, true);
      state_space.set_filter_parameters(motion_noise_, measurement_noise_, controller_);
      path_ = state_space.get_path(state_start_, state_goal_);
    }
    else if (path_type_ == "CC0pm_Dubins")
    {
      CC0pm_Dubins_State_Space state_space(kappa_max_, sigma_max_, discretization_, true);
      state_space.set_filter_parameters(motion_noise_, measurement_noise_, controller_);
      path_ = state_space.get_path(state_start_, state_goal_);
    }
    else if (path_type_ == "CCpm0_Dubins")
    {
      CCpm0_Dubins_State_Space state_space(kappa_max_, sigma_max_, discretization_, true);
      state_space.set_filter_parameters(motion_noise_, measurement_noise_, controller_);
      path_ = state_space.get_path(state_start_, state_goal_);
    }
    else if (path_type_ == "CCpmpm_Dubins")
    {
      CCpmpm_Dubins_State_Space state_space(kappa_max_, sigma_max_, discretization_, true);
      state_space.set_filter_parameters(motion_noise_, measurement_noise_, controller_);
      path_ = state_space.get_path(state_start_, state_goal_);
    }
    else if (path_type_ == "Dubins")
    {
      Dubins_State_Space state_space(kappa_max_, discretization_, true);
      state_space.set_filter_parameters(motion_noise_, measurement_noise_, controller_);
      path_ = state_space.get_path(state_start_, state_goal_);
    }
    else if (path_type_ == "CC00_RS")
    {
      CC00_Reeds_Shepp_State_Space state_space(kappa_max_, sigma_max_, discretization_);
      state_space.set_filter_parameters(motion_noise_, measurement_noise_, controller_);
      path_ = state_space.get_path(state_start_, state_goal_);
    }
    else if (path_type_ == "HC_RS")
    {
      HC_Reeds_Shepp_State_Space state_space(kappa_max_, sigma_max_, discretization_);
      state_space.set_filter_parameters(motion_noise_, measurement_noise_, controller_);
      path_ = state_space.get_path(state_start_, state_goal_);
    }
    else if (path_type_ == "HC00_RS")
    {
      HC00_Reeds_Shepp_State_Space state_space(kappa_max_, sigma_max_, discretization_);
      state_space.set_filter_parameters(motion_noise_, measurement_noise_, controller_);
      path_ = state_space.get_path(state_start_, state_goal_);
    }
    else if (path_type_ == "HC0pm_RS")
    {
      HC0pm_Reeds_Shepp_State_Space state_space(kappa_max_, sigma_max_, discretization_);
      state_space.set_filter_parameters(motion_noise_, measurement_noise_, controller_);
      path_ = state_space.get_path(state_start_, state_goal_);
    }
    else if (path_type_ == "HCpm0_RS")
    {
      HCpm0_Reeds_Shepp_State_Space state_space(kappa_max_, sigma_max_, discretization_);
      state_space.set_filter_parameters(motion_noise_, measurement_noise_, controller_);
      path_ = state_space.get_path(state_start_, state_goal_);
    }
    else if (path_type_ == "HCpmpm_RS")
    {
      HCpmpm_Reeds_Shepp_State_Space state_space(kappa_max_, sigma_max_, discretization_);
      state_space.set_filter_parameters(motion_noise_, measurement_noise_, controller_);
      path_ = state_space.get_path(state_start_, state_goal_);
    }
    else if (path_type_ == "RS")
    {
      Reeds_Shepp_State_Space state_space(kappa_max_, discretization_);
      state_space.set_filter_parameters(motion_noise_, measurement_noise_, controller_);
      path_ = state_space.get_path(state_start_, state_goal_);
    }
  }
};

class RobotClass
{
public:
  // robot config
  double kappa_max_;
  double sigma_max_;
  double wheel_base_;
  double track_width_;
  double wheel_radius_;
  double wheel_width_;

  // measurement noise
  Measurement_Noise measurement_noise_;

  // visualization
  string frame_id_;
  bool animate_;

  // constructor
  explicit RobotClass() : frame_id_(FRAME_ID), animate_(ANIMATE)
  {
  }
};

int main(int argc, char** argv)
{
  RobotClass robot;

  int seed(5);
  srand(seed);
    State start;
    start.x = random(-OPERATING_REGION_X / 2.0, OPERATING_REGION_X / 2.0);
    start.y = random(-OPERATING_REGION_Y / 2.0, OPERATING_REGION_Y / 2.0);
    start.theta = random(-OPERATING_REGION_THETA / 2.0, OPERATING_REGION_THETA / 2.0);
    start.kappa = random(-robot.kappa_max_, robot.kappa_max_);
    start.d = 0.0;

    State start_wout_curv;
    start_wout_curv.x = start.x;
    start_wout_curv.y = start.y;
    start_wout_curv.theta = start.theta;
    start_wout_curv.kappa = 0.01;
    start_wout_curv.d = start.d;

    State goal;
    goal.x = random(-OPERATING_REGION_X / 2.0, OPERATING_REGION_X / 2.0);
    goal.y = random(-OPERATING_REGION_Y / 2.0, OPERATING_REGION_Y / 2.0);
    goal.theta = random(-OPERATING_REGION_THETA / 2.0, OPERATING_REGION_THETA / 2.0);
    goal.kappa = random(-robot.kappa_max_, robot.kappa_max_);
    goal.d = 0.0;

    State goal_wout_curv;
    goal_wout_curv.x = goal.x;
    goal_wout_curv.y = goal.y;
    goal_wout_curv.theta = goal.theta;
    goal_wout_curv.kappa = 0.0;
    goal_wout_curv.d = goal.d;

    PathClass cc_dubins_path("CC_Dubins", start, goal, robot.kappa_max_, robot.sigma_max_);
    PathClass cc00_dubins_path("CC00_Dubins", start_wout_curv, goal_wout_curv, robot.kappa_max_, robot.sigma_max_);
    //PathClass cc0pm_dubins_path("CC0pm_Dubins", start_wout_curv, goal_wout_curv, robot.kappa_max_, robot.sigma_max_);
    //PathClass ccpm0_dubins_path("CCpm0_Dubins", start_wout_curv, goal_wout_curv, robot.kappa_max_, robot.sigma_max_);
    //PathClass ccpmpm_dubins_path("CCpmpm_Dubins", start_wout_curv, goal_wout_curv, robot.kappa_max_, robot.sigma_max_);
    //PathClass dubins_path("Dubins", start_wout_curv, goal_wout_curv, robot.kappa_max_, robot.sigma_max_);
    //PathClass cc00_rs_path("CC00_RS", start_wout_curv, goal_wout_curv, robot.kappa_max_, robot.sigma_max_);
    //PathClass hc_rs_path("HC_RS", start, goal, robot.kappa_max_, robot.sigma_max_);
    //PathClass hc00_rs_path("HC00_RS", start_wout_curv, goal_wout_curv, robot.kappa_max_, robot.sigma_max_);
    //PathClass hc0pm_rs_path("HC0pm_RS", start_wout_curv, goal_wout_curv, robot.kappa_max_, robot.sigma_max_);
    //PathClass hcpm0_rs_path("HCpm0_RS", start_wout_curv, goal_wout_curv, robot.kappa_max_, robot.sigma_max_);
    //PathClass hcpmpm_rs_path("HCpmpm_RS", start_wout_curv, goal_wout_curv, robot.kappa_max_, robot.sigma_max_);
    PathClass rs_path("RS", start_wout_curv, goal_wout_curv, robot.kappa_max_, robot.sigma_max_);
  return 0;
}
