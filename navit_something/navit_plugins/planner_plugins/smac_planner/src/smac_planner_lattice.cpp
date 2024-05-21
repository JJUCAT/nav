// Copyright (c) 2021, Samsung Research America
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

#include <string>
#include <memory>
#include <vector>
#include <algorithm>
#include <limits>

#include "Eigen/Core"
#include "ros/package.h"
#include "ros/time.h"
#include "ros/ros.h"
//#include "rospkg/rospkg.h"
#include "smac_planner/smac_planner_lattice.hpp"

// #define BENCHMARK_TESTING

namespace nav2_smac_planner
{

using namespace std::chrono;  // NOLINT
//using rcl_interfaces::msg::ParameterType;

SmacPlannerLattice::SmacPlannerLattice()
: _a_star(nullptr),
  _collision_checker(nullptr, 1),//nullptr
  _smoother(nullptr),
  _costmap(nullptr)
{
}

SmacPlannerLattice::~SmacPlannerLattice()
{
  ROS_INFO(
    "Destroying plugin %s of type SmacPlannerLattice",
    _name.c_str());
}

void SmacPlannerLattice::initialize(
  //const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & name,
  const std::shared_ptr<navit_costmap::Costmap2DROS> & costmap_ros)
{
  // _node = parent;
  // auto node = parent.lock();
  // _logger = node->get_logger();
  // _clock = node->get_clock();
  _costmap = costmap_ros->getCostmap();
  _costmap_ros = costmap_ros;
  _name = name;
  _global_frame = costmap_ros->getGlobalFrameID();
  //_raw_plan_publisher = node->create_publisher<nav_msgs::Path>("unsmoothed_plan", 1);
  ros::NodeHandle pnh("~/" + _name);
  _raw_plan_publisher = pnh.advertise<nav_msgs::Path>("unsmoothed_plan", 1);

  ROS_INFO("Configuring %s of type SmacPlannerLattice", name.c_str());

  // General planner params
  double analytic_expansion_max_length_m;
  bool smooth_path;

  pnh.param("tolerance", _tolerance, 0.25f);
  pnh.param("allow_unknown", _allow_unknown, true);
  pnh.param("max_iterations", _max_iterations, 1000000);
  pnh.param("max_on_approach_iterations", _max_on_approach_iterations, 1000);
  pnh.param("smooth_path", smooth_path, true);

  pnh.param("cache_obstacle_heuristic", _search_info.cache_obstacle_heuristic, false);
  pnh.param("reverse_penalty", _search_info.reverse_penalty, 2.0f);
  pnh.param("change_penalty", _search_info.change_penalty, 0.05f);
  pnh.param("non_straight_penalty", _search_info.non_straight_penalty, 1.05f);
  pnh.param("cost_penalty", _search_info.cost_penalty, 2.0f);
  pnh.param("retrospective_penalty", _search_info.retrospective_penalty, 0.015f);
  pnh.param("analytic_expansion_ratio", _search_info.analytic_expansion_ratio, 3.5f);
  pnh.param("analytic_expansion_max_length", analytic_expansion_max_length_m, 3.0);
  _search_info.analytic_expansion_max_length =  analytic_expansion_max_length_m / _costmap->getResolution();
  pnh.param("max_planning_time", _max_planning_time, 5.0);
  pnh.param("lookup_table_size", _lookup_table_size, 20.0);
  pnh.param("rotation_penalty", _search_info.rotation_penalty, 5.0f);
  pnh.param("allow_reverse_expansion", _search_info.allow_reverse_expansion, false);
  //pnh.param("motion_model_for_search", _motion_model_for_search, std::string("DUBIN"));

  // nav2_util::declare_parameter_if_not_declared(
  //   node, name + ".tolerance", rclcpp::ParameterValue(0.25));
  // _tolerance = static_cast<float>(node->get_parameter(name + ".tolerance").as_double());
  // nav2_util::declare_parameter_if_not_declared(
  //   node, name + ".allow_unknown", rclcpp::ParameterValue(true));
  // node->get_parameter(name + ".allow_unknown", _allow_unknown);
  // nav2_util::declare_parameter_if_not_declared(
  //   node, name + ".max_iterations", rclcpp::ParameterValue(1000000));
  // node->get_parameter(name + ".max_iterations", _max_iterations);
  // nav2_util::declare_parameter_if_not_declared(
  //   node, name + ".max_on_approach_iterations", rclcpp::ParameterValue(1000));
  // node->get_parameter(name + ".max_on_approach_iterations", _max_on_approach_iterations);
  // nav2_util::declare_parameter_if_not_declared(
  //   node, name + ".smooth_path", rclcpp::ParameterValue(true));
  // node->get_parameter(name + ".smooth_path", smooth_path);

  // Default to a well rounded model: 16 bin, 0.4m turning radius, ackermann model
  // rospkg::Rospag rp;
  std::string path = "lattice_primitives/sample_primitives/5cm_resolution/0.5m_turning_radius/ackermann/output.json";
  std::string full_path = ros::package::getPath("smac_planner") + "/" + path;
  pnh.param("lattice_filepath", _search_info.lattice_filepath,full_path);
  //pnh.param("lattice_filepath", _search_info.lattice_filepath, ros::package::getPath("smac_planner/lattice_primitives/sample_primitives/5cm_resolution/0.5m_turning_radius/ackermann/output.json"));
  // nav2_util::declare_parameter_if_not_declared(
  //   node, name + ".lattice_filepath", rclcpp::ParameterValue(
  //     ament_index_cpp::get_package_share_directory("nav2_smac_planner") +
  //     "/sample_primitives/5cm_resolution/0.5m_turning_radius/ackermann/output.json"));
  // node->get_parameter(name + ".lattice_filepath", _search_info.lattice_filepath);
  
  // nav2_util::declare_parameter_if_not_declared(
  //   node, name + ".cache_obstacle_heuristic", rclcpp::ParameterValue(false));
  // node->get_parameter(name + ".cache_obstacle_heuristic", _search_info.cache_obstacle_heuristic);
  // nav2_util::declare_parameter_if_not_declared(
  //   node, name + ".reverse_penalty", rclcpp::ParameterValue(2.0));
  // node->get_parameter(name + ".reverse_penalty", _search_info.reverse_penalty);
  // nav2_util::declare_parameter_if_not_declared(
  //   node, name + ".change_penalty", rclcpp::ParameterValue(0.05));
  // node->get_parameter(name + ".change_penalty", _search_info.change_penalty);
  // nav2_util::declare_parameter_if_not_declared(
  //   node, name + ".non_straight_penalty", rclcpp::ParameterValue(1.05));
  // node->get_parameter(name + ".non_straight_penalty", _search_info.non_straight_penalty);
  // nav2_util::declare_parameter_if_not_declared(
  //   node, name + ".cost_penalty", rclcpp::ParameterValue(2.0));
  // node->get_parameter(name + ".cost_penalty", _search_info.cost_penalty);
  // nav2_util::declare_parameter_if_not_declared(
  //   node, name + ".retrospective_penalty", rclcpp::ParameterValue(0.015));
  // node->get_parameter(name + ".retrospective_penalty", _search_info.retrospective_penalty);
  // nav2_util::declare_parameter_if_not_declared(
  //   node, name + ".rotation_penalty", rclcpp::ParameterValue(5.0));
  // node->get_parameter(name + ".rotation_penalty", _search_info.rotation_penalty);
  // nav2_util::declare_parameter_if_not_declared(
  //   node, name + ".analytic_expansion_ratio", rclcpp::ParameterValue(3.5));
  // node->get_parameter(name + ".analytic_expansion_ratio", _search_info.analytic_expansion_ratio);
  // nav2_util::declare_parameter_if_not_declared(
  //   node, name + ".analytic_expansion_max_length", rclcpp::ParameterValue(3.0));
  // node->get_parameter(name + ".analytic_expansion_max_length", analytic_expansion_max_length_m);
  // _search_info.analytic_expansion_max_length =
  //   analytic_expansion_max_length_m / _costmap->getResolution();

  // nav2_util::declare_parameter_if_not_declared(
  //   node, name + ".max_planning_time", rclcpp::ParameterValue(5.0));
  // node->get_parameter(name + ".max_planning_time", _max_planning_time);
  // nav2_util::declare_parameter_if_not_declared(
  //   node, name + ".lookup_table_size", rclcpp::ParameterValue(20.0));
  // node->get_parameter(name + ".lookup_table_size", _lookup_table_size);
  // nav2_util::declare_parameter_if_not_declared(
  //   node, name + ".allow_reverse_expansion", rclcpp::ParameterValue(false));
  // node->get_parameter(name + ".allow_reverse_expansion", _search_info.allow_reverse_expansion);

  _metadata = LatticeMotionTable::getLatticeMetadata(_search_info.lattice_filepath);
  _search_info.minimum_turning_radius =
    _metadata.min_turning_radius / (_costmap->getResolution());
  _motion_model = MotionModel::STATE_LATTICE;

  if (_max_on_approach_iterations <= 0) {
    ROS_INFO(
      "On approach iteration selected as <= 0, "
      "disabling tolerance and on approach iterations.");
    _max_on_approach_iterations = std::numeric_limits<int>::max();
  }

  if (_max_iterations <= 0) {
    ROS_INFO(
      "maximum iteration selected as <= 0, "
      "disabling maximum iterations.");
    _max_iterations = std::numeric_limits<int>::max();
  }

  float lookup_table_dim =
    static_cast<float>(_lookup_table_size) /
    static_cast<float>(_costmap->getResolution());

  // Make sure its a whole number
  lookup_table_dim = static_cast<float>(static_cast<int>(lookup_table_dim));

  // Make sure its an odd number
  if (static_cast<int>(lookup_table_dim) % 2 == 0) {
    ROS_INFO(
      "Even sized heuristic lookup table size set %f, increasing size by 1 to make odd",
      lookup_table_dim);
    lookup_table_dim += 1.0;
  }

  // Initialize collision checker using 72 evenly sized bins instead of the lattice
  // heading angles. This is done so that we have precomputed angles every 5 degrees.
  // If we used the sparse lattice headings (usually 16), then when we attempt to collision
  // check for intermediary points of the primitives, we're forced to round to one of the 16
  // increments causing "wobbly" checks that could cause larger robots to virtually show collisions
  // in valid configurations. This approximation helps to bound orientation error for all checks
  // in exchange for slight inaccuracies in the collision headings in terminal search states.
  //_collision_checker = GridCollisionChecker(_costmap, 72u, node);
  _collision_checker = GridCollisionChecker(_costmap, 72u);
  _collision_checker.setFootprint(
    costmap_ros->getRobotFootprint(),
    costmap_ros->getUseRadius(),
    findCircumscribedCost(costmap_ros));

  // Initialize A* template
  _a_star = std::make_unique<AStarAlgorithm<NodeLattice>>(_motion_model, _search_info);
  _a_star->initialize(
    _allow_unknown,
    _max_iterations,
    _max_on_approach_iterations,
    _max_planning_time,
    lookup_table_dim,
    _metadata.number_of_headings);

  // Initialize path smoother
  if (smooth_path) {
    SmootherParams params;
    //params.get(node, name);
    params.get(name);
    _smoother = std::make_unique<Smoother>(params);
    _smoother->initialize(_metadata.min_turning_radius);
  }

  ROS_INFO(
    "Configured plugin %s of type SmacPlannerLattice with "
    "maximum iterations %i, max on approach iterations %i, "
    "and %s. Tolerance %.2f. Using motion model: %s. State lattice file: %s.",
    _name.c_str(), _max_iterations, _max_on_approach_iterations,
    _allow_unknown ? "allowing unknown traversal" : "not allowing unknown traversal",
    _tolerance, toString(_motion_model).c_str(), _search_info.lattice_filepath.c_str());
}

/*void SmacPlannerLattice::activate()
{
  ROS_INFO(
    _logger, "Activating plugin %s of type SmacPlannerLattice",
    _name.c_str());
  _raw_plan_publisher->on_activate();
  auto node = _node.lock();
  // Add callback for dynamic parameters
  _dyn_params_handler = node->add_on_set_parameters_callback(
    std::bind(&SmacPlannerLattice::dynamicParametersCallback, this, std::placeholders::_1));
}

void SmacPlannerLattice::deactivate()
{
  ROS_INFO(
    _logger, "Deactivating plugin %s of type SmacPlannerLattice",
    _name.c_str());
  _raw_plan_publisher->on_deactivate();
  _dyn_params_handler.reset();
}

void SmacPlannerLattice::cleanup()
{
  ROS_INFO(
    _logger, "Cleaning up plugin %s of type SmacPlannerLattice",
    _name.c_str());
  _a_star.reset();
  _smoother.reset();
  _raw_plan_publisher.reset();
}*/

nav_msgs::Path SmacPlannerLattice::createPlan(
  const geometry_msgs::PoseStamped & start,
  const geometry_msgs::PoseStamped & goal)
{
  std::lock_guard<std::mutex> lock_reinit(_mutex);
  //steady_clock::time_point a = steady_clock::now();
  ros::Time a = ros::Time::now();
  std::string error;
  std::unique_lock<navit_costmap::Costmap2D::mutex_t> lock(*(_costmap->getMutex()));

  // Set collision checker and costmap information
  _a_star->setCollisionChecker(&_collision_checker);

  // Set starting point, in A* bin search coordinates
  unsigned int mx, my;
  if (!_costmap->worldToMap(start.pose.position.x, start.pose.position.y, mx, my)) {
    // throw nav2_core::StartOutsideMapBounds(
    //         "Start Coordinates of(" + std::to_string(start.pose.position.x) + ", " +
    //         std::to_string(start.pose.position.y) + ") was outside bounds");
    error = std::string("Start Coordinates of(" + std::to_string(start.pose.position.x) + ", " +
            std::to_string(start.pose.position.y) + ") was outside bounds");
  }
  _a_star->setStart(
    mx, my,
    NodeLattice::motion_table.getClosestAngularBin(tf2::getYaw(start.pose.orientation)));

  // Set goal point, in A* bin search coordinates
  if (!_costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my)) {
    // throw nav2_core::GoalOutsideMapBounds(
    //         "Goal Coordinates of(" + std::to_string(goal.pose.position.x) + ", " +
    //         std::to_string(goal.pose.position.y) + ") was outside bounds");
    error = std::string("Goal Coordinates of(" + std::to_string(goal.pose.position.x) + ", " +
            std::to_string(goal.pose.position.y) + ") was outside bounds");
  }
  _a_star->setGoal(
    mx, my,
    NodeLattice::motion_table.getClosestAngularBin(tf2::getYaw(goal.pose.orientation)));

  // Setup message
  nav_msgs::Path plan;
  plan.header.stamp = ros::Time::now();
  plan.header.frame_id = _global_frame;
  geometry_msgs::PoseStamped pose;
  pose.header = plan.header;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;

  // Compute plan
  NodeLattice::CoordinateVector path;
  int num_iterations = 0;

  // Note: All exceptions thrown are handled by the planner server and returned to the action
  if (!_a_star->createPath(
      path, num_iterations, _tolerance / static_cast<float>(_costmap->getResolution())))
  {
    if (num_iterations < _a_star->getMaxIterations()) {
      //throw navit_core::NoValidPathCouldBeFound("no valid path found");
      error = std::string("no valid path found");
    } else {
      //throw navit_core::PlannerTimedOut("exceeded maximum iterations");
      error = std::string("exceeded maximum iterations");
    }
  }

  // Convert to world coordinates
  plan.poses.reserve(path.size());
  geometry_msgs::PoseStamped last_pose = pose;
  for (int i = path.size() - 1; i >= 0; --i) {
    pose.pose = getWorldCoords(path[i].x, path[i].y, _costmap);
    pose.pose.orientation = getWorldOrientation(path[i].theta);
    if (fabs(pose.pose.position.x - last_pose.pose.position.x) < 1e-4 &&
      fabs(pose.pose.position.y - last_pose.pose.position.y) < 1e-4 &&
      fabs(tf2::getYaw(pose.pose.orientation) - tf2::getYaw(last_pose.pose.orientation)) < 1e-4)
    {
      ROS_DEBUG(
        "Removed a path from the path due to replication. "
        "Make sure your minimum control set does not contain duplicate values!");
      continue;
    }
    last_pose = pose;
    plan.poses.push_back(pose);
  }

  // Publish raw path for debug
  //if (_raw_plan_publisher.get_subscription_count() > 0) {
    _raw_plan_publisher.publish(plan);
  //}

  // Find how much time we have left to do smoothing
  // steady_clock::time_point b = steady_clock::now();
  // duration<double> time_span = duration_cast<duration<double>>(b - a);
  // double time_remaining = _max_planning_time - static_cast<double>(time_span.count());
  ros::Time b = ros::Time::now();
  ros::Duration time_span = b - a;
  double time_remaining = _max_planning_time - time_span.toSec();

#ifdef BENCHMARK_TESTING
  std::cout << "It took " << time_span.count() * 1000 <<
    " milliseconds with " << num_iterations << " iterations." << std::endl;
#endif

  // Smooth plan
  if (_smoother && num_iterations > 1) {
    _smoother->smooth(plan, _costmap, time_remaining);
  }

#ifdef BENCHMARK_TESTING
  steady_clock::time_point c = steady_clock::now();
  duration<double> time_span2 = duration_cast<duration<double>>(c - b);
  std::cout << "It took " << time_span2.count() * 1000 <<
    " milliseconds to smooth path." << std::endl;
#endif

  return plan;
}

// rcl_interfaces::msg::SetParametersResult
// SmacPlannerLattice::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
// {
//   rcl_interfaces::msg::SetParametersResult result;
//   std::lock_guard<std::mutex> lock_reinit(_mutex);

//   bool reinit_a_star = false;
//   bool reinit_smoother = false;

//   for (auto parameter : parameters) {
//     const auto & type = parameter.get_type();
//     const auto & name = parameter.get_name();

//     if (type == ParameterType::PARAMETER_DOUBLE) {
//       if (name == _name + ".max_planning_time") {
//         reinit_a_star = true;
//         _max_planning_time = parameter.as_double();
//       } else if (name == _name + ".tolerance") {
//         _tolerance = static_cast<float>(parameter.as_double());
//       } else if (name == _name + ".lookup_table_size") {
//         reinit_a_star = true;
//         _lookup_table_size = parameter.as_double();
//       } else if (name == _name + ".reverse_penalty") {
//         reinit_a_star = true;
//         _search_info.reverse_penalty = static_cast<float>(parameter.as_double());
//       } else if (name == _name + ".change_penalty") {
//         reinit_a_star = true;
//         _search_info.change_penalty = static_cast<float>(parameter.as_double());
//       } else if (name == _name + ".non_straight_penalty") {
//         reinit_a_star = true;
//         _search_info.non_straight_penalty = static_cast<float>(parameter.as_double());
//       } else if (name == _name + ".cost_penalty") {
//         reinit_a_star = true;
//         _search_info.cost_penalty = static_cast<float>(parameter.as_double());
//       } else if (name == _name + ".rotation_penalty") {
//         reinit_a_star = true;
//         _search_info.rotation_penalty = static_cast<float>(parameter.as_double());
//       } else if (name == _name + ".analytic_expansion_ratio") {
//         reinit_a_star = true;
//         _search_info.analytic_expansion_ratio = static_cast<float>(parameter.as_double());
//       } else if (name == _name + ".analytic_expansion_max_length") {
//         reinit_a_star = true;
//         _search_info.analytic_expansion_max_length =
//           static_cast<float>(parameter.as_double()) / _costmap->getResolution();
//       }
//     } else if (type == ParameterType::PARAMETER_BOOL) {
//       if (name == _name + ".allow_unknown") {
//         reinit_a_star = true;
//         _allow_unknown = parameter.as_bool();
//       } else if (name == _name + ".cache_obstacle_heuristic") {
//         reinit_a_star = true;
//         _search_info.cache_obstacle_heuristic = parameter.as_bool();
//       } else if (name == _name + ".allow_reverse_expansion") {
//         reinit_a_star = true;
//         _search_info.allow_reverse_expansion = parameter.as_bool();
//       } else if (name == _name + ".smooth_path") {
//         if (parameter.as_bool()) {
//           reinit_smoother = true;
//         } else {
//           _smoother.reset();
//         }
//       }
//     } else if (type == ParameterType::PARAMETER_INTEGER) {
//       if (name == _name + ".max_iterations") {
//         reinit_a_star = true;
//         _max_iterations = parameter.as_int();
//         if (_max_iterations <= 0) {
//           ROS_INFO(
//             _logger, "maximum iteration selected as <= 0, "
//             "disabling maximum iterations.");
//           _max_iterations = std::numeric_limits<int>::max();
//         }
//       }
//     } else if (name == _name + ".max_on_approach_iterations") {
//       reinit_a_star = true;
//       _max_on_approach_iterations = parameter.as_int();
//       if (_max_on_approach_iterations <= 0) {
//         ROS_INFO(
//           _logger, "On approach iteration selected as <= 0, "
//           "disabling tolerance and on approach iterations.");
//         _max_on_approach_iterations = std::numeric_limits<int>::max();
//       }
//     } else if (type == ParameterType::PARAMETER_STRING) {
//       if (name == _name + ".lattice_filepath") {
//         reinit_a_star = true;
//         if (_smoother) {
//           reinit_smoother = true;
//         }
//         _search_info.lattice_filepath = parameter.as_string();
//         _metadata = LatticeMotionTable::getLatticeMetadata(_search_info.lattice_filepath);
//         _search_info.minimum_turning_radius =
//           _metadata.min_turning_radius / (_costmap->getResolution());
//       }
//     }
//   }

//   // Re-init if needed with mutex lock (to avoid re-init while creating a plan)
//   if (reinit_a_star || reinit_smoother) {
//     // convert to grid coordinates
//     _search_info.minimum_turning_radius =
//       _metadata.min_turning_radius / (_costmap->getResolution());
//     float lookup_table_dim =
//       static_cast<float>(_lookup_table_size) /
//       static_cast<float>(_costmap->getResolution());

//     // Make sure its a whole number
//     lookup_table_dim = static_cast<float>(static_cast<int>(lookup_table_dim));

//     // Make sure its an odd number
//     if (static_cast<int>(lookup_table_dim) % 2 == 0) {
//       ROS_INFO(
//         _logger,
//         "Even sized heuristic lookup table size set %f, increasing size by 1 to make odd",
//         lookup_table_dim);
//       lookup_table_dim += 1.0;
//     }

//     // Re-Initialize smoother
//     if (reinit_smoother) {
//       auto node = _node.lock();
//       SmootherParams params;
//       params.get(node, _name);
//       _smoother = std::make_unique<Smoother>(params);
//       _smoother->initialize(_metadata.min_turning_radius);
//     }

//     // Re-Initialize A* template
//     if (reinit_a_star) {
//       _a_star = std::make_unique<AStarAlgorithm<NodeLattice>>(_motion_model, _search_info);
//       _a_star->initialize(
//         _allow_unknown,
//         _max_iterations,
//         _max_on_approach_iterations,
//         _max_planning_time,
//         lookup_table_dim,
//         _metadata.number_of_headings);
//     }
//   }

//   result.successful = true;
//   return result;
// }

}  // namespace nav2_smac_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_smac_planner::SmacPlannerLattice, navit_core::GlobalPlanner)
