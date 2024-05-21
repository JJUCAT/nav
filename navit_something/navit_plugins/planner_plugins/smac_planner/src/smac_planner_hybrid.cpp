// Copyright (c) 2020, Samsung Research America
// Copyright (c) 2023, Open Navigation LLC
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

#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include <algorithm>
#include <limits>

#include "Eigen/Core"
#include "ros/publisher.h"
#include "smac_planner/smac_planner_hybrid.hpp"

// #define BENCHMARK_TESTING

namespace nav2_smac_planner
{

using namespace std::chrono;  // NOLINT
//using rcl_interfaces::msg::ParameterType;
//using std::placeholders::_1;

SmacPlannerHybrid::SmacPlannerHybrid()
: _a_star(nullptr),
  _collision_checker(nullptr, 1),//nullptr
  _smoother(nullptr),
  _costmap(nullptr),
  _costmap_downsampler(nullptr)
{
}

SmacPlannerHybrid::~SmacPlannerHybrid()
{
  ROS_INFO(
    "Destroying plugin %s of type SmacPlannerHybrid",
    _name.c_str());
}

void SmacPlannerHybrid::initialize(
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

  ROS_INFO("Configuring %s of type SmacPlannerHybrid", name.c_str());

  int angle_quantizations;
  double analytic_expansion_max_length_m;
  bool smooth_path;

  // General planner params
  ros::NodeHandle pnh("~/" + _name);
  pnh.param("downsample_costmap", _downsample_costmap, false);
  pnh.param("downsampling_factor", _downsampling_factor, 1);
  pnh.param("angle_quantization_bins", angle_quantizations, 72);
  _angle_bin_size = 2.0 * M_PI / angle_quantizations;
  _angle_quantizations = static_cast<unsigned int>(angle_quantizations);
  pnh.param("tolerance", _tolerance, 0.25);
  pnh.param("allow_unknown", _allow_unknown, false);
  pnh.param("max_iterations", _max_iterations, 1000000);
  pnh.param("max_on_approach_iterations", _max_on_approach_iterations, 1000);
  pnh.param("smooth_path", smooth_path, true);

  pnh.param("minimum_turning_radius", _minimum_turning_radius_global_coords, 0.4);
  pnh.param("allow_primitive_interpolation", _search_info.allow_primitive_interpolation, true);
  pnh.param("cache_obstacle_heuristic", _search_info.cache_obstacle_heuristic, true);
  pnh.param("reverse_penalty", _search_info.reverse_penalty, 2.0f);
  pnh.param("change_penalty", _search_info.change_penalty, 0.0f);
  pnh.param("non_straight_penalty", _search_info.non_straight_penalty, 1.2f);
  pnh.param("cost_penalty", _search_info.cost_penalty, 0.5f);
  pnh.param("retrospective_penalty", _search_info.retrospective_penalty, 0.015f);
  pnh.param("analytic_expansion_ratio", _search_info.analytic_expansion_ratio, 3.5f);
  pnh.param("use_quadratic_cost_penalty", _search_info.use_quadratic_cost_penalty, true);
  pnh.param("downsample_obstacle_heuristic", _search_info.downsample_obstacle_heuristic, true);
  pnh.param("analytic_expansion_max_length", analytic_expansion_max_length_m, 3.0);
  _search_info.analytic_expansion_max_length =  analytic_expansion_max_length_m / _costmap->getResolution();
  pnh.param("max_planning_time", _max_planning_time, 5.0);
  pnh.param("lookup_table_size", _lookup_table_size, 20.0);
  pnh.param("debug_visualizations", _debug_visualizations, true);
  pnh.param("motion_model_for_search", _motion_model_for_search, std::string("DUBIN"));
  _motion_model = fromString(_motion_model_for_search);
  // nav2_util::declare_parameter_if_not_declared(
  //   node, name + ".downsample_costmap", rclcpp::ParameterValue(false));
  // node->get_parameter(name + ".downsample_costmap", _downsample_costmap);
  // nav2_util::declare_parameter_if_not_declared(
  //   node, name + ".downsampling_factor", rclcpp::ParameterValue(1));
  // node->get_parameter(name + ".downsampling_factor", _downsampling_factor);

  // nav2_util::declare_parameter_if_not_declared(
  //   node, name + ".angle_quantization_bins", rclcpp::ParameterValue(72));
  // node->get_parameter(name + ".angle_quantization_bins", angle_quantizations);
  // _angle_bin_size = 2.0 * M_PI / angle_quantizations;
  // _angle_quantizations = static_cast<unsigned int>(angle_quantizations);

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

  // nav2_util::declare_parameter_if_not_declared(
  //   node, name + ".minimum_turning_radius", rclcpp::ParameterValue(0.4));
  // node->get_parameter(name + ".minimum_turning_radius", _minimum_turning_radius_global_coords);
  // nav2_util::declare_parameter_if_not_declared(
  //   node, name + ".allow_primitive_interpolation", rclcpp::ParameterValue(false));
  // node->get_parameter(
  //   name + ".allow_primitive_interpolation", _search_info.allow_primitive_interpolation);
  // nav2_util::declare_parameter_if_not_declared(
  //   node, name + ".cache_obstacle_heuristic", rclcpp::ParameterValue(false));
  // node->get_parameter(name + ".cache_obstacle_heuristic", _search_info.cache_obstacle_heuristic);
  // nav2_util::declare_parameter_if_not_declared(
  //   node, name + ".reverse_penalty", rclcpp::ParameterValue(2.0));
  // node->get_parameter(name + ".reverse_penalty", _search_info.reverse_penalty);
  // nav2_util::declare_parameter_if_not_declared(
  //   node, name + ".change_penalty", rclcpp::ParameterValue(0.0));
  // node->get_parameter(name + ".change_penalty", _search_info.change_penalty);
  // nav2_util::declare_parameter_if_not_declared(
  //   node, name + ".non_straight_penalty", rclcpp::ParameterValue(1.2));
  // node->get_parameter(name + ".non_straight_penalty", _search_info.non_straight_penalty);
  // nav2_util::declare_parameter_if_not_declared(
  //   node, name + ".cost_penalty", rclcpp::ParameterValue(2.0));
  // node->get_parameter(name + ".cost_penalty", _search_info.cost_penalty);
  // nav2_util::declare_parameter_if_not_declared(
  //   node, name + ".retrospective_penalty", rclcpp::ParameterValue(0.015));
  // node->get_parameter(name + ".retrospective_penalty", _search_info.retrospective_penalty);
  // nav2_util::declare_parameter_if_not_declared(
  //   node, name + ".analytic_expansion_ratio", rclcpp::ParameterValue(3.5));
  // node->get_parameter(name + ".analytic_expansion_ratio", _search_info.analytic_expansion_ratio);
  // nav2_util::declare_parameter_if_not_declared(
  //   node, name + ".use_quadratic_cost_penalty", rclcpp::ParameterValue(false));
  // node->get_parameter(
  //   name + ".use_quadratic_cost_penalty", _search_info.use_quadratic_cost_penalty);
  // nav2_util::declare_parameter_if_not_declared(
  //   node, name + ".downsample_obstacle_heuristic", rclcpp::ParameterValue(true));
  // node->get_parameter(
  //   name + ".downsample_obstacle_heuristic", _search_info.downsample_obstacle_heuristic);

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
  //   node, name + ".debug_visualizations", rclcpp::ParameterValue(false));
  // node->get_parameter(name + ".debug_visualizations", _debug_visualizations);

  // nav2_util::declare_parameter_if_not_declared(
  //   node, name + ".motion_model_for_search", rclcpp::ParameterValue(std::string("DUBIN")));
  // node->get_parameter(name + ".motion_model_for_search", _motion_model_for_search);
  // _motion_model = fromString(_motion_model_for_search);

  if (_motion_model == MotionModel::UNKNOWN) {
    ROS_WARN(
      "Unable to get MotionModel search type. Given '%s', "
      "valid options are MOORE, VON_NEUMANN, DUBIN, REEDS_SHEPP, STATE_LATTICE.",
      _motion_model_for_search.c_str());
  }

  if (_max_on_approach_iterations <= 0) {
    ROS_WARN(
      "On approach iteration selected as <= 0, "
      "disabling tolerance and on approach iterations.");
    _max_on_approach_iterations = std::numeric_limits<int>::max();
  }

  if (_max_iterations <= 0) {
    ROS_WARN(
      "maximum iteration selected as <= 0, "
      "disabling maximum iterations.");
    _max_iterations = std::numeric_limits<int>::max();
  }

  if (_minimum_turning_radius_global_coords < _costmap->getResolution() * _downsampling_factor) {
    ROS_WARN(
      "Min turning radius cannot be less than the search grid cell resolution!");
    _minimum_turning_radius_global_coords = _costmap->getResolution() * _downsampling_factor;
  }

  // convert to grid coordinates
  if (!_downsample_costmap) {
    _downsampling_factor = 1;
  }
  _search_info.minimum_turning_radius =
    _minimum_turning_radius_global_coords / (_costmap->getResolution() * _downsampling_factor);
  _lookup_table_dim =
    static_cast<float>(_lookup_table_size) /
    static_cast<float>(_costmap->getResolution() * _downsampling_factor);

  // Make sure its a whole number
  _lookup_table_dim = static_cast<float>(static_cast<int>(_lookup_table_dim));

  // Make sure its an odd number
  if (static_cast<int>(_lookup_table_dim) % 2 == 0) {
    ROS_INFO(
      "Even sized heuristic lookup table size set %f, increasing size by 1 to make odd",
      _lookup_table_dim);
    _lookup_table_dim += 1.0;
  }

  // Initialize collision checker
  _collision_checker = GridCollisionChecker(_costmap, _angle_quantizations); //node
  _collision_checker.setFootprint(
    _costmap_ros->getRobotFootprint(),
    _costmap_ros->getUseRadius(),
    findCircumscribedCost(_costmap_ros));

  // Initialize A* template
  _a_star = std::make_unique<AStarAlgorithm<NodeHybrid>>(_motion_model, _search_info);
  _a_star->initialize(
    _allow_unknown,
    _max_iterations,
    _max_on_approach_iterations,
    _max_planning_time,
    _lookup_table_dim,
    _angle_quantizations);

  // Initialize path smoother
  if (smooth_path) {
    SmootherParams params;
    //params.get(node, name);
    params.get(name);
    _smoother = std::make_unique<Smoother>(params);
    _smoother->initialize(_minimum_turning_radius_global_coords);
  }

  // Initialize costmap downsampler
  if (_downsample_costmap && _downsampling_factor > 1) {
    _costmap_downsampler = std::make_unique<CostmapDownsampler>();
    std::string topic_name = "downsampled_costmap";
    _costmap_downsampler->initialize(
      &pnh, _global_frame, topic_name, _costmap, _downsampling_factor);
  }

  //_raw_plan_publisher = node->create_publisher<nav_msgs::Path>("unsmoothed_plan", 1);
  _raw_plan_publisher = pnh.advertise<nav_msgs::Path>("unsmoothed_plan", 10);

  if (_debug_visualizations) {
    //_expansions_publisher = node->create_publisher<geometry_msgs::PoseArray>("expansions", 1);
    _expansions_publisher = pnh.advertise<geometry_msgs::PoseArray>("expansions", 1);
    _planned_footprints_publisher = pnh.advertise<visualization_msgs::MarkerArray>("planned_footprints", 1);
    // _planned_footprints_publisher = node->create_publisher<visualization_msgs::MarkerArray>(
    //   "planned_footprints", 1);
  }

  ROS_INFO(
    "Configured plugin %s of type SmacPlannerHybrid with "
    "maximum iterations %i, max on approach iterations %i, and %s. Tolerance %.2f."
    "Using motion model: %s.",
    _name.c_str(), _max_iterations, _max_on_approach_iterations,
    _allow_unknown ? "allowing unknown traversal" : "not allowing unknown traversal",
    _tolerance, toString(_motion_model).c_str());
}

/*void SmacPlannerHybrid::activate()
{
  ROS_INFO(
    _logger, "Activating plugin %s of type SmacPlannerHybrid",
    _name.c_str());
  _raw_plan_publisher->on_activate();
  if (_debug_visualizations) {
    _expansions_publisher->on_activate();
    _planned_footprints_publisher->on_activate();
  }
  if (_costmap_downsampler) {
    _costmap_downsampler->on_activate();
  }
  auto node = _node.lock();
  // Add callback for dynamic parameters
  _dyn_params_handler = node->add_on_set_parameters_callback(
    std::bind(&SmacPlannerHybrid::dynamicParametersCallback, this, _1));
}

void SmacPlannerHybrid::deactivate()
{
  ROS_INFO(
    _logger, "Deactivating plugin %s of type SmacPlannerHybrid",
    _name.c_str());
  _raw_plan_publisher->on_deactivate();
  if (_debug_visualizations) {
    _expansions_publisher->on_deactivate();
    _planned_footprints_publisher->on_deactivate();
  }
  if (_costmap_downsampler) {
    _costmap_downsampler->on_deactivate();
  }
  _dyn_params_handler.reset();
}

void SmacPlannerHybrid::cleanup()
{
  ROS_INFO(
    _logger, "Cleaning up plugin %s of type SmacPlannerHybrid",
    _name.c_str());
  _a_star.reset();
  _smoother.reset();
  if (_costmap_downsampler) {
    _costmap_downsampler->on_cleanup();
    _costmap_downsampler.reset();
  }
  _raw_plan_publisher.reset();
  _expansions_publisher.reset();
  _planned_footprints_publisher.reset();
}*/

nav_msgs::Path SmacPlannerHybrid::createPlan(
  const geometry_msgs::PoseStamped & start,
  const geometry_msgs::PoseStamped & goal)
{
  std::lock_guard<std::mutex> lock_reinit(_mutex);
  //steady_clock::time_point a = steady_clock::now();
  ros::Time a = ros::Time::now();
  std::string error;
  std::unique_lock<navit_costmap::Costmap2D::mutex_t> lock(*(_costmap->getMutex()));

  // Downsample costmap, if required
  navit_costmap::Costmap2D * costmap = _costmap;
  if (_costmap_downsampler) {
    costmap = _costmap_downsampler->downsample(_downsampling_factor);
    _collision_checker.setCostmap(costmap);
  }

  // Set collision checker and costmap information
  _a_star->setCollisionChecker(&_collision_checker);

  // Set starting point, in A* bin search coordinates
  unsigned int mx, my;
  if (!costmap->worldToMap(start.pose.position.x, start.pose.position.y, mx, my)) {
    // throw navit_core::StartOutsideMapBounds(
    //         "Start Coordinates of(" + std::to_string(start.pose.position.x) + ", " +
    //         std::to_string(start.pose.position.y) + ") was outside bounds");
    error = std::string("Start Coordinates of(" + std::to_string(start.pose.position.x) + ", " +
            std::to_string(start.pose.position.y) + ") was outside bounds");
  }

  double orientation_bin = tf2::getYaw(start.pose.orientation) / _angle_bin_size;
  while (orientation_bin < 0.0) {
    orientation_bin += static_cast<float>(_angle_quantizations);
  }
  // This is needed to handle precision issues
  if (orientation_bin >= static_cast<float>(_angle_quantizations)) {
    orientation_bin -= static_cast<float>(_angle_quantizations);
  }
  unsigned int orientation_bin_id = static_cast<unsigned int>(floor(orientation_bin));
  _a_star->setStart(mx, my, orientation_bin_id);

  // Set goal point, in A* bin search coordinates
  if (!costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my)) {
    // throw nav2_core::GoalOutsideMapBounds(
    //         "Goal Coordinates of(" + std::to_string(goal.pose.position.x) + ", " +
    //         std::to_string(goal.pose.position.y) + ") was outside bounds");
    error = std::string("Goal Coordinates of(" + std::to_string(goal.pose.position.x) + ", " +
            std::to_string(goal.pose.position.y) + ") was outside bounds");
  }
  orientation_bin = tf2::getYaw(goal.pose.orientation) / _angle_bin_size;
  while (orientation_bin < 0.0) {
    orientation_bin += static_cast<float>(_angle_quantizations);
  }
  // This is needed to handle precision issues
  if (orientation_bin >= static_cast<float>(_angle_quantizations)) {
    orientation_bin -= static_cast<float>(_angle_quantizations);
  }
  orientation_bin_id = static_cast<unsigned int>(floor(orientation_bin));
  _a_star->setGoal(mx, my, orientation_bin_id);

  // Setup message
  nav_msgs::Path plan;
  //plan.header.stamp = _clock->now();
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
  NodeHybrid::CoordinateVector path;
  int num_iterations = 0;
  std::unique_ptr<std::vector<std::tuple<float, float, float>>> expansions = nullptr;
  if (_debug_visualizations) {
    expansions = std::make_unique<std::vector<std::tuple<float, float, float>>>();
  }
  // Note: All exceptions thrown are handled by the planner server and returned to the action
  if (!_a_star->createPath(
      path, num_iterations,
      _tolerance / static_cast<float>(costmap->getResolution()), expansions.get()))
  {
    if (_debug_visualizations) {
      geometry_msgs::PoseArray msg;
      geometry_msgs::Pose msg_pose;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = _global_frame;
      for (auto & e : *expansions) {
        msg_pose.position.x = std::get<0>(e);
        msg_pose.position.y = std::get<1>(e);
        msg_pose.orientation = getWorldOrientation(std::get<2>(e));
        msg.poses.push_back(msg_pose);
      }
      _expansions_publisher.publish(msg);
    }
    try{
      if (num_iterations < _a_star->getMaxIterations()) {
        //throw NoValidPathCouldBeFound("no valid path found");
        throw std::runtime_error("no valid path found");
      } else {
        //throw navit_core::PlannerTimedOut("exceeded maximum iterations");
        throw std::runtime_error("exceeded maximum iterations");
      }
      }
    catch(const std::runtime_error & e) {
      return plan;
    }
  }

  // Convert to world coordinates
  plan.poses.reserve(path.size());
  for (int i = path.size() - 1; i >= 0; --i) {
    pose.pose = getWorldCoords(path[i].x, path[i].y, costmap);
    pose.pose.orientation = getWorldOrientation(path[i].theta);
    plan.poses.push_back(pose);
  }

  // Publish raw path for debug
  //if (_raw_plan_publisher->get_subscription_count() > 0) {
    _raw_plan_publisher.publish(plan);
  //}

  if (_debug_visualizations) {
    // Publish expansions for debug
    geometry_msgs::PoseArray msg;
    geometry_msgs::Pose msg_pose;
    //msg.header.stamp = _clock->now();
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = _global_frame;
    for (auto & e : *expansions) {
      msg_pose.position.x = std::get<0>(e);
      msg_pose.position.y = std::get<1>(e);
      msg_pose.orientation = getWorldOrientation(std::get<2>(e));
      msg.poses.push_back(msg_pose);
    }
    _expansions_publisher.publish(msg);

    // plot footprint path planned for debug
    //if (_planned_footprints_publisher->get_subscription_count() > 0) {
      auto marker_array = std::make_unique<visualization_msgs::MarkerArray>();
      visualization_msgs::MarkerArray marker_array_ros;
      // std::unique_ptr<visualization_msgs::MarkerArray> marker_array = std::make_unique<visualization_msgs::MarkerArray>();
      // ros::Publisher::publish(*marker_array);
      for (size_t i = 0; i < plan.poses.size(); i++) {
        const std::vector<geometry_msgs::Point> edge =
          transformFootprintToEdges(plan.poses[i].pose, _costmap_ros->getRobotFootprint());
        // marker_array->markers.push_back(createMarker(edge, i, _global_frame, ros::Time::now()));
        marker_array_ros.markers.push_back(createMarker(edge, i, _global_frame, ros::Time::now()));
      }

      // if (marker_array->markers.empty()) {
      //   visualization_msgs::Marker clear_all_marker;
      //   clear_all_marker.action = visualization_msgs::Marker::DELETEALL;
      //   marker_array->markers.push_back(clear_all_marker);
      // }
      // _planned_footprints_publisher.publish(std::move(marker_array));
      _planned_footprints_publisher.publish(marker_array_ros);
  }
  //}

  // Find how much time we have left to do smoothing
  // steady_clock::time_point b = steady_clock::now();
  // duration<double> time_span = duration_cast<duration<double>>(b - a);
  // double time_remaining = _max_planning_time - static_cast<double>(time_span.count());
  ros::Time b = ros::Time::now();
  ros::Duration time_span = b - a;
  double time_remaining = _max_planning_time - time_span.toSec();
  std::cout << "It took " << time_span.toSec() << " seconds with " << num_iterations << " iterations." << std::endl;
#ifdef BENCHMARK_TESTING
  std::cout << "It took " << time_span.count() * 1000 <<
    " milliseconds with " << num_iterations << " iterations." << std::endl;
#endif

  // Smooth plan
  if (_smoother && num_iterations > 1) {
    _smoother->smooth(plan, costmap, time_remaining);
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
// SmacPlannerHybrid::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
// {
//   rcl_interfaces::msg::SetParametersResult result;
//   std::lock_guard<std::mutex> lock_reinit(_mutex);

//   bool reinit_collision_checker = false;
//   bool reinit_a_star = false;
//   bool reinit_downsampler = false;
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
//       } else if (name == _name + ".minimum_turning_radius") {
//         reinit_a_star = true;
//         if (_smoother) {
//           reinit_smoother = true;
//         }

//         if (parameter.as_double() < _costmap->getResolution() * _downsampling_factor) {
//           ROS_ERROR(
//             _logger, "Min turning radius cannot be less than the search grid cell resolution!");
//           result.successful = false;
//         }

//         _minimum_turning_radius_global_coords = static_cast<float>(parameter.as_double());
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
//       } else if (name == _name + ".analytic_expansion_ratio") {
//         reinit_a_star = true;
//         _search_info.analytic_expansion_ratio = static_cast<float>(parameter.as_double());
//       } else if (name == _name + ".analytic_expansion_max_length") {
//         reinit_a_star = true;
//         _search_info.analytic_expansion_max_length =
//           static_cast<float>(parameter.as_double()) / _costmap->getResolution();
//       }
//     } else if (type == ParameterType::PARAMETER_BOOL) {
//       if (name == _name + ".downsample_costmap") {
//         reinit_downsampler = true;
//         _downsample_costmap = parameter.as_bool();
//       } else if (name == _name + ".allow_unknown") {
//         reinit_a_star = true;
//         _allow_unknown = parameter.as_bool();
//       } else if (name == _name + ".cache_obstacle_heuristic") {
//         reinit_a_star = true;
//         _search_info.cache_obstacle_heuristic = parameter.as_bool();
//       } else if (name == _name + ".allow_primitive_interpolation") {
//         _search_info.allow_primitive_interpolation = parameter.as_bool();
//         reinit_a_star = true;
//       } else if (name == _name + ".smooth_path") {
//         if (parameter.as_bool()) {
//           reinit_smoother = true;
//         } else {
//           _smoother.reset();
//         }
//       }
//     } else if (type == ParameterType::PARAMETER_INTEGER) {
//       if (name == _name + ".downsampling_factor") {
//         reinit_a_star = true;
//         reinit_downsampler = true;
//         _downsampling_factor = parameter.as_int();
//       } else if (name == _name + ".max_iterations") {
//         reinit_a_star = true;
//         _max_iterations = parameter.as_int();
//         if (_max_iterations <= 0) {
//           ROS_INFO(
//             _logger, "maximum iteration selected as <= 0, "
//             "disabling maximum iterations.");
//           _max_iterations = std::numeric_limits<int>::max();
//         }
//       } else if (name == _name + ".max_on_approach_iterations") {
//         reinit_a_star = true;
//         _max_on_approach_iterations = parameter.as_int();
//         if (_max_on_approach_iterations <= 0) {
//           ROS_INFO(
//             _logger, "On approach iteration selected as <= 0, "
//             "disabling tolerance and on approach iterations.");
//           _max_on_approach_iterations = std::numeric_limits<int>::max();
//         }
//       } else if (name == _name + ".angle_quantization_bins") {
//         reinit_collision_checker = true;
//         reinit_a_star = true;
//         int angle_quantizations = parameter.as_int();
//         _angle_bin_size = 2.0 * M_PI / angle_quantizations;
//         _angle_quantizations = static_cast<unsigned int>(angle_quantizations);
//       }
//     } else if (type == ParameterType::PARAMETER_STRING) {
//       if (name == _name + ".motion_model_for_search") {
//         reinit_a_star = true;
//         _motion_model = fromString(parameter.as_string());
//         if (_motion_model == MotionModel::UNKNOWN) {
//           ROS_WARN(
//             _logger,
//             "Unable to get MotionModel search type. Given '%s', "
//             "valid options are MOORE, VON_NEUMANN, DUBIN, REEDS_SHEPP.",
//             _motion_model_for_search.c_str());
//         }
//       }
//     }
//   }

//   // Re-init if needed with mutex lock (to avoid re-init while creating a plan)
//   if (reinit_a_star || reinit_downsampler || reinit_collision_checker || reinit_smoother) {
//     // convert to grid coordinates
//     if (!_downsample_costmap) {
//       _downsampling_factor = 1;
//     }
//     _search_info.minimum_turning_radius =
//       _minimum_turning_radius_global_coords / (_costmap->getResolution() * _downsampling_factor);
//     _lookup_table_dim =
//       static_cast<float>(_lookup_table_size) /
//       static_cast<float>(_costmap->getResolution() * _downsampling_factor);

//     // Make sure its a whole number
//     _lookup_table_dim = static_cast<float>(static_cast<int>(_lookup_table_dim));

//     // Make sure its an odd number
//     if (static_cast<int>(_lookup_table_dim) % 2 == 0) {
//       ROS_INFO(
//         _logger,
//         "Even sized heuristic lookup table size set %f, increasing size by 1 to make odd",
//         _lookup_table_dim);
//       _lookup_table_dim += 1.0;
//     }

//     auto node = _node.lock();

//     // Re-Initialize A* template
//     if (reinit_a_star) {
//       _a_star = std::make_unique<AStarAlgorithm<NodeHybrid>>(_motion_model, _search_info);
//       _a_star->initialize(
//         _allow_unknown,
//         _max_iterations,
//         _max_on_approach_iterations,
//         _max_planning_time,
//         _lookup_table_dim,
//         _angle_quantizations);
//     }

//     // Re-Initialize costmap downsampler
//     if (reinit_downsampler) {
//       if (_downsample_costmap && _downsampling_factor > 1) {
//         std::string topic_name = "downsampled_costmap";
//         _costmap_downsampler = std::make_unique<CostmapDownsampler>();
//         _costmap_downsampler->on_configure(
//           node, _global_frame, topic_name, _costmap, _downsampling_factor);
//       }
//     }

//     // Re-Initialize collision checker
//     if (reinit_collision_checker) {
//       _collision_checker = GridCollisionChecker(_costmap, _angle_quantizations, node);
//       _collision_checker.setFootprint(
//         _costmap_ros->getRobotFootprint(),
//         _costmap_ros->getUseRadius(),
//         findCircumscribedCost(_costmap_ros));
//     }

//     // Re-Initialize smoother
//     if (reinit_smoother) {
//       SmootherParams params;
//       params.get(node, _name);
//       _smoother = std::make_unique<Smoother>(params);
//       _smoother->initialize(_minimum_turning_radius_global_coords);
//     }
//   }
//   result.successful = true;
//   return result;
// }

}  // namespace nav2_smac_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_smac_planner::SmacPlannerHybrid, navit_core::GlobalPlanner)
