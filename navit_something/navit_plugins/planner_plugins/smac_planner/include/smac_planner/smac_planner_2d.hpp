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

#ifndef NAV2_SMAC_PLANNER__SMAC_PLANNER_2D_HPP_
#define NAV2_SMAC_PLANNER__SMAC_PLANNER_2D_HPP_

#include <memory>
#include <vector>
#include <string>
#include <mutex>

#include "smac_planner/a_star.hpp"
#include "smac_planner/smoother.hpp"
#include "smac_planner/utils.hpp"
#include "smac_planner/costmap_downsampler.hpp"
//#include "navit_msgs/occupancy_grid.h"
#include "navit_core/base_global_planner.h"
#include "nav_msgs/Path.h"
#include "navit_costmap/costmap_2d_ros.h"
#include "navit_costmap/costmap_2d.h"
#include "geometry_msgs/PoseStamped.h"
//#include "navit_util/lifecycle_node.hpp"
//#include "navit_util/node_utils.hpp"
#include "tf2/utils.h"
//#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <ros/ros.h>

namespace nav2_smac_planner
{

class SmacPlanner2D : public navit_core::GlobalPlanner
{
public:
  /**
   * @brief constructor
   */
  SmacPlanner2D();

  /**
   * @brief destructor
   */
  ~SmacPlanner2D();

  /**
   * @brief Configuring plugin
   * @param parent Lifecycle node pointer
   * @param name Name of plugin map
   * @param tf Shared ptr of TF2 buffer
   * @param costmap_ros Costmap2DROS object
   */
  void initialize(
    //const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string& name, 
    const std::shared_ptr<navit_costmap::Costmap2DROS>& costmap_ros) override;

  /**
   * @brief Cleanup lifecycle node
   */
  //void cleanup() override;

  /**
   * @brief Activate lifecycle node
   */
  //void activate() override;

  /**
   * @brief Deactivate lifecycle node
   */
  //void deactivate() override;

  /**
   * @brief Creating a plan from start and goal poses
   * @param start Start pose
   * @param goal Goal pose
   * @return nav2_msgs::Path of the generated path
   */

  nav_msgs::Path makePlan(
    const geometry_msgs::PoseStamped & start,
    const geometry_msgs::PoseStamped & goal) override
  {
      return createPlan(start, goal);
  }

  nav_msgs::Path createPlan(
    const geometry_msgs::PoseStamped & start,
    const geometry_msgs::PoseStamped & goal); //override;

protected:
  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  // rcl_interfaces::msg::SetParametersResult
  // dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
  inline geometry_msgs::Quaternion orientationAroundZAxis(double angle)
  {
    tf2::Quaternion q;
    q.setRPY(0, 0, angle);  // void returning function
    return tf2::toMsg(q);
  }
  std::unique_ptr<AStarAlgorithm<Node2D>> _a_star;
  GridCollisionChecker _collision_checker;
  std::unique_ptr<Smoother> _smoother;
  navit_costmap::Costmap2D * _costmap;
  std::unique_ptr<CostmapDownsampler> _costmap_downsampler;
  //rclcpp::Clock::SharedPtr _clock;
  //rclcpp::Logger _logger{rclcpp::get_logger("SmacPlanner2D")};
  std::string _global_frame, _name;
  float _tolerance;
  int _downsampling_factor;
  bool _downsample_costmap;
  //rclcpp_lifecycle::LifecyclePublisher<nav_msgs::Path>::SharedPtr _raw_plan_publisher;
  ros::Publisher _raw_plan_publisher;
  double _max_planning_time;
  bool _allow_unknown;
  int _max_iterations;
  int _max_on_approach_iterations;
  bool _use_final_approach_orientation;
  SearchInfo _search_info;
  std::string _motion_model_for_search;
  MotionModel _motion_model;
  std::mutex _mutex;
  boost::shared_ptr<ros::NodeHandle> _node;
  //rclcpp_lifecycle::LifecycleNode::WeakPtr _node;

  // Dynamic parameters handler
  //roscpp::NodeParameters::CallbackHandle::UniquePtr _dyn_params_handler;
  // rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr _dyn_params_handler;
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__SMAC_PLANNER_2D_HPP_
