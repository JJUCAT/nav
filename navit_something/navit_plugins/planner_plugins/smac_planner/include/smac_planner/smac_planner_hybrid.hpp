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

#ifndef NAV2_SMAC_PLANNER__SMAC_PLANNER_HYBRID_HPP_
#define NAV2_SMAC_PLANNER__SMAC_PLANNER_HYBRID_HPP_

#include <memory>
#include <vector>
#include <string>

#include "smac_planner/a_star.hpp"
#include "smac_planner/smoother.hpp"
#include "smac_planner/utils.hpp"
#include "smac_planner/costmap_downsampler.hpp"
//#include "nav_msgs/msg/occupancy_grid.hpp"
#include "navit_core/base_global_planner.h"
#include "nav_msgs/Path.h"
#include "navit_costmap/costmap_2d_ros.h"
#include "navit_costmap/costmap_2d.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
// #include "navit_util/lifecycle_node.hpp"
// #include "navit_util/node_utils.hpp"
#include "tf2/utils.h"

namespace nav2_smac_planner
{

class SmacPlannerHybrid : public navit_core::GlobalPlanner
{
public:
  /**
   * @brief constructor
   */
  SmacPlannerHybrid();

  /**
   * @brief destructor
   */
  ~SmacPlannerHybrid();

  /**
   * @brief Configuring plugin
   * @param parent Lifecycle node pointer
   * @param name Name of plugin map
   * @param tf Shared ptr of TF2 buffer
   * @param costmap_ros Costmap2DROS object
   */
  void initialize(
    //const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name,
    const std::shared_ptr<navit_costmap::Costmap2DROS> & costmap_ros) override;

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
    const geometry_msgs::PoseStamped & goal) ;

protected:
  /**
   * @brief Callback executed when a paramter change is detected
   * @param parameters list of changed parameters
   */
  // rcl_interfaces::msg::SetParametersResult
  // dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  std::unique_ptr<AStarAlgorithm<NodeHybrid>> _a_star;
  GridCollisionChecker _collision_checker;
  std::unique_ptr<Smoother> _smoother;
  //rclcpp::Clock::SharedPtr _clock;
  //rclcpp::Logger _logger{rclcpp::get_logger("SmacPlannerHybrid")};
  navit_costmap::Costmap2D * _costmap;
  std::shared_ptr<navit_costmap::Costmap2DROS> _costmap_ros;
  std::unique_ptr<CostmapDownsampler> _costmap_downsampler;
  std::string _global_frame, _name;
  float _lookup_table_dim;
  double _tolerance;
  bool _downsample_costmap;
  int _downsampling_factor;
  double _angle_bin_size;
  unsigned int _angle_quantizations;
  bool _allow_unknown;
  int _max_iterations;
  int _max_on_approach_iterations;
  SearchInfo _search_info;
  double _max_planning_time;
  double _lookup_table_size;
  double _minimum_turning_radius_global_coords;
  bool _debug_visualizations;
  std::string _motion_model_for_search;
  MotionModel _motion_model;
  ros::Publisher _raw_plan_publisher;
  ros::Publisher _planned_footprints_publisher;
  ros::Publisher _expansions_publisher;
  //rclcpp_lifecycle::LifecyclePublisher<nav_msgs::Path>::SharedPtr _raw_plan_publisher;
  //rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
  //   _planned_footprints_publisher;
  // rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::PoseArray>::SharedPtr
  //   _expansions_publisher;
  std::mutex _mutex;
  boost::shared_ptr<ros::NodeHandle> _node;
  //rclcpp_lifecycle::LifecycleNode::WeakPtr _node;

  // Dynamic parameters handler
  // rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr _dyn_params_handler;
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__SMAC_PLANNER_HYBRID_HPP_
