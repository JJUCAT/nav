// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
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
// limitations under the License.

#ifndef NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__REGULATED_PURE_PURSUIT_CONTROLLER_HPP_
#define NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__REGULATED_PURE_PURSUIT_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>

//#include "nav2_core/controller.hpp"
//#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
//#include "nav2_util/odometry_utils.hpp"
//#include "nav2_util/geometry_utils.hpp"
//#include "geometry_msgs/msg/pose2_d.hpp"

#include <navit_core/base_controller.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

#include <angles/angles.h>
#include <tf2/utils.h>
namespace nav2_regulated_pure_pursuit_controller
{  
   /**
   * copied from nav2_utils
   * @brief Get the L2 distance between 2 geometry_msgs::Poses
   * @param pos1 First pose
   * @param pos1 Second pose
   * @return double L2 distance
   */
  inline double euclidean_distance(
    const geometry_msgs::Pose & pos1,
    const geometry_msgs::Pose & pos2)
  {
    double dx = pos1.position.x - pos2.position.x;
    double dy = pos1.position.y - pos2.position.y;
    double dz = pos1.position.z - pos2.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }
  
  inline double euclidean_distance(
  const geometry_msgs::PoseStamped & pos1,
  const geometry_msgs::PoseStamped & pos2)
  {
    return euclidean_distance(pos1.pose, pos2.pose);
  }
    
  /**
   * copied from nav2_utils
   * Find element in iterator with the minimum calculated value
   */
  template<typename Iter, typename Getter>
  inline Iter min_by(Iter begin, Iter end, Getter getCompareVal)
  {
    if (begin == end) {
      return end;
    }
    auto lowest = getCompareVal(*begin);
    Iter lowest_it = begin;
    for (Iter it = ++begin; it != end; ++it) {
      auto comp = getCompareVal(*it);
      if (comp < lowest) {
        lowest = comp;
        lowest_it = it;
      }
    }
    return lowest_it;
  }

/**
 * @class nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
 * @brief Regulated pure pursuit controller plugin
 */
class RegulatedPurePursuitController : public navit_core::Controller
{
public:
  /**
   * @brief Constructor for nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
   */
  RegulatedPurePursuitController() = default;

  /**
   * @brief Destrructor for nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
   */
  ~RegulatedPurePursuitController() override = default;

  // navit_core interface
  void initialize( const std::string& name,
                   const std::shared_ptr<tf2_ros::Buffer>& tf,
                   const std::shared_ptr<navit_costmap::Costmap2DROS>& costmap_ros) override;
  /**
   * @brief Configure controller state machine
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
    const std::shared_ptr<navit_costmap::Costmap2DROS> & costmap_ros) override;
   */

  /**
   * @brief Cleanup controller state machine
  void cleanup() override;
   */

  /**
   * @brief Activate controller state machine
  void activate() override;
   */

  /**
   * @brief Deactivate controller state machine
  void deactivate() override;
   */

  /**
   * @brief Compute the best command given the current pose and velocity, with possible debug information
   *
   * Same as above computeVelocityCommands, but with debug results.
   * If the results pointer is not null, additional information about the twists
   * evaluated will be in results after the call.
   *
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param goal_checker   Ptr to the goal checker for this task in case useful in computing commands
   * @return          Best command
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker ) override;
   */
  geometry_msgs::Twist
  computeVelocityCommands(const geometry_msgs::PoseStamped& current_pose,
                          const geometry_msgs::Twist& current_vel) override;

  /**
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
  void setPlan(const nav_msgs::msg::Path & path) override;
   */
  bool setPlan(const nav_msgs::Path& path) override;

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage);
  bool setSpeedLimit(const double& speed_limit) override
  {
      //set speed limit in percentage
    setSpeedLimit(speed_limit, true);
    return true;
  }

  bool isGoalReached() override;

protected:

/**
   * @brief Transforms global plan into same frame as pose, clips far away poses and possibly prunes passed poses
   * @param pose pose to transform
   * @return Path in new frame
  nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);
   */
  nav_msgs::Path transformGlobalPlan(const geometry_msgs::PoseStamped& pose);

  /**
   * @brief Transform a pose to another frame.
   * @param frame Frame ID to transform to
   * @param in_pose Pose input to transform
   * @param out_pose transformed output
   * @return bool if successful
  bool transformPose(
    const std::string frame,
    const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose) const;
   */
  bool transformPose(
    const std::string frame,
    const geometry_msgs::PoseStamped & in_pose,
    geometry_msgs::PoseStamped & out_pose) const;

  /**
   * @brief Get lookahead distance
   * @param cmd the current speed to use to compute lookahead point
   * @return lookahead distance
  double getLookAheadDistance(const geometry_msgs::msg::Twist &);
   */
  double getLookAheadDistance(const geometry_msgs::Twist&);

  /**
   * @brief Compute the steering angle for front wheel
   * @param wheel_base the length of wheel_base (in meters)
   * @param kappa curvature of trajectory between the rear-wheel and lookahead distance
   * @return steering angle in rad
  double convertCurvatureToSteeringAngle(const double& linear_vel, const double& ang_vel, const double& wheel_base, const double& kappa)
  */
 double convertCurvatureToSteeringAngle(const double& linear_vel, const double& ang_vel, const double& wheel_base, const double& kappa);

  /**
   * @brief Creates a PointStamped message for visualization
   * @param carrot_pose Input carrot point as a PoseStamped
   * @return CarrotMsg a carrot point marker, PointStamped
  std::unique_ptr<geometry_msgs::msg::PointStamped> createCarrotMsg(
    const geometry_msgs::msg::PoseStamped & carrot_pose);
   */
  geometry_msgs::PointStamped createCarrotMsg(
          const geometry_msgs::PoseStamped& carrot_pose);

  /**
   * @brief Whether robot should rotate to rough path heading
   * @param carrot_pose current lookahead point
   * @param angle_to_path Angle of robot output relatie to carrot marker
   * @return Whether should rotate to path heading
  bool shouldRotateToPath(
    const geometry_msgs::msg::PoseStamped & carrot_pose, double & angle_to_path);
   */
  bool shouldRotateToPath(
    const geometry_msgs::PoseStamped & carrot_pose, double & angle_to_path);

  /**
   * @brief Whether robot should rotate to final goal orientation
   * @param carrot_pose current lookahead point
   * @return Whether should rotate to goal heading
  bool shouldRotateToGoalHeading(const geometry_msgs::msg::PoseStamped & carrot_pose);
   */
  bool shouldRotateToGoalHeading(const geometry_msgs::PoseStamped & carrot_pose);

  /**
   * @brief Create a smooth and kinematically smoothed rotation command
   * @param linear_vel linear velocity
   * @param angular_vel angular velocity
   * @param angle_to_path Angle of robot output relatie to carrot marker
   * @param curr_speed the current robot speed
  void rotateToHeading(
    double & linear_vel, double & angular_vel,
    const double & angle_to_path, const geometry_msgs::msg::Twist & curr_speed);
   */
  void rotateToHeading(
    double & linear_vel, double & angular_vel,
    const double & angle_to_path, const geometry_msgs::Twist & curr_speed);

  /**
   * @brief Whether collision is imminent
   * @param robot_pose Pose of robot
   * @param carrot_pose Pose of carrot
   * @param linear_vel linear velocity to forward project
   * @param angular_vel angular velocity to forward project
   * @return Whether collision is imminent
  bool isCollisionImminent(
    const geometry_msgs::msg::PoseStamped &,
    const double &, const double &);
   */
  bool isCollisionImminent(
    const geometry_msgs::PoseStamped &,
    const double &, const double &);

  /**
   * @brief Whether point is in collision
   * @param x Pose of pose x
   * @param y Pose of pose y
   * @return Whether in collision
   */
  bool inCollision(const double & x, const double & y);

  /**
   * @brief Cost at a point
   * @param x Pose of pose x
   * @param y Pose of pose y
   * @return Cost of pose in costmap
   */
  double costAtPose(const double & x, const double & y);

  /**
   * @brief apply regulation constraints to the system
   * @param linear_vel robot command linear velocity input
   * @param dist_error error in the carrot distance and lookahead distance
   * @param lookahead_dist optimal lookahead distance
   * @param curvature curvature of path
   * @param speed speed of robot
   * @param pose_cost cost at this pose
   * @param curr_pose current pose of robot
  void applyConstraints(
    const double & dist_error, const double & lookahead_dist,
    const double & curvature, const geometry_msgs::msg::Twist & speed,
    const double & pose_cost, double & linear_vel);
   */
  void applyConstraints(
    const double & dist_error, const double & lookahead_dist,
    const double & curvature, const geometry_msgs::Twist & speed,
    const double & pose_cost, double & linear_vel);

  /**
   * @brief Get lookahead point
   * @param lookahead_dist Optimal lookahead distance
   * @param path Current global path
   * @return Lookahead point
  geometry_msgs::msg::PoseStamped getLookAheadPoint(const double &, const nav_msgs::msg::Path &);
   */
  geometry_msgs::PoseStamped getLookAheadPoint(const double &, const nav_msgs::Path &);

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<navit_costmap::Costmap2DROS> costmap_ros_;
  navit_costmap::Costmap2D * costmap_;

  double desired_linear_vel_, base_desired_linear_vel_;
  double lookahead_dist_;
  double rotate_to_heading_angular_vel_;
  double max_lookahead_dist_;
  double min_lookahead_dist_;
  double lookahead_time_;
  double max_linear_accel_;
  double max_linear_decel_;
  bool use_velocity_scaled_lookahead_dist_;
  ros::Duration transform_tolerance_;
  bool use_approach_vel_scaling_;
  double min_approach_linear_velocity_;
  double control_duration_;
  double control_frequency_;
  double max_allowed_time_to_collision_;
  bool use_regulated_linear_velocity_scaling_;
  bool use_cost_regulated_linear_velocity_scaling_;
  double cost_scaling_dist_;
  double cost_scaling_gain_;
  double inflation_cost_scaling_factor_;
  double regulated_linear_scaling_min_radius_;
  double regulated_linear_scaling_min_speed_;
  bool use_rotate_to_heading_;
  double max_angular_accel_;
  double rotate_to_heading_min_angle_;
  double goal_dist_tol_;
  double xy_goal_tolerance_sq_, yaw_goal_tolerance_;
  bool cmd_angle_instead_rotvel_;
  double wheel_base_;
  bool use_angle_limit_;
  double max_steering_angle_;
  double speed_up_acc_;
  bool follow_path_;

  //nav_msgs::msg::Path global_plan_;
  //std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_path_pub_;
  //std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>>
  //carrot_pub_;
  //std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> carrot_arc_pub_;
  nav_msgs::Path global_plan_;
  ros::Publisher global_path_pub_;
  ros::Publisher carrot_pub_;
  ros::Publisher carrot_arc_pub_;

  geometry_msgs::PoseStamped goal_pose_, current_pose_;
};

}  // namespace nav2_regulated_pure_pursuit_controller

#endif  // NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__REGULATED_PURE_PURSUIT_CONTROLLER_HPP_
