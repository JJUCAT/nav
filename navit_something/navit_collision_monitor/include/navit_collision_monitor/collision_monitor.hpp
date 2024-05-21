// Copyright (c) 2022 Samsung R&D Institute Russia
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

#ifndef COLLISION_MONITOR__COLLISION_MONITOR_NODE_HPP_
#define COLLISION_MONITOR__COLLISION_MONITOR_NODE_HPP_

#include "navit_collision_monitor/circle.hpp"
#include "navit_collision_monitor/pointcloud.hpp"
#include "navit_collision_monitor/polygon.hpp"
#include "navit_collision_monitor/range.hpp"
#include "navit_collision_monitor/scan.hpp"
#include "navit_collision_monitor/source.hpp"
#include "navit_collision_monitor/types.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <geometry_msgs/Twist.h>
#include <memory>
#include <ros/ros.h>
#include <string>
#include <vector>
#include <std_srvs/SetBool.h>

namespace navit_collision_monitor {

/**
 * @brief Collision Monitor ROS2 node
 */
class CollisionMonitor {
public:
  /**
   * @brief Constructor for the collision_monitor::CollisionMonitor
   * @param options Additional options to control creation of the node.
   */
  explicit CollisionMonitor(ros::NodeHandle &node);
  /**
   * @brief Destructor for the collision_monitor::CollisionMonitor
   */
  ~CollisionMonitor();

protected:
  /**
   * @brief: Initializes and obtains ROS-parameters, creates main
   subscribers
   * and publishers, creates polygons and data sources objects
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  bool on_configure();
  /**
   * @brief: Activates LifecyclePublishers, polygons and main processor,
   creates
   * bond connection
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  bool on_activate();
  /**
   * @brief: Deactivates LifecyclePublishers, polygons and main processor,
   * destroys bond connection
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  bool on_deactivate();
  /**
   * @brief: Resets all subscribers/publishers, polygons/data sources arrays
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  bool on_cleanup();
  /**
   * @brief Called in shutdown state
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  bool on_shutdown();

protected:
  /**
   * @brief Callback for input cmd_vel
   * @param msg Input cmd_vel message
   */
  void cmdVelInCallback(geometry_msgs::Twist::ConstPtr msg);
  /**
   * @brief Publishes output cmd_vel. If robot was stopped more than
   * stop_pub_timeout_ seconds, quit to publish 0-velocity.
   * @param robot_action Robot action to publish
   */
  void publishVelocity(const Action &robot_action);

  /**
   * @brief Supporting routine obtaining all ROS-parameters
   * @param cmd_vel_in_topic Output name of cmd_vel_in topic
   * @param cmd_vel_out_topic Output name of cmd_vel_out topic
   * is required.
   * @return True if all parameters were obtained or false in failure case
   */
  bool getParameters(std::string &cmd_vel_in_topic,
                     std::string &cmd_vel_out_topic, std::string &state_topic);
  /**
   * @brief Supporting routine creating and configuring all polygons
   * @param base_frame_id Robot base frame ID
   * @param transform_tolerance Transform tolerance
   * @return True if all polygons were configured successfully or false in
   * failure case
   */
  bool configurePolygons(const std::string &base_frame_id,
                         const ros::Duration &transform_tolerance);
  /**
   * @brief Supporting routine creating and configuring all data sources
   * @param base_frame_id Robot base frame ID
   * @param odom_frame_id Odometry frame ID. Used as global frame to get
   * source->base time interpolated transform.
   * @param transform_tolerance Transform tolerance
   * @param source_timeout Maximum time interval in which data is considered
   * valid
   * @param base_shift_correction Whether to correct source data towards to base
   * frame movement, considering the difference between current time and latest
   * source time
   * @return True if all sources were configured successfully or false in
   * failure case
   */
  bool configureSources(const std::string &base_frame_id,
                        const std::string &odom_frame_id,
                        const ros::Duration &transform_tolerance,
                        const ros::Duration &source_timeout,
                        const bool base_shift_correction);

  /**
   * @brief Main processing routine
   * @param cmd_vel_in Input desired robot velocity
   */
  void process(const Velocity &cmd_vel_in);

  /**
   * @brief Processes the polygon of STOP, SLOWDOWN and LIMIT action type
   * @param polygon Polygon to process
   * @param collision_points Array of 2D obstacle points
   * @param velocity Desired robot velocity
   * @param robot_action Output processed robot action
   * @return True if returned action is caused by current polygon, otherwise
   * false
   */
  bool processStopSlowdownLimit(const std::shared_ptr<Polygon> polygon,
                                const std::vector<Point> &collision_points,
                                const Velocity &velocity,
                                Action &robot_action) const;

  /**
   * @brief Processes APPROACH action type
   * @param polygon Polygon to process
   * @param collision_points Array of 2D obstacle points
   * @param velocity Desired robot velocity
   * @param robot_action Output processed robot action
   * @return True if returned action is caused by current polygon, otherwise
   * false
   */
  bool processApproach(const std::shared_ptr<Polygon> polygon,
                       const std::vector<Point> &collision_points,
                       const Velocity &velocity, Action &robot_action) const;

  /**
   * @brief Log and publish current robot action and polygon
   * @param robot_action Robot action to notify
   * @param action_polygon Pointer to a polygon causing a selected action
   */
  void notifyActionState(const Action &robot_action,
                         const std::shared_ptr<Polygon> action_polygon) const;

  /**
   * @brief Polygons publishing routine. Made for visualization.
   */
  void publishPolygons() const;

  // ----- Variables -----

  /// @brief TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  /// @brief TF listener
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /// @brief Polygons array
  std::vector<std::shared_ptr<Polygon>> polygons_;

  /// @brief Data sources array
  std::vector<std::shared_ptr<Source>> sources_;

  // Input/output speed controls
  /// @beirf Input cmd_vel subscriber
  ros::Subscriber cmd_vel_in_sub_;
  /// @brief Output cmd_vel publisher
  ros::Publisher cmd_vel_out_pub_;

  /// @brief CollisionMonitor state publisher
  ros::Publisher state_pub_;

  /// @brief Whether main routine is active
  bool process_active_;

  /// @brief Previous robot action
  Action robot_action_prev_;
  /// @brief Latest timestamp when robot has 0-velocity
  ros::Time stop_stamp_;
  /// @brief Timeout after which 0-velocity ceases to be published
  ros::Duration stop_pub_timeout_;

  double min_linear_vel_ = 0.0;

  double min_angular_vel_ = 0.0;

  ros::NodeHandle node_;
  ros::ServiceServer ctrl_service_;
  bool onCtrl(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
}; // class CollisionMonitor

} // namespace navit_collision_monitor

#endif // COLLISION_MONITOR__COLLISION_MONITOR_NODE_HPP_
