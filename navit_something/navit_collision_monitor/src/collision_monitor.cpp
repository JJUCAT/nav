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

#include "navit_collision_monitor/collision_monitor.hpp"
#include "navit_collision_monitor/CollisionMonitorState.h"
#include "navit_collision_monitor/kinematics.hpp"
#include "navit_collision_monitor/utils.hpp"
#include <exception>
#include <functional>

namespace navit_collision_monitor
{

CollisionMonitor::CollisionMonitor(ros::NodeHandle& node)
  : process_active_(false), robot_action_prev_{ DO_NOTHING, { -1.0, -1.0, -1.0 }, "" }, stop_pub_timeout_(1.0, 0.0), node_(node)
{
  on_configure();
  on_activate();
}

CollisionMonitor::~CollisionMonitor()
{
  on_deactivate();
  on_shutdown();
  on_cleanup();
  polygons_.clear();
  sources_.clear();
}

bool CollisionMonitor::on_configure()
{
  ROS_INFO("Configuring");

  // Transform buffer and listener initialization
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  std::string cmd_vel_in_topic;
  std::string cmd_vel_out_topic;
  std::string state_topic;

  // Obtaining ROS parameters
  if (!getParameters(cmd_vel_in_topic, cmd_vel_out_topic, state_topic))
  {
    return false;
  }
  ROS_INFO("Parameters obtained.");

  cmd_vel_in_sub_ = node_.subscribe<geometry_msgs::Twist>(
      cmd_vel_in_topic, 1, boost::bind(&CollisionMonitor::cmdVelInCallback, this, boost::placeholders::_1));
  ROS_INFO("Cmd Vel In Call back.");
  cmd_vel_out_pub_ = node_.advertise<geometry_msgs::Twist>(cmd_vel_out_topic, 1);
  ROS_INFO("Cmd Vel Out Pub.");
  if (!state_topic.empty())
  {
    state_pub_ = node_.advertise<navit_collision_monitor::CollisionMonitorState>(state_topic, 1);
  }
  ctrl_service_ = node_.advertiseService("ctrl", &CollisionMonitor::onCtrl, this);

  return true;
}

bool CollisionMonitor::on_activate()
{
  ROS_INFO("Activating");

  // Activating polygons
  for (std::shared_ptr<Polygon> polygon : polygons_)
  {
    polygon->activate();
  }

  // Since polygons are being published when cmd_vel_in appears,
  // we need to publish polygons first time to display them at startup
  publishPolygons();

  // Activating main worker
  process_active_ = true;

  return true;
}

bool CollisionMonitor::on_deactivate()
{
  ROS_INFO("Deactivating");

  // Deactivating main worker
  process_active_ = false;

  // Reset action type to default after worker deactivating
  robot_action_prev_ = { DO_NOTHING, { -1.0, -1.0, -1.0 }, "" };

  // Deactivating polygons
  for (std::shared_ptr<Polygon> polygon : polygons_)
  {
    polygon->deactivate();
  }
  return true;
}

bool CollisionMonitor::on_cleanup()
{
  ROS_INFO("Cleaning up");

  cmd_vel_in_sub_.shutdown();
  cmd_vel_out_pub_.shutdown();
  state_pub_.shutdown();

  polygons_.clear();
  sources_.clear();

  tf_listener_.reset();
  tf_buffer_.reset();

  return true;
}

bool CollisionMonitor::on_shutdown()
{
  ROS_INFO("Shutting down");

  return true;
}

bool CollisionMonitor::onCtrl(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  bool control = req.data;
  // 在这里进行你想要的处理操作
  ROS_INFO_STREAM("Received control: " << control);

  bool succeed = control ? on_activate() : on_deactivate();
  res.success = succeed;
  res.message = "Control received";
  return succeed;
}

void CollisionMonitor::cmdVelInCallback(geometry_msgs::Twist::ConstPtr msg)
{
  // If message contains NaN or Inf, ignore
  if (!collision_monitor_utils::validateTwist(*msg))
  {
    ROS_ERROR("Velocity message contains NaNs or Infs! Ignoring as invalid!");
    return;
  }
  process({ msg->linear.x, msg->linear.y, msg->angular.z });
}

void CollisionMonitor::publishVelocity(const Action& robot_action)
{
  if (robot_action.req_vel.isZero())
  {
    if (!robot_action_prev_.req_vel.isZero())
    {
      // Robot just stopped: saving stop timestamp and continue
      stop_stamp_ = ros::Time::now();
    }
    else if (ros::Time::now() - stop_stamp_ > stop_pub_timeout_)
    {
      // More than stop_pub_timeout_ passed after robot has been stopped.
      // Cease publishing output cmd_vel.
      return;
    }
  }

  geometry_msgs::Twist::Ptr cmd_vel_out_msg(new geometry_msgs::Twist());
  cmd_vel_out_msg->linear.x = robot_action.req_vel.x;
  cmd_vel_out_msg->linear.y = robot_action.req_vel.y;
  cmd_vel_out_msg->angular.z = robot_action.req_vel.tw;
  // linear.z, angular.x and angular.y will remain 0.0

  cmd_vel_out_pub_.publish(cmd_vel_out_msg);
}

bool CollisionMonitor::getParameters(std::string& cmd_vel_in_topic, std::string& cmd_vel_out_topic,
                                     std::string& state_topic)
{
  std::string base_frame_id, odom_frame_id;
  ros::Duration transform_tolerance;
  ros::Duration source_timeout(2.0, 0.0);

  node_.param("cmd_vel_in_topic", cmd_vel_in_topic, std::string("cmd_vel_raw"));
  // cmd_vel_out_topic
  node_.param("cmd_vel_out_topic", cmd_vel_out_topic, std::string("cmd_vel"));
  // state_topic
  node_.param("state_topic", state_topic, std::string(""));
  // base_frame_id
  node_.param("base_frame_id", base_frame_id, std::string("base_footprint"));
  // odom_frame_id
  node_.param("odom_frame_id", odom_frame_id, std::string("odom"));
  // transform_tolerance
  double temp_transform_tolerance = 0.5;
  node_.param("transform_tolerance", temp_transform_tolerance, 0.5);
  transform_tolerance = ros::Duration(temp_transform_tolerance);
  // source_timeout
  double temp_source_timeout = 2.0;
  node_.param("source_timeout", temp_source_timeout, temp_source_timeout);
  source_timeout = ros::Duration(temp_source_timeout);
  // base_shift_correction
  bool base_shift_correction = true;
  node_.param("base_shift_correction", base_shift_correction, true);
  // stop_pub_timeout_
  double temp_stop_pub_timeout = 2.0;
  node_.param("stop_pub_timeout", temp_stop_pub_timeout, temp_stop_pub_timeout);
  stop_pub_timeout_ = ros::Duration(temp_stop_pub_timeout);
  // min_linear_vel (m/s)
  node_.param("min_linear_vel", min_linear_vel_, 0.2);

  ROS_INFO("min_linear_vel is %f", min_linear_vel_);
  // min_angular_vel (rad/s)
  node_.param("min_angular_vel", min_angular_vel_, 0.0);
  // max_linear_vel (m/s)
  ROS_INFO("min angular vel is %f", min_angular_vel_);
  // configurePolygons
  if (!configurePolygons(base_frame_id, transform_tolerance))
  {
    ROS_ERROR("[getParameters]: configurePolygons failed");
    return false;
  }
  // configureSources
  if (!configureSources(base_frame_id, odom_frame_id, transform_tolerance, source_timeout, base_shift_correction))
  {
    ROS_ERROR("[getParameters]: configureSources failed");
    return false;
  }

  return true;
}

bool CollisionMonitor::configurePolygons(const std::string& base_frame_id, const ros::Duration& transform_tolerance)
{
  try
  {
    // Leave it to be not initialized: to intentionally cause an error if it
    // will not set
    std::vector<std::string> polygon_names;
    node_.param("polygons", polygon_names, polygon_names);
    // 给polygon_names赋值node_的namespace下的polygons参数值
    for (auto& polygon_name : polygon_names)
    {
      polygon_name = node_.getNamespace() + "/" + polygon_name;
    }

    if (polygon_names.empty())
    {
      ROS_ERROR("[configurePolygons]: polygon_names is empty");
      return false;
    }

    for (std::string polygon_name : polygon_names)
    {
      // Leave it not initialized: the will cause an error if it will not set
      std::string polygon_type;

      node_.param(polygon_name + "/type", polygon_type, polygon_type);

      if (polygon_type == "polygon")
      {
        polygons_.push_back(
            std::make_shared<Polygon>(node_, polygon_name, tf_buffer_, base_frame_id, transform_tolerance));
      }
      else if (polygon_type == "circle")
      {
        polygons_.push_back(
            std::make_shared<Circle>(node_, polygon_name, tf_buffer_, base_frame_id, transform_tolerance));
      }
      else
      {  // Error if something else
        ROS_ERROR("[%s]: Unknown polygon type: %s", polygon_name.c_str(), polygon_type.c_str());
        return false;
      }

      // Configure last added polygon
      if (!polygons_.back()->configure())
      {
        ROS_ERROR("[%s]: polygon configure failed with type: %s", polygon_name.c_str(), polygon_type.c_str());
        return false;
      }

      // warn if the added polygon's action_type_ is not different than "none"
      // auto action_type = polygons_.back()->getActionType();
      // std::cout << "action_type is" << action_type << std::endl;
      // if (action_type != DO_NOTHING)
      // {
      //   ROS_ERROR(
      //       "[%s]: The action_type of the polygon is different than "
      //       "\"none\" which is "
      //       "not supported in the collision detector.",
      //       polygon_name.c_str());
      //   return false;
      // }
    }
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR("Error while getting parameters: %s", ex.what());
    return false;
  }

  return true;
}

bool CollisionMonitor::configureSources(const std::string& base_frame_id, const std::string& odom_frame_id,
                                        const ros::Duration& transform_tolerance, const ros::Duration& source_timeout,
                                        const bool base_shift_correction)
{
  try
  {
    // Leave it to be not initialized to intentionally cause an error if it will
    // not set
    std::vector<std::string> source_names;
    node_.getParam("observation_sources", source_names);

    if (source_names.empty())
    {
      ROS_ERROR("[configureSources]: observation_sources is empty");
      return false;
    }

    for (std::string source_name : source_names)
    {
      std::string source_type = "scan";
      node_.param(source_name + "/type", source_type, source_type);
      if (source_type == "scan")
      {
        std::shared_ptr<Scan> s = std::make_shared<Scan>(node_, source_name, tf_buffer_, base_frame_id, odom_frame_id,
                                                         transform_tolerance, source_timeout, base_shift_correction);

        s->configure();

        sources_.push_back(s);
      }
      else if (source_type == "pointcloud")
      {
        std::shared_ptr<PointCloud> p =
            std::make_shared<PointCloud>(node_, source_name, tf_buffer_, base_frame_id, odom_frame_id,
                                         transform_tolerance, source_timeout, base_shift_correction);

        p->configure();

        sources_.push_back(p);
      }
      else if (source_type == "range")
      {
        std::shared_ptr<Range> r = std::make_shared<Range>(node_, source_name, tf_buffer_, base_frame_id, odom_frame_id,
                                                           transform_tolerance, source_timeout, base_shift_correction);

        r->configure();

        sources_.push_back(r);
      }
      else
      {  // Error if something else
        ROS_ERROR("[%s]: Unknown source type: %s", source_name.c_str(), source_type.c_str());
        return false;
      }
      ROS_INFO("[%s]: config source succeed with type: %s", source_name.c_str(), source_type.c_str());
    }
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR("Error while getting parameters: %s", ex.what());
    return false;
  }

  return true;
}

void CollisionMonitor::process(const Velocity& cmd_vel_in)
{
  // Current timestamp for all inner routines prolongation
  ros::Time curr_time = ros::Time::now();

  // Do nothing if main worker in non-active state
  if (!process_active_)
  {
    return;
  }

  // Points array collected from different data sources in a robot base frame
  std::vector<Point> collision_points;

  // Fill collision_points array from different data sources
  for (std::shared_ptr<Source> source : sources_)
  {
    source->getData(curr_time, collision_points);
  }

  // By default - there is no action
  Action robot_action{ DO_NOTHING, cmd_vel_in, "" };
  // Polygon causing robot action (if any)
  std::shared_ptr<Polygon> action_polygon;

  for (std::shared_ptr<Polygon> polygon : polygons_)
  {
    if (robot_action.action_type == STOP)
    {
      // If robot already should stop, do nothing
      break;
    }

    // Update polygon coordinates
    polygon->updatePolygon();

    const ActionType at = polygon->getActionType();
    if (at == STOP || at == SLOWDOWN || at == LIMIT)
    {
      // Process STOP/SLOWDOWN for the selected polygon
      if (processStopSlowdownLimit(polygon, collision_points, cmd_vel_in, robot_action))
      {
        action_polygon = polygon;
      }
    }
    else if (at == APPROACH)
    {
      // Process APPROACH for the selected polygon
      if (processApproach(polygon, collision_points, cmd_vel_in, robot_action))
      {
        action_polygon = polygon;
      }
    }
  }

  if (robot_action.polygon_name != robot_action_prev_.polygon_name)
  {
    // Report changed robot behavior
    notifyActionState(robot_action, action_polygon);
  }

  // Publish requred robot velocity
  publishVelocity(robot_action);

  // Publish polygons for better visualization
  publishPolygons();

  robot_action_prev_ = robot_action;
}

bool CollisionMonitor::processStopSlowdownLimit(const std::shared_ptr<Polygon> polygon,
                                                const std::vector<Point>& collision_points, const Velocity& velocity,
                                                Action& robot_action) const
{
  if (!polygon->isShapeSet())
  {
    return false;
  }

  if (polygon->getPointsInside(collision_points) >= polygon->getMinPoints())
  {
    if (polygon->getActionType() == STOP)
    {
      // Setting up zero velocity for STOP model
      robot_action.polygon_name = polygon->getName();
      robot_action.action_type = STOP;
      robot_action.req_vel.x = 0.0;
      robot_action.req_vel.y = 0.0;
      robot_action.req_vel.tw = 0.0;
      return true;
    }
    else if (polygon->getActionType() == SLOWDOWN)
    {
      Velocity safe_vel;
      double ratio = 1.0;
      ratio = polygon->getSlowdownRatio();
      safe_vel.x = velocity.x * ratio;
      safe_vel.y = velocity.y * ratio;
      safe_vel.tw = velocity.tw;
      // Check that currently calculated velocity is safer than
      // chosen for previous shapes one
      if (safe_vel < robot_action.req_vel)
      {
        robot_action.polygon_name = polygon->getName();
        robot_action.action_type = SLOWDOWN;
        robot_action.req_vel = safe_vel;
        return true;
      }
    }
    else
    {  // Limit
      // Compute linear velocity
      const double linear_vel = std::hypot(velocity.x, velocity.y);  // absolute
      Velocity safe_vel;
      double ratio = 1.0;
      if (linear_vel != 0.0)
      {
        ratio = std::clamp(polygon->getLinearLimit() / linear_vel, 0.0, 1.0);
      }

      safe_vel.x = std::max(velocity.x * ratio, min_linear_vel_);
      velocity.x * ratio;
      safe_vel.y = velocity.y * ratio;
      safe_vel.tw = std::clamp(velocity.tw, -polygon->getAngularLimit(), polygon->getAngularLimit());
      // Check that currently calculated velocity is safer than
      // chosen for previous shapes one
      if (safe_vel < robot_action.req_vel)
      {
        robot_action.polygon_name = polygon->getName();
        robot_action.action_type = LIMIT;
        robot_action.req_vel = safe_vel;
        return true;
      }
    }
  }

  return false;
}

bool CollisionMonitor::processApproach(const std::shared_ptr<Polygon> polygon,
                                       const std::vector<Point>& collision_points, const Velocity& velocity,
                                       Action& robot_action) const
{
  if (!polygon->isShapeSet())
  {
    return false;
  }

  // Obtain time before a collision
  const double collision_time = polygon->getCollisionTime(collision_points, velocity);
  if (collision_time >= 0.0)
  {
    // If collision will occurr, reduce robot speed
    const double change_ratio = collision_time / polygon->getTimeBeforeCollision();
    const Velocity safe_vel = velocity * change_ratio;
    // Check that currently calculated velocity is safer than
    // chosen for previous shapes one
    if (safe_vel < robot_action.req_vel)
    {
      robot_action.polygon_name = polygon->getName();
      robot_action.action_type = APPROACH;
      robot_action.req_vel = safe_vel;
      return true;
    }
  }

  return false;
}

void CollisionMonitor::notifyActionState(const Action& robot_action,
                                         const std::shared_ptr<Polygon> action_polygon) const
{
  if (robot_action.action_type == STOP)
  {
    ROS_INFO("Robot to stop due to %s polygon", action_polygon->getName().c_str());
  }
  else if (robot_action.action_type == SLOWDOWN)
  {
    ROS_INFO("Robot to slowdown for %f percents due to %s polygon", action_polygon->getSlowdownRatio() * 100,
             action_polygon->getName().c_str());
  }
  else if (robot_action.action_type == LIMIT)
  {
    ROS_INFO("Robot to limit speed due to %s polygon", action_polygon->getName().c_str());
  }
  else if (robot_action.action_type == APPROACH)
  {
    ROS_INFO("Robot to approach for %f seconds away from collision", action_polygon->getTimeBeforeCollision());
  }
  else
  {  // robot_action.action_type == DO_NOTHING
    ROS_INFO("Robot to continue normal operation");
  }

  if (state_pub_)
  {
    navit_collision_monitor::CollisionMonitorState::Ptr state_msg(new navit_collision_monitor::CollisionMonitorState());
    state_msg->polygon_name = robot_action.polygon_name;
    state_msg->action_type = robot_action.action_type;
    state_pub_.publish(state_msg);
  }
}

void CollisionMonitor::publishPolygons() const
{
  for (std::shared_ptr<Polygon> polygon : polygons_)
  {
    polygon->publish();
  }
}

}  // namespace navit_collision_monitor
