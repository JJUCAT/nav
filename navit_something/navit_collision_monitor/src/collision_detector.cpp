// Copyright (c) 2022 Samsung R&D Institute Russia
// Copyright (c) 2023 Pixel Robotics GmbH
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

#include "navit_collision_monitor/collision_detector.hpp"
#include "navit_collision_monitor/CollisionDetectorState.h"
#include <exception>
#include <functional>

using namespace std::chrono_literals;

namespace navit_collision_monitor
{

CollisionDetector::CollisionDetector(ros::NodeHandle& node) : node_(node)
{
  on_configure();
  on_activate();
}

CollisionDetector::~CollisionDetector()
{
  on_deactivate();
  on_shutdown();
  on_cleanup();
  polygons_.clear();
  sources_.clear();
}

bool CollisionDetector::on_configure()
{
  ROS_INFO("Configuring");

  // Transform buffer and listener initialization
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  state_pub_ = node_.advertise<navit_collision_monitor::CollisionDetectorState>("collision_detector_state", 10);
  ctrl_service_ = node_.advertiseService("ctrl", &CollisionDetector::onCtrl, this);

  // Obtaining ROS parameters
  if (!getParameters())
  {
    return false;
  }

  return true;
}

bool CollisionDetector::on_activate()
{
  ROS_INFO("Activating");

  // Activating polygons
  for (std::shared_ptr<Polygon> polygon : polygons_)
  {
    polygon->activate();
  }

  // Creating timer
  timer_ = node_.createTimer(ros::Duration(1.0 / frequency_), boost::bind(&CollisionDetector::process, this));
  return true;
}

bool CollisionDetector::on_deactivate()
{
  ROS_INFO("Deactivating");

  // Resetting timer
  timer_.stop();

  // Deactivating polygons
  for (std::shared_ptr<Polygon> polygon : polygons_)
  {
    polygon->deactivate();
  }
  return true;
}

bool CollisionDetector::on_cleanup()
{
  ROS_INFO("Cleaning up");

  state_pub_.shutdown();

  polygons_.clear();
  sources_.clear();

  tf_listener_.reset();
  tf_buffer_.reset();

  return true;
}

bool CollisionDetector::on_shutdown()
{
  ROS_INFO("Shutting down");
  return true;
}

bool CollisionDetector::onCtrl(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  bool control = req.data;
  // 在这里进行你想要的处理操作
  ROS_INFO_STREAM("Received control: " << control);

  bool succeed = control ? on_activate() : on_deactivate();
  res.success = succeed;
  res.message = "Control received";
  return succeed;
}

bool CollisionDetector::getParameters()
{
  std::string base_frame_id, odom_frame_id;
  ros::Duration transform_tolerance;
  ros::Duration source_timeout(2.0, 0.0);

  // frequency
  node_.param("frequency", frequency_, 10.0);
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
  // configurePolygons
  if (!configurePolygons(base_frame_id, transform_tolerance))
  {
    return false;
  }
  // configureSources
  if (!configureSources(base_frame_id, odom_frame_id, transform_tolerance, source_timeout, base_shift_correction))
  {
    return false;
  }

  return true;
}

bool CollisionDetector::configurePolygons(const std::string& base_frame_id, const ros::Duration& transform_tolerance)
{
  try
  {
    // Leave it to be not initialized: to intentionally cause an error if it
    // will not set
    std::vector<std::string> polygon_names;
    node_.param("polygons", polygon_names, polygon_names);
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
      auto action_type = polygons_.back()->getActionType();
      if (action_type != DO_NOTHING)
      {
        ROS_ERROR(
            "[%s]: The action_type of the polygon is different than "
            "\"none\" which is "
            "not supported in the collision detector.",
            polygon_name.c_str());
        return false;
      }
    }
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR("Error while getting parameters: %s", ex.what());
    return false;
  }

  return true;
}

bool CollisionDetector::configureSources(const std::string& base_frame_id, const std::string& odom_frame_id,
                                         const ros::Duration& transform_tolerance, const ros::Duration& source_timeout,
                                         const bool base_shift_correction)
{
  try
  {
    // Leave it to be not initialized to intentionally cause an error if it will
    // not set
    std::vector<std::string> source_names;
    
    node_.param("observation_sources", source_names, source_names);

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

void CollisionDetector::process()
{
  // Current timestamp for all inner routines prolongation
  ros::Time curr_time = ros::Time::now();

  // Points array collected from different data sources in a robot base frame
  std::vector<Point> collision_points;

  // Fill collision_points array from different data sources
  for (std::shared_ptr<Source> source : sources_)
  {
    source->getData(curr_time, collision_points);
  }

  navit_collision_monitor::CollisionDetectorState::Ptr state_msg(new navit_collision_monitor::CollisionDetectorState());

  for (std::shared_ptr<Polygon> polygon : polygons_)
  {
    state_msg->polygons.push_back(polygon->getName());
    state_msg->detections.push_back(polygon->getPointsInside(collision_points) > polygon->getMinPoints());
  }

  state_pub_.publish(state_msg);

  // Publish polygons for better visualization
  publishPolygons();
}

void CollisionDetector::publishPolygons() const
{
  for (std::shared_ptr<Polygon> polygon : polygons_)
  {
    polygon->publish();
  }
}

}  // namespace navit_collision_monitor
