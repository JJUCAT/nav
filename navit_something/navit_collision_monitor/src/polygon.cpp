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

#include "navit_collision_monitor/polygon.hpp"
#include "navit_collision_monitor/kinematics.hpp"
#include "navit_collision_monitor/utils.hpp"
#include <exception>
#include <geometry_msgs/Point32.h>

namespace navit_collision_monitor {

Polygon::Polygon(ros::NodeHandle &node, const std::string &polygon_name,
                 const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                 const std::string &base_frame_id,
                 const ros::Duration &transform_tolerance)
    : node_(node), polygon_name_(polygon_name), action_type_(DO_NOTHING),
      slowdown_ratio_(0.0), linear_limit_(0.0), angular_limit_(0.0),
      tf_buffer_(tf_buffer), base_frame_id_(base_frame_id),
      transform_tolerance_(transform_tolerance) {
  ROS_INFO("[%s]: Creating Polygon", polygon_name_.c_str());
}

Polygon::~Polygon() {
  ROS_INFO("[%s]: Destroying Polygon", polygon_name_.c_str());
  polygon_sub_.shutdown();
  polygon_pub_.shutdown();
  poly_.clear();
}

bool Polygon::configure() {

  std::string polygon_sub_topic, polygon_pub_topic, footprint_topic;

  if (!getParameters(polygon_sub_topic, polygon_pub_topic, footprint_topic)) {
    ROS_ERROR("[%s]: Error while getting polygon parameters",
              polygon_name_.c_str());
    return false;
  }

  if (!polygon_sub_topic.empty()) {
    ROS_INFO("[%s]: Subscribing on %s topic for polygon", polygon_name_.c_str(),
             polygon_sub_topic.c_str());
    polygon_sub_ = node_.subscribe<geometry_msgs::PolygonStamped>(
        polygon_sub_topic, 1,
        boost::bind(&Polygon::polygonCallback, this, boost::placeholders::_1));
  }

  if (!footprint_topic.empty()) {
    ROS_INFO("[%s]: Making footprint subscriber on %s topic",
            polygon_name_.c_str(), footprint_topic.c_str());
    footprint_sub_ = std::make_unique<navit_collision_checker::FootprintSub>(
        node_, footprint_topic);
  }

  if (visualize_) {
    // Fill polygon_ for future usage
    polygon_.header.frame_id = base_frame_id_;
    std::vector<Point> poly;
    getPolygon(poly);
    for (const Point &p : poly) {
      geometry_msgs::Point32 p_s;
      p_s.x = p.x;
      p_s.y = p.y;
      // p_s.z will remain 0.0
      polygon_.polygon.points.push_back(p_s);
    }

    polygon_pub_ =
        node_.advertise<geometry_msgs::PolygonStamped>(polygon_pub_topic, 1);
  }

  return true;
}

std::string Polygon::getName() const { return polygon_name_; }

ActionType Polygon::getActionType() const { return action_type_; }

int Polygon::getMinPoints() const { return min_points_; }

double Polygon::getSlowdownRatio() const { return slowdown_ratio_; }

double Polygon::getLinearLimit() const { return linear_limit_; }

double Polygon::getAngularLimit() const { return angular_limit_; }

double Polygon::getTimeBeforeCollision() const {
  return time_before_collision_;
}

void Polygon::getPolygon(std::vector<Point> &poly) const { poly = poly_; }

bool Polygon::isShapeSet() {
  if (poly_.empty()) {
    ROS_WARN("[%s]: Polygon shape is not set yet", polygon_name_.c_str());
    return false;
  }
  return true;
}

void Polygon::updatePolygon() {
  //  if (footprint_sub_ != nullptr) {
  //    // Get latest robot footprint from footprint subscriber
  //    std::vector<geometry_msgs::Point> footprint_vec;
  //    footprint_sub_->getFootprint(footprint_vec);
  
  //    std::size_t new_size = footprint_vec.size();
  //    poly_.resize(new_size);
     
  //    polygon_.header.frame_id = base_frame_id_;
  //    polygon_.polygon.points.resize(new_size);
  
  //    geometry_msgs::Point32 p_s;
  //    for (std::size_t i = 0; i < new_size; i++) {
  //      poly_[i] = {footprint_vec[i].x, footprint_vec[i].y};
  //      p_s.x = footprint_vec[i].x;
  //      p_s.y = footprint_vec[i].y;
  //      std::cout << "footprint point " << i << " is " << p_s.x << ", " << p_s.y << std::endl;
  //      polygon_.polygon.points[i] = p_s;
  //    }
  //  } else 
   if (!polygon_.header.frame_id.empty() &&
      polygon_.header.frame_id != base_frame_id_) {
    // Polygon is published in another frame: correct poly_ vertices to the
    // latest frame state
    std::size_t new_size = polygon_.polygon.points.size();

    // Get the transform from PolygonStamped frame to base_frame_id_
    tf2::Transform tf_transform;
    if (!collision_monitor_utils::getTransform(
            polygon_.header.frame_id, base_frame_id_, transform_tolerance_,
            tf_buffer_, tf_transform)) {
      return;
    }

    // Correct main poly_ vertices
    poly_.resize(new_size);
    for (std::size_t i = 0; i < new_size; i++) {
      // Transform point coordinates from PolygonStamped frame -> to base frame
      tf2::Vector3 p_v3_s(polygon_.polygon.points[i].x,
                          polygon_.polygon.points[i].y, 0.0);
      tf2::Vector3 p_v3_b = tf_transform * p_v3_s;

      // Fill poly_ array
      poly_[i] = {p_v3_b.x(), p_v3_b.y()};
    }
  }
}

int Polygon::getPointsInside(const std::vector<Point> &points) const {
  int num = 0;
  for (const Point &point : points) {
    if (isPointInside(point)) {
      num++;
    }
  }
  return num;
}

double Polygon::getCollisionTime(const std::vector<Point> &collision_points,
                                 const Velocity &velocity) const {
  // Initial robot pose is {0,0} in base_footprint coordinates
  Pose pose = {0.0, 0.0, 0.0};
  Velocity vel = velocity;

  // Array of points transformed to the frame concerned with pose on each
  // simulation step
  std::vector<Point> points_transformed;

  // Robot movement simulation
  for (double time = 0.0; time <= time_before_collision_;
       time += simulation_time_step_) {
    // Shift the robot pose towards to the vel during simulation_time_step_ time
    // interval NOTE: vel is changing during the simulation
    projectState(simulation_time_step_, pose, vel);
    // Transform collision_points to the frame concerned with current robot pose
    points_transformed = collision_points;
    transformPoints(pose, points_transformed);
    // If the collision occurred on this stage, return the actual time before a
    // collision as if robot was moved with given velocity
    if (getPointsInside(points_transformed) >= min_points_) {
      return time;
    }
  }

  // There is no collision
  return -1.0;
}

void Polygon::publish() {
  if (!visualize_) {
    return;
  }

  // Actualize the time to current and publish the polygon
  polygon_.header.stamp = ros::Time::now();
  geometry_msgs::PolygonStamped::Ptr msg(
      new geometry_msgs::PolygonStamped(polygon_));
  polygon_pub_.publish(msg);
}

bool Polygon::getCommonParameters(std::string &polygon_pub_topic) {

  try {
    // Get action type.
    // Leave it not initialized: the will cause an error if it will not set.
    std::string at_str;
    node_.param(polygon_name_ + "/action_type", at_str, at_str);
    if (at_str == "stop") {
      action_type_ = STOP;
    } else if (at_str == "slowdown") {
      action_type_ = SLOWDOWN;
    } else if (at_str == "limit") {
      action_type_ = LIMIT;
    } else if (at_str == "approach") {
      action_type_ = APPROACH;
    } else if (at_str == "none") {
      action_type_ = DO_NOTHING;
    } else { // Error if something else
      ROS_ERROR("[%s]: Unknown action type: %s", polygon_name_.c_str(),
                at_str.c_str());
      return false;
    }

    node_.param(polygon_name_ + "/min_points", min_points_, 4);

    if (action_type_ == SLOWDOWN) {
      node_.param(polygon_name_ + "/slowdown_ratio", slowdown_ratio_, 0.5);
    }

    if (action_type_ == LIMIT) {
      node_.param(polygon_name_ + "/linear_limit", linear_limit_, 0.5);
      node_.param(polygon_name_ + "/angular_limit", angular_limit_, 0.5);
    }

    if (action_type_ == APPROACH) {
      node_.param(polygon_name_ + "/time_before_collision",
                  time_before_collision_, 2.0);
      node_.param(polygon_name_ + "/simulation_time_step",
                  simulation_time_step_, 0.1);
    }

    node_.param(polygon_name_ + "/visualize", visualize_, false);
    if (visualize_) {
      // Get polygon topic parameter in case if it is going to be published
      node_.param(polygon_name_ + "/polygon_pub_topic", polygon_pub_topic,
                  polygon_name_);
    }
  } catch (const std::exception &ex) {
    ROS_ERROR("[%s]: Error while getting common polygon parameters: %s",
              polygon_name_.c_str(), ex.what());
    return false;
  }

  return true;
}

bool Polygon::getParameters(std::string &polygon_sub_topic,
                            std::string &polygon_pub_topic,
                            std::string &footprint_topic) {
  if (!getCommonParameters(polygon_pub_topic)) {
    ROS_ERROR("[%s]: Error while getting common polygon parameters",
              polygon_name_.c_str());
    return false;
  }

  // Clear the subscription topics. They will be set later, if necessary.
  polygon_sub_topic.clear();
  footprint_topic.clear();

  try {
    try {
      // Leave it uninitialized: it will throw an inner exception if the
      // parameter is not set
      std::vector<double> poly_row;
      
      node_.param(polygon_name_ + "/points", poly_row, poly_row);

      // Check for points format correctness
      if (poly_row.size() <= 6 || poly_row.size() % 2 != 0) {
        ROS_ERROR("[%s]: Polygon has incorrect points description",
                  polygon_name_.c_str());
        return false;
      }

      // Obtain polygon vertices
      Point point;
      bool first = true;
      for (double val : poly_row) {
        if (first) {
          point.x = val;
        } else {
          point.y = val;
          poly_.push_back(point);
        }
        first = !first;
      }

      // Do not need to proceed further, if "points" parameter is defined.
      // Static polygon will be used.
      return true;
    } catch (...) {
      ROS_INFO("[%s]: Polygon points are not defined. Using dynamic "
               "subscription instead.",
               polygon_name_.c_str());
    }
    if (action_type_ == STOP || action_type_ == SLOWDOWN ||
        action_type_ == LIMIT || action_type_ == DO_NOTHING) {
      // Dynamic polygon will be used
      node_.param(polygon_name_ + "/polygon_sub_topic", polygon_sub_topic,
                  std::string());
    } else if (action_type_ == APPROACH) {
      // Obtain the footprint topic to make a footprint subscription
      // for approach polygon
      node_.param(polygon_name_ + "/footprint_topic", footprint_topic,
                  std::string("local_costmap/published_footprint"));
    }
  } catch (const std::exception &ex) {
    ROS_ERROR("[%s]: Error while getting polygon parameters: %s",
              polygon_name_.c_str(), ex.what());
    return false;
  }

  return true;
}

void Polygon::updatePolygon(geometry_msgs::PolygonStamped::ConstPtr msg) {
  std::size_t new_size = msg->polygon.points.size();

  if (new_size < 3) {
    ROS_ERROR("[%s]: Polygon should have at least 3 points",
              polygon_name_.c_str());
    return;
  }

  // Get the transform from PolygonStamped frame to base_frame_id_
  tf2::Transform tf_transform;
  if (!collision_monitor_utils::getTransform(
          msg->header.frame_id, base_frame_id_, transform_tolerance_,
          tf_buffer_, tf_transform)) {
    return;
  }

  // Set main poly_ vertices first time
  poly_.resize(new_size);
  for (std::size_t i = 0; i < new_size; i++) {
    // Transform point coordinates from PolygonStamped frame -> to base frame
    tf2::Vector3 p_v3_s(msg->polygon.points[i].x, msg->polygon.points[i].y,
                        0.0);
    tf2::Vector3 p_v3_b = tf_transform * p_v3_s;

    // Fill poly_ array
    poly_[i] = {p_v3_b.x(), p_v3_b.y()};
  }

  // Store incoming polygon for further (possible) poly_ vertices corrections
  // from PolygonStamped frame -> to base frame
  polygon_ = *msg;
}

void Polygon::polygonCallback(geometry_msgs::PolygonStamped::ConstPtr msg) {
  ROS_INFO("[%s]: Polygon shape update has been arrived",
           polygon_name_.c_str());
  updatePolygon(msg);
}

inline bool Polygon::isPointInside(const Point &point) const {
  // Adaptation of Shimrat, Moshe. "Algorithm 112: position of point relative to
  // polygon." Communications of the ACM 5.8 (1962): 434. Implementation of ray
  // crossings algorithm for point in polygon task solving. Y coordinate is
  // fixed. Moving the ray on X+ axis starting from given point. Odd number of
  // intersections with polygon boundaries means the point is inside polygon.
  const int poly_size = poly_.size();
  int i, j;         // Polygon vertex iterators
  bool res = false; // Final result, initialized with already inverted value

  // Starting from the edge where the last point of polygon is connected to the
  // first
  i = poly_size - 1;
  for (j = 0; j < poly_size; j++) {
    // Checking the edge only if given point is between edge boundaries by Y
    // coordinates. One of the condition should contain equality in order to
    // exclude the edges parallel to X+ ray.
    if ((point.y <= poly_[i].y) == (point.y > poly_[j].y)) {
      // Calculating the intersection coordinate of X+ ray
      const double x_inter = poly_[i].x + (point.y - poly_[i].y) *
                                              (poly_[j].x - poly_[i].x) /
                                              (poly_[j].y - poly_[i].y);
      // If intersection with checked edge is greater than point.x
      // coordinate, inverting the result
      if (x_inter > point.x) {
        res = !res;
      }
    }
    i = j;
  }
  return res;
}

} // namespace navit_collision_monitor
