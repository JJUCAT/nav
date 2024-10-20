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

#include "navit_collision_monitor/circle.hpp"
#include <cmath>
#include <exception>

namespace navit_collision_monitor {

Circle::Circle(ros::NodeHandle &node, const std::string &polygon_name,
               const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
               const std::string &base_frame_id,
               const ros::Duration &transform_tolerance)
    : Polygon::Polygon(node, polygon_name, tf_buffer, base_frame_id,
                       transform_tolerance) {
  ROS_INFO("[%s]: Creating Circle", polygon_name_.c_str());
}

Circle::~Circle() {
  ROS_INFO("[%s]: Destroying Circle", polygon_name_.c_str());
}

void Circle::getPolygon(std::vector<Point> &poly) const {
  // Number of polygon points. More edges means better approximation.
  const double polygon_edges = 16;
  // Increment of angle during points position calculation
  double angle_increment = 2 * M_PI / polygon_edges;

  // Clear polygon before filling
  poly.clear();

  // Making new polygon looks like a circle
  Point p;
  for (double angle = 0.0; angle < 2 * M_PI; angle += angle_increment) {
    p.x = radius_ * std::cos(angle);
    p.y = radius_ * std::sin(angle);
    poly.push_back(p);
  }
}

int Circle::getPointsInside(const std::vector<Point> &points) const {
  int num = 0;
  for (Point point : points) {
    if (point.x * point.x + point.y * point.y < radius_squared_) {
      num++;
    }
  }

  return num;
}

bool Circle::getParameters(std::string &polygon_sub_topic,
                           std::string &polygon_pub_topic,
                           std::string &footprint_topic) {
  if (!getCommonParameters(polygon_pub_topic)) {
    return false;
  }

  // There is no polygon or footprint subscription for the Circle. Thus, set
  // strings as empty.
  polygon_sub_topic.clear();
  footprint_topic.clear();

  try {
    // Leave it not initialized: the will cause an error if it will not set
    node_.param(polygon_name_ + "/radius", radius_, 0.1);
    radius_squared_ = radius_ * radius_;
  } catch (const std::exception &ex) {
    ROS_ERROR("[%s]: Error while getting circle parameters: %s",
              polygon_name_.c_str(), ex.what());
    return false;
  }

  return true;
}

} // namespace navit_collision_monitor
