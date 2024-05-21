/*
* @Author: czk
* @Date:   2022-10-04 10:14:49
* @Last Modified by:   chenzongkui
* @Last Modified time: 2022-11-20 15:56:57
*/
#include "path_follower/coordinate_convert.h"

namespace planner {
namespace path {

CoordinateConvert::CoordinateConvert() {}
bool CoordinateConvert::setPath(const proto::Path& path) {
  maps_x_.clear();
  maps_y_.clear();
  maps_s_.clear();
  x_s_.clear();
  y_s_.clear();
  if (path.path_points_size() < 3) return false;
  for (int i = 0; i < path.path_points_size(); i++) {
    maps_x_.push_back(path.path_points(i).position().x());
    maps_y_.push_back(path.path_points(i).position().y());
    maps_s_.push_back(path.path_points(i).s());
  }
  std::vector<double> ss_vals(maps_s_), xs_vals(maps_x_), ys_vals(maps_y_);
  for (size_t i = 1; i < ss_vals.size(); i++) {
    if (ss_vals[i] < ss_vals[i - 1]) {
      return false;
    }
  }
  x_s_.set_points(ss_vals, xs_vals);
  y_s_.set_points(ss_vals, ys_vals);
  return true;
}

//Cartesian coordinates to frenet coordinates.
void CoordinateConvert::xyToSl(const Vec2f& xy_point, Vec2f* sl_point) {
  double x = xy_point.x;
  double y = xy_point.y;
  int wp1 = ClosestWaypoint(x, y);
  double frenet_s = maps_s_[wp1];
  double ds = 0.001;
  double norm_distance = 1000;
  int d_index = 0;
  while (fabs(norm_distance) > 0.01 && d_index < 2000) {
    d_index++;
    double x0 = x_s_(frenet_s);
    double y0 = y_s_(frenet_s);
    double s1 = frenet_s + ds;
    double dx = x_s_(s1) - x0;
    double dy = y_s_(s1) - y0;
    double slope = atan2(dy, dx);
    norm_distance = (y - y0) * sin(slope) + (x - x0) * cos(slope);
    frenet_s = frenet_s + norm_distance;
  }
  double x0 = x_s_(frenet_s);
  double y0 = y_s_(frenet_s);
  double s1 = frenet_s + ds;
  double dx = x_s_(s1) - x0;
  double dy = y_s_(s1) - y0;
  double slope = atan2(dy, dx);
  double frenet_d = (y - y0) * cos(slope) - (x - x0) * sin(slope);
  sl_point->x = frenet_s;
  sl_point->y = -frenet_d;
}

//Frenet coordinates to cartesian coordinates.
void CoordinateConvert::slToXy(const Vec2f& sl_point, Vec2f* xy_point) {
  double s = sl_point.x;
  double d = sl_point.y;
  double frenet_s = s;
  double x = x_s_(frenet_s);
  double y = y_s_(frenet_s);
  double ds = 0.01;
  double s1 = frenet_s + ds;
  double dx = x_s_(s1) - x;
  double dy = y_s_(s1) - y;

  double norm = atan2(dx, dy);

  x += d * cos(norm);
  y -= d * sin(norm);
  xy_point->x = x;
  xy_point->y = y;
}

int CoordinateConvert::ClosestWaypoint(double x, double y) {
  double closest_len = 100000;  // large number
  int closest_waypoint = 0;
  for (size_t i = 0; i < maps_x_.size(); i++) {
    double map_x = maps_x_[i];
    double map_y = maps_y_[i];
    double dist = (Vec2f(x, y)).distance(Vec2f(map_x, map_y));
    if (dist < closest_len) {
      closest_len = dist;
      closest_waypoint = i;
    }
    if (closest_len < 0.05) break;
  }
  return closest_waypoint;
}

void CoordinateConvert::clear() {
  maps_x_.clear();
  maps_y_.clear();
  maps_s_.clear();
}

CoordinateConvert::~CoordinateConvert() {}

}  // namespace path
}  // namespace planner
