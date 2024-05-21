/*
* @Author: czk
* @Date:   2022-10-04 10:15:25
* @Last Modified by:   chenzongkui
* @Last Modified time: 2022-11-20 15:50:34
*/
#pragma once

#include <memory.h>
#include <cmath>
#include <iostream>
#include <vector>
#include "spline.h"

#include "../geometry2.h"
#include "path.pb.h"

using namespace geometry2;

namespace planner {
namespace path {

class CoordinateConvert {
 public:
  CoordinateConvert();

  bool setPath(const proto::Path& path);

  void xyToSl(const Vec2f& xy_point, Vec2f* sl_point);

  void slToXy(const Vec2f& sl_point, Vec2f* xy_point);

  void clear();

  ~CoordinateConvert();

 private:
  int ClosestWaypoint(double x, double y);

  tk::spline x_s_;
  tk::spline y_s_;
  std::vector<double> maps_x_;
  std::vector<double> maps_y_;
  std::vector<double> maps_s_;
};
}  // namespace path
}  // namespace planner
