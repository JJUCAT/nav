/*
* @Author: czk
* @Date:   2022-10-02 12:03:58
* @Last Modified by:   chenzongkui
* @Last Modified time: 2023-03-23 15:21:11
*/
#pragma once
#include <iostream>

#include <Eigen/Core>
#include <navit_core/base_global_planner.h>

#include <gsl/gsl_spline.h>

#include "geometry.pb.h"
#include "path.pb.h"
#include "math.h"

#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
namespace navit_planner {

class PathGslSpline : public navit_core::SmoothPathPlanner {
public:
    using Ptr = boost::shared_ptr<PathGslSpline>;

    PathGslSpline(double step_s) : step_s_(step_s) {}
    PathGslSpline(){
        nh_.param("/navit_planner_node/smooth_path/step_s", step_s_, 0.02);
        nh_.param("/navit_planner_node/smooth_path/step_point", step_point_, 2);
     }
    ~PathGslSpline() {
        // gsl_spline_free(spline_x_);
        // gsl_spline_free(spline_y_);
        // gsl_interp_accel_free(acc_);
    }
    void initialize(const std::string& name){}

    bool makePlan(const nav_msgs::Path& rough_path, nav_msgs::Path& smooth_path) {
        return pathSmoother(rough_path, smooth_path);
    }

    bool pathSmoother(const nav_msgs::Path& rough_path, nav_msgs::Path& smooth_path);

    bool pathSmoother(const nav_msgs::Path& rough_path, proto::Path& smooth_path);

    bool pathSmoother(const proto::Path& rough_path, proto::Path& smooth_path);

    bool setStep(const int8_t step) {
        step_point_ = step;
        return true;
    }
 private:
  gsl_spline* spline_x_;
  gsl_spline* spline_y_;
  gsl_interp_accel* acc_;
  double step_s_;

  int step_point_;

  ros::NodeHandle nh_;

  bool setPath(const proto::Polyline& line, proto::Path* path);

  bool evaluate(double s, proto::PathPoint* path_point);

  // DISALLOW_COPY_AND_ASSIGN(PathGslSpline);
};

}  // namespace navit_planner
