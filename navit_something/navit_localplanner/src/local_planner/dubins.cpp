/**
 * @copyright Copyright (c) {2022} LMR
 * @author LMR (lmr2887@163.com)
 * @date 2024-04-01
 * @brief 
 */



#include <ros/ros.h>
#include <local_planner/dubins.h>


namespace local_planner {


void Dubins::SetStart(const Eigen::Vector3d& start)
{
  start_ = start;
}

void Dubins::SetTerminal(const Eigen::Vector3d& terminal)
{
  terminal_ = terminal;
}

void Dubins::SetRadius(const double radius)
{
  radius_ = radius;
}

size_t Dubins::GetPath(double stepSize, std::vector<Eigen::Vector3d>& poses) const
{
  double s[3] = {start_(0), start_(1), start_(2)};
  double t[3] = {terminal_(0), terminal_(1), terminal_(2)};
  DubinsPath dpath;
  dubins_shortest_path( &dpath, s, t, radius_);
  
  double q[3];
  double x = 0.0;
  double length = dubins_path_length(&dpath);
  while( x <  length ) {
    dubins_path_sample(&dpath, x, q );
    Eigen::Vector3d p(q[0], q[1], q[2]);
    poses.push_back(p);
    x += stepSize;
  }

  return poses.size();
}




} // namespace local_planner
