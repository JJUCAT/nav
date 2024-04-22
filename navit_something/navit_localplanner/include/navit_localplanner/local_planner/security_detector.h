#ifndef NAVIT_LOCAL_PLANNER__SECURITY_DETETOR_HPP_
#define NAVIT_LOCAL_PLANNER__SECURITY_DETETOR_HPP_

#include <ros/ros.h>

namespace local_planner {

class SecurityDetector
{
 public:

  SecurityDetector(
    const int base_collision, const int head_collision, const int collision_offset,
    const int base_security, const int head_security, const double security_ignore_dist);

  ~SecurityDetector();

  void SetCollisionCost(const int base_cost, const int head_cost);

  int base_collision_cost;
  int head_collision_cost;
  int collision_cost_offset;
  int base_security_cost;
  int head_security_cost;
  double security_ignore_dist;

 private:

}; // class Configuration

} // namespace local_planner

#endif // NAVIT_LOCAL_PLANNER__SECURITY_DETETOR_HPP_
