#include <local_planner/security_detector.h>

namespace local_planner {


SecurityDetector::SecurityDetector(
  const int base_collision, const int head_collision, const int collision_offset,
  const int base_security, const int head_security, const double security_ignore) :
  base_collision_cost(base_collision), head_collision_cost(head_collision), collision_cost_offset(collision_offset),
  base_security_cost(base_security), head_security_cost(head_security), security_ignore_dist(security_ignore) {}

SecurityDetector::~SecurityDetector() {}

void SecurityDetector::SetCollisionCost(const int base_cost, const int head_cost)
{
  base_collision_cost = base_cost;
  head_collision_cost = head_cost;
}


} // namespace local_planner
