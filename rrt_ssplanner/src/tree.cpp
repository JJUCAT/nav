/**
 * @copyright Copyright (c) {2022} JUCAT
 * @author jucat (lmr2887@163.com)
 * @date 2024-01-14
 * @brief 
 */

#include "rrt_ssplanner/node.h"
#include <limits>
#include <memory>
#include <rrt_ssplanner/tree.h>

namespace rrt_planner {

void Tree::InsertNode(const size_t index, const geometry_msgs::Point point,
  const std::vector<size_t> near_points)
{
  Insert(index, point);
  Rewire(index, near_points);
}

void Tree::Insert(const size_t index, const geometry_msgs::Point point)
{
  nodes_.push_back(rrt_planner::Node(index, point));
}

void  Tree::Rewire(const size_t node, const std::vector<size_t>& near_points)
{
  auto point = nodes_.at(node).point();
  double cost_min = std::numeric_limits<double>::max();
  size_t parent_index;
  double c2p_cost;
  for (auto p : near_points) {
    auto pp = nodes_.at(p).point();
    double new_cost = std::hypot(pp.y-point.y, pp.x-point.x);
    double total_cost = new_cost + nodes_.at(p).trajectory_cost();
    if (total_cost < cost_min) {
      cost_min = total_cost;
      parent_index = p;
      c2p_cost = new_cost;
    }
  }
  nodes_.at(node).set_cost(c2p_cost);
  nodes_.at(node).set_parent(&nodes_.at(parent_index));

  for (auto p : near_points) {
    if (p == parent_index) continue;
    double original_cost = nodes_.at(p).trajectory_cost();
    auto pp = nodes_.at(p).point();
    double new_cost = std::hypot(pp.y-point.y, pp.x-point.x) + cost_min;
    if (original_cost > new_cost) {
      nodes_.at(p).set_parent(&nodes_.at(node));
      nodes_.at(p).set_cost(new_cost);
    }
  }
}

} // namespace rrt_planner
