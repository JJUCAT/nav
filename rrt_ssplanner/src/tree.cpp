/**
 * @copyright Copyright (c) {2022} JUCAT
 * @author jucat (lmr2887@163.com)
 * @date 2024-01-14
 * @brief 
 */

#include "rrt_ssplanner/node.h"
#include "tf/transform_datatypes.h"
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
  nodes_.insert(std::make_pair(index, rrt_planner::Node(index, point)));
}

void Tree::Rewire(const size_t node, const std::vector<size_t>& near_points)
{
  auto point = nodes_.at(node).point();
  double cost_min = std::numeric_limits<double>::max();
  size_t parent_index;
  double c2p_cost;
  for (auto p : near_points) {
    auto pp = nodes_.at(p).point();
    double new_cost = std::hypot(pp.y-point.y, pp.x-point.x);
    double total_cost = new_cost + TrajectoryCost(&nodes_.at(p));
    if (total_cost < cost_min) {
      cost_min = total_cost;
      parent_index = p;
      c2p_cost = new_cost;
    }
  }
  nodes_.at(node).set_cost(c2p_cost);
  nodes_.at(node).set_parent(&nodes_.at(parent_index));
  ROS_INFO("[RRT] insert point [%lu], child [%f,%f] to parent [%f,%f]", node, point.x, point.y, nodes_.at(parent_index).point().x, nodes_.at(parent_index).point().y);
  PubArrow(node, point, nodes_.at(parent_index).point(), 1, 1, 0);

  for (auto p : near_points) {
    if (p == parent_index) continue;
    if (nodes_.at(p).is_root()) continue;
    double original_cost = TrajectoryCost(&nodes_.at(p));
    auto pp = nodes_.at(p).point();
    double new_cost = std::hypot(pp.y-point.y, pp.x-point.x);
    if (original_cost > new_cost+cost_min) {
      nodes_.at(p).set_parent(&nodes_.at(node));
      nodes_.at(p).set_cost(new_cost);
      ROS_INFO("[RRT] rewire index [%lu], child [%f,%f] to parent [%f,%f]", p, nodes_.at(p).point().x, nodes_.at(p).point().y, nodes_.at(node).point().x, nodes_.at(node).point().y);
      PubArrow(p, nodes_.at(p).point(), nodes_.at(node).point(), 1, 0, 1);
    }
  }
}

std::map<size_t, rrt_planner::Node>* Tree::nodes()
{
  return &nodes_;
}

void Tree::SetTerminal(const size_t terminal)
{
  terminal_ = terminal;
}

rrt_planner::Node* Tree::GetTerminal()
{
  return &nodes_.at(terminal_);
}

double Tree::TrajectoryCost(rrt_planner::Node* node)
{
  double cost = 0.0;
  auto p = node;
  while (!p->is_root()) {
    cost += p->cost();
    p = p->parent();
  }
  return cost;
}

std::vector<geometry_msgs::Point> Tree::GeTrajectory(rrt_planner::Node* node)
{
  std::vector<geometry_msgs::Point> points;
  auto p = node;
  do {
    points.push_back(p->point());
    p = p->parent();
  } while (!p->is_root());
  points.push_back(p->point());
  return std::move(points);
}


void Tree::PubArrow(const size_t index,
  const geometry_msgs::Point& child, const geometry_msgs::Point& parent,
  const double r, const double g, const double b)
{
  visualization_msgs::Marker v_trajectory;
  v_trajectory.header.frame_id = map_frame_;
  v_trajectory.header.stamp = ros::Time::now();
  v_trajectory.color.r = r;
  v_trajectory.color.g = g;
  v_trajectory.color.b = b;
  v_trajectory.color.a = 1;
  v_trajectory.pose.position.x = child.x;
  v_trajectory.pose.position.y = child.y;
  v_trajectory.pose.position.z = 0;
  double yaw = atan2(parent.y-child.y, parent.x-child.x);
  v_trajectory.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
  double dist = std::hypot(parent.y-child.y, parent.x-child.x);
  v_trajectory.scale.x = dist - 0.05;
  v_trajectory.scale.y = 0.03;
  v_trajectory.scale.z = 0.05;
  v_trajectory.type = visualization_msgs::Marker::ARROW;
  v_trajectory.action = visualization_msgs::Marker::ADD;
  v_trajectory.lifetime = ros::Duration();
  v_trajectory.id = index;
  arrows_pub_.publish(v_trajectory);
}

} // namespace rrt_planner
