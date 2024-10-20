/**
 * @copyright Copyright (c) {2022} LMRCAT
 * @author lmrcat (lmr2887@163.com)
 * @date 2024-01-14
 * @brief 
 */

# include <rrt_ssplanner/node.h>

namespace rrt_planner {

Node::Node(const size_t index, const geometry_msgs::Point point)
  : parent_{nullptr}, point_{point}, cost_{0.0}, index_(index){}

void Node::set_parent(Node* p)
{
  parent_ = p;
}

Node* Node::parent()
{
  return parent_;
}

void Node::set_cost(const double cost)
{
  cost_ = cost;
}

double Node::cost()
{
  return cost_;
}

geometry_msgs::Point Node::point()
{
  return point_;
}

bool Node::is_root()
{
  return index_ == 0;
}

} // namespace rrt_planner

