/**
 * @copyright Copyright (c) {2022} JUCAT
 * @author jucat (lmr2887@163.com)
 * @date 2024-01-14
 * @brief 
 */

#ifndef TREE_H_
#define TREE_H_

#include <geometry_msgs/Point.h>

#include <rrt_ssplanner/node.h>

namespace rrt_planner {

class Tree
{
 public:
  Tree() = default;
  ~Tree() = default;

  void InsertNode(const size_t index, const geometry_msgs::Point point,
                  const std::vector<size_t> near_points);

  void Insert(const size_t index, const geometry_msgs::Point point);

  void Rewire(const size_t node, const std::vector<size_t>& near_points);

  std::vector<rrt_planner::Node>* nodes();

 private:
  std::vector<rrt_planner::Node> nodes_;

}; // class Tree

} // namespace rrt_planner

#endif // TREE_H_
