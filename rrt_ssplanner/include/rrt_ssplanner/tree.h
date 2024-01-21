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

  std::map<size_t, rrt_planner::Node>* nodes();

  /**
   * @brief  从节点向父节点一直遍历到根节点，累积节点移动代价
   * @param  node  要遍历的起始节点
   * @return double  总移动代价
   */
  double TrajectoryCost(rrt_planner::Node* node);

  /**
   * @brief  获取轨迹
   * @param  node  轨迹的终点节点
   * @return std::vector<geometry_msgs::Point> 
   */
  std::vector<geometry_msgs::Point> GeTrajectory(rrt_planner::Node* node);

 private:
  std::map<size_t, rrt_planner::Node> nodes_;

}; // class Tree

} // namespace rrt_planner

#endif // TREE_H_
