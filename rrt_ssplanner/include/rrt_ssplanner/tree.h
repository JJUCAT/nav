/**
 * @copyright Copyright (c) {2022} JUCAT
 * @author jucat (lmr2887@163.com)
 * @date 2024-01-14
 * @brief 
 */

#ifndef TREE_H_
#define TREE_H_

#include "ros/publisher.h"
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <string>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <rrt_ssplanner/node.h>

namespace rrt_planner {

class Tree
{
 public:
  Tree() = delete;
  Tree(const std::string& map_frame, const ros::Publisher& arrows_pub) {
    map_frame_ = map_frame;
    arrows_pub_ = arrows_pub;
  }
  ~Tree() = default;

  void InsertNode(const size_t index, const geometry_msgs::Point point,
                  const std::vector<size_t> near_points);

  void Insert(const size_t index, const geometry_msgs::Point point);

  void Rewire(const size_t node, const std::vector<size_t>& near_points);

  std::map<size_t, rrt_planner::Node>* nodes();

  void SetTerminal(const size_t terminal);

  rrt_planner::Node* GetTerminal();

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

  /**
   * @brief  发布箭头，从子节点指向父节点
   * @param  index  子节点下标
   * @param  child  子节点
   * @param  parent  父节点
   */
  void PubArrow(const size_t index,
    const geometry_msgs::Point& child, const geometry_msgs::Point& parent,
    const double r, const double g, const double b);

 private:
  std::map<size_t, rrt_planner::Node> nodes_;

  std::string map_frame_;
  ros::Publisher arrows_pub_;
  size_t terminal_;

}; // class Tree

} // namespace rrt_planner

#endif // TREE_H_
