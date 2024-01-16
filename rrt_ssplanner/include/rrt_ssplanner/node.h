/**
 * @copyright Copyright (c) {2022} JUCAT
 * @author jucat (lmr2887@163.com)
 * @date 2024-01-14
 * @brief 
 */
#ifndef NODE_H_
#define NODE_H_

#include <geometry_msgs/Point.h>

namespace rrt_planner {

class Node
{
 public:

  Node() = delete;

  Node(const size_t index, const geometry_msgs::Point point);
  
  ~Node() = default;

  /**
   * @brief  设置父节点
   * @param  p  父节点
   */
  void set_parent(Node* p);

  /**
   * @brief  获取父节点
   * @return Node* 
   */
  Node* parent();

  /**
   * @brief  设置到父节点的移动代价
   * @param  cost  移动代价
   */
  void set_cost(const double cost);

  /**
   * @brief  获取节点到父节点的移动代价
   * @return double 
   */
  double cost();
  
  /**
   * @brief  返回节点所在位置
   * @return geometry_msgs::Point 
   */
  geometry_msgs::Point point();

 protected:

  Node* parent_;
  geometry_msgs::Point point_;
  double cost_;
  const size_t index_;
  
}; // class Ndoe

} // namespace nanoflann_port_ns


#endif // NODE_H_
