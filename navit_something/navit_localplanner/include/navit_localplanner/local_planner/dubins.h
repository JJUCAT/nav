/**
 * @copyright Copyright (c) {2022} LMR
 * @author LMR (lmr2887@163.com)
 * @date 2024-04-01
 * @brief 
 */

#ifndef _DUBINS_H_
#define _DUBINS_H_

#include <ros/ros.h>
#include <dubins/include/dubins.h>
#include <Eigen/Core>


namespace local_planner {

/**
 * @brief  dubins 路径规划
 */
class Dubins
{
 public:

  /**
   * @brief  设置起点
   * @param  start  起点 [x, y, yaw] [m, m, rad]
   */
  void SetStart(const Eigen::Vector3d& start);
  
  /**
   * @brief  设置终点
   * @param  terminal  终点 [x, y, yaw] [m, m, rad]
   */
  void SetTerminal(const Eigen::Vector3d& terminal);

  /**
   * @brief  设置转弯半径
   * @param  radius  转弯半径, [rad]
   */
  void SetRadius(const double radius);

  /**
   * @brief  获取 dubins 路径
   * @param  stepSize 采样步长
   * @param  poses 采样路径
   * @return size_t 
   */
  size_t GetPath(double stepSize, std::vector<Eigen::Vector3d>& poses) const;

 private:

  Eigen::Vector3d start_;

  Eigen::Vector3d terminal_;

  double radius_;

}; // class Dubins

} // namespace local_planner

#endif // _DUBINS_H_
