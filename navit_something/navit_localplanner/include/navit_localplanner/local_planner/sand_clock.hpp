#ifndef SAND_CLOCK_HPP_
#define SAND_CLOCK_HPP_

#include <ros/ros.h>
#include "ros/duration.h"

namespace local_planner {
/**
 * @brief 沙漏负责局部规划的超时错误检测
 */
class SandClock
{
 public:
  SandClock() = default;
  ~SandClock() = default;
  
  /**
   * @brief  设置超时时间
   * @param  timeout  超时时间
   */
  void Load(double timeout) {
    timeout_ = ros::Duration(timeout);
  }

  /**
   * @brief  沙漏开始计时
   */
  void Overturn() {
    timer_ = ros::Time::now();
  }
  
  /**
   * @brief  是否超时
   * @return true 
   * @return false 
   */
  bool Timeout() {
    if (ros::Time::now()-timer_ >= timeout_) return true;
    return false;
  }

 private:
  ros::Time timer_; // 计时器
  ros::Duration timeout_; // 等待障碍物消失时间
}; // class SandClock

}; // namespace local_planner

#endif // SAND_CLOCK_HPP_
