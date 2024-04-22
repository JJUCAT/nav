#ifndef DISPATCHER_H_
#define DISPATCHER_H_

#include <ros/ros.h>
#include <mutex>

namespace local_planner {
/**
 * @brief 调度器负责何时调用局部规划，何时停止局部规划，不关心规划成功与否
 */
class Dispatcher
{
 public:
  Dispatcher() = default;
  ~Dispatcher() = default;
  
  /**
   * @brief  调度器初始化
   * @param  wait  等待障碍物消失的时间
   * @param  sleep  调度器休眠时间
   */
  void Reset(const ros::Duration wait, const ros::Duration sleep) {
    timer_ = ros::Time(0);
    wait_ = wait;
    sleep_ = sleep;
    sleeping_ = false;
  }
  
  /**
   * @brief  设置等待时间
   * @param  wait             My Param doc
   */
  void SetWait(const ros::Duration wait) {
    wait_ = wait;
  }

  /**
   * @brief 调度器计时
   */
  void Tick() {
    if (timer_ == ros::Time(0)) {
      std::unique_lock<std::mutex> lock(timer_mutex_);
      timer_ = ros::Time::now();
    }
  }

  /**
   * @brief  是否等待再走
   * @return true 
   * @return false 
   */
  bool Wait() {
    std::unique_lock<std::mutex> lock(timer_mutex_);
    if (timer_ == ros::Time(0)) return false;

    ros::Duration time_lapses = ros::Time::now()-timer_;
    ros::Duration timeout = sleeping_ ? sleep_ : wait_;
    bool is_timeout = time_lapses > timeout;

    if (is_timeout) {
      if (!sleeping_) timer_ = ros::Time::now();
      else timer_ = ros::Time(0);
      sleeping_ = !sleeping_;
    }

    return !sleeping_;
  }
  
  /**
   * @brief  唤醒调度器
   */
  void Wakeup() {
    std::unique_lock<std::mutex> lock(timer_mutex_);
    if (sleeping_) {
      sleeping_ = false;
      timer_ = ros::Time(0);
    }
  }

 private:
  std::mutex timer_mutex_;
  ros::Time timer_; // 计时器
  ros::Duration wait_; // 等待障碍物消失时间
  ros::Duration sleep_; // 调度器休眠时间，需要大于局部规划器的发布间隔
  bool sleeping_; // 调度器是否休眠
}; // class Dispatcher

}; // namespace local_planner

#endif // DISPATCHER_
