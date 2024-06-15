#ifndef HEIGHT_TRACKING_HPP_
#define HEIGHT_TRACKING_HPP_

#include <random>
#include <ros/ros.h>


namespace lqr_test {

class Height_tracker
{
 public:
  Height_tracker() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0.0, 10.0);
    height_ = dist(gen);

    std::uniform_real_distribution<double> dist0(0.0, 5.0);
    tracker_ = dist0(gen);

    P_ = Q_;
  };

  // 更新目标高度
  double UpdateHeight(const double step=0.5) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(-step/2, step/2);
    double next_step = dist(gen);
    height_ += next_step;
    return height_;
  };

  /**
   * @brief  
   * @param  acc              My Param doc
   * @return double 
   */
  double GetConsume(const double acc) {
    double consume = fabs(acc) * 0.2 + 0.3;
    return consume;
  };

  // 系统状态方程，跟随器高度 h_{k+1} = 1 * h_{k} + dt * acc
  // 系统性能/代价方程，J = 状态代价：跟随器和目标高度差 0.5*dh^2 + 控制代价：由 GetConsume() 提供
  double GetTrackerU(const double dt) {
    double A = 1, B = dt;
    double P = Q_ + A*P_*A - (A*P_*B)/(R_+B*P_*B)*(B*P_*A);
    double K = (B*P*A)/(R_+B*P*B);
    double u = -K*tracker_;
    ROS_INFO("A:%f, B:%f, P:%f, K:%f, u:%f", A, B, P, K, u);
    return u;
  };


 private:
  double height_; // 目标高度
  double tracker_; // 跟随器高度
  double acc_ = 3.0; // 最大加速度
  double Q_ = 1;
  double R_ = 1;
  double P_ = 1;


}; // Height_tracker

} // namespace lqr_test


#endif // HEIGHT_TRACKING_HPP_
