#ifndef HEIGHT_TRACKING_HPP_
#define HEIGHT_TRACKING_HPP_

#include "geometry_msgs/PoseStamped.h"
#include <cmath>
#include <random>
#include <ros/ros.h>
#include <nav_msgs/Path.h>

namespace lqr_test {

class Height_tracker
{
 public:
  Height_tracker(ros::NodeHandle* n) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0.0, 10.0);
    height_ = dist(gen);
    ROS_INFO("height init %f", height_);

    std::uniform_real_distribution<double> dist0(0.0, 5.0);
    tracker_ = dist0(gen);
    ROS_INFO("tracker init %f", tracker_);
    if (fabs(height_-tracker_)<5) height_ += 5;

    Pf_ = Q_;

    height_pub_ = n->advertise<nav_msgs::Path>("target", 1);
    tracker_pub_ = n->advertise<nav_msgs::Path>("tracker", 1);
    J_pub_ = n->advertise<nav_msgs::Path>("J", 1);
    height_path_.header.frame_id = "map";
    tracker_path_.header = height_path_.header;
    J_path_.header = height_path_.header;
  };


  /**
   * @brief  测试 LQR 跟随
   * @param  dt  控制时间
   * @return double 
   */
  bool Run(const double dt, const double err) {
    UpdateHeight(1.0);
    double u = GetTrackerU(1000, dt);
    UpdateTracker(dt, u);
    UpdateConsume(tracker_, dt, u);
    ROS_INFO("J : %.3f", J_);
    timestamp_ += dt;

    geometry_msgs::PoseStamped ps;
    ps.header = height_path_.header;
    ps.pose.position.x = timestamp_;
    ps.pose.position.y = height_;
    ps.pose.position.z = 0;
    height_path_.poses.push_back(ps);
    ps.pose.position.y = tracker_;
    tracker_path_.poses.push_back(ps);
    ps.pose.position.y = J_;
    J_path_.poses.push_back(ps);
    height_pub_.publish(height_path_);
    tracker_pub_.publish(tracker_path_);
    J_pub_.publish(J_path_);
    // if (fabs(tracker_ - height_) <= err) {
    //   ROS_ERROR("tracker finished !");
    //   return true;
    // }
    return false;
  };


  /**
   * @brief  更新目标高度，[m]
   * @param  step 更新的高度步进范围,[m]
   * @return double 
   */
  double UpdateHeight(const double step=0.5) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(-step/2, step/2);
    double next_step = dist(gen);
    height_ += next_step;
    return height_;
  };


  /**
   * @brief  更新跟踪器高度，[m]
   * @param  dt  控制时间，[s]
   * @param  u  加速度控制，[m/(s^2)]
   * @return double
   */
  double UpdateTracker(const double dt, const double u) {
    tracker_ += 0.5*u*dt*dt;
    return tracker_;
  };


  /**
   * @brief  更新总消耗
   * @param  dt  控制的时间
   * @param  u  控制量
   * @return double 
   */
  double UpdateConsume(const double h, const double dt, double u) {
    double c = 0.005 * fabs(h) + 0.1 * fabs(u) * dt;
    J_ += c;
    return J_;
  };


  /**
   * @brief  计算控制量
   *         系统状态方程，跟随器高度 h_{k+1} = 1 * h_{k} + 0.5 * dt * dt * acc
   *         系统性能/代价方程，J = 状态代价：跟随器和目标高度差 0.5*dh^2 + 控制代价：由 GetConsume() 提供
   * @param  dt  控制时间长度
   * @return double，加速度
   */
  double GetTrackerU(int loop, const double dt) {
    double A = height_-tracker_, B = -0.5*dt*dt;    
    double P = Pf_;
    double diffP;
    while (loop-- > 0) {
      double tP = A*P*A - (A*P*B)/(R_+B*P*B)*(B*P*A) + Q_;
      // 当迭代插值小的时候可以结束计算，但是这个需要先测试判断。
      diffP = fabs(tP-P);
      if (diffP < 500) { P = tP; break; }
      P = tP;
    }
    double K = (B*P*A)/(R_+B*P*B);
    double u = -K*tracker_;
    ROS_INFO("A:%f, B:%f, P:%f, K:%f, u:%f, diffP:%f", A, B, P, K, u, diffP);
    return u;
  };


 private:

  double height_; // 目标高度
  double tracker_; // 跟随器高度
  double acc_ = 3.0; // 最大加速度
  double J_ = 0.0;
  double Q_ = 20; // Q矩阵元素变大意味着希望跟踪偏差能够快速趋近于零
  double R_ = 1; // R矩阵元素变大意味着希望控制输入能够尽可能小
  double Pf_ = 1;
  double timestamp_ = 0.0;

  ros::Publisher height_pub_;
  ros::Publisher tracker_pub_;
  ros::Publisher J_pub_;
  nav_msgs::Path height_path_;
  nav_msgs::Path tracker_path_;
  nav_msgs::Path J_path_;
}; // Height_tracker

} // namespace lqr_test


#endif // HEIGHT_TRACKING_HPP_
