#ifndef DIFF_TRACKING_HPP_
#define DIFF_TRACKING_HPP_

#include "geometry_msgs/PoseStamped.h"
#include <random>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <Eigen/Dense>

namespace lqr_test {


class Diff_tracker
{
 public:
  Diff_tracker(ros::NodeHandle* n) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0.0, 5.0);
    x_ = dist(gen);
    y_ = dist(gen);
    ROS_INFO("target init [%f, %f]", x_, y_);

    std::uniform_real_distribution<double> dist0(0.0, 3.0);
    tracker_x_ = dist0(gen);
    tracker_y_ = dist0(gen);
    ROS_INFO("tracker init [%f, %f]", tracker_x_, tracker_y_);

    // 矩阵初始化
    Q_ << 1, 1, 1,
          1, 1, 1,
          1, 1, 1;
    R_ << 1, 1, 1,
          1, 1, 1,
          1, 1, 1;
    P_ = Q_;

    target_pub_ = n->advertise<nav_msgs::Path>("diff_target", 1);
    tracker_pub_ = n->advertise<nav_msgs::Path>("diff_tracker", 1);
    J_pub_ = n->advertise<nav_msgs::Path>("diff_J", 1);
    target_path_.header.frame_id = "map";
    tracker_path_.header = target_path_.header;
    J_path_.header = target_path_.header;
  };


  /**
   * @brief  测试 LQR 跟随
   * @param  dt  控制时间
   * @return double 
   */
  bool Run(const double dt, const double err) {
    UpdateTarget();
    Eigen::Vector3d u = GetTrackerU(100, dt);
    UpdateTracker(dt, u);
    // UpdateConsume(tracker_, dt, u);
    // ROS_INFO("J : %.3f", J_);
    timestamp_ += dt;

    geometry_msgs::PoseStamped ps;
    ps.header = target_path_.header;
    ps.pose.position.x = x_;
    ps.pose.position.y = y_;
    ps.pose.position.z = 0;
    target_path_.poses.push_back(ps);
    ps.pose.position.x = tracker_x_;
    ps.pose.position.y = tracker_y_;
    tracker_path_.poses.push_back(ps);
    ps.pose.position.y = J_;
    J_path_.poses.push_back(ps);
    target_pub_.publish(target_path_);
    tracker_pub_.publish(tracker_path_);
    J_pub_.publish(J_path_);

    return false;
  };


  /**
   * @brief  更新目标高度，[m]
   * @param  step 更新的高度步进范围,[m]
   */
  void UpdateTarget(const double step=0.5, const double offset=0.2) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(-step/2+offset, step/2+offset);
    x_ += dist(gen);
    y_ += dist(gen);
  };


  /**
   * @brief  更新跟踪器高度，[m]
   * @param  dt  控制时间，[s]
   * @param  u  [vl, vr, 0]
   */
  void UpdateTracker(const double dt, const Eigen::Vector3d u) {
    tracker_x_ += 0.5*(u[0]+u[1])*dt*cos(tracker_theta_);
    tracker_y_ += 0.5*(u[0]+u[1])*dt*sin(tracker_theta_);
    double w = (u[1]-u[0])*D_;
    tracker_theta_ += dt*w;
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
   * @param  loop  P 矩阵计算迭代次数
   * @param  dt  控制时间长度
   * @return [vl, vr, 0]
   */
  Eigen::Vector3d GetTrackerU(int loop, const double dt) {
    Eigen::Matrix3d A;
    A << x_-tracker_x_, 0, 0,
         0, y_-tracker_y_, 0,
         0, 0, theta_-tracker_theta_;
    Eigen::Matrix3d B;
    B << -0.5*dt*cos(tracker_theta_), -0.5*dt*sin(tracker_theta_), 0,
         -0.5*dt*cos(tracker_theta_), -0.5*dt*sin(tracker_theta_), 0,
         dt*D_, -dt*D_, 0;
 
    Eigen::Matrix3d P = P_;
    while (loop-- > 0) {
      P = A.transpose()*P_*A - (A.transpose()*P_*B)*(R_+B.transpose()*P_*B).reverse()*(B.transpose()*P_*A) + Q_;
      Eigen::Matrix3d diff = P-P_;
      Eigen::Matrix3d abs = diff.array().abs();
      ROS_INFO_STREAM("ABS:" << abs);
      if (abs.maxCoeff() < 1000) break;
    }
    P_ = P;
    Eigen::Matrix3d K = (R_+B.transpose()*P_*B).reverse()*(B.transpose()*P_*A);
    Eigen::Vector3d xk(tracker_x_, tracker_y_, tracker_theta_);
    Eigen::Vector3d u = -K*xk;
    ROS_INFO_STREAM("A:" << A << ",B:" << B << "P:" << P_ << "K:" << K << "u:" << u);
    return u;
  };


 private:

  double x_{0.0}; // 目标坐标
  double y_{0.0};
  double theta_{0.0};
  double tracker_x_{0.0}; // 跟随器坐标
  double tracker_y_{0.0};
  double tracker_theta_{0.0};
  double acc_ = 3.0; // 最大加速度
  double D_ = 0.3; // 轮距
  double J_ = 0.0;
  Eigen::Matrix3d Q_; // Q矩阵元素变大意味着希望跟踪偏差能够快速趋近于零
  Eigen::Matrix3d R_; // R矩阵元素变大意味着希望控制输入能够尽可能小
  Eigen::Matrix3d P_;
  double timestamp_ = 0.0;

  ros::Publisher target_pub_;
  ros::Publisher tracker_pub_;
  ros::Publisher J_pub_;
  nav_msgs::Path target_path_;
  nav_msgs::Path tracker_path_;
  nav_msgs::Path J_path_;
}; // Diff_tracker


} // namespace lqr_test


#endif // DIFF_TRACKING_HPP_
