#ifndef LKF_H
#define LKF_H

#include <navit_auto_dock/plugins/approach_dock_filter.h>
#include <cmath>
#include <ros/ros.h>
#include <tf2/utils.h>

namespace navit_auto_dock {

  /**
   * @brief 观测静态系统的一维卡尔曼滤波器
   */
  class Static_LKF
  {
    public:

      /**
       * @brief 初始化系统状态和估计方差
       * @param  x        系统初始状态
       * @param  p_sigma  估计的标准差
       */
      void init(const double x) { x_ = x; }

      /**
       * @brief 设置噪声的标准差
       * @param  p_sigma  系统状态噪声，来自估计的不确定性
       * @param  r_sigma  测量噪声，来自传感器
       * @param  q_sigma  过程噪声，来自 odom 偏移和传感器数据 odom 数据时间不同步
       */
      void setNoise(const double p_sigma, const double r_sigma, const double q_sigma) { 
        p_ = p_sigma*p_sigma;
        r_ = r_sigma*r_sigma;
        q_ = q_sigma*q_sigma;
      }

      /**
       * @brief 更新
       * @param  z  观测值
       */
      void update(const double z) {
        k_ = p_ / (p_ + r_);
        x_ = x_ + k_ * (z - x_);
        p_ = (1-k_) * p_;
      }

      /**
       * @brief 获取系统状态，update 后获取的是估计，predict 后获取的是预测
       * @return double 
       */
      double get() { return x_; }

      /**
       * @brief 预测系统状态和估计方差
       */
      void predict() {
        // x_ = x_;
        p_ = p_ + q_;
      }

    private:
      double x_; // 系统状态
      double k_; // 卡尔曼增益
      double r_; // 测量噪声方差
      double q_; // 过程噪声方差
      double p_; // 估计方差
  }; // Static_LKF

namespace plugins {
  class PerceptionLKF : public ApproachDockFilter
  {
    public:

      ~PerceptionLKF(){
        x_filter_.reset();
        y_filter_.reset();
        yaw_filter_.reset();
      }

      void initialize(const std::string& name,
                      const std::shared_ptr<tf2_ros::Buffer>& tf) override;

      void reset() override;

      /**
       * @brief 滤波，先更新后预测，返回的更新后的估计值
       * @param  dock_pose  充电桩位姿
       * @param  current_vel  机器控制
       */
      void update(geometry_msgs::PoseStamped& dock_pose,
                  const geometry_msgs::Twist& current_vel) override;

    protected:
      void init();    
      geometry_msgs::PoseStamped updateFilter(const geometry_msgs::PoseStamped& pose);
      void predictFilter(const geometry_msgs::Twist& cmd_vel);

      std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
      std::shared_ptr<navit_auto_dock::Static_LKF> x_filter_;
      std::shared_ptr<navit_auto_dock::Static_LKF> y_filter_;
      std::shared_ptr<navit_auto_dock::Static_LKF> yaw_filter_;
      ros::Publisher estimate_pose_pub_;
      ros::Publisher predict_pose_pub_;
      std::string pose_frame_;

      struct filter_noise {
        double estimate_sigma = 0.07; // 系统估计标准差
        double measure_sigma = 0.03; // 测量噪声标准差
        double project_sigma = 0.002; // 过程噪声标准差
      };

      filter_noise x_noise_;
      filter_noise y_noise_;
      filter_noise yaw_noise_;

      bool filter_init_ = false;
  }; // PerceptionLKF

} // namespace plugins

}
#endif
