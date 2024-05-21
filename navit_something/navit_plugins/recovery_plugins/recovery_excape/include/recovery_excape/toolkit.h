#ifndef RECOVERY_EXCAPE__TOOLKIT_HPP_
#define RECOVERY_EXCAPE__TOOLKIT_HPP_

#include "Eigen/Core"
#include "angles/angles.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"
#include "navit_costmap/costmap_2d.h"


namespace recovery_excape {

/**
  * @brief  根据机器状态预测下个 dt 时刻的位姿
  * @param  pose  当前位姿
  * @param  v  线速度
  * @param  ws  角速度或前轮转角（阿克曼模型）
  * @param  dt  时间步进
  * @param  wheel_base  阿克曼轮距（前轮到后轮中心）
  * @param  use_steering  是否是阿克曼模型
  * @return Eigen::Vector3d 
  */
Eigen::Vector3d Predict(const Eigen::Vector3d pose, const double v,
  const double ws, const double dt, const double wheel_base, const bool use_steering = false);


/**
 * @brief 差速轮和阿克曼运动模型预测位姿
 * @param  pose             当前位姿
 * @param  v                当前线速度
 * @param  ws               当前角速度或阿克曼的转向轮角度
 * @param  dt               运动所长时间
 * @param  wheel_base       阿克曼的轴距
 * @param  use_steering     是否将 ws 参数作为阿克曼转向轮角度
 * @return geometry_msgs::Pose 返回运动 dt 时间后的位姿
 */
geometry_msgs::Pose Predict(const geometry_msgs::Pose pose, const double v,
  const double ws, const double dt, const double wheel_base, const bool use_steering = false);


/**
  * @brief  根据控制序列和机器人位姿，时间步进计算轨迹
  * @param  u_sequence  控制序列 [v, w]
  * @param  robot_pose  机器人在 recovery 地图中的位姿
  * @param  time_step  时间步进
  * @param  x_sequence  状态序列 [x, y, yaw]
  * @return size_t  状态序列数量
  */
size_t GenPath(const std::vector<Eigen::Vector2d>* u_sequence,
  const geometry_msgs::PoseStamped& robot_pose,
  const double time_step, std::vector<Eigen::Vector3d>& x_sequence);

/**
 * @brief  角速度转前轮转角
 * @param  v  线速度
 * @param  w  角速度
 * @param  wheel_base  轮距
 * @param  s_limit  前轮转向范围 [-s_limit, s_limit]
 * @return double 
 */
double W2S(const double v, const double w, const double wheel_base, const double s_limit);

int GetDir(const geometry_msgs::Pose p, const navit_costmap::Costmap2D& map, const double r);

} // namespace recovery_excape

#endif // RECOVERY_EXCAPE__TOOLKIT_HPP_
