/**
 * @copyright Copyright (c) {2022} LZY
 * @author LZY (linziyan@yijiahe.com)
 * @date 2023-11-08
 * @brief
 */


#ifndef LOCAL_PLANNER_H_
#define LOCAL_PLANNER_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <navit_costmap/costmap_2d_ros.h>
#include <angles/angles.h>

#include <state_lattice_planner/state_lattice_planner.h>
#include <local_planner/dispatcher.hpp>
#include <local_planner/sand_clock.hpp>
#include <local_planner/dubins.h>
#include <local_planner/config.hpp>
#include <local_planner/security_detector.h>

namespace local_planner {

class LocalPlanner
{
 public:

  struct TrackingPath
  {
    nav_msgs::Path plan;
    size_t idx;
  };

  struct SamplingIndex
  {
    size_t idx;
    size_t end;
    size_t jump;
    double jump_dist;
    double cache_dist;
    bool init;
  };

  LocalPlanner() = delete;

  /**
   * @brief Construct a new Local Planner object
   */
  LocalPlanner(std::shared_ptr<navit_costmap::Costmap2DROS>& costmap_ros,
               std::shared_ptr<tf2_ros::Buffer>& tf);

  /**
   * @brief  设置全局参考路径
   * @param  plan  全局参考路径
   */
  void SetPlan(const nav_msgs::Path& plan);

  /**
   * @brief  参考路径是否规划完成
   * @return true
   * @return false
   */
  bool IsFinished();

  /**
   * @brief  获取局部规划路径
   * @param  plan  局部路径
   * @return true
   * @return false
   */
  bool GetLocalPlan(nav_msgs::Path& plan);

  /**
   * @brief  通知控制器需要先等下
   * @return true
   * @return false
   */
  bool Wait()
  {
    std::unique_lock<std::mutex> lock(wait_mutex_);
    if (wait_) ROS_WARN_THROTTLE(2, "[LP] notify controller to stop.");
    return wait_;
  }

  /**
   * @brief  局部规划完成再取消控制的等待
   */
  void Pass()
  {
    std::unique_lock<std::mutex> lock(wait_mutex_);
    wait_ = false;
  }

  /**
   * @brief  是否堵塞超时
   * @return true
   * @return false
   */
  bool Block()
  {
    std::unique_lock<std::mutex> lock(wait_mutex_);
    if (wait_ && sand_clock_.Timeout()) {
      ROS_ERROR("[LP] local planner check blocked !");
      return true;
    }
    return false;
  }

  /**
   * @brief  设置终点等待参数
   * @param  terminal_check  终点前等待的检测距离
   * @param  terminal_wait  终点前等待时间
   */
  void SetWaitConfig(double terminal_check, double terminal_wait);

  /**
   * @brief  设置局部规划频率
   * @param  hz  局部规划频率
   */
  void SetHz(const double hz) { cfg_->HZ = hz; }

  /**
   * @brief  终止这次 refplan 的绕障
   */
  void Terminate() { ClearRefPlan(); }

  /**
   * @brief  设置碰撞检测值
   * @param  level  碰撞检测值等级 [0,1]，等级 0 安全要求最高
   */
  void SetCollisionDetectLevel(const int level = 0);

 private:
  /**
   * @brief  odom 回调，获取速度，角速度信息
   * @param  msg  odom 数据
   */
  void OdomCallback(const nav_msgs::OdometryConstPtr& msg);

  /**
   * @brief 更新“参考路径-地图”和“地图-机器”的坐标变换
   * @return true
   * @return false
   */
  bool UpdateFrame();

  /**
   * @brief  更新地图
   * @return true
   * @return false
   */
  bool UpdateMap();

  /**
   * @brief 更新机器在 map 中的位姿
   * @return true
   * @return false
   */
  bool UpdateRobotPose();


  bool UpdatePlanIndex(TrackingPath& plan, const double rang, const double check_dist, bool use_closest_idx = false);

  /**
   * @brief 更新机器在 local plan 的下标
   * @return true
   * @return false  没有在 local plan 找到位置，认为机器走完局部规划路径了
   */
  bool UpdateLocalPlanIndex(const double rang);

  /**
   * @brief 更新机器在 ref plan 的下标
   * @return true
   * @return false  没有在 local plan 找到位置
   */
  bool UpdateRefPlanIndex(const double rang);

  /**
   * @brief 获取在参考路径上的投影距离

   * @return double
   */
  double GetShadow(const bool shadow_on_refplan);

  bool InsideSamplingIndexCircle(const double circle_r);

  /**
   * @brief 更新在 ref plan 的采样区间
   * @return true
   * @return false
   */
  bool UpdateSamplingIndex();

  void ResetSamplingJumpDist();

  void SetSamplingIndex(const size_t index);

  void SamplingJumpForward();

  /**
   * @brief  设置局部路径
   * @param  plan  局部路径，基于地图坐标系
   */
  void SetLocalPlan(const nav_msgs::Path& plan);

  /**
   * @brief 清空局部规划路径
   */
  void ClearLocalPlan();

  /**
   * @brief 清空参考规划路径
   */
  void ClearRefPlan();

  /**
   * @brief 清空采样数据
   */
  void ClearSamplingIndex();

  /**
   * @brief  在参考路径上的采样点
   * @param  num            采样数量
   * @return std::vector<Eigen::Vector3d>
   */
  std::vector<Eigen::Vector3d> RefForwardSampling(const size_t num);

  /**
   * @brief  平行采样
   * @param  sample  采样点
   * @param  states  返回的采样点
   */
  void Parallel_Sampling(
    const std::vector<Eigen::Vector3d>& sample, std::vector<Eigen::Vector3d>& states);

  /**
   * @brief  回归 ref plan 的采样，回归排在 states 的前面，方便优化的时候区分参数
   * @param  sample  采样点
   * @param  states  返回的采样点
   * @return size_t  参考路径上的采样数量
   */
  size_t Reference_Sampling(
    const std::vector<Eigen::Vector3d>& sample,
    std::vector<Eigen::Vector3d>& states);

  /**
   * @brief 清空历史采样点
   */
  void ClearRefplanSample() {
    refplan_sample_index_.clear();
  }

  /**
   * @brief  是否更新局部规划，局部规划可能规划的比较长，走一段后就得更新
   * @param  check_dist  局部规划走了 check_dist 距离就可以再更新了
   * @return true
   * @return false
   */
  bool IsNeedToUpdateLocalPlan(const float check_dist);

  /**
   * @brief  检测路径是否堵塞
   * @param  check_dist      检测距离 [m]
   * @param  plan_connected  局部路径和参考路径是否连接
   * @param  local_plan_end  局部路径碰撞检测的结尾下标
   * @param  ref_plan_end    参考路径碰撞检测的结尾下标
   * @param  is_collid_soon  即将发生碰撞
   * @return true   需要参考路径前向采样
   * @return false  不需要
   */
  bool IsPlanBlocked(
    const float check_dist, const bool plan_connected,
    size_t& local_plan_end, size_t& ref_plan_end, bool& is_collid_soon);

  /**
   * @brief  求两个 pose 的 x，y 距离
   * @param  p0  pose0
   * @param  p1  pose1
   * @return float
   */
  float Poses2DDistance(
    const geometry_msgs::PoseStamped& p0, const geometry_msgs::PoseStamped& p1) {
    return std::hypot(p0.pose.position.x - p1.pose.position.x,
                      p0.pose.position.y - p1.pose.position.y);
  }

  float Poses2DDistance(
    const geometry_msgs::PoseStamped& p0, const Eigen::Vector3d& p1) {
    return std::hypot(p0.pose.position.x - p1(0),
                      p0.pose.position.y - p1(1));
  }

  /**
   * @brief  把 src_pose 转到 OriginInTargetFrame 的坐标系中
   * @param  transformer 原坐标点的坐标系 O 在目标坐标系中的坐标
   * @param  src_pose  原坐标
   * @param  tgt_pose  转换后的坐标
   */
  void PoseTransform(const geometry_msgs::TransformStamped& transformer,
    const geometry_msgs::PoseStamped& src_pose, geometry_msgs::PoseStamped& tgt_pose);

  void PoseTransform(const geometry_msgs::TransformStamped& transformer,
    const Eigen::Vector3d& src_pose, Eigen::Vector3d& tgt_pose);

  void PoseTransform(const geometry_msgs::TransformStamped& transformer,
    const Eigen::Vector3d& src_pose, geometry_msgs::PoseStamped& tgt_pose);

  void PoseTransform(const geometry_msgs::TransformStamped& transformer,
    const geometry_msgs::PoseStamped& src_pose, Eigen::Vector3d& tgt_pose);

  /**
   * @brief  检查机器是否在地图中有碰撞
   * @param  pose            在地图中的位姿
   * @param  collision_cost  碰撞值
   * @return true
   * @return false
   */
  bool IsRobotCollision(const geometry_msgs::PoseStamped& pose,
    const unsigned int base_cost, const unsigned int head_cost);

  bool IsRobotCollision(const Eigen::Vector3d& pose,
    const unsigned int base_cost, const unsigned int head_cost);

  /**
   * @brief  局部规划成功后更新要发布的路径结束下标
   * @param  local_plan_end   局部路径检测结束位置
   * @param  ref_plan_end     参考路径检测结束位置
   * @param  plan_connected   局部路径和参考路径是否连接
   * @param  ref_plan_dist    参考路径检测距离
   */
  void UpdatePubPlanIndex(size_t& local_plan_end, size_t& ref_plan_end,
    const bool plan_connected, double ref_plan_dist);

  /**
   * @brief  融合局部路径和参考路径
   * @param  plan_connected  局部路径和参考路径是否连接
   * @param  local_plan_end  局部路径检测结束位置
   * @param  ref_plan_end    参考路径检测结束位置
   * @param  plan            融合后的路径
   */
  void MergePlan(const bool plan_connected, const size_t local_plan_end,
                 const size_t ref_plan_end, nav_msgs::Path& plan);

  /**
   * @brief  检查局部路径和全局路径是否相连，在局部规划成功时候使用
   * @param  trajectory_end  局部路径终点，基于机器坐标系
   */
  void CheckPlanConnected(const Eigen::Vector3d trajectory_end);

  /**
   * @brief  state lattice planner 的轨迹转 map 下的路径
   * @param  trajectory  基于 base_link 的轨迹
   * @param  plan        基于 map 的路径
   */
  void Trajectory2Plan(const std::vector<Eigen::Vector3d>& trajectory,
                       nav_msgs::Path& plan);

  /**
   * @brief  可视化目标，基于机器坐标系
   * @param  goal  目标点，基于机器坐标系
   */
  void VizGoal(const Eigen::Vector3d& goal);

  /**
   * @brief  可视化目标采样点
   * @param  states  目标采样点，基于机器坐标系
   */
  void VizGoalSampling(const std::vector<Eigen::Vector3d>& states);

  /**
   * @brief  可视化检测路径
   * @param  plan_connected  局部路径和参考路径是否连接
   * @param  local_plan_end  局部路径结尾
   * @param  ref_plan_end    参考路径结尾
   */
  void VizCheckPlan(const bool plan_connected,
    const size_t local_plan_end, const size_t ref_plan_end);

  void VizCostTrajectories(
    const std::map<double, MotionModelDiffDrive::Trajectory>& trajectories,
    const ros::Publisher& pub);

  void ClearCostTrajectories(const ros::Publisher& pub);

  double GetTargetV(const nav_msgs::Odometry& cur_odom);
  void WaitForOdom();

  void VizMarker(const ros::Publisher& pub,
    const geometry_msgs::PoseStamped& pose, const int type, const float size,
    const float r, const float g, const float b, const float a);

  /**
   * @brief  更新地图，tf，机器在地图中位姿，采样区间
   * @param  is_lost  是否走丢
   * @return true
   * @return false
   */
  bool Update(bool& is_lost);

  /**
   * @brief  采样
   * @param  goal    参考路径上的目标点，基于机器坐标系
   * @param  states  由 goal 展开的备选采样，参考路径上的采样点排在前面，基于机器坐标系
   * @return size_t  参考路径上的采样数量
   */
  size_t Sampling(Eigen::Vector3d& goal , std::vector<Eigen::Vector3d>& states);

  /**
   * @brief  根据样本生成轨迹
   * @param  states          样本，参考路径上的采样点排在前面，基于机器坐标系
   * @param  ref_sample_num  参考轨迹上的采样数量
   * @param  trajectories    轨迹，基于机器坐标系
   * @return true
   * @return false
   */
  bool GenerateTrajectoriesToSample(const std::vector<Eigen::Vector3d>& states,
    const size_t ref_sample_num, std::vector<MotionModelDiffDrive::Trajectory>& trajectories);

  /**
   * @brief  dubins 路径规划
   * @param  states          样本，参考路径上的采样点排在前面，基于机器坐标系
   * @param  ref_sample_num  参考轨迹上的采样数量
   * @param  trajectories    轨迹，基于机器坐标系
   * @return true 
   * @return false 
   */
  bool GenerateDubinsToSample(const std::vector<Eigen::Vector3d>& states,
    const size_t ref_sample_num, std::vector<MotionModelDiffDrive::Trajectory>& trajectories);

  /**
   * @brief  挑选出最佳轨迹
   * @param  trajectories       候选轨迹
   * @param  goal               参考轨迹上的目标点，基于机器坐标系
   * @return true
   * @return false
   */
  bool GetBetterTrajectories(const std::vector<MotionModelDiffDrive::Trajectory>& trajectories, const Eigen::Vector3d& goal);

  /**
   * @brief  获取安全的轨迹
   * @param  cost_trajectory_map  带代价的轨迹
   * @return const_iterator  代价轨迹的下标，@end 表示没有合适的轨迹
   */
  std::map<double, MotionModelDiffDrive::Trajectory>::const_iterator
  GetSecurePlan(const std::map<double, MotionModelDiffDrive::Trajectory>& cost_trajectory_map);

  /**
   * @brief  局部路径末尾一段路径改为线性插值，连接到参考路径上
   * @param  plan_connected  局部路径是否与参考路径连接
   * @param  insert_dist  局部路径末尾要修改为线性插值的长度
   * @return size_t
   */
  size_t LinearInsert2RefPlan(const bool plan_connected, const double insert_dist);

  /**
   * @brief  是否接近终点
   * @return true
   * @return false
   */
  bool IsApproachTerminal();

  /**
   * @brief  判断是否要停下等待
   * @param  is_collid_soon  即将碰撞
   * @return true
   * @return false
   */
  bool Wait(bool is_collid_soon);

  /**
   * @brief  是否把终点发布出去了
   * @param  ref_plan_end  发布的参考路径结尾
   * @return true
   * @return false
   */
  bool CheckTerminal(const size_t ref_plan_end);

  /**
   * @brief  已经发布过最后的路径了
   * @return true
   * @return false
   */
  bool HasPubFinalPlan();

  /**
   * @brief  无异常情况下，是否要发布局部路径
   * @return true
   * @return false
   */
  bool PubLocalPlan();

  ros::NodeHandle nh_;
  ros::NodeHandle local_nh_;
  tf::TransformListener listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  Eigen::Vector3d plan_in_map_frame_;
  Eigen::Vector3d plan_in_robot_frame_;
  Eigen::Vector3d robot_in_map_frame_;
  geometry_msgs::TransformStamped plan_to_map_transform_;
  geometry_msgs::TransformStamped plan_to_robot_transform_;
  geometry_msgs::TransformStamped robot_to_map_transform_;
  geometry_msgs::PoseStamped robot_pose_;
  std::shared_ptr<navit_costmap::Costmap2DROS> costmap_ros_;
  std::shared_ptr<navit_costmap::Costmap2D> costmap_;
  std::vector<size_t> refplan_sample_index_;
  TrackingPath ref_plan_;
  TrackingPath local_plan_; // 和 costmap_ros_ 同个 frame_id
  SamplingIndex sampling_idx_;
  bool plan_connected_ = true; // 一开始在参考轨迹上认为就是连接的
  StateLatticePlanner slp_;
  ros::Publisher candidate_trajectories_pub_;
  ros::Publisher candidate_trajectories_no_collision_pub_;
  ros::Publisher selected_trajectory_pub_;
  ros::Publisher cost_trajectories_pub_;
  ros::Publisher goal_sampling_pub_;
  ros::Publisher goal_in_robot_frame_pub_;
  ros::Publisher check_plan_pub_;
  ros::Publisher ref_plan_pub_;
  ros::Publisher local_plan_pub_;
  ros::Subscriber odom_sub_;
  std::vector<Eigen::Vector3d> xyyaw_table_;
  nav_msgs::Odometry cur_odom_;
  std::shared_ptr<StateLatticePlanner::Critic> critic_;
  bool subscribed_odom_ = false;
  bool odom_udpated_ = false;
  Dispatcher dispatcher_;
  std::mutex wait_mutex_;
  bool wait_ = false; // 通知控制器需要停下
  std::mutex wait_config_mutex_;
  bool final_plan_ = false;
  double check_dist_;
  ros::Time pub_timer_;
  SandClock sand_clock_;
  bool finished_{false};
  bool first_pub_{false};

  ros::Publisher sample_start_pub_;
  ros::Publisher sample_jump_pub_;
  ros::Publisher sample_end_pub_;

  std::shared_ptr<Configuration> cfg_;
  std::shared_ptr<SecurityDetector> security_detector_;

}; // class LocalPlanner

} // namespace local_planner

#endif /* LOCAL_PLANNER_H_ */
