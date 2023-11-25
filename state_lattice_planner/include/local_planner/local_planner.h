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

#include <state_lattice_planner/state_lattice_planner.h>

namespace local_planner {

class LocalPlanner
{
 public:

  struct TrackingPath
  {
    nav_msgs::Path plan;
    size_t idx;
  };

  LocalPlanner() = delete;

  /**
   * @brief Construct a new Local Planner object
   */
  LocalPlanner(std::shared_ptr<navit_costmap::Costmap2DROS>& costmap_ros);

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

 private:
  /**
   * @brief  odom 回调，获取速度，角速度信息
   * @param  msg  odom 数据
   */
  void OdomCallback(const nav_msgs::OdometryConstPtr& msg);

  /**
   * @brief 加载参数
   */
  void LoadParams();

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
   * @brief  在参考路径上选择一个点作为局部规划的 goal
   * @param  step        选点的步进 [m]
   * @param  update_idx  更新 ref plan 的参考采样点
   * @return Eigen::Vector3d 
   */
  Eigen::Vector3d RefForwardSampling(const float step, const bool update_idx = true);

  /**
   * @brief  平行采样
   * @param  goal    目标
   * @param  states  返回的采样点
   */
  void Parallel_Sampling(
    const Eigen::Vector3d goal, std::vector<Eigen::Vector3d>& states);

  /**
   * @brief  ref plan 后退采样
   * @param  back_step     后退步长 [m]
   * @param  forward_step  前进步长 [m]
   * @param  states  返回的采样点
   */
  void Lookback_Sampling(
    const double back_step, const double forward_step,
    std::vector<Eigen::Vector3d>& states);

  /**
   * @brief 清空历史采样点
   */
  void ClearLookbackSample() {
    lookback_sample_index_.clear();
  }

  /**
   * @brief  判断是否需要在参考路径上前向选点采样
   * @param  check_dist      检测距离 [m]
   * @param  plan_connected  局部路径和参考路径是否连接
   * @param  local_plan_end  局部路径碰撞检测的结尾下标
   * @param  ref_plan_end    参考路径碰撞检测的结尾下标
   * @return true   需要参考路径前向采样
   * @return false  不需要
   */
  bool IsNeedForwardSampling(
    const float check_dist, const bool plan_connected,
    size_t& local_plan_end, size_t& ref_plan_end);

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
  void PoseTransform(const Eigen::Vector3d& transformer,
                     const geometry_msgs::PoseStamped& src_pose,
                     geometry_msgs::PoseStamped& tgt_pose);

  void PoseTransform(const Eigen::Vector3d& transformer,
                     const Eigen::Vector3d& src_pose,
                     Eigen::Vector3d& tgt_pose);

  void PoseTransform(const Eigen::Vector3d& transformer,
                     const Eigen::Vector3d& src_pose,
                     geometry_msgs::PoseStamped& tgt_pose);

  void PoseTransform(const Eigen::Vector3d& transformer,
                     const geometry_msgs::PoseStamped& src_pose,
                     Eigen::Vector3d& tgt_pose);

  /**
   * @brief  检查机器是否在地图中有碰撞
   * @param  pose            在地图中的位姿
   * @param  collision_cost  碰撞值
   * @return true 
   * @return false 
   */
  bool IsRobotCollision(const geometry_msgs::PoseStamped& pose,
                        const unsigned int collision_cost);

  bool IsRobotCollision(const Eigen::Vector3d& pose,
                        const unsigned int collision_cost);

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
   * @brief  检查局部路径和全局路径是否相连
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

  void VizTrajectories(
    const std::vector<MotionModelDiffDrive::Trajectory>& trajectories,
    const double r, const double g, const double b,
    const int trajectories_size, const ros::Publisher& pub);

  void VizTrajectory(
    const MotionModelDiffDrive::Trajectory& trajectory,
    const double r, const double g, const double b, const ros::Publisher& pub);

  ros::NodeHandle nh_;
  ros::NodeHandle local_nh_;
  tf::TransformListener listener_;
  Eigen::Vector3d plan_in_map_frame_;
  Eigen::Vector3d plan_in_robot_frame_;
  Eigen::Vector3d robot_in_map_frame_;
  geometry_msgs::PoseStamped robot_pose_; 
  std::shared_ptr<navit_costmap::Costmap2DROS> costmap_ros_;  
  std::shared_ptr<navit_costmap::Costmap2D> costmap_;
  std::vector<size_t> lookback_sample_index_;
  TrackingPath ref_plan_;
  TrackingPath local_plan_; // 和 costmap_ros_ 同个 frame_id
  bool plan_connected_ = true;
  StateLatticePlanner slp_;
  ros::Publisher candidate_trajectories_pub_;
  ros::Publisher candidate_trajectories_no_collision_pub_;
  ros::Publisher selected_trajectory_pub_;
  ros::Publisher goal_sampling_pub_;
  ros::Publisher goal_in_robot_frame_pub_;
  ros::Publisher check_plan_pub_;
  ros::Publisher local_plan_pub_;
  ros::Subscriber odom_sub_;
  std::vector<Eigen::Vector3d> xyyaw_table_;
  nav_msgs::Odometry cur_odom_;
  std::shared_ptr<StateLatticePlanner::Critic> critic_;

  // 参数
  double HZ;
  std::string ROBOT_FRAME;
  int N_P;
  int N_H;
  int N_S;
  double MAX_ALPHA;
  double MAX_PSI;
  double MAX_ACCELERATION;
  double TARGET_VELOCITY;
  std::string LOOKUP_TABLE_FILE_NAME;
  int MAX_ITERATION;
  double OPTIMIZATION_TOLERANCE;
  double MAX_YAWRATE;
  double MAX_D_YAWRATE;
  double MAX_WHEEL_ANGULAR_VELOCITY;
  double WHEEL_RADIUS;
  double TREAD;
  double IGNORABLE_OBSTACLE_RANGE;
  bool VERBOSE;
  int CONTROL_DELAY;
  double TURN_DIRECTION_THRESHOLD;
  bool ENABLE_SHARP_TRAJECTORY;
  bool ENABLE_CONTROL_SPACE_SAMPLING;
  bool DRIVE;
  bool ACKERMANN;
  double WHEEL_BASE;
  bool NEW_GOAL_SAMPLING;
  double NGS_R;
  int NGS_NA;
  int NGS_NR;
  double NGS_GY;
  int NGS_NY;
  double SAMPLE_STEP; // 采样步进 [m]
  double SMAPLE_MAX_DIS; // 采样点与机器最大距离 [m]
  double LOOKBACK; // 历史采样距离 [m]
  double HEAD; // 车头碰撞检测位置
  int COLLISION_COST; // 碰撞检测值，[-1,100]
  double LOCAL_REACH_RANG; // 该半径内认为到达 local plan [m]
  double REF_REACH_RANG; // 该半径内认为到达 ref plan [m]
  double PARALLEL_R; // 平行采样宽 [m]
  int PARALLEL_N; // 平行采样数量
  double PARALLEL_B; // 后置平行采样距离 [m]

  // 轨迹评价
  double DIST_ERR;
  double YAW_ERR;
  double ANGULAR_ERR;
  double DIST_SCALE;
  double YAW_SCALE;
  double ANGULAR_SCALE;
  double CHECK_DISTANCE; // 前向采样检测的距离 [m]

}; // class LocalPlanner

} // namespace local_planner

#endif /* LOCAL_PLANNER_H_ */
