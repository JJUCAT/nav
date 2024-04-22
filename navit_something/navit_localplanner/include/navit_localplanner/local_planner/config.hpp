#ifndef NAVIT_LOCAL_PLANNER__CONFIG_HPP_
#define NAVIT_LOCAL_PLANNER__CONFIG_HPP_

#include <ros/ros.h>

namespace local_planner {

class Configuration
{
 public:

  Configuration(ros::NodeHandle& nh)
    : nh_(nh)
  {
    Get();
  }

  ~Configuration() {}

  void Get()
  {
    nh_.param("HZ", HZ, {5});
    nh_.param("PUB_HZ", PUB_HZ, {1});
    nh_.param("ROBOT_FRAME", ROBOT_FRAME, {"base_link"});
    nh_.param("N_P", N_P, {10});
    nh_.param("N_H", N_H, {3});
    nh_.param("MAX_ALPHA", MAX_ALPHA, {M_PI / 3.0});
    nh_.param("MAX_PSI", MAX_PSI, {M_PI / 6.0});
    nh_.param("N_S", N_S, {1000});
    nh_.param("MAX_ACCELERATION", MAX_ACCELERATION, {1.0});
    nh_.param("TARGET_VELOCITY", TARGET_VELOCITY, {0.8});
    nh_.param("LOOKUP_TABLE_FILE_NAME", LOOKUP_TABLE_FILE_NAME, {std::string(std::getenv("HOME")) + "/lookup_table.csv"});
    nh_.param("MAX_ITERATION", MAX_ITERATION, {100});
    nh_.param("OPTIMIZATION_TOLERANCE", OPTIMIZATION_TOLERANCE, {0.1});
    nh_.param("MAX_YAWRATE", MAX_YAWRATE, {0.8});
    nh_.param("MAX_D_YAWRATE", MAX_D_YAWRATE, {2.0});
    nh_.param("MAX_WHEEL_ANGULAR_VELOCITY", MAX_WHEEL_ANGULAR_VELOCITY, {11.6});
    nh_.param("WHEEL_RADIUS", WHEEL_RADIUS, {0.125});
    nh_.param("TREAD", TREAD, {0.5});
    nh_.param("IGNORABLE_OBSTACLE_RANGE", IGNORABLE_OBSTACLE_RANGE, {1.0});
    nh_.param("VERBOSE", VERBOSE, {false});
    nh_.param("CONTROL_DELAY", CONTROL_DELAY, {1});
    nh_.param("TURN_DIRECTION_THRESHOLD", TURN_DIRECTION_THRESHOLD, {M_PI*3/4.0});
    nh_.param("ENABLE_SHARP_TRAJECTORY", ENABLE_SHARP_TRAJECTORY, {false});
    nh_.param("ENABLE_CONTROL_SPACE_SAMPLING", ENABLE_CONTROL_SPACE_SAMPLING, {false});
    nh_.param("DRIVE", DRIVE, {true});
    nh_.param("ACKERMANN", ACKERMANN, {false});
    nh_.param("WHEEL_BASE", WHEEL_BASE, {0.95});
    nh_.param("NEW_GOAL_SAMPLING", NEW_GOAL_SAMPLING, {false});
    nh_.param("NGS_R", NGS_R, {1.5});
    nh_.param("NGS_NA", NGS_NA, {8});
    nh_.param("NGS_NR", NGS_NR, {3});
    nh_.param("NGS_GY", NGS_GY, {0.785398});
    nh_.param("NGS_NY", NGS_NY, {3});
    nh_.param("SAMPLE_BEGIN", SAMPLE_BEGIN, {2.5});
    nh_.param("SAMPLE_STEP", SAMPLE_STEP, {1.0});
    nh_.param("PARALLEL_R", PARALLEL_R, {5.0});
    nh_.param("PARALLEL_N", PARALLEL_N, {2});
    nh_.param("PARALLEL_B", PARALLEL_B, {3.0});
    nh_.param("SAMPLE_MAX_DIS", SAMPLE_MAX_DIS, {5.0});
    nh_.param("SAMPLE_ENDLINE_TOUCH", SAMPLE_ENDLINE_TOUCH, {3.0});
    nh_.param("SAMPLE_PUSH", SAMPLE_PUSH, {1.8});
    nh_.param("SAMPLE_CLEAR_RADIUS", SAMPLE_CLEAR_RADIUS, {2.0});
    nh_.param("SAMPLE_SHADOW_K", SAMPLE_SHADOW_K, {1.2});
    nh_.param("REF_PLAN_SAMPLE_NUM", REF_PLAN_SAMPLE_NUM, {2});
    nh_.param("HEAD", HEAD, {-1});
    nh_.param("HEAD_COLLISION_COST", HEAD_COLLISION_COST, {240});
    nh_.param("BASE_COLLISION_COST", BASE_COLLISION_COST, {240});
    nh_.param("HEAD_SECURITY_COST", HEAD_SECURITY_COST, {220});
    nh_.param("BASE_SECURITY_COST", BASE_SECURITY_COST, {200});
    nh_.param("COLLISION_COST_OFFSET", COLLISION_COST_OFFSET, {15});
    nh_.param("SECURITY_IGNORE_DIST", SECURITY_IGNORE_DIST, {1.7});
    nh_.param("DIST_ERR", DIST_ERR, {6.0});
    nh_.param("YAW_ERR", YAW_ERR, {0.785398});
    nh_.param("RIGHT_PUNISHMENT", RIGHT_PUNISHMENT, {1.0});
    nh_.param("DIST_SCALE", DIST_SCALE, {60.0});
    nh_.param("YAW_SCALE", YAW_SCALE, {20.0});
    nh_.param("RIGHT_PUNISHMENT_SCALE", RIGHT_PUNISHMENT_SCALE, {40.0});
    nh_.param("CHECK_DISTANCE", CHECK_DISTANCE, {5.0});
    nh_.param("LOCAL_REACH_RANG", LOCAL_REACH_RANG, {0.8});
    nh_.param("REF_REACH_RANG", REF_REACH_RANG, {0.5});
    nh_.param("TARGET_V_FREE", TARGET_V_FREE, {false});
    nh_.param("FORBID_RIGHT_SAMPLE", FORBID_RIGHT_SAMPLE, {false});
    nh_.param("FORBID_RIGHT_RANG", FORBID_RIGHT_RANG, {2.5});
    nh_.param("REF_OPT_DIST_ERROR", REF_OPT_DIST_ERROR, {0.1});
    nh_.param("REF_OPT_YAW_ERROR", REF_OPT_YAW_ERROR, {0.17});
    nh_.param("AVOID_OPT_DIST_ERROR", AVOID_OPT_DIST_ERROR, {0.3});
    nh_.param("AVOID_OPT_YAW_ERROR", AVOID_OPT_YAW_ERROR, {0.54});
    nh_.param("DISPATCHER_WAIT", DISPATCHER_WAIT, {5.0});
    nh_.param("DISPATCHER_SLEEP", DISPATCHER_SLEEP, {1.0});
    nh_.param("TERMINAL_WAIT", TERMINAL_WAIT, {10.0});
    nh_.param("TERMINAL_CHECK", TERMINAL_CHECK, {6.0});
    nh_.param("BLOCK_TIMEOUT", BLOCK_TIMEOUT, {10.0});
    nh_.param("DUBINS_RADIUS", DUBINS_RADIUS, {1.0});
  }

  double HZ;
  double PUB_HZ; // 无异常情况下的局部路径发布频率，需要确保在发布频率内，机器走不完 CHECK_DISTANCE 距离
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
  double SAMPLE_BEGIN; // 采样起始距离 [m] # 2.5
  double SAMPLE_STEP; // 采样步进 [m]
  double SAMPLE_MAX_DIS; // 采样点与机器最大距离 [m]
  double SAMPLE_ENDLINE_TOUCH; // 采样区间终点线的挤压距离，机器到终点线距离小于该值就会挤压，推进采样区间，需要小于 SAMPLE_MAX_DIS
  double SAMPLE_PUSH; // 推进采样区间的距离
  double SAMPLE_CLEAR_RADIUS; // 清除靠近机器的采样点
  double SAMPLE_SHADOW_K;
  int REF_PLAN_SAMPLE_NUM; // 参考轨迹上的采样数 [1, +∞)
  double HEAD; // 车头碰撞检测位置
  int HEAD_COLLISION_COST; // 车头碰撞检测值
  int BASE_COLLISION_COST; // 车尾碰撞检测值
  int HEAD_SECURITY_COST; //  车头安全检测值
  int BASE_SECURITY_COST; // 车尾安全检测值
  int COLLISION_COST_OFFSET; // 碰撞检测值偏移，局部规划路径的碰撞检测值需要剪去该值，以提高局部绕障的距离
  double SECURITY_IGNORE_DIST; // 安全检测的忽视距离 [m]，碰撞检测值比安全检测值高，规划路径要求更高需要忽视一段机器附近的路径，否则规划的路径都不符合要求
  double LOCAL_REACH_RANG; // 该半径内认为到bool LocalPlanner::UpdateSamplingIndex() {达 local plan [m]
  double REF_REACH_RANG; // 该半径内认为到达 ref plan [m]
  double PARALLEL_R; // 平行采样宽 [m]
  int PARALLEL_N; // 平行采样数量
  double PARALLEL_B; // 后置平行采样距离 [m]
  bool TARGET_V_FREE; // 是否把当前 odom 速度设置为目标速度，这样 slp 少了加速过程
  bool FORBID_RIGHT_SAMPLE; // 禁止右侧采样，防止进入雷达盲区
  double FORBID_RIGHT_RANG; // 禁止右侧部分采样，防止进入雷达盲区，需要关闭 FORBID_RIGHT_SAMPLE，负数表示关闭该功能
  double REF_OPT_DIST_ERROR; // 回归采样的轨迹优化欧式距离容错
  double REF_OPT_YAW_ERROR; // 回归采样的轨迹优化航向角容错
  double AVOID_OPT_DIST_ERROR; // 绕障采样的轨迹优化欧式距离容错
  double AVOID_OPT_YAW_ERROR; // 绕障采样的轨迹优化航向角容错
  double CHECK_DISTANCE; // 前向采样检测的距离 [m]
  double BLOCK_TIMEOUT; // 堵塞超时时间
  double DUBINS_RADIUS; // dubins 半径 [m]

  // 轨迹评价
  double DIST_ERR;
  double YAW_ERR;
  double RIGHT_PUNISHMENT;
  double DIST_SCALE;
  double YAW_SCALE;
  double RIGHT_PUNISHMENT_SCALE;

  // 调度器
  double DISPATCHER_WAIT;
  double DISPATCHER_SLEEP;
  double TERMINAL_WAIT;
  double TERMINAL_CHECK;

 private:

  ros::NodeHandle nh_;

}; // class Configuration

} // namespace local_planner

#endif // NAVIT_LOCAL_PLANNER__CONFIG_HPP_
