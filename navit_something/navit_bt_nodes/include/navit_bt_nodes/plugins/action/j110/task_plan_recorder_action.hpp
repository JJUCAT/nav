#ifndef TASK_PLAN_RECORDER_ACTION_HPP_
#define TASK_PLAN_RECORDER_ACTION_HPP_

#include "geometry_msgs/PoseStamped.h"
#include "navit_bt_nodes/bt_action_node.h"
#include <string>
#include <google/protobuf/util/json_util.h>
#include <navit_bt_nodes/bt_exception.hpp>
#include <navit_common/log.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include "nav_msgs/Path.h"
#include "ros/assert.h"
#include "ros/node_handle.h"

namespace navit_bt_nodes {

class TaskPlanRecorderAction : public BT::ActionNodeBase
{
 public:

  TaskPlanRecorderAction(const std::string& xml_tag_name, const BT::NodeConfiguration& conf)
    : BT::ActionNodeBase(xml_tag_name, conf), tf_listener_(tf_buffer_)
  {
    ros::NodeHandle node("~");
    lastest_recorder_pub_ = node.advertise<geometry_msgs::PoseStamped>("BT_task_recorder", 1);
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::Path>("task_plan", "task_plan"), // 任务路径
      BT::BidirectionalPort<std::vector<size_t>>("recorder", "recorder"), // 记录任务路径执行情况
      BT::BidirectionalPort<bool>("on_taskplan", "on_taskplan"), // 机器是否抵达任务路径上
      BT::InputPort<double>("near", "near"), // 在 near 半径范围内的最近点是机器的最新记录位置，更新 recorder
      BT::InputPort<double>("search", "search"), // 当 near 内无法搜索到机器位置，就会循环遍历 search 搜索机器
      BT::InputPort<double>("capture", "capture"), // 在 search 时候的 capture 半径范围内找到机器就估计机器可能回到这里
      BT::InputPort<double>("attach", "attach"), // 连续 capture 了 attach 距离就确认机器回到这里，更新 recorder
    };
  }

 private:

  /**
   * @brief  加载树节点参数
   * @param  task_plan  任务路径 [in]
   * @param  recorder   任务记录 [in/out]
   * @param  near       更新半径 [in]
   * @param  search     搜索距离 [in]
   * @param  capture    捕获半径 [in]
   * @param  attach     回归确认距离 [in]
   * @param  on_taskplan  是否在任务路径上 [in/out]
   */
  void LoadArg(nav_msgs::Path& task_plan, std::vector<size_t>& recorder,
    double& near, double& search, double& capture, double& attach, bool& on_taskplan);

  /**
   * @brief  更新机器在任务路径坐标系中的位姿
   * @param  robot_frame  机器人坐标系
   * @param  plan_frame  任务路径坐标系
   * @param  robot  任务路径坐标系中的机器位姿
   * @return true 
   * @return false 
   */
  bool UpdateRobotPose(const std::string robot_frame,
    const std::string plan_frame, geometry_msgs::PoseStamped& robot);

  /**
   * @brief  在 [begin, end) 之间搜索 radius 范围内的最近点，没有找到最近点返回 end
   * @param  begin  起点
   * @param  end  终点
   * @param  robot  机器
   * @param  radius  搜索半径
   * @return std::vector<geometry_msgs::PoseStamped>::iterator 
   */
  std::vector<geometry_msgs::PoseStamped>::const_iterator
  FindClosest(std::vector<geometry_msgs::PoseStamped>::const_iterator begin,
    std::vector<geometry_msgs::PoseStamped>::const_iterator end,
    const geometry_msgs::Point& robot, const double radius);

  /**
   * @brief  在 poses 上起点 start 开始 distance 距离的下标
   * @param  poses  路径点
   * @param  start_index  起点下标
   * @param  distance  距离
   * @param  offset_index  检测距离结束的下标
   * @return double  返回检查的距离
   */
  double GetDistanceOffset(const std::vector<geometry_msgs::PoseStamped>& poses,
    const size_t start_index, const double distance, size_t& offset_index);

  /**
   * @brief  在 near 半径内更新任务记录
   * @param  task_plan  任务路径
   * @param  recorder  任务记录
   * @param  robot  机器位置
   * @param  near  更新半径
   * @return true 
   * @return false 
   */
  bool UpdateRecorderInNear(const nav_msgs::Path& task_plan, std::vector<size_t>& recorder,
    const geometry_msgs::Point& robot, const double near);

  /**
   * @brief  搜索机器
   * @param  task_plan  任务路径
   * @param  robot  机器位置
   * @param  reset_index  重置搜索的起点下标
   * @param  search  搜索距离
   * @param  capture  捕获半径
   * @param  attach  回归确认距离
   * @return true 
   * @return false 
   */
  bool Search(const nav_msgs::Path& task_plan, const geometry_msgs::Point& robot,
    const size_t reset_index, const double search, const double capture, const double attach, size_t& captured);

  /**
   * @brief  插入捕获点的下标
   * @param  index  捕获的下标
   */
  void InsertCapturedIndex(const size_t index);

  /**
   * @brief  是否捕获机器一段距离了
   * @param  poses  检查路径点
   * @param  dist  检测距离
   * @return true 
   * @return false 
   */
  bool IsCapturedDistanceLonggerThan(
    const std::vector<geometry_msgs::PoseStamped>& poses, const double dist);

  /**
   * @brief  是否捕获了一段时间
   * @param  time_limit  捕获时间上限
   * @return true 
   * @return false 
   */
  bool IsCapturedTimeLonggerThan(const double time_limit);

  /**
   * @brief  开始捕获计时
   */
  void TimerTick();

  /**
   * @brief  获取计时重置
   */
  void TimerReset();

  /**
   * @brief  发布路径记录
   * @param  plan  路径
   * @param  recorder  记录走过的下标
   */
  void PubRecord(const nav_msgs::Path& plan, const std::vector<size_t>& recorder);

  /**
   * @brief  检测机器是否在任务路径上
   * @param  robot  机器位姿
   * @param  pose  检测的任务路径点
   * @param  dist_err  欧式距离误差
   * @param  yaw_err  角度误差
   * @return true 
   * @return false 
   */
  bool IsOnTaskplan(const geometry_msgs::PoseStamped& robot,
    const geometry_msgs::PoseStamped& pose, const double dist_err, const double yaw_err);

  void halt() override;

  BT::NodeStatus tick() override;

 private:
  const char* node_name_ = "task_plan_recorder_action";
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  size_t last_search_index_ = 0;
  std::vector<size_t> captured_index_;
  ros::Time timer_;
  ros::Publisher lastest_recorder_pub_;

}; // class TaskPlanRecorderAction

}  // namespace navit_bt_nodes

#endif  // TASK_PLAN_RECORDER_ACTION_HPP_
