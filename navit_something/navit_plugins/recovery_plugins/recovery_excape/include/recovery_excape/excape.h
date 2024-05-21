#ifndef RECOVERY_EXCAPE_H
#define RECOVERY_EXCAPE_H

#include <navit_recovery/recovery_action.h>
#include <navit_msgs/ExcapeAction.h>
#include <recovery_excape/typedef.hpp>
#include <recovery_excape/environment.h>
#include <recovery_excape/individuals.h>
#include <recovery_excape/genotype.h>
#include <recovery_excape/toolkit.h>
#include <recovery_excape/config.hpp>
#include "Eigen/Core"
#include "geometry_msgs/PoseStamped.h"
#include "ros/publisher.h"

using namespace navit_recovery;

namespace recovery_excape {

using ActionT = navit_msgs::ExcapeAction;
using ActionGoalT = navit_msgs::ExcapeGoal;
using ActionFeedbackT = navit_msgs::ExcapeFeedback;
using ActionResultT = navit_msgs::ExcapeResult;

class RecoveryExcape : public RecoveryAction<ActionT, ActionGoalT, ActionResultT>
{
 public:
  RecoveryExcape(){}
  ~RecoveryExcape(){}

  Status onRun(const ActionGoalT::ConstPtr& action_goal) override;
  Status onCycleUpdate() override;
  Status onCancel() override;
  void loadParams(const std::string& name) override;

 protected:
  ActionFeedbackT feedback_;
  ActionResultT result_;

  ros::Publisher cmd_pub_;
  ros::Time end_;
  geometry_msgs::Twist cmd_vel_;

 private:

  /**
   * @brief  是否超时
   * @param  timeout  时间上限 [s]
   * @return true 
   * @return false 
   */
  bool Timeout(const double timeout);

  /**
   * @brief  获取机器在 recovery 地图中的位姿
   * @param  robot_pose  机器人位姿
   * @return true 
   * @return false 
   */
  bool GetRobotPose(geometry_msgs::PoseStamped& robot_pose);

  /**
   * @brief  脱困
   * @param  robot  机器
   * @param  map  地图   
   * @return true 
   * @return false 
   */
  bool GetOutOfStuck(const geometry_msgs::PoseStamped& robot,
    const navit_costmap::Costmap2D& map);

  /**
   * @brief 定时器
   */
  void TimerCallback(const ros::TimerEvent&);
  void DirTimerCallback(const ros::TimerEvent&);

  void TimeGoesby();

  /**
   * @brief  创建原始控制序列
   * @return Individuals 
   */
  Individuals CreateAncestor();

  /**
   * @brief  生成随机控制序列
   * @param  parents  原始控制序列
   * @param  size  生成随机序列数量
   * @param  population  序列集合
   * @param  include_parents  是否包含原始控制序列
   * @return size_t  序列数量
   */
  size_t Reproduce(const Individuals& parents, const size_t size, Population& population,
    const bool include_parents = true);

  /**
   * @brief  控制序列集评分
   * @param  env  评分器
   * @param  pp  控制序列集合
   * @param  pose  机器位姿
   * @param  ppf  带分数的控制序列集合
   * @return size_t 
   */
  size_t Evaluate(Environment& env, Population& pp,
    geometry_msgs::PoseStamped pose, PopulationWithFitness& ppf);

  /**
   * @brief  筛选适应度高的控制
   * @param  ppf  带分数的控制序列集合
   * @param  excellent  优秀分数，达到该分数认为优化已经完成
   * @param  good  良好分数，达到该分数的集合会被添加到下一次迭代中
   * @param  size  elite 数量上限
   * @param  elite  良好分数以上的控制集合
   * @return true  有优秀分数的控制
   * @return false 
   */
  bool Select(const PopulationWithFitness& ppf, const double excellent, const double good, const size_t size, Population& elite);

  /**
   * @brief  混合控制序列
   * @param  seeds  控制序列集合
   * @return Individuals 
   */
  Individuals Cross(const Population& seeds);

  /**
   * @brief  获取 ppf 中适应分最高的一个控制序列
   * @param  ppf  带分数的控制序列集合
   * @return Individuals 
   */
  Individuals SelectElite(const PopulationWithFitness& ppf);

  size_t CrossRandom(const size_t size);

  /**
   * @brief  发布脱困路径
   * @param  pub  发布器
   * @param  u  控制序列
   * @param  robot  机器位姿
   */
  void PubExcapePath(const ros::Publisher& pub, Individuals& u, const geometry_msgs::PoseStamped& robot);

  /**
   * @brief  检测脱困路径是否碰撞
   * @param  x_sequence  路径上的位姿状态
   * @return true 
   * @return false 
   */
  bool IsExcapePathCollised(Individuals& u, const geometry_msgs::PoseStamped& robot);

  std::shared_ptr<Configuration> config_;
  ros::Publisher excape_poses_pub_;
  ros::Time start_time_;
  double timeout_;
  geometry_msgs::PoseStamped original_pose_;
  bool create_ancestor_{false};
  Individuals parents_;
  Population elite_;
  ros::Timer timer_;
  ros::Timer dir_timer_;
  geometry_msgs::PoseStamped robot_pose_;
  Individuals better_;
  std::mutex u_mutex_;
  std::mutex r_mutex_;
  int dir_{-1};
  double v_;
  double free_;
}; // class RecoveryExcape


} // recovery_excape

#endif // RECOVERY_EXCAPE_H
