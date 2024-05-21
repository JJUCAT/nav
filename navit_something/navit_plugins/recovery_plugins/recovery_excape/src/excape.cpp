#include <recovery_excape/excape.h>


#include "angles/angles.h"
#include <random>
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "recovery_excape/individuals.h"
#include "ros/node_handle.h"
#include "tf/transform_datatypes.h"


using namespace navit_recovery;

namespace recovery_excape {

void RecoveryExcape::loadParams(const std::string& name)
{
  ros::NodeHandle pnh("~/" + name);  
  config_ = std::make_shared<Configuration>(pnh);
  excape_poses_pub_ = pnh.advertise<geometry_msgs::PoseArray>("excape_poses", 1);

  ros::NodeHandle nh;
  cmd_pub_ = nh.advertise<geometry_msgs::Twist>(config_->cmd_vel_topic, 1);
  timer_ = nh.createTimer(ros::Duration(0.5), &RecoveryExcape::TimerCallback, this);
  timer_.stop();
  dir_timer_ = nh.createTimer(ros::Duration(5), &RecoveryExcape::DirTimerCallback, this);
  dir_timer_.stop();
  ROS_INFO("Recovery Action Excape params loaded!");
}

Status RecoveryExcape::onRun(const ActionGoalT::ConstPtr& goal)
{
  ROS_INFO("[Rcvr][Excape] receive excape action, "
    "timeout %f, free %f, v %f, w %f, r %f",
    goal->timeout, goal->free, goal->v, goal->w, goal->r);

  result_.is_excape = false;
  
  create_ancestor_ = false;
  timeout_ = goal->timeout;
  free_ = goal->free; // 暂时不用
  v_ = goal->v;  
  start_time_ = ros::Time::now();
  if (!GetRobotPose(original_pose_)) {
    ROS_ERROR("[Rcvr][Excape] get original robot pose from map failed !");
    return navit_recovery::Status::FAILED;
  } 
  robot_pose_ = original_pose_;

  timer_.start();
  dir_timer_.start();
  return navit_recovery::Status::SUCCEEDED;
}

Status RecoveryExcape::onCycleUpdate()
{
  geometry_msgs::Twist stop;
  feedback_.elapsed_time = (ros::Time::now()-start_time_).toSec();
  as_->publishFeedback(feedback_);
  // result_.total_time = ros::Duration(feedback_.elapsed_time);

  if (Timeout(timeout_)) {
    ROS_ERROR("[Rcvr][Excape] timeout !");
    cmd_pub_.publish(stop);
    timer_.stop();
    dir_timer_.stop();
    as_->setSucceeded(result_);
    return navit_recovery::Status::FAILED;
  }

  {
    std::unique_lock<std::mutex> lock(r_mutex_);
    if (!GetRobotPose(robot_pose_)) {
      ROS_ERROR("[Rcvr][Excape] get robot pose from map failed !");
      cmd_pub_.publish(stop);
      timer_.stop();
      dir_timer_.stop();
      as_->setSucceeded(result_);
      return navit_recovery::Status::FAILED;
    }
  }

  Individuals new_u;
  ros::Rate r(10);
  while (new_u.GetHereditaryInformation().empty()) {
    {
      std::unique_lock<std::mutex> lock(u_mutex_);
      new_u = better_;
    }
    r.sleep();
  }

  navit_costmap::Costmap2D map;
  {
    auto map_mutex = map_ros_->getCostmap()->getMutex();
    boost::recursive_mutex::scoped_lock lock(*map_mutex);
    map = *map_ros_->getCostmap();
  }

  if (GetOutOfStuck(robot_pose_, map)) {
    ROS_WARN("[Rcvr][Excape] excape !");
    timer_.stop();
    dir_timer_.stop();
    cmd_vel_.linear.x = 0;
    cmd_vel_.angular.z = 0;
    cmd_pub_.publish(cmd_vel_);
    result_.is_excape = true;
    as_->setSucceeded(result_);
    return navit_recovery::Status::SUCCEEDED;
  }

  if (IsExcapePathCollised(new_u, robot_pose_)) {
    ROS_INFO("[Rcvr][Excape] collised.");
    cmd_vel_.linear.x = 0;
    cmd_vel_.angular.z = 0;
  } else {
    auto info = new_u.GetHereditaryInformation();
    double ws; int v_signed = dir_;    
    new_u.GetBehavior(info.front(), robot_pose_.pose, map, config_->wheel_base, ws, v_signed);
    cmd_vel_.linear.x = v_ * v_signed;
    cmd_vel_.angular.z = ws * v_signed;
  }
  cmd_pub_.publish(cmd_vel_);

  return navit_recovery::Status::RUNNING;
}

Status RecoveryExcape::onCancel()
{
  cmd_vel_.linear.x = 0;
  cmd_vel_.angular.z = 0;
  cmd_pub_.publish(cmd_vel_);
  timer_.stop();
  dir_timer_.stop();
  return navit_recovery::Status::SUCCEEDED;
}

// -------------------- private --------------------

bool RecoveryExcape::Timeout(const double timeout)
{
  ros::Duration to(timeout);
  return ros::Time::now() - start_time_ >= to;
}

bool RecoveryExcape::GetRobotPose(geometry_msgs::PoseStamped& robot_pose)
{
  return map_ros_->getRobotPose(robot_pose);
}

bool RecoveryExcape::GetOutOfStuck(const geometry_msgs::PoseStamped& robot,
  const navit_costmap::Costmap2D& map)
{
  int head_cost = 255, base_cost = 255;
  double yaw = tf::getYaw(robot.pose.orientation);
  double head = config_->head;
  unsigned int mx, my;
  for (double r = 0; r <= head; r += head) {
    double wx = robot.pose.position.x + cos(yaw) * r;
    double wy = robot.pose.position.y + sin(yaw) * r;    
    if (map.worldToMap(wx, wy, mx, my)) {
      if (r < 0.1) base_cost = map.getCost(mx, my);
      else head_cost = map.getCost(mx, my);
    }
  }
  ROS_INFO_THROTTLE(1, "[Rcvr][Excape] current base cost %d, head cost is %d",
    config_->head_free, config_->base_free);
  if (base_cost < config_->base_free && head_cost < config_->head_free) return true;
  return false;
}

void RecoveryExcape::TimerCallback(const ros::TimerEvent&)
{
  ROS_INFO_THROTTLE(0.5, "[Rcvr][Excape] timer tictok ...");

  geometry_msgs::PoseStamped robot;
  {
    std::unique_lock<std::mutex> lock(r_mutex_);
    robot = robot_pose_;   
  }

  int loop = config_->loop;
  while (loop-- > 0) {
    if (!create_ancestor_) {
      create_ancestor_ = true;
      parents_ = CreateAncestor();
    }

    Population pp;
    PopulationWithFitness ppf;
    Environment env(map_ros_);    
    Reproduce(parents_, 30, pp); 
    size_t esize = elite_.size(), inherit_size = 3;
    esize = esize >= inherit_size ? inherit_size : esize;
    pp.insert(pp.end(), elite_.begin(), elite_.begin()+esize);
    Evaluate(env, pp, robot, ppf);

    {
      std::unique_lock<std::mutex> lock(u_mutex_);
      better_ = SelectElite(ppf);
    }
    PubExcapePath(excape_poses_pub_, better_, robot);    

    if (Select(ppf, 500.0, 450.0, 10, elite_)) {
      ROS_INFO("[Rcvr][Excape] Find path to excape !");
    }
    parents_ = Cross(elite_);
    parents_.Mutation();
    TimeGoesby();    
  }
}

void RecoveryExcape::DirTimerCallback(const ros::TimerEvent&)
{
  navit_costmap::Costmap2D map;
  {
    auto map_mutex = map_ros_->getCostmap()->getMutex();
    boost::recursive_mutex::scoped_lock lock(*map_mutex);
    map = *map_ros_->getCostmap();
  }
  dir_ = GetDir(robot_pose_.pose, map, 3.0);
}

void RecoveryExcape::TimeGoesby()
{
  parents_.Survive();
  for (auto it = elite_.begin(); it != elite_.end(); it ++) {
    it->Survive();
  }
}

Individuals RecoveryExcape::CreateAncestor()
{
  Genotype geno_w(0.8, 0.5, 1.0, 0.4);
  Genotype geno_t(1.0, 1.0, 1.0, 0.5);

  Chromosome chromosome;
  chromosome.emplace_back(geno_w);
  chromosome.emplace_back(geno_t);

  HereditaryInformation info(20, chromosome);
  Individuals indiv(info);
  return indiv;
}

size_t RecoveryExcape::Reproduce(const Individuals& parents, const size_t size,
  Population& population, const bool include_parents)
{
  population.clear();
  size_t total = size + (include_parents ? 1 : 0);
  population.reserve(total);

  if (include_parents) population.push_back(parents);
  for (size_t i = 0; i < size; i ++) {
    auto children(parents);
    children.Reproduce();
    population.push_back(children);
  }

  return population.size();
}

size_t RecoveryExcape::Evaluate(Environment& env, Population& pp,
  geometry_msgs::PoseStamped pose, PopulationWithFitness& ppf)
{
  navit_costmap::Costmap2D map;
  {
    auto map_mutex = map_ros_->getCostmap()->getMutex();
    boost::recursive_mutex::scoped_lock lock(*map_mutex);
    map = *map_ros_->getCostmap();
  }

  double v = v_, wb = config_->wheel_base;
  bool use_steering = true;
  ppf.clear();
  for (size_t i = 0; i < pp.size(); i ++) {
    auto poses = pp.at(i).GenCharacter(pose, map, v*dir_, wb, use_steering);
    double fitness = env.Evaluate(poses, pose.pose);
    ppf.insert(std::make_pair(fitness, pp.at(i)));
  }
  return ppf.size();
}

bool RecoveryExcape::Select(const PopulationWithFitness& ppf, const double excellent,
  const double good, const size_t size, Population& elite)
{
  elite.clear();
  for (auto iter = ppf.begin(); iter != ppf.end(); iter ++) {
    // ROS_INFO("[Rcvr][Excape] ppf score %.3f", iter->first);
    if (elite.size() >= size) break;

    if (iter->first > good) {
      elite.push_back(iter->second);
      if (iter->first > excellent) {
        // ROS_INFO("[Rcvr][Excape] find excellent solve (%f)", iter->first);
        // return true;
      }
    } else if (elite.empty()) {
      elite.push_back(iter->second);
    } else {
      break;
    }
  }
  return false;
}

Individuals RecoveryExcape::Cross(const Population& seeds)
{
  if (seeds.size()==1) {
    Individuals c {seeds.front()};
    return c;
  }
  size_t mother = CrossRandom(seeds.size());  
  size_t father = CrossRandom(seeds.size());
  if (father == mother) return seeds.front().GetHereditaryInformation();
  
  Individuals m {seeds.at(mother)};
  Individuals f {seeds.at(father)};
  Individuals child = m.Cross(f);
  return child;
}

Individuals RecoveryExcape::SelectElite(const PopulationWithFitness& ppf)
{
  Individuals elite;
  for (auto it = ppf.begin(); it != ppf.end();
    it = ppf.upper_bound(it->first)) {
    elite = it->second;
    break;
  }
  return elite;
}

size_t RecoveryExcape::CrossRandom(const size_t size)
{
  double u = size-1, d = 2.0 * u / 3;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<double> distribution(u, d);
  double r = distribution(gen);
  r = fabs(r-u);
  r = std::min(std::max(r, 0.0), u);
  return size_t(r);
}

void RecoveryExcape::PubExcapePath(const ros::Publisher& pub,
  Individuals& u, const geometry_msgs::PoseStamped& robot)
{
  navit_costmap::Costmap2D map;
  {
    auto map_mutex = map_ros_->getCostmap()->getMutex();
    boost::recursive_mutex::scoped_lock lock(*map_mutex);
    map = *map_ros_->getCostmap();
  }

  geometry_msgs::PoseArray poses;
  poses.header.frame_id = map_ros_->getGlobalFrameID();
  poses.header.stamp = ros::Time::now();
  poses.poses = u.GenCharacter(robot, map, v_*dir_, config_->wheel_base);
  pub.publish(poses);
}

bool RecoveryExcape::IsExcapePathCollised(Individuals& u, const geometry_msgs::PoseStamped& robot)
{
  navit_costmap::Costmap2D map;
  {
    auto map_mutex = map_ros_->getCostmap()->getMutex();
    boost::recursive_mutex::scoped_lock lock(*map_mutex);
    map = *map_ros_->getCostmap();
  }

  geometry_msgs::PoseArray poses;
  poses.header.frame_id = map_ros_->getGlobalFrameID();
  poses.header.stamp = ros::Time::now();
  poses.poses = u.GenCharacter(robot, map, v_*dir_, config_->wheel_base);
  geometry_msgs::PoseStamped pose = robot;
  for (auto p : poses.poses) {
    pose.pose = p;
    if (!collision_checker_->isCollisionFree(pose)) return true;
  }
  return false;
}

} // namespace recovery_excape

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(recovery_excape::RecoveryExcape, navit_core::Recovery)

