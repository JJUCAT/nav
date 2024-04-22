#include "navit_bt_nodes/plugins/action/j110/task_plan_recorder_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/duration.h"
#include "tf2/utils.h"
#include "angles/angles.h"

namespace navit_bt_nodes {

void TaskPlanRecorderAction::halt()
{
  last_search_index_ = 0;
  captured_index_.clear();
  TimerReset();
}

BT::NodeStatus TaskPlanRecorderAction::tick()
{
  nav_msgs::Path task_plan;
  std::vector<size_t> recorder;
  double near, search, capture, attach;
  bool on_taskplan;
  LoadArg(task_plan, recorder, near, search, capture, attach, on_taskplan);

  geometry_msgs::PoseStamped robot;
  if (!UpdateRobotPose("base_link", task_plan.header.frame_id, robot)) {
    ROS_ERROR("[BT][%s] update robot pose failed.", node_name_);
    return BT::NodeStatus::FAILURE;
  }

  size_t captured;
  if (!UpdateRecorderInNear(task_plan, recorder, robot.pose.position, near)) {
    size_t reset_index = recorder.empty() ? 0 : recorder.back();
    if (Search(task_plan, robot.pose.position, reset_index, search, capture, attach, captured)) {
      if (captured > reset_index)
        recorder.push_back(captured);
    }
  }
  // ROS_INFO("[BT][%s] latest recorder size:%lu.", node_name_, recorder.size());
  PubRecord(task_plan, recorder);

  if (!on_taskplan && !recorder.empty()) {
    if (recorder.back() > 0) {
      double dist_err = 0.3, yaw_err = 0.51;
      on_taskplan = IsOnTaskplan(robot, task_plan.poses.at(recorder.back()), dist_err, yaw_err);
      if (on_taskplan)
        setOutput("on_taskplan", on_taskplan);
    }
  }

  setOutput("recorder", recorder);
  return BT::NodeStatus::SUCCESS;
};

void TaskPlanRecorderAction::LoadArg(nav_msgs::Path& task_plan, std::vector<size_t>& recorder,
  double& near, double& search, double& capture, double& attach, bool& on_taskplan)
{
  if (!getInput<nav_msgs::Path>("task_plan", task_plan)) {
    std::string msg("missing arg [task_plan]");
    ROS_ERROR("[BT][%s] %s", node_name_,  msg.c_str());
    throw(BT_Exception(msg));
  }
  
  if (!getInput<std::vector<size_t>>("recorder", recorder)) {
    std::string msg("missing arg [recorder]");
    ROS_ERROR("[BT][%s] %s", node_name_, msg.c_str());
    throw(BT_Exception(msg));
  }
  
  if (!getInput<double>("near", near)) {
    std::string msg("missing arg [near]");
    ROS_ERROR("[BT][%s] %s", node_name_, msg.c_str());
    throw(BT_Exception(msg));
  }
  
  if (!getInput<double>("search", search)) {
    std::string msg("missing arg [search]");
    ROS_ERROR("[BT][%s] %s", node_name_, msg.c_str());
    throw(BT_Exception(msg));
  }

  if (!getInput<double>("capture", capture)) {
    std::string msg("missing arg [capture]");
    ROS_ERROR("[BT][%s] %s", node_name_, msg.c_str());
    throw(BT_Exception(msg));
  }

  if (!getInput<double>("attach", attach)) {
    std::string msg("missing arg [attach]");
    ROS_ERROR("[BT][%s] %s", node_name_, msg.c_str());
    throw(BT_Exception(msg));
  }

  if (!getInput<bool>("on_taskplan", on_taskplan)) {
    std::string msg("missing arg [on_taskplan]");
    ROS_ERROR("[BT][%s] %s", node_name_, msg.c_str());
    throw(BT_Exception(msg));
  }
}

bool TaskPlanRecorderAction::UpdateRobotPose(
  const std::string robot_frame, const std::string plan_frame, geometry_msgs::PoseStamped& robot)
{
  try {
    // ROS_INFO("[BT][%s] robot frame: %s, plan frame: %s", node_name_, robot_frame.c_str(), plan_frame.c_str());
    // if (tf_buffer_.canTransform(plan_frame, robot_frame, ros::Time::now(), ros::Duration(10.0))) {
      geometry_msgs::TransformStamped robot_to_plan_transform =
        tf_buffer_.lookupTransform(plan_frame, robot_frame, ros::Time(0));
      geometry_msgs::PoseStamped origin;
      origin.pose.orientation.w = 1;
      tf2::doTransform(origin, robot, robot_to_plan_transform);
      // ROS_INFO("[BT][%s] robot in plan frame [%s][%.3f,%.3f]",
      //   node_name_, plan_frame.c_str(), robot.pose.position.x, robot.pose.position.y);
      return true;
    // }
    // ROS_ERROR("[BT][%s] missing frame transform.\n", node_name_);
    // return false;
  } catch (const tf2::TransformException& ex) {
    ROS_ERROR("[BT][%s] update robot pose failed, %s\n", node_name_, ex.what());
    return false;
  }
  return true;
}

std::vector<geometry_msgs::PoseStamped>::const_iterator
TaskPlanRecorderAction::FindClosest(std::vector<geometry_msgs::PoseStamped>::const_iterator begin,
  std::vector<geometry_msgs::PoseStamped>::const_iterator end,
  const geometry_msgs::Point& robot, const double radius)
{
  std::vector<geometry_msgs::PoseStamped>::const_iterator closest = end;
  double drp, min = radius;
  for (auto i = begin; i != end; i ++) {
    drp = std::hypot(i->pose.position.y - robot.y, i->pose.position.x - robot.x);
    if (drp < min) {
      closest = i;
      min = drp;
    }
  }
  return closest;
}

double TaskPlanRecorderAction::GetDistanceOffset(
  const std::vector<geometry_msgs::PoseStamped>& poses,
  const size_t start_index, const double distance, size_t& offset_index)
{
  double s = 0.0;
  size_t i = start_index;
  for (size_t j = i; i < poses.size(); j = i, i ++) {
    s += std::hypot(poses.at(i).pose.position.y - poses.at(j).pose.position.y,
                    poses.at(i).pose.position.x - poses.at(j).pose.position.x);
    if (s > distance) break;    
  }
  offset_index = i;
  return s;
}

bool TaskPlanRecorderAction::UpdateRecorderInNear(const nav_msgs::Path& task_plan,
  std::vector<size_t>& recorder, const geometry_msgs::Point& robot, const double near)
{
  // ROS_INFO("[BT][%s] update recorder in near, task plan size %lu, recorder size %lu, robot [%.3f,%.3f], near %.3f",
  //   node_name_, task_plan.poses.size(), recorder.size(), robot.x, robot.y, near);

  auto& poses = task_plan.poses;
  size_t start = 0, offset = 1;
  if (!recorder.empty()) {
    start = recorder.back();
    GetDistanceOffset(poses, start, 1.5, offset);
  }
  // ROS_INFO("[BT][%s] offset is %lu", node_name_, offset);

  std::vector<geometry_msgs::PoseStamped>::const_iterator closest;
  std::vector<geometry_msgs::PoseStamped>::const_iterator begin = poses.begin()+start;
  std::vector<geometry_msgs::PoseStamped>::const_iterator end = poses.begin() + offset;
  closest = FindClosest(begin, end, robot, near);
  if (closest == end) {
    ROS_WARN("[BT][%s] missing robot.", node_name_);
    return false;
  }

  size_t latest = std::distance(poses.begin(), closest);
  // ROS_INFO("[BT][%s] update near, last record index %lu, latest record index %lu, plan size %lu.",
  //   node_name_, start, latest, poses.size());
  if (recorder.empty()) {
    recorder.push_back(latest);
  } else {
    for (size_t i = start+1; i <= latest; i ++)
      recorder.push_back(i);
  }
  return true;
}

bool TaskPlanRecorderAction::Search(const nav_msgs::Path& task_plan, const geometry_msgs::Point& robot,
  const size_t reset_index, const double search, double capture, double attach, size_t& captured)
{
  ROS_INFO("[BT][%s] search, task plan size %lu, robot [%.3f,%.3f],"
    " reset index %lu, search %.3f, capture %.3f, attach %.3f",
    node_name_, task_plan.poses.size(), robot.x, robot.y, reset_index, search, capture, attach);

  auto& poses = task_plan.poses;  
  size_t start = last_search_index_, offset = start+1;
  std::vector<geometry_msgs::PoseStamped>::const_iterator closest;
  if (poses.size() > 1) {
    for (double s = 0.0f, d; s < search; s += d, start = reset_index) {
      d = GetDistanceOffset(poses, start, search, offset);
      closest = FindClosest(poses.begin()+start, poses.begin()+offset, robot, capture);
      if (closest != poses.begin()+offset) break;
    }
  } else {
    closest = FindClosest(poses.begin()+start, poses.begin()+offset, robot, capture);
  }
  last_search_index_ = offset-1; // 更新下次的搜索起点
  if (closest == poses.begin()+offset) {
    ROS_WARN("[BT][%s] not capture.", node_name_);
    captured_index_.clear();
    TimerReset();
    return false;
  }

  size_t search_index = std::distance(poses.begin(), closest);
  last_search_index_ = search_index; // 找到可能的返回点，记录做下次搜索的地点
  InsertCapturedIndex(search_index);
  ROS_INFO("[BT][%s] capture ! search index %lu, captured size %lu",
    node_name_, last_search_index_, captured_index_.size());

  bool goback = false;
  goback |= IsCapturedDistanceLonggerThan(poses, attach);
  goback |= IsCapturedTimeLonggerThan(20.0);

  if (goback) {
    captured = search_index;
    captured_index_.clear();
    TimerReset();
    ROS_INFO("[BT][%s] robot already go back ! captured index %lu", node_name_, captured);
    return true;
  }
  return false;
}

void TaskPlanRecorderAction::InsertCapturedIndex(const size_t index)
{
  if (captured_index_.empty()) {
    captured_index_.push_back(index);
    TimerTick();
  } else if (index > captured_index_.back()) {
    captured_index_.push_back(index);
  }
}

bool TaskPlanRecorderAction::IsCapturedDistanceLonggerThan(
  const std::vector<geometry_msgs::PoseStamped>& poses, const double dist)
{
  double captured_dist = 0.0;
  for (size_t i = 0, j = i; i < captured_index_.size(); j = i, i ++) {
    captured_dist += std::hypot(poses.at(i).pose.position.y-poses.at(j).pose.position.y,
                                poses.at(i).pose.position.x-poses.at(j).pose.position.x);
    if (captured_dist >= dist) return true;
  }
  return false;
}

bool TaskPlanRecorderAction::IsCapturedTimeLonggerThan(const double time_limit)
{
  ros::Duration alarm(time_limit);
  if (ros::Time::now() - timer_ >= alarm) {
    ROS_INFO("[BT][%s] captured for a time !", node_name_);
    return true;
  }
  return false;
}

void TaskPlanRecorderAction::TimerTick()
{
  timer_ = ros::Time::now();
}

void TaskPlanRecorderAction::TimerReset()
{
  timer_ = ros::Time(0);
}

void TaskPlanRecorderAction::PubRecord(const nav_msgs::Path& plan, const std::vector<size_t>& recorder)
{
  size_t i = recorder.empty() ? 0 : recorder.back();
  auto p = plan.poses.at(i);
  p.header = plan.header;
  lastest_recorder_pub_.publish(p);
}

bool TaskPlanRecorderAction::IsOnTaskplan(const geometry_msgs::PoseStamped& robot,
  const geometry_msgs::PoseStamped& pose, const double dist_err, const double yaw_err)
{
  bool on_taskplan{false};

  double robot_yaw = tf2::getYaw(robot.pose.orientation);
  double pose_yaw = tf2::getYaw(pose.pose.orientation);
  double yaw_diff = fabs(angles::shortest_angular_distance(robot_yaw, pose_yaw));
  double dist_diff = std::hypot(robot.pose.position.x-pose.pose.position.x,
                                robot.pose.position.y-pose.pose.position.y);
  if (yaw_diff < yaw_err && dist_diff < dist_err) {
    on_taskplan = true;
    ROS_WARN("[BT][%s] robot is on task plan !", node_name_);
  }

  return on_taskplan;
}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navit_bt_nodes::TaskPlanRecorderAction>("TaskPlanRecorderAction");
}

} // namespace navit_bt_nodes

