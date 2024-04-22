//
// Created by fan on 23-8-15.
//

#ifndef NAVIT_BT_NODES_TASK_CONTROL_GRPC_SERVER_HPP
#define NAVIT_BT_NODES_TASK_CONTROL_GRPC_SERVER_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include "task_grpc_server.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <google/protobuf/util/json_util.h>
#include <navit_msgs/SimpleCmd.h>
#include <geometry_msgs/Pose2D.h>

geometry_msgs::PoseStamped pose2dToPoseStamped(const geometry_msgs::Pose2D& pose2d)
{
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.stamp = ros::Time::now();
  pose_stamped.header.frame_id = "map";

  pose_stamped.pose.position.x = pose2d.x;
  pose_stamped.pose.position.y = pose2d.y;
  pose_stamped.pose.orientation.z = pose2d.theta;
  pose_stamped.pose.orientation.w = 1.0;
  return pose_stamped;
}

// 定义Action节点类
class TaskReceiverAction : public BT::ActionNodeBase
{
public:
  // 构造函数
  TaskReceiverAction(const std::string& name, const BT::NodeConfiguration& config) : BT::ActionNodeBase(name, config)
  {
    ros::NodeHandle nh;
    move_base_sub_ = nh.subscribe("/move_base_simple/goal", 1, &TaskReceiverAction::onReceivedGoal, this);
    task_control_sub_ = nh.subscribe("/navit/task_control", 1, &TaskReceiverAction::onReceivedTask, this);
    task_control_server_ = nh.advertiseService("/navit/task_control", &TaskReceiverAction::onReceivedTask, this);

    const bool grpc_enable = getInput<bool>("grpc_enable").value();
    const int grpc_port = getInput<int>("grpc_port").value();
    if (grpc_enable)
    {
      task_grpc_server_ = std::make_unique<TaskGrpcServer>();
      task_grpc_server_->start(grpc_port);
    }
  }

  // 重写行为树Action节点的执行函数
  BT::NodeStatus tick() override
  {
    std::lock_guard<std::mutex> l(mutex_);
    if (received_new_cmd_)
    {
      received_new_cmd_ = false;
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("grpc_enable", "grpc enable."),
      BT::InputPort<int>("grpc_port", "grpc port such as 50051."),
      BT::OutputPort<std::string>("cmd", "receive cmd."),
      BT::OutputPort<geometry_msgs::PoseStamped>("cmd_param_go_pose", "go pose param."),
      BT::OutputPort<std::vector<std::string>>("cmd_param_go_path", "go path param."),
      BT::OutputPort<std::vector<std::string>>("cmd_param_go_coverage", "go area param."),
    };
  }
  void halt() override
  {
    received_new_cmd_ = false;
  }

private:
  void onReceivedGoal(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> l(mutex_);
    if ((status() != BT::NodeStatus::RUNNING) || received_new_cmd_)
    {
      return;
    }

    setOutput("cmd", "go_pose");
    setOutput("cmd_param_go_pose", *msg);
    received_new_cmd_ = true;
  }

  void onReceivedTask(const std_msgs::String::ConstPtr& msg)
  {
    addTask(msg->data);
  }

  bool onReceivedTask(navit_msgs::SimpleCmd::Request& req, navit_msgs::SimpleCmd::Response& resp)
  {
    bool succeed = addTask(req.cmd);
    resp.succeed = succeed;
    return succeed;
  }

private:
  bool addTask(const std::string& task_string)
  {
    std::lock_guard<std::mutex> l(mutex_);
    if ((status() != BT::NodeStatus::RUNNING) || received_new_cmd_)
    {
      return false;
    }

    bool parse_ok = false;
    navigation::TaskControl taskControl;
    if (google::protobuf::util::JsonStringToMessage(task_string, &taskControl).ok())
    {
      auto task_cmd = taskControl.cmd();
      setOutput("cmd", task_cmd);
      if (taskControl.has_cmd_param_go_pose() && (task_cmd == "go_pose"))
      {
        geometry_msgs::Pose2D pose_2d;
        pose_2d.x = taskControl.cmd_param_go_pose().x();
        pose_2d.y = taskControl.cmd_param_go_pose().y();
        pose_2d.theta = taskControl.cmd_param_go_pose().theta();
        setOutput("cmd_param_go_pose", pose2dToPoseStamped(pose_2d));
        parse_ok = true;
      }
      if (taskControl.has_cmd_param_go_path() && (task_cmd == "go_path"))
      {
        auto& stations = taskControl.cmd_param_go_path().stations();
        std::vector<std::string> out_stations;
        out_stations.assign(stations.begin(), stations.end());
        setOutput("cmd_param_go_path", out_stations);
        parse_ok = true;
      }
      if (taskControl.has_cmd_param_go_coverage() && (task_cmd == "go_coverage"))
      {
        auto& areas = taskControl.cmd_param_go_coverage().areas();
        std::vector<std::string> out_areas;
        out_areas.assign(areas.begin(), areas.end());
        setOutput("cmd_param_go_coverage", out_areas);
        parse_ok = true;
      }
    }
    else
    {
      std::cout << "TaskControlGrpcServer::addTask JsonStringToMessage Error!" << std::endl;
      return false;
    }

    if (!parse_ok)
    {
      return false;
    }

    received_new_cmd_ = true;
    return true;
  }

private:
  std::mutex mutex_;
  bool received_new_cmd_{};
  ros::Subscriber move_base_sub_;
  ros::Subscriber task_control_sub_;
  ros::ServiceServer task_control_server_;

  std::unique_ptr<TaskGrpcServer> task_grpc_server_;
};

// 注册节点
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<TaskReceiverAction>("TaskReceiverAction");
}

#endif  // NAVIT_BT_NODES_TASK_CONTROL_GRPC_SERVER_HPP
