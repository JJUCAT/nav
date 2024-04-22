//
// Created by fan on 23-8-16.
//

#ifndef NAVIT_BT_NODES_TASK_GRPC_SERVER_HPP
#define NAVIT_BT_NODES_TASK_GRPC_SERVER_HPP

#include "navigation.grpc.pb.h"
#include "grpcpp/grpcpp.h"
#include <ros/ros.h>
#include <google/protobuf/util/json_util.h>
#include <navit_msgs/SimpleCmd.h>

// 导航服务类
class NavigationServiceImpl final : public navigation::NavigationService::Service
{
public:
  NavigationServiceImpl()
  {
    ros::NodeHandle nh;
    task_control_client_ = nh.serviceClient<navit_msgs::SimpleCmd>("/navit/task_control");
  }

  bool sendTaskControl(const navigation::TaskControl& taskControl)
  {
    navit_msgs::SimpleCmd simple_cmd;
    google::protobuf::util::MessageToJsonString(taskControl, &simple_cmd.request.cmd);
    if (task_control_client_.call(simple_cmd))
    {
      simple_cmd.response.succeed = true;
      return true;
    }

    return false;
  }

  grpc::Status GoPose(grpc::ServerContext* context, const navigation::CmdGoPoseRequest* request,
                      navigation::CmdCommonResponse* response) override
  {
    navigation::TaskControl task_control;
    task_control.set_cmd("go_pose");
    task_control.mutable_cmd_param_go_pose()->CopyFrom(*request);
    response->set_succeed(sendTaskControl(task_control));  // 设置响应结果
    return grpc::Status::OK;
  }

  grpc::Status GoPath(grpc::ServerContext* context, const navigation::CmdGoPathRequest* request,
                      navigation::CmdCommonResponse* response) override
  {
    navigation::TaskControl task_control;
    task_control.set_cmd("go_path");
    task_control.mutable_cmd_param_go_path()->CopyFrom(*request);
    response->set_succeed(sendTaskControl(task_control));  // 设置响应结果
    return grpc::Status::OK;
  }

  grpc::Status GoCoverage(grpc::ServerContext* context, const navigation::CmdGoCoverageRequest* request,
                          navigation::CmdCommonResponse* response) override
  {
    navigation::TaskControl task_control;
    task_control.set_cmd("go_coverage");
    task_control.mutable_cmd_param_go_coverage()->CopyFrom(*request);
    response->set_succeed(sendTaskControl(task_control));  // 设置响应结果
    return grpc::Status::OK;
  }

  grpc::Status GoTranslate(grpc::ServerContext* context, const navigation::CmdGoTranslateRequest* request,
                           navigation::CmdCommonResponse* response) override
  {
    navigation::TaskControl task_control;
    task_control.set_cmd("go_translate");
    task_control.mutable_cmd_param_go_translate()->CopyFrom(*request);
    response->set_succeed(sendTaskControl(task_control));  // 设置响应结果
    return grpc::Status::OK;
  }

  grpc::Status GoSpin(grpc::ServerContext* context, const navigation::CmdGoSpinRequest* request,
                      navigation::CmdCommonResponse* response) override
  {
    navigation::TaskControl task_control;
    task_control.set_cmd("go_spin");
    task_control.mutable_cmd_param_go_spin()->CopyFrom(*request);
    response->set_succeed(sendTaskControl(task_control));  // 设置响应结果
    return grpc::Status::OK;
  }

private:
  ros::ServiceClient task_control_client_;
};

class TaskGrpcServer
{
public:
  ~TaskGrpcServer()
  {
    stop();
  }

  void start(int port = 50051)
  {
    // 定义常量
    const std::string kServerAddress = "0.0.0.0:" + std::to_string(port);

    // 构造服务
    navigation_service_ = std::make_shared<NavigationServiceImpl>();

    // 创建gRPC服务器
    builder_ = std::make_shared<grpc::ServerBuilder>();
    builder_->AddListeningPort(kServerAddress, grpc::InsecureServerCredentials());

    // 将导航服务添加到服务器中
    // 将其他服务添加到服务器中
    builder_->RegisterService(navigation_service_.get());
    // ...

    // 创建并启动服务
    server_ = builder_->BuildAndStart();
    std::cout << "Server listening on " << kServerAddress << std::endl;
  }

  void stop()
  {
    if (server_)
    {
      server_->Shutdown();
      server_->Wait();
      server_ = nullptr;
    }
  }

private:
  std::unique_ptr<grpc::Server> server_;
  std::shared_ptr<grpc::ServerBuilder> builder_;
  std::shared_ptr<NavigationServiceImpl> navigation_service_;
};

#endif  // NAVIT_BT_NODES_TASK_GRPC_SERVER_HPP
