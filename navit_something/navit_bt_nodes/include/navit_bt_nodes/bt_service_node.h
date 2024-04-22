// Copyright (c) 2019 Samsung Research America
// Copyright (c) 2020 Davide Faconti
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BEHAVIOR_TREE_BT_SERVICE_NODE_HPP_
#define BEHAVIOR_TREE_BT_SERVICE_NODE_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>
#include <ros/service_client.h>

namespace BT
{

/**
 * Base Action to implement a ROS Service
 */
template <class ServiceT>
class RosServiceNode : public BT::SyncActionNode
{
protected:
  RosServiceNode(ros::NodeHandle& nh, const std::string& name, const BT::NodeConfiguration& conf)
    : BT::SyncActionNode(name, conf), node_(nh)
  {
    double second = getInput<double>("server_timeout").value();
    timeout_ = ros::Duration(second);

    const std::string service_name = getInput<std::string>("service_name").value();
    service_client_ = node_.serviceClient<ServiceT>(service_name);
  }

public:
  using BaseClass = RosServiceNode<ServiceT>;
  using ServiceType = ServiceT;
  using RequestType = typename ServiceT::Request;
  using ResponseType = typename ServiceT::Response;

  RosServiceNode() = delete;

  virtual ~RosServiceNode() = default;

  /**
   * @brief Any subclass of BtActionNode that accepts parameters must provide a
   * providedPorts method and call providedBasicPorts in it.
   * @param addition Additional ports to add to BT port list
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = { BT::InputPort<std::string>("service_name", "Action server name"),
                            BT::InputPort<double>("server_timeout") };
    basic.insert(addition.begin(), addition.end());

    return basic;
  }

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  /// User must implement this method.
  virtual bool prepareRequest() = 0;

  /// Method (to be implemented by the user) to receive the reply.
  /// User can decide which NodeStatus it will return (SUCCESS or FAILURE).
  virtual NodeStatus onResponse(const ResponseType& rep) = 0;

  enum FailureCause
  {
    MISSING_SERVER = 0,
    FAILED_CALL = 1
  };

  /// Called when a service call failed. Can be overriden by the user.
  virtual NodeStatus onFailedRequest(FailureCause failure)
  {
    return NodeStatus::FAILURE;
  }

protected:
  void increment_recovery_count()
  {
    int recovery_count = 0;
    config().blackboard->template get<int>("number_recoveries", recovery_count);  // NOLINT
    recovery_count += 1;
    config().blackboard->template set<int>("number_recoveries", recovery_count);  // NOLINT
  }

  BT::NodeStatus tick() override
  {
    bool connected = service_client_.waitForExistence(timeout_);
    if (!connected)
    {
      return onFailedRequest(MISSING_SERVER);
    }

    if (!prepareRequest())
    {
      return onFailedRequest(FAILED_CALL);
    }

    std::future<bool> service_ok =
        std::async(std::launch::async, [this]() { return service_client_.call(request_, response_); });

    auto start_time = ros::Time::now();
    while (ros::ok() && ((ros::Time::now() - start_time) < timeout_))
    {
      if (service_ok.wait_for(std::chrono::milliseconds(1)) == std::future_status::ready)
      {
        return service_ok.get() ? onResponse(response_) : onFailedRequest(FAILED_CALL);
      }
    }
    return onFailedRequest(FAILED_CALL);
  }

  RequestType request_;
  ResponseType response_;
  ros::ServiceClient service_client_;
  
  // The node that will be used for any ROS operations
  ros::NodeHandle& node_;
  ros::Duration timeout_;
};

/// Method to register the service into a factory.
/// It gives you the opportunity to set the ros::NodeHandle.
// template <class DerivedT> static
//     void RegisterRosService(BT::BehaviorTreeFactory& factory,
//                        const std::string& registration_ID,
//                        ros::NodeHandle& node_handle)
// {
//   NodeBuilder builder = [&node_handle](const std::string& name, const NodeConfiguration& config) {
//     return std::make_unique<DerivedT>(node_handle, name, config );
//   };
//
//   TreeNodeManifest manifest;
//   manifest.type = getType<DerivedT>();
//   manifest.ports = DerivedT::providedPorts();
//   manifest.registration_ID = registration_ID;
//   const auto& basic_ports = RosServiceNode< typename DerivedT::ServiceType>::providedPorts();
//   manifest.ports.insert( basic_ports.begin(), basic_ports.end() );
//
//   factory.registerBuilder( manifest, builder );
// }

}  // namespace BT

#endif  // BEHAVIOR_TREE_BT_SERVICE_NODE_HPP_
