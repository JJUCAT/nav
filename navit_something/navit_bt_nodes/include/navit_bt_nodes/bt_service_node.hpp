// Copyright (c) 2019 Samsung Research America
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

#ifndef NAVIT_BT_NODES__BT_SERVICE_NODE_HPP_
#define NAVIT_BT_NODES__BT_SERVICE_NODE_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/action_node.h"
// #include "nav2_util/node_utils.hpp"
// #include "rclcpp/rclcpp.hpp"
#include <ros/ros.h>
#include "navit_bt_nodes/bt_conversions.hpp"

namespace navit_bt_nodes
{

/**
 * @brief Abstract class representing a service based BT node
 * @tparam ServiceT Type of service
 */
template<class ServiceT>
class BtServiceNode : public BT::ActionNodeBase
{
public:
  /**
   * @brief A navit_bt_nodes::BtServiceNode constructor
   * @param service_node_name Service name this node creates a client for ros service
   * @param conf BT node configuration
   */
  BtServiceNode(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf)
  : BT::ActionNodeBase(service_node_name, conf), service_node_name_(service_node_name)
  {
    node_ = config().blackboard->template get<ros::NodeHandle>("node");

    // Get the required items from the blackboard
    bt_loop_duration_ = config().blackboard->template get<double>("bt_loop_duration");
    server_timeout_ = config().blackboard->template get<double>("server_timeout");
    getInput<double>("server_timeout", server_timeout_);

    // Now that we have node_ to use, create the service client for this BT service
    getInput("service_name", service_name_);
    service_client_ = node_.serviceClient<ServiceT> (service_name_);

    // Make sure the server is actually there before continuing
    ROS_DEBUG("Waiting for \"%s\" service", service_name_.c_str());
    // service_client_->wait_for_service(); // no member‘wait_for_service’

    ROS_INFO("\"%s\" BtServiceNode initialized", service_node_name_.c_str());
  }

  BtServiceNode() = delete;

  virtual ~BtServiceNode()
  {
  }

  /**
   * @brief Any subclass of BtServiceNode that accepts parameters must provide a
   * providedPorts method and call providedBasicPorts in it.
   * @param addition Additional ports to add to BT port list
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {
      BT::InputPort<std::string>("service_name", "please_set_service_name_in_BT_Node"),
      BT::InputPort<double>("server_timeout")
    };
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

  /**
   * @brief The main override required by a BT service
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override
  {
    if (!request_sent_) {
      on_tick();
      if (request_valid_)
      {
        if (async_service_)
          // service_ok_ = std::async(std::launch::async, [&](ServiceT srv)->bool{service_client_.call(srv);}, srv_);
          service_ok_ = std::async(std::launch::async, [this](){
            return service_client_.call(srv_);
          });
        else
          service_done_ = service_client_.call(srv_);
      }
      else
        return BT::NodeStatus::FAILURE;
      sent_time_ = ros::Time::now();
      request_sent_ = true;
    }
    return check_future();
  }

  /**
   * @brief The other (optional) override required by a BT service.
   */
  void halt() override
  {
    request_sent_ = false;
    setStatus(BT::NodeStatus::IDLE);
  }

  /**
   * @brief Function to perform some user-defined operation on tick -- (request valid check)
   * Fill in service request with information if necessary
   */
  virtual void on_tick()
  {
  }

  /**
   * @brief Check the future and decide the status of BT
   * @return BT::NodeStatus SUCCESS if future complete before timeout, FAILURE otherwise
   */
  virtual BT::NodeStatus check_future()
  {
    // auto elapsed = (node_->now() - sent_time_).to_chrono<std::chrono::milliseconds>();
    auto elapsed = (ros::Time::now() - sent_time_).toSec(); //.to_chrono<std::chrono::milliseconds>();
    auto remaining = server_timeout_ - elapsed;

    bool service_status = true;
    if (async_service_)
    {
      if (service_ok_.wait_for(std::chrono::nanoseconds(1)) == std::future_status::ready){
        service_status = service_ok_.get();
      }
      else{
        service_status = false;
      }
    }
    else
      service_status = service_done_;
    

    if (service_status) {
      request_sent_ = false;
      return BT::NodeStatus::SUCCESS;
    }
    
    if (!service_status){
      on_wait_for_result();
      elapsed = (ros::Time::now() - sent_time_).toSec();
      if (elapsed < server_timeout_) {
        ROS_WARN("Node timed out while executing service call to %s.", service_name_.c_str());
        return BT::NodeStatus::RUNNING;
      }
    }

    ROS_WARN("Node timed out while executing service call to %s.", service_name_.c_str());
    request_sent_ = false;
    return BT::NodeStatus::FAILURE;
  }

  /**
   * @brief Function to perform some user-defined operation after a timeout waiting
   * for a result that hasn't been received yet
   */
  virtual void on_wait_for_result()
  {
  }

protected:
  /**
   * @brief Function to increment recovery count on blackboard if this node wraps a recovery
   */
  void increment_recovery_count()
  {
    int recovery_count = 0;
    config().blackboard->template get<int>("number_recoveries", recovery_count);  // NOLINT
    recovery_count += 1;
    config().blackboard->template set<int>("number_recoveries", recovery_count);  // NOLINT
  }

  std::string service_name_, service_node_name_;
  ServiceT srv_;

  // The node that will be used for any ROS operations
  ros::NodeHandle node_;
  ros::ServiceClient service_client_; 

  // The timeout value while to use in the tick loop while waiting for
  // a result from the server
  double server_timeout_;
  double bt_loop_duration_;

  // To track the server response when a new request is sent
  std::future<bool> service_ok_;
  bool service_done_;
  bool async_service_{true};
  bool request_valid_{true};
  bool request_sent_{false};
  ros::Time sent_time_;
};

}  // namespace navit_bt_nodes

#endif  // NAVIT_BT_NODES__BT_SERVICE_NODE_HPP_
