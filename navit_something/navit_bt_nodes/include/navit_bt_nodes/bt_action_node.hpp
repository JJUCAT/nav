// Copyright (c) 2018 Intel Corporation
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

#ifndef NAVIT_BT_NODES__BT_ACTION_NODE_HPP_
#define NAVIT_BT_NODES__BT_ACTION_NODE_HPP_

#include <memory>
#include <string>
#include <iostream>

#include "behaviortree_cpp_v3/action_node.h"
#include "navit_bt_nodes/bt_conversions.hpp"

#include <actionlib/client/simple_action_client.h>

namespace navit_bt_nodes
{

/**
 * @brief Abstract class representing an action based BT node
 * @tparam ActionT Type of action
 * @tparam ActionTGoal Type of action goal
 *  @tparam ActionTResult Type of action result
 */
template<class ActionT, class ActionTGoal, class ActionTResult>
class BtActionNode : public BT::ActionNodeBase
{
public:
  /**
   * @brief A navit_bt_nodes::BtActionNode constructor
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  BtActionNode(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BT::ActionNodeBase(xml_tag_name, conf), action_name_(action_name)
  {
    node_ = config().blackboard->template get<ros::NodeHandle>("node");

    bt_loop_duration_ = config().blackboard->template get<double>("bt_loop_duration");
    server_timeout_ = config().blackboard->template get<double>("server_timeout");
    getInput<double>("server_timeout", server_timeout_);

    std::string remapped_action_name;
    if (getInput("server_name", remapped_action_name)) {
      action_name_ = remapped_action_name;
    }

    ROS_WARN( "Try to create instance of  \"%s\" action client", action_name_.c_str());

    createActionClient(action_name_);

    ROS_WARN( "Node \"%s\" BtActionNode initialized", action_name_.c_str());
  }

  BtActionNode() = delete;

  virtual ~BtActionNode()
  {
  }

  /**
   * @brief Create instance of an action client
   * @param action_name Action name to create client for
   */
  void createActionClient(const std::string & action_name)
  {
    // Now that we have the ROS node to use, create the action client for this BT action
    action_client_ =std::make_shared<actionlib::SimpleActionClient<ActionT>> (action_name, true);

    ROS_WARN( "Waiting for \"%s\" action server", action_name.c_str());
    action_client_->waitForServer(); //will wait for infinite time
  }

  /**
   * @brief Any subclass of BtActionNode that accepts parameters must provide a
   * providedPorts method and call providedBasicPorts in it.
   * @param addition Additional ports to add to BT port list
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {
      BT::InputPort<std::string>("server_name", "Action server name"),
      BT::InputPort<std::chrono::milliseconds>("server_timeout")
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

  // Derived classes can override any of the following methods to hook into the
  // processing for the action: on_tick, on_wait_for_result, and on_success

  /**
   * @brief Function to perform some user-defined operation on tick
   * Could do dynamic checks, such as getting updates to values on the blackboard -- [Goal valid check]
   */
  virtual void on_tick()
  {
  } 

  /**
   * @brief Function to perform some user-defined operation after a timeout
   * waiting for a result that hasn't been received yet
   */
  virtual void on_wait_for_result()
  {
  }

  /**
   * @brief Function to perform some user-defined operation upon successful
   * completion of the action. Could put a value on the blackboard.
   * @return BT::NodeStatus Returns SUCCESS by default, user may override return another value
   */
  virtual BT::NodeStatus on_success()
  {
    return BT::NodeStatus::SUCCESS;
  }

  /**
   * @brief Function to perform some user-defined operation whe the action is aborted.
   * @return BT::NodeStatus Returns FAILURE by default, user may override return another value
   */
  virtual BT::NodeStatus on_aborted()
  {
    ROS_ERROR("Action client \"%s\" on_aborted", action_name_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  /**
   * @brief Function to perform some user-defined operation when the action is cancelled.
   * @return BT::NodeStatus Returns SUCCESS by default, user may override return another value
   */
  virtual BT::NodeStatus on_cancelled()
  {
    action_client_->cancelGoal();
    return BT::NodeStatus::SUCCESS;
  }

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override
  {
    // ROS_WARN( "Action node  \"%s\" ticking", action_name_.c_str());
    // first step to be done only at the beginning of the Action
    if (status() == BT::NodeStatus::IDLE) {
      setStatus(BT::NodeStatus::RUNNING);

      // user defined callback
      on_tick();

      if (goal_valid_)
        send_new_goal();
    }

    // if (action_client_->getState().isDone())
    //   ROS_INFO("Action node  \"%s\"  request is DONE, Client get status is [%s]", 
    //                             action_name_.c_str(), action_client_->getState().toString().c_str());
    // else
    //   ROS_WARN("Action node  \"%s\"  request is NOT DONE, Client get status is [%s]", 
    //                              action_name_.c_str(), action_client_->getState().toString().c_str());

    // if new goal was sent and action server has not yet responded check the future goal handle
    // isDone(): True if in RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, or LOST. False otherwise (PENDING, ACTIVE...)
    if (action_client_->isServerConnected() && !action_client_->getState().isDone() && 
          action_client_->getState() != actionlib::SimpleClientGoalState::StateEnum::ACTIVE) {
      auto elapsed = (ros::Time::now() - time_goal_sent_).toSec(); 
      if (!is_future_goal_handle_complete(elapsed)) {
        // return RUNNING if there is still some time before timeout happens
        if (elapsed < server_timeout_) {
          ROS_WARN("Action node  \"%s\"  still some time before timeout happens", action_name_.c_str());
          return BT::NodeStatus::RUNNING;
        }
        // if server has taken more time to respond than the specified timeout value return FAILURE  
        ROS_WARN("Timed out 1: while waiting for action server to acknowledge goal request for %s",action_name_.c_str());
        return BT::NodeStatus::FAILURE;
      }
    }

    // ROS_INFO("Action node  \"%s\" server connected .", action_name_.c_str());

    // The following code corresponds to the "RUNNING" loop
    if (ros::ok() && !goal_result_available_) {
      // user defined callback. May modify the value of "goal_updated_"
      on_wait_for_result();

      if (goal_updated_ && (action_client_->getState() == actionlib::SimpleClientGoalState::StateEnum::ACTIVE || 
          action_client_->getState() == actionlib::SimpleClientGoalState::StateEnum::PENDING)) // PENDING means server been killed
      {
        goal_updated_ = false;
        send_new_goal();
        auto elapsed = (ros::Time::now() - time_goal_sent_).toSec(); //.to_chrono<std::chrono::milliseconds>();
        if (!is_future_goal_handle_complete(elapsed)  && !action_client_->getState().isDone() && 
              action_client_->getState() != actionlib::SimpleClientGoalState::StateEnum::ACTIVE) {
          if (elapsed < server_timeout_) {
            ROS_WARN("Action node  \"%s\"  still some time before timeout happens", action_name_.c_str());
            return BT::NodeStatus::RUNNING;
          }
          if (action_client_->getState() == actionlib::SimpleClientGoalState::StateEnum::PENDING)
          {
            // action_client_->cancelGoal();
            action_client_->cancelAllGoals();
            ROS_WARN("Timed out 2: %s state is PENDING, cancel all goals and return FAILURE.",action_name_.c_str());
          }
          ROS_WARN("Timed out 2:  while waiting for action server to acknowledge goal request for %s",action_name_.c_str());
          // return BT::NodeStatus::FAILURE;
        }
        // ROS_INFO("Action node  \"%s\" NEW GOAL PREEMPTED FINISHED.", action_name_.c_str());
      }
    }    
    
    ROS_INFO( "Action node  \"%s\": State  [ \"%s\" ]  now", 
                            action_name_.c_str(),action_client_->getState().toString().c_str());

    BT::NodeStatus status;
    actionlib::SimpleClientGoalState action_status = action_client_->getState();
    if (action_status == actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED)
      status = on_success();
    else if (action_status == actionlib::SimpleClientGoalState::StateEnum::ACTIVE ||
                  action_status == actionlib::SimpleClientGoalState::StateEnum::PREEMPTED ||
                  action_status == actionlib::SimpleClientGoalState::StateEnum::PENDING)
      status = BT::NodeStatus::RUNNING;
    else
      status = on_aborted();

    return status;
  }

  /**
   * @brief The other (optional) override required by a BT action. In this case, we
   * make sure to cancel the ROS2 action if it is still running.
   */
  void halt() override
  {
    // if (should_cancel_goal()) {
    //   action_client_->cancelGoalsAtAndBeforeTime(ros::Time::now());
    //   ROS_WARN("Failed to cancel all goals for action server before NOW.");
    // }
    action_client_->cancelGoalsAtAndBeforeTime(ros::Time::now());
    setStatus(BT::NodeStatus::IDLE);
  }

protected:
  /**
   * @brief Function to check if current goal should be cancelled
   * @return bool True if current goal should be cancelled, false otherwise
   */
  bool should_cancel_goal()
  {
    // Shut the node down if it is currently running
    if (status() != BT::NodeStatus::RUNNING) {
      ROS_WARN("Action node  \"%s\"  CHECK should_cancel_goal, node is currently running.", action_name_.c_str());
      return false;
    }

    action_client_->waitForResult(ros::Duration(server_timeout_));
    ROS_ERROR("Action node  \"%s\"  CHECK should_cancel_goal, Client get status is [%s]", 
                                action_name_.c_str(), action_client_->getState().toString().c_str());

    bool should_cancel = false;
    if (action_client_->getState() != actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED ||
                  action_client_->getState() != actionlib::SimpleClientGoalState::StateEnum::ACTIVE)
    {
      ROS_INFO("Action node  \"%s\"  CHECK should_cancel_goal: [%d]", action_name_.c_str(), should_cancel ? 1 : 0);
    }

    return should_cancel;
  }

  /**
   * @brief Function to send new goal to action server
   */
  void send_new_goal()
  {
    goal_result_available_ = false;
    
    action_client_->sendGoal(goal_);

    // ROS_INFO( "BT action node  \"%s\" send a new goal.", action_name_.c_str());

    time_goal_sent_ = ros::Time::now();

    action_client_->waitForResult(ros::Duration(server_timeout_));

    result_ = action_client_->getResult();

    // ROS_INFO( "Action node  \"%s\" got result: State  [ \"%s\" ]  now", 
    //                         action_name_.c_str(),action_client_->getState().toString().c_str());
  }

  /**
   * @brief Function to check if the action server acknowledged a new goal
   * @param elapsed Duration since the last goal was sent and future goal handle has not completed.
   * After waiting for the future to complete, this value is incremented with the timeout value.
   * @return boolean True if future_goal_handle_ returns SUCCESS, False otherwise
   */
  bool is_future_goal_handle_complete(double & elapsed)
  {
    auto remaining = server_timeout_ - elapsed;

    // ROS_WARN( "BT action node  \"%s\" check future goal complete, elapsed time = [%f], remaining = [%f].", 
    //                           action_name_.c_str(), elapsed, remaining);

    auto timeout = remaining > bt_loop_duration_ ? bt_loop_duration_ : remaining;
    timeout = remaining < epsilon_time ? epsilon_time : remaining;
    action_client_->waitForResult(ros::Duration(timeout));
    elapsed += timeout;

    actionlib::SimpleClientGoalState state = action_client_->getState();
    if (state == actionlib::SimpleClientGoalState::StateEnum::ACTIVE)
    {
      ROS_INFO("Action node  \"%s\" is not SUCCESS, while state: ACTIVE.", action_name_.c_str());
      return true;
    }      

    return false;
  }

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

  std::string action_name_;
  typename  std::shared_ptr<actionlib::SimpleActionClient<ActionT>> action_client_;

  ActionTGoal goal_;
  ActionTResult result_;

  ros::NodeHandle node_;
  ros::Time time_goal_sent_;

  double server_timeout_;
  double bt_loop_duration_{0.5};
  double epsilon_time = 0.01;
  bool goal_updated_{false};
  bool goal_result_available_{false};  
  bool goal_valid_{true};
};

}  // namespace navit_bt_nodes

#endif  // NAVIT_BT_NODES__BT_ACTION_NODE_HPP_
