#include <iostream>
#include <future>
#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/server/simple_action_server.h>
#include <navit_msgs/SimpleNavigatorAction.h>

#include "navit_bt_nodes/behavior_tree_engine.hpp"

#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_minitrace_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#ifdef ZMQ_FOUND
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#endif

class SimpleNavigatorActionServer
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<navit_msgs::SimpleNavigatorAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  navit_msgs::SimpleNavigatorFeedback feedback_;
  navit_msgs::SimpleNavigatorResult result_;

public:

    SimpleNavigatorActionServer(std::string name):
        as_(nh_, name, boost::bind(&SimpleNavigatorActionServer::executeCB, this, _1), false),
        action_name_(name)
    {
        ROS_WARN("Waiting for navigator client ...");
        ROS_WARN("Waiting for navigator client ...");
        ROS_WARN("Waiting for navigator client ...");
        
        as_.start();
    }

    ~SimpleNavigatorActionServer(void)
    {
        // halt all BT nodes
        //
    }

    const std::vector<std::string> plugin_lib_names_ = {
        "navit_bt_nodes_pipeline_sequence",
        "navit_bt_nodes_rate_controller",
        "navit_bt_nodes_goal_updater_node",
        "navit_bt_nodes_truncate_path_action",
        "navit_bt_nodes_compute_path_to_pose_action",
        "navit_bt_nodes_follow_path_action",
        "navit_bt_nodes_approach_dock_action",
        "navit_bt_nodes_final_dock_action",
        "navit_bt_nodes_is_battery_low_condition",
        "navit_bt_nodes_wait_action",
        "navit_bt_nodes_back_up_action",
        "navit_bt_nodes_clear_costmap_service",
        "navit_bt_nodes_recovery_node",
    };

    template<typename T>
    T getParam(const std::string& name,const T& defaultValue) //This name must be namespace+parameter_name
    {
        T v;
        if(ros::param::get(name,v)) //get parameter by name depend on ROS.
        {
            ROS_INFO_STREAM("Found parameter: "<<name<<",\tvalue: "<<v);
            return v;
        }
        else
            ROS_WARN_STREAM("Cannot find value for parameter: "<<name<<",\tassigning default: "<<defaultValue);
        return defaultValue; //if the parameter haven't been set,it's value will return defaultValue.
    };
    
    void executeCB(const navit_msgs::SimpleNavigatorGoalConstPtr &goal);

};