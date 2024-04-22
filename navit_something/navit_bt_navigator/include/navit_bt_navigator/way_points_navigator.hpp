#include "navit_bt_nodes/behavior_tree_engine.hpp"

#include <ros/ros.h>
#include <ros/package.h>

#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_minitrace_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#ifdef ZMQ_FOUND
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#endif


const std::vector<std::string> plugin_lib_names_ = {
    "navit_bt_nodes_pipeline_sequence",
    "navit_bt_nodes_rate_controller",
    "navit_bt_nodes_way_points_node",
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