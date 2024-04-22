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
  "navit_bt_nodes_approach_dock_action",
  "navit_bt_nodes_get_polygon_from_service",
  "navit_bt_nodes_load_polygon_json_service",
  "navit_bt_nodes_back_to_reference_line_action",
  "navit_bt_nodes_back_up_action",
  "navit_bt_nodes_clear_costmap_service",
  "navit_bt_nodes_compute_path_to_pose_action",
  "navit_bt_nodes_coverage_path_action",
  "navit_bt_nodes_fetch_item_action",
  "navit_bt_nodes_final_dock_action",
  "navit_bt_nodes_follow_path_action",
  "navit_bt_nodes_get_area_edge_service",
  "navit_bt_nodes_get_route_path_service",
  "navit_bt_nodes_get_station_pose_service",
  "navit_bt_nodes_get_closest_id_from_node_files",
  "navit_bt_nodes_get_current_controller_plugin_service",
  "navit_bt_nodes_get_current_planner_plugin_service",
  "navit_bt_nodes_nem_compute_path_to_pose_action",
  "navit_bt_nodes_synchronize_ne_files_service",
  "navit_bt_nodes_move_through_poses_action",
  "navit_bt_nodes_move_to_pose_action",
  "navit_bt_nodes_print_black_board_value",
  "navit_bt_nodes_remove_passed_goals_action",
  "navit_bt_nodes_rotate_action",
  "navit_bt_nodes_truncate_path_action",
  "navit_bt_nodes_wait_action",
  "navit_bt_nodes_wall_path_action",
  "navit_bt_nodes_goal_updated_condition",
  "navit_bt_nodes_is_battery_low_condition",
  "navit_bt_nodes_need_plan_condition",
  "navit_bt_nodes_pipeline_sequence",
  "navit_bt_nodes_recovery_node",
  "navit_bt_nodes_get_position_in_frame",
  "navit_bt_nodes_goal_updater_node",
  "navit_bt_nodes_rate_controller",
  "navit_bt_nodes_select_goal_action",
  "navit_bt_nodes_via_points_node",
  "navit_bt_nodes_way_points_node",
  "navit_bt_nodes_task_receiver",
  "navit_bt_nodes_follow_coverage_path_action",
  "navit_bt_nodes_task_map_parser_action",
  "navit_bt_nodes_task_command_service_action",
  "navit_bt_nodes_polygon_coverage_path_planner_action",
  "navit_bt_nodes_get_path_first_point_action",
  "navit_bt_nodes_switch_map_service",
  "navit_bt_nodes_get_driveable_area_id",
  "navit_bt_nodes_routing_action",
  "navit_bt_nodes_handle_via_points",
  "navit_bt_nodes_reset_value",
  "navit_bt_nodes_inverse_path",
  "navit_bt_nodes_set_clean_mode_action",
  "navit_bt_nodes_communicate_with_supply_action",
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
