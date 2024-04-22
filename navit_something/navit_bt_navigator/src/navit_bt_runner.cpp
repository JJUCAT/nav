//
// Created by fan on 23-1-30.
//

#include "navit_bt_navigator/navit_bt_runner.h"
#include <vector>
#include <navit_bt_nodes/behavior_tree_engine.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>
//#include <filesystem>

namespace navit_bt_navigator
{

struct NavitBtRunner::Impl
{
  // 这里的插件命名规则是 navit_bt_nodes_ + 节点文件名
  inline static const std::vector<std::string> default_plugins = {
    "navit_bt_nodes_pipeline_sequence",
    "navit_bt_nodes_rate_controller",
    "navit_bt_nodes_way_points_node",
    "navit_bt_nodes_via_points_node",
    "navit_bt_nodes_goal_updater_node",
    "navit_bt_nodes_truncate_path_action",
    "navit_bt_nodes_compute_path_to_pose_action",
    "navit_bt_nodes_follow_path_action",
    "navit_bt_nodes_approach_dock_action",
    "navit_bt_nodes_final_dock_action",
    "navit_bt_nodes_coverage_path_action",
    "navit_bt_nodes_wall_path_action",
    "navit_bt_nodes_need_plan_condition",
    "navit_bt_nodes_goal_updated_condition",
    "navit_bt_nodes_wait_action",
    "navit_bt_nodes_back_up_action",
    "navit_bt_nodes_clear_costmap_service",
    "navit_bt_nodes_recovery_node",
    "navit_bt_nodes_get_station_pose_service",
    "navit_bt_nodes_get_route_path_service",
    "navit_bt_nodes_get_position_in_frame",
    "navit_bt_nodes_print_black_board_value",
    "navit_bt_nodes_select_goal_action",
    "navit_bt_nodes_task_plan_editor_action",
    "navit_bt_nodes_task_plan_recorder_action",
    "navit_bt_nodes_task_plan_reporter_action",
    "navit_bt_nodes_clear_map_action",
    "navit_bt_nodes_merge_plans_action",
    "navit_bt_nodes_move_back_action",
    "navit_bt_nodes_select_goal_on_plan_action",
    "navit_bt_nodes_final_succeed_loop",
    "navit_bt_nodes_ctrl_obstacle_layer_action",
    "navit_bt_nodes_plan_security_checking_action",
    "navit_bt_nodes_path_to_point_action",
    "navit_bt_nodes_seq_execute_final_recovery",
    "navit_bt_nodes_select_start_on_plan_action",
    "navit_bt_nodes_compute_coverage_path_with_pwh",
    "navit_bt_nodes_ignore",
    "navit_bt_nodes_close_obstacle_avoidance_service",
    "navit_bt_nodes_excape_action",
    "navit_bt_nodes_is_precise_arrival_action",
  };

  BT::Tree tree;
  std::unique_ptr<navit_bt_nodes::BehaviorTreeEngine> bt_;

  std::string name_;
  std::mutex mutex_;
  std::unique_ptr<BT::StdCoutLogger> stdcout_logger;
  std::unique_ptr<BT::FileLogger> file_logger;

#ifdef ZMQ_FOUND
  std::unique_ptr<PublisherZMQ> publisher_zmq;
#endif

  explicit Impl(const std::string& name) : name_(name)
  {
  }

  bool init(const std::string& xml_string, const std::string& log_path, BT::Blackboard::Ptr blackboard)
  {
    std::scoped_lock<std::mutex> l(mutex_);
    if (!bt_)
    {
      ROS_WARN("Plugins Loading.....");
      bt_ = std::make_unique<navit_bt_nodes::BehaviorTreeEngine>(default_plugins);
      if (!bt_)
      {
        ROS_ERROR("navit_bt_nodes::BehaviorTreeEngine init error.");
        return false;
      }
      ROS_WARN("Plugins Loading Finished.");
    }

    // try
    // {
      tree = bt_->createTreeFromText(xml_string, blackboard);
    //}
    // catch (...)
    // {
    //   ROS_ERROR("createTreeFromText error.");
    //   return false;
    // }

    stdcout_logger = std::make_unique<BT::StdCoutLogger>(tree);
    // if (std::filesystem::path(log_path).has_filename())
    //{
      file_logger = std::make_unique<BT::FileLogger>(tree, log_path.c_str());
    //}

#ifdef ZMQ_FOUND
    publisher_zmq = std::make_unique<PublisherZMQ>(tree);
#endif
    printTreeRecursively(tree.rootNode());
    return true;
  }
};

NavitBtRunner::NavitBtRunner(const std::string& name) : impl_(std::make_shared<Impl>(name))
{
}

NavitBtRunner::~NavitBtRunner()
{
  stop();
}

bool NavitBtRunner::init(const std::string& xml_string, const std::string& log_path, BT::Blackboard::Ptr blackboard)
{
  return impl_->init(xml_string, log_path, blackboard);
}

BT::NodeStatus NavitBtRunner::tickRoot()
{
  return impl_->tree.tickRoot();
}

void NavitBtRunner::stop()
{
  if (impl_->tree.rootNode() && !impl_->tree.rootNode()->isHalted())
  {
    ROS_WARN(" ** stop BT tree!");
    impl_->tree.haltTree();
  }
}
}  // namespace navit_bt_navigator