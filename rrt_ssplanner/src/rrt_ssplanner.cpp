/**
 * @copyright Copyright (c) {2022} JUCAT
 * @author jucat (lmr2887@163.com)
 * @date 2024-01-01
 * @brief 
 */
#include <angles/angles.h>
#include <rrt_ssplanner/rrt_ssplanner.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_planner::RrtStarSmartPlanner, nav_core::BaseGlobalPlanner)
//TODO:cmake 依赖添加，完成单独编译

namespace rrt_planner {
RrtStarSmartPlanner::~RrtStarSmartPlanner() {

}


RrtStarSmartPlanner::RrtStarSmartPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : costmap_ros_(NULL), costmap_(NULL), world_model_(NULL), initialized_(false){
  initialize(name, costmap_ros);
}

void RrtStarSmartPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){

}

double RrtStarSmartPlanner::footprintCost(double x_i, double y_i, double theta_i){
  return 0.f;
}

bool RrtStarSmartPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
    const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){

  return true;
}

}; // namespace rrt_planner
