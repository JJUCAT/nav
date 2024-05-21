#include "prm_planner/prm_planner.h"
#include "load_json.hpp"

#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "planning_scene_loader_node");
    ros::NodeHandle nh;

    // PlanningSceneLoader loader;
    // PlanningScene scene = loader.LoadFromFile("/home/yjh/map.json");
    
    // // Set the bounds and obstacles of the scene...
    // navit_planner::PRMPlanner planner(scene);

    // planner.plan();

    // ros::spin();  // Start message processing loop
    return 0;   
}