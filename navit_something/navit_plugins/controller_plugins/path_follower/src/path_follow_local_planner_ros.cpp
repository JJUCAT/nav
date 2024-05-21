//
// Created by yjh on 23-2-24.
//
#include <ros/ros.h>

#include <path_planner/astar_search.h>

namespace planner {

class PathFollowLocalPlanner {

public:
    
    PathFollowLocalPlanner(){};
    
    ~PathFollowLocalPlanner(){};
    
    void run();

private:
	AstarSearch* astar_search_ptr_;
}; 


void PathFollowLocalPlanner::run() {
	
	ros::spin();
}

} //namespace planner

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "path_follower_node");

    planner::PathFollowLocalPlanner path_follow_local_planner;

    path_follow_local_planner.run();

    return 0;
}
