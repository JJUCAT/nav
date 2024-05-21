//
// Created by yjh on 23-2-24.
//
#include <ros/ros.h>

#include <path_planner/astar_search.h>

namespace Planner {

class hybridAStarPlanner {
    public:
        void run();
    private:

};

void hybridAStarPlanner::run() {

}

} //hybridAStarPlanner

int main(int argc, char **argv) {

    ros::init(argc, argv, "hybrid_astar_node");

    Planner::hybridAStarPlanner hybrid_astar_planner;

    hybrid_astar_planner.run();

    return 0;
}
