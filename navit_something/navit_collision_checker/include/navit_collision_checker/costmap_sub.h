#ifndef COSTMAP_SUB_H
#define COSTMAP_SUB_H

#include <ros/ros.h>
#include <navit_costmap/costmap_2d.h>
#include <nav_msgs/OccupancyGrid.h>
#include <mutex>

using namespace navit_costmap;

namespace navit_collision_checker {

class CostmapSub 
{
    public:
        CostmapSub(const ros::NodeHandle& nh, const std::string& topic_name);

        ~CostmapSub() = default;

        std::shared_ptr<Costmap2D> getCostmap();

        std::mutex costmap_lock_;

    protected:
        void toCostmap2D();

        void costmapCallback(const nav_msgs::OccupancyGridConstPtr& msg);

        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        std::shared_ptr<Costmap2D> costmap_;
        nav_msgs::OccupancyGrid::ConstPtr costmap_msg_;
        std::string topic_name_;
        bool costmap_received_;
};


}
#endif
