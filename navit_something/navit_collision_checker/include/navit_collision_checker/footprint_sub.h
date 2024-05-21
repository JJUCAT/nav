#ifndef FOOTPRINT_SUB_H
#define FOOTPRINT_SUB_H

#include <navit_costmap/footprint.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>
#include <mutex>

using namespace navit_costmap;

namespace navit_collision_checker {

class FootprintSub
{
    public:
        FootprintSub(const ros::NodeHandle& nh, const std::string& topic_name);

        ~FootprintSub() = default;

        bool getFootprint(std::vector<geometry_msgs::Point>& footprint);

        std::mutex footprint_lock_;
    protected:

        ros::NodeHandle nh_;
        ros::Subscriber sub_;

        void footprintCallback(const geometry_msgs::PolygonStamped::ConstPtr& msg);

        geometry_msgs::PolygonStamped::ConstPtr footprint_;

        std::string topic_name_;

        bool footprint_received_;

};


}

#endif
