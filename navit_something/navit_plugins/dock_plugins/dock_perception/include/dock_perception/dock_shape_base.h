#ifndef DOCK_SHAPE_BASE_H
#define DOCK_SHAPE_BASE_H

#include <geometry_msgs/Point.h>
#include <ros/ros.h>

namespace dock_perception{
    class DockShapeBase
    {
        public:
        using Cloud = std::vector<geometry_msgs::Point>;
        using Ptr = boost::shared_ptr<DockShapeBase>;

        virtual ~DockShapeBase(){}

        virtual Cloud getIdealCloud(ros::NodeHandle &pnh) = 0; 
    };
}
#endif
