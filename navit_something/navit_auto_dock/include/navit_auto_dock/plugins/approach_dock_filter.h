#ifndef APPROACH_DOCK_FILTER_H
#define APPROACH_DOCK_FILTER_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/buffer.h>

namespace navit_auto_dock {
namespace plugins{

    class ApproachDockFilter
    {
        public:    
            using Ptr = boost::shared_ptr<ApproachDockFilter>;

            virtual ~ApproachDockFilter(){}

            virtual void initialize(const std::string& name,
                                    const std::shared_ptr<tf2_ros::Buffer>& tf = nullptr) = 0;

            virtual void reset() = 0;

            virtual void update(geometry_msgs::PoseStamped& dock_pose,
                                const geometry_msgs::Twist& current_vel = geometry_msgs::Twist())=0;
    };
}
}
#endif
