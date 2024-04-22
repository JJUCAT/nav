#ifndef APPROCH_DOCK_FILTER_AVERAGE_FILTER_H
#define APPROCH_DOCK_FILTER_AVERAGE_FILTER_H

#include <navit_auto_dock/plugins/approach_dock_filter.h>
#include <list>
#include <angles/angles.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <ros/ros.h>

namespace navit_auto_dock {
namespace plugins {
    class AverageFilter : public ApproachDockFilter
    {
        public:
            ~AverageFilter(){
            }
            void initialize(const std::string& name,
                            const std::shared_ptr<tf2_ros::Buffer>& tf) override;

            void reset() override;

            void update(geometry_msgs::PoseStamped& dock_pose,
                        const geometry_msgs::Twist& current_vel) override;

        protected:
            
            struct {
                int window_lenght = 20;
                } config_; 

	    std::list<geometry_msgs::PoseStamped> filter_; 
    };
}
}
#endif
