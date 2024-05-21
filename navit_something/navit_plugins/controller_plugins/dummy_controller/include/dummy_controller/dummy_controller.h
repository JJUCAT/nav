#ifndef DUMMY_CONTROLLER_H
#define DUMMY_CONTROLLER_H

#include <navit_core/base_controller.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <navit_costmap/costmap_2d_ros.h>

namespace dummy_controller {
    class DummyController : public navit_core::Controller
    {
        public:
            DummyController(){}

            ~DummyController(){}

            geometry_msgs::Twist 
            computeVelocityCommands(const geometry_msgs::PoseStamped& current_pose,
                                    const geometry_msgs::Twist& current_vel);

            void initialize(const std::string& name,
                            const std::shared_ptr<tf2_ros::Buffer>& tf,
                            const std::shared_ptr<navit_costmap::Costmap2DROS>& costmap_ros);

            bool setPlan(const nav_msgs::Path& plan){ 
                return true;
            }
            
            bool isGoalReached(){
                if (dummy_counter_ > dummy_param_)
                {
                    dummy_counter_ = 0;
                    return true;
                }
                else
                    return false;
            }

            bool setSpeedLimit(const double& limit){ return true;}
        protected:
            int dummy_counter_, dummy_param_;
            std::string name_;
            std::shared_ptr<tf2_ros::Buffer> tf_;
            std::shared_ptr<navit_costmap::Costmap2DROS> costmap_ros_;
    };

}
#endif
