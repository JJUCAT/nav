#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <navit_costmap/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/Path.h>

namespace navit_core {
  // navit controller interface
  class Controller {
    public:
        using Ptr = boost::shared_ptr<Controller>;
        virtual ~Controller(){}

        virtual geometry_msgs::Twist computeVelocityCommands(const geometry_msgs::PoseStamped& current_pose,
                                                             const geometry_msgs::Twist& current_vel) = 0;

        virtual bool computeVelocity(geometry_msgs::Twist& current_vel) { return true; };

        virtual void initialize(const std::string& name,
                                const std::shared_ptr<tf2_ros::Buffer>& tf,
                                const std::shared_ptr<navit_costmap::Costmap2DROS>& costmap_ros) = 0;


        virtual bool setPlan(const nav_msgs::Path& plan) = 0;
        
        virtual bool isGoalReached() = 0;

        virtual bool setSpeedLimit(const double& speed_limit) = 0;

        // 传统点边模式需要 
        virtual bool initNemData(const std::string &node_file_path, const std::string &edge_file_path)
        {
            return false;
        }

  };

};  // namespace navit_core

#endif  // NAV_CORE_BASE_LOCAL_PLANNER_H
