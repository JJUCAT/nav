#ifndef NAV_CORE_BASE_LOCAL_PLANNER_H
#define NAV_CORE_BASE_LOCAL_PLANNER_H

#warning navit_core/base_local_planner.h has been deprecated

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <navit_costmap/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>

namespace nav_core {
  /**
   * @class BaseLocalPlanner
   * @brief Provides an interface for local planners used in navigation. All local planners written as plugins for the navigation stack must adhere to this interface.
   */
  class BaseLocalPlanner{
    public:
      /**
       * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid velocity command was found, false otherwise
       */
      virtual bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) = 0;

      /**
       * @brief  Check if the goal pose has been achieved by the local planner
       * @return True if achieved, false otherwise
       */
      virtual bool isGoalReached() = 0;

      /**
       * @brief  Set the plan that the local planner is following
       * @param plan The plan to pass to the local planner
       * @return True if the plan was updated successfully, false otherwise
       */
      virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) = 0;

      /**
       * @brief  Constructs the local planner
       * @param name The name to give this instance of the local planner
       * @param tf A pointer to a transform listener
       * @param costmap_ros The cost map to use for assigning costs to local plans
       */
      virtual void initialize(std::string name, tf2_ros::Buffer* tf, navit_costmap::Costmap2DROS* costmap_ros) = 0;

      virtual bool initNemData(const std::string &node_file_path, const std::string &edge_file_path){}

      /**
       * @brief  Virtual destructor for the interface
       */
      virtual ~BaseLocalPlanner(){}

    protected:
      BaseLocalPlanner(){}
  };
};  // namespace nav_core

#endif  // NAV_CORE_BASE_LOCAL_PLANNER_H
