/**
 * @copyright Copyright (c) {2022} JUCAT
 * @author jucat (lmr2887@163.com)
 * @date 2024-01-01
 * @brief 
 */
#ifndef RRT_STAR_SMART_PLANNER_H_
#define RRT_STAR_SMART_PLANNER_H_
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

namespace rrt_star_smart {

/**
  * @brief 
  */
class RrtStarSmartPlanner : public nav_core::BaseGlobalPlanner {
  public:
    /**
      * @brief  Constructor for the CarrotPlanner
      */
    RrtStarSmartPlanner() = default;

    /**
      * @brief Destructor
      */
    virtual ~RrtStarSmartPlanner();

    /**
      * @brief  Constructor for the CarrotPlanner
      * @param  name The name of this planner
      * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
      */
    RrtStarSmartPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /**
      * @brief  Initialization function for the CarrotPlanner
      * @param  name The name of this planner
      * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
      */
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /**
      * @brief Given a goal pose in the world, compute a plan
      * @param start The start pose 
      * @param goal The goal pose 
      * @param plan The plan... filled by the planner
      * @return True if a valid plan was found, false otherwise
      */
    bool makePlan(const geometry_msgs::PoseStamped& start, 
        const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

  private:
    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_;
    base_local_planner::WorldModel* world_model_; ///< @brief The world model that the controller will use

    /**
      * @brief  Checks the legality of the robot footprint at a position and orientation using the world model
      * @param x_i The x position of the robot 
      * @param y_i The y position of the robot 
      * @param theta_i The orientation of the robot
      * @return 
      */
    double footprintCost(double x_i, double y_i, double theta_i);

    bool initialized_;
};

}; // namespace rrt_star_smart

#endif // RRT_STAR_SMART_PLANNER_H_
