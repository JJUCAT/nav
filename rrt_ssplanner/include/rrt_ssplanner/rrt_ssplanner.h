/**
 * @copyright Copyright (c) {2022} JUCAT
 * @author jucat (lmr2887@163.com)
 * @date 2024-01-01
 * @brief 
 */
#ifndef RRT_PLANNER_H_
#define RRT_PLANNER_H_
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseArray.h"
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <rrt_ssplanner/tree.h>

namespace rrt_planner {

/**
  * @brief rrt star smart planner
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
    /**
      * @brief  Checks the legality of the robot footprint at a position and orientation using the world model
      * @param x_i The x position of the robot 
      * @param y_i The y position of the robot 
      * @param theta_i The orientation of the robot
      * @return 
      */
    double footprintCost(double x_i, double y_i, double theta_i);

    /**
     * @brief  采样一个随机点
     * @return geometry_msgs::Point  采样点
     */
    geometry_msgs::Point Sample();

    /**
     * @brief  搜索与 rand_point 最近的点
     * @param  rand_point  随机点
     * @return size_t  最近点下标
     */
    size_t GetNearestPoint(const geometry_msgs::Point random_point);

    /**
     * @brief  从 rand_point 到 nestest_point 移动产生新节点 new_point，这个根据系统而定
     * @param  nestest_point  最近点
     * @param  random_point  随机采样点
     * @param  new_point   新节点
     * @return  新节点是否有效
     */
    bool Steer(const geometry_msgs::Point nestest_point,
               const geometry_msgs::Point random_point,
               geometry_msgs::Point& new_point);

    /**
     * @brief  获取邻近半径
     * @param  n  迭代次数
     * @return double  邻近半径
     */
    double GetNearRadius(size_t n);

    /**
     * @brief  获取邻近点
     * @param  center_point  搜索的中心点
     * @param  radius  搜索半径
     * @return std::vector<size_t> 
     */
    std::vector<size_t> GetNearPoints(const geometry_msgs::Point center_point, const double radius);

    /**
     * @brief  给 new_point 更新父节点
     * @param  near_points  邻近节点
     * @param  new_point  准备插入的新节点
     * @return size_t
     */
    size_t ChooseParent(const std::vector<size_t>& near_points, const geometry_msgs::Point new_point);

    /**
     * @brief  重新布局网络
     * @param  new_point_index  新节点下标
     * @param  new_point_parent_index  新节点的父节点下标
     * @param  near_points  邻近节点
     */
    void Rewire(const size_t new_point_index,
                const size_t new_point_parent_index,
                const std::vector<size_t>& near_points);

    /**
     * @brief  是否碰撞，根据系统而定
     * @param  p  检查的位姿
     * @return true 
     * @return false 
     */
    bool IsCollised(const geometry_msgs::Pose p);
    
    /**
     * @brief  移动到新节点的检测步长
     */
    void calculateCheckDeltaStep();

    bool initialized_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_;
    std::shared_ptr<base_local_planner::WorldModel> world_model_;
    std::shared_ptr<rrt_planner::Tree> tree_;
    double check_dstep_;




    // -------------------- parameters --------------------
    int max_iterations_;
    double timeout_;
    double step_;
    double r_gamma_;



};  // class RrtStarSmartPlanner

} // namespace rrt_planner

#endif // RRT_PLANNER_H_
