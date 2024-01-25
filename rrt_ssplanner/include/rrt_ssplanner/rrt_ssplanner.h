/**
 * @copyright Copyright (c) {2022} LMRCAT
 * @author lmrcat (lmr2887@163.com)
 * @date 2024-01-01
 * @brief 
 */
#ifndef RRT_PLANNER_H_
#define RRT_PLANNER_H_
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseArray.h"
#include "ros/publisher.h"
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <rrt_ssplanner/node.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
     * @brief 采样器
     */
    class Sample
    {
      public:

        Sample() = delete;
        Sample(costmap_2d::Costmap2D* map, const double ratio, const double r)
          : costmap_(map), ratio_(ratio), r_(r) {}
        ~Sample() = default;

        /**
         * @brief  采样
         * @return geometry_msgs::Point 
         */
        geometry_msgs::Point SamplePoint();

        /**
        * @brief  获取信标采样的间隔
        * @param  n  终点采样的迭代次数
        * @return size_t
        */
        void SetBiasing(const size_t n);

        /**
         * @brief  设置信标
         * @param  beacons  信标节点
         * @return size_t 
         */
        void SetBeacons(const std::vector<geometry_msgs::Point> beacons);

      private:

        /**
        * @brief  采样一个随机点
        * @return geometry_msgs::Point  采样点
        */
        geometry_msgs::Point RandomSample();

        /**
        * @brief  信标采样
        * @return geometry_msgs::Point 
        */
        geometry_msgs::Point BeaconSample();

        costmap_2d::Costmap2D* costmap_;
        double ratio_;
        std::vector<geometry_msgs::Point> beacons_;
        bool reach_;
        size_t b_;
        size_t c_ = 0;
        double r_;
    };

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
     * @brief  插入节点
     * @param  index            My Param doc
     * @param  point            My Param doc
     */
    void Insert(const size_t index, const geometry_msgs::Point point);

    /**
     * @brief  重新布局
     * @param  node             My Param doc
     * @param  near_points      My Param doc
     */
    void Rewire(const size_t node, const std::vector<size_t>& near_points);

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
     * @brief  获取终点
     * @return rrt_planner::Node* 
     */
    rrt_planner::Node* GetTerminal();

    /**
    * @brief  从节点向父节点一直遍历到根节点，累积节点移动代价
    * @param  node  要遍历的起始节点
    * @return double  总移动代价
    */
    double TrajectoryCost(rrt_planner::Node* node);

    /**
    * @brief  获取轨迹
    * @param  node  轨迹的终点节点
    * @return std::vector<geometry_msgs::Point> 
    */
    std::vector<geometry_msgs::Point> GeTrajectory(rrt_planner::Node* node);

    /**
     * @brief  是否碰撞，根据系统而定
     * @param  p  检查的位姿
     * @return true 
     * @return false 
     */
    bool IsCollised(const geometry_msgs::Pose p);

    /**
     * @brief  碰撞检测步长
     * @param  dist  检测总距离
     * @return double  检测步长
     */
    double CalculateCheckDeltaStep(const double dist);

    /**
     * @brief  是否搜索到终点了
     * @param  goal  终点
     * @param  point  当前节点
     * @return true 
     * @return false 
     */
    bool IsReach(const geometry_msgs::PoseStamped& goal, const geometry_msgs::Point point);

    /**
     * @brief  获取路径
     * @param  plan  路径
     * @return size_t
     */
    size_t GetPlan(std::vector<geometry_msgs::PoseStamped>& plan);

    /**
     * @brief  可视化节点
     * @param  point  节点
     * @param  index  id 
     */
    void PubNode(const geometry_msgs::Point& point, const size_t index);

    /**
     * @brief  可视化路径
     * @param  pub  发布器
     * @param  points  路径点
     */
    void PubPlan(const ros::Publisher pub, const std::vector<geometry_msgs::PoseStamped>& points);

    /**
    * @brief  发布箭头，从子节点指向父节点
    * @param  index  子节点下标
    * @param  child  子节点
    * @param  parent  父节点
    */
    void PubArrow(const size_t index,
      const geometry_msgs::Point& child, const geometry_msgs::Point& parent,
      const double r, const double g, const double b);

    /**
     * @brief  连接两点
     * @param  s  起点
     * @param  e  终点
     * @return true
     * @return false
     */
    bool Connect(const geometry_msgs::Point& s, const geometry_msgs::Point& e);

    /**
     * @brief  路径优化
     * @param  points  返回的信标
     * @return double 优化后的路径代价
     */
    double PathOptimization(std::vector<geometry_msgs::Point>& beacons);

    /**
     * @brief  获取路径代价
     * @return double 
     */
    double GetPlanCost();

    bool initialized_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_;
    std::shared_ptr<base_local_planner::WorldModel> world_model_;
    double check_dstep_;
    size_t terminal_;

    ros::Publisher nodes_pub_;
    ros::Publisher plan_pub_;
    ros::Publisher arrows_pub_;
    ros::Publisher opt_plan_pub_;
    std::map<size_t, rrt_planner::Node> nodes_;

    // -------------------- parameters --------------------
    int max_iterations_;
    double timeout_;
    double step_;
    double r_gamma_;
    double goal_err_;
    double r_beacon_;
    double biasing_ratio_;

};  // class RrtStarSmartPlanner

} // namespace rrt_planner

#endif // RRT_PLANNER_H_
