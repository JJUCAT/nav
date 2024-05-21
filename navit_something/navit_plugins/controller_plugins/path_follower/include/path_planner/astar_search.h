/*
* @Author: czk
* @Date:   2022-11-11 16:27:19
* @Last Modified by:   chenzongkui
* @Last Modified time: 2023-03-13 13:51:50
*/
#ifndef ASTAR_NAVI_NODE_H
#define ASTAR_NAVI_NODE_H

#include <path_planner/astar_util.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <navit_costmap/costmap_2d_ros.h>

#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <geometry2.h>
#include "path.pb.h"
#include "config_graph_search.pb.h"
// #include "proto/proto_utils.h"
#include <iostream>
#include <vector>
#include <queue>
#include <string>
#include <chrono>

namespace planner{
class AstarSearch
{
 public:
  AstarSearch();
  ~AstarSearch();

  //-- FOR DEBUG -------------------------
  void publishPoseArray(const ros::Publisher &pub, const std::string &frame);
  geometry_msgs::PoseArray debug_pose_array_;
  //--------------------------------------

  bool makePlan(const geometry_msgs::Pose &start_pose, const geometry_msgs::Pose &goal_pose, const nav_msgs::OccupancyGrid &map);
  bool makePlan(const geometry_msgs::Pose &start_pose, const geometry_msgs::Pose &goal_pose, const navit_costmap::Costmap2DROS* costmap_ros);
  void reset();
  void setTolerance(const int &type);
  void broadcastPathTF();
  bool getPath(const double &angle, proto::Polyline *path);
  nav_msgs::Path getRosPath() {
    return path_;
  }
  bool init(proto::ConfigGraphSearch &graph_search_parameters);
  bool init(std::string name);
  void breakSearch(const bool flag){
    break_search_flag_ = flag;
  }
  bool getPath(proto::Polyline* local_path);
 private:
  bool search();
  void resizeNode(int width, int height, int angle_size);
  void createStateUpdateTable(int angle_size);
  void createStateUpdateTableLocal(int angle_size); //
  void poseToIndex(const geometry_msgs::Pose &pose, int *index_x, int *index_y, int *index_theta);
  bool isOutOfRange(int index_x, int index_y);
  void setPath(const SimpleNode &goal);
  void setMap(const nav_msgs::OccupancyGrid &map);
  void setMap(const navit_costmap::Costmap2DROS* costmap_ros);
  bool setStartNode();
  bool setGoalNode();
  bool isGoal(double x, double y, double theta);
  bool detectCollision(const SimpleNode &sn, bool start =false);
  bool calcWaveFrontHeuristic(const SimpleNode &sn);
  bool detectCollisionWaveFront(const WaveFrontNode &sn);
  // ROS param
  std::string path_frame_;        // publishing path frame
  int angle_size_;                // descritized angle size
  double minimum_turning_radius_; // varying by vehicles
  int obstacle_threshold_;        // more than this value is regarded as obstacles
  double goal_radius_,goal_radius_init_;            // meter
  double goal_angle_,goal_angle_init_;             // degree
  bool use_back_,use_back_inital_;                 // use backward driving
  double robot_length_;
  double robot_width_;
  double base2back_;
  double curve_weight_;
  double reverse_weight_;
  bool use_wavefront_heuristic_;
  bool use_2dnav_goal_;

  bool node_initialized_;
  std::vector<std::vector<NodeUpdate>> state_update_table_;
  nav_msgs::MapMetaData map_info_;
  std::vector<std::vector<std::vector<AstarNode>>> nodes_;
  std::priority_queue<SimpleNode, std::vector<SimpleNode>, std::greater<SimpleNode>> openlist_;
  std::vector<SimpleNode> goallist_;

  // Pose in global(/map) frame
  geometry_msgs::PoseStamped start_pose_;
  geometry_msgs::PoseStamped goal_pose_;

  // Pose in OccupancyGrid frame
  geometry_msgs::PoseStamped start_pose_local_;
  geometry_msgs::PoseStamped goal_pose_local_;

  // Transform which converts OccupancyGrid frame to global frame
  tf::Transform map2ogm_;

  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;

  // Searched path
  nav_msgs::Path path_;

  bool break_search_flag_ = false;

  struct AvoidanceCfg {
    int angle_size;
    double minimum_turning_radius;
    double obstacle_threshold;
    double goal_radius;
    double goal_angle;
    bool use_back;
    double robot_length;
    double robot_width;
    double base2back;
    double curve_weight;
    double reverse_weight;
    bool use_wavefront_heuristic;
  } avoidance_cfg_;

};
} //namespace planner
#endif // ASTAR_NAVI_NODE_H
