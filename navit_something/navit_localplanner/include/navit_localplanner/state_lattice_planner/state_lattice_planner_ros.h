#ifndef __STATE_LATTICE_PLANNER_ROS_H
#define __STATE_LATTICE_PLANNER_ROS_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "state_lattice_planner/state_lattice_planner.h"

class StateLatticePlannerROS
{
public:
    StateLatticePlannerROS(void);

    void process(void);
    void local_goal_callback(const geometry_msgs::PoseStampedConstPtr&);
    void local_map_callback(const nav_msgs::OccupancyGridConstPtr&);
    void odom_callback(const nav_msgs::OdometryConstPtr&);
    void target_velocity_callback(const geometry_msgs::TwistConstPtr&);
    template<typename TYPE>
    void get_obstacle_map(const nav_msgs::OccupancyGrid&, state_lattice_planner::ObstacleMap<TYPE>&);

protected:
    void visualize_trajectories(
      const std::vector<MotionModelDiffDrive::Trajectory>&,
      const double, const double, const double, const int, const ros::Publisher&);
    void visualize_trajectory(const MotionModelDiffDrive::Trajectory&,
      const double, const double, const double, const ros::Publisher&);
    geometry_msgs::Twist diff_cmd2ackermann_cmd(const geometry_msgs::Twist& cmd_vel);
    void init_xyyaw_table();
    void viz_goal_sampling(const std::vector<Eigen::Vector3d>& states);
    void new_goal_sampling(const Eigen::Vector3d goal, const float ratio, std::vector<Eigen::Vector3d>& states);
    void parallel_goal_sampling(const Eigen::Vector3d goal, const float ratio, std::vector<Eigen::Vector3d>& states);
    void PubBestTrajectory(const std::vector<Eigen::Vector3d>& trajectory, const std::string target_frame_id);
    void update_pose_in_map(const std::string map_frame_id);

    double HZ;
    std::string ROBOT_FRAME;
    int N_P;
    int N_H;
    int N_S;
    double MAX_ALPHA;
    double MAX_PSI;
    double MAX_ACCELERATION;
    double TARGET_VELOCITY;
    std::string LOOKUP_TABLE_FILE_NAME;
    int MAX_ITERATION;
    double OPTIMIZATION_TOLERANCE;
    double MAX_YAWRATE;
    double MAX_D_YAWRATE;
    double MAX_WHEEL_ANGULAR_VELOCITY;
    double WHEEL_RADIUS;
    double TREAD;
    double IGNORABLE_OBSTACLE_RANGE;
    bool VERBOSE;
    int CONTROL_DELAY;
    double TURN_DIRECTION_THRESHOLD;
    bool ENABLE_SHARP_TRAJECTORY;
    bool ENABLE_CONTROL_SPACE_SAMPLING;
    bool DRIVE;
    bool ACKERMANN;
    double WHEEL_BASE;
    bool NEW_GOAL_SAMPLING;
    double NGS_R;
    int NGS_NA;
    int NGS_NR;
    double NGS_GY;
    int NGS_NY;
    double HEAD;
    int COLLISION_COST; // 碰撞检测值，[-1,100]
    // 轨迹评价
    double DIST_ERR;
    double YAW_ERR;
    double ANGULAR_ERR;
    double DIST_SCALE;
    double YAW_SCALE;
    double ANGULAR_SCALE;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;

    ros::Publisher best_trajectory_pub;
    ros::Publisher xyyaw_table_pub;
    ros::Publisher goal_pub;
    ros::Publisher goal_sampling_pub;
    ros::Publisher velocity_pub;
    ros::Publisher candidate_trajectories_pub;
    ros::Publisher candidate_trajectories_no_collision_pub;
    ros::Publisher selected_trajectory_pub;
    ros::Subscriber local_map_sub;
    ros::Subscriber local_goal_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber target_velocity_sub;
    tf::TransformListener listener;
    geometry_msgs::PoseStamped local_goal;
    nav_msgs::OccupancyGrid local_map;
    geometry_msgs::Twist current_velocity;
    bool local_goal_subscribed;
    bool local_map_updated;
    bool odom_updated;

    StateLatticePlanner planner;
    sensor_msgs::PointCloud pc_table_;
    std::vector<Eigen::Vector3d> xyyaw_table_;
    geometry_msgs::PoseStamped robot_pose_in_map_;
    Eigen::Vector3d robot_in_map_;
    std::shared_ptr<StateLatticePlanner::Critic> critic_;
};

#endif //__STATE_LATTICE_PLANNER_ROS_H
