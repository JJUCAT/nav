/*
* @Author: czk
* @Date:   2022-10-03 09:57:30
* @Last Modified by:   chenzongkui
* @Last Modified time: 2023-03-28 09:25:30
*/
#pragma once

#include <ros/ros.h>
// transforms
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <navit_core/base_controller.h>
#include <path_smoother/path_spline.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Quaternion.h>
#include "coordinate_convert.h"
#include <pid_solver/pid_solver.h>
#include <vector>
#include <thread>
#include <visualization_msgs/Marker.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <navit_costmap/costmap_2d.h>
#include <path_planner/astar_search.h>
#include <std_srvs/Empty.h>
#include <path_follower/trajectory_view.h>
#include <path_follower/path_matcher.h>
#include <nav_msgs/Odometry.h>
#include <navit_common/log.h>

#include "path_follow_cfg.pb.h"
#include "config_control.pb.h"
#include "control_command.pb.h"
#include "path.pb.h"
#include "robot_state.pb.h"
#include "macros.h"
#include "math.h"
#include "geometry2.h"
#include <smac_planner/smac_planner_hybrid.hpp>
namespace control {
typedef boost::geometry::model::d2::point_xy<double> point_type;
typedef boost::geometry::model::polygon<point_type> polygon_type;
// tracking error from car state to ref path point
class ControllerPathFollow : public navit_core::Controller {
 public:
  ControllerPathFollow(const proto::ConfigControlLateral& config, int hz) : config_(config), hz_(hz) {
    controller_yaw_pid_ptr_.reset(new ControllerPid2Dof(1.0, 0.2, 0.3, 1.0f / hz_, 0.0, 1.0, 0.4, 1.0));
    path_matcher_ptr_.reset(new PathMatcher(8.0));
  }
  ControllerPathFollow(){
    coordinate_convert_ptr_.reset(new planner::path::CoordinateConvert());
    path_matcher_ptr_.reset(new PathMatcher(4.0));
    path_spline_ptr_.reset(new navit_planner::PathGslSpline());
    controller_yaw_pid_ptr_.reset(new ControllerPid2Dof(0.8, 0.0, 0.0, 1.0f / 100.0f, 0.0, 1.0, 0.4, 1.0));
  }

  ControllerPathFollow(const proto::ConfigControlLateral& config, const proto::ConfigControlRotation& config_rotation, int hz)
      : config_(config), config_rotation_control_(config_rotation), hz_(hz) {

    controller_yaw_pid_ptr_.reset(new ControllerPid2Dof(
        config_rotation_control_.rotation_pid_parameter().kp(), config_rotation_control_.rotation_pid_parameter().ki(),
        config_rotation_control_.rotation_pid_parameter().kd(), 1.0f / hz_, config_rotation_control_.rotation_pid_parameter().kt(),
        config_rotation_control_.rotation_pid_parameter().wp(), config_rotation_control_.rotation_pid_parameter().tau(),
        config_rotation_control_.rotation_pid_parameter().saturation()));

    path_matcher_ptr_.reset(new PathMatcher(18.0));
  }

  ~ControllerPathFollow() {
    if (path_matcher_ptr_ != NULL) {
        path_matcher_ptr_.reset();
    }
    if (controller_yaw_pid_ptr_ != NULL) {
        controller_yaw_pid_ptr_.reset();
    }
    if (coordinate_convert_ptr_ != NULL) {
        coordinate_convert_ptr_.reset();
    }
    replanHandle_done_ = false;
    replan_thread_ptr_->join();
    tf_handle_thread_ptr_->join();

    if (path_spline_ptr_ != NULL) {
        path_spline_ptr_.reset();
    }
  }
  /*
  * @ brief: initialize the controller
  * @ param: name: the name of the controller
  * @ param: tf: the tf2_ros::Buffer
  * @ param: costmap_ros: the navit_costmap::Costmap2DROS
  */
  void initialize(const std::string& name,
                  const std::shared_ptr<tf2_ros::Buffer>& tf,
                  const std::shared_ptr<navit_costmap::Costmap2DROS>& costmap_ros) {
    costmap_ros_ = costmap_ros;
    initialize(name, tf.get(), costmap_ros.get());
  }


  void initialize(const std::string name, tf2_ros::Buffer* tf, navit_costmap::Costmap2DROS* costmap_ros);
  /*
  * @ brief: compute the velocity command based on the current robot state and reference path
  * @ param: current_pose: the current robot pose
  * @ param: current_vel: the current robot velocity
  * @ return: the velocity command
  */
  geometry_msgs::Twist computeVelocityCommands(const geometry_msgs::PoseStamped& current_pose,
                                               const geometry_msgs::Twist& current_vel);
  /*
  * @ brief: set the reference path
  * @ param: plan: the reference path
  * @ return: true if successful, false otherwise
  * @ note: the reference path should be in the same frame as the robot
  */
  bool setPlan(const nav_msgs::Path& plan);

  /*
  * @ brief: if robot reaches the goal
  * @ param: empty:
  * @ return: true if successful, false otherwise
  */
  bool isGoalReached() { return goal_reached_; }

  /*
   * @ brief: set the speed limit (Deprecated)
   * @ param: speed_limit: the speed limit in m/s
   * @ return: true if successful, false otherwise
  */
  bool setSpeedLimit(const double& speed_limit){ return true; }


  /*
   * @ brief: transform the proto::Pose to geometry_msgs::Pose
   * @ param: proto_pose: the proto::Pose to be transformed
   * @ param: ros_pose: the transformed geometry_msgs::Pose
   * @ return: void
  */
  void transProto2GeometryMsgsPose (const proto::Pose proto_pose, geometry_msgs::Pose& ros_pose);

  enum CollsionCheckType{
    EMERGENCY_STOP = 1,
    CUSHION = 2,
    REPLAN = 3
  };
  void resetYawPid() {
    steering_angle_setpoint_ = 0;
    controller_yaw_pid_ptr_->reset();
  }
  enum ControlMode {
    INVALID = 0,
    FOLLOW_PATH = 1,
    SLOW_DOWN = 2,
    ROTATE_FOR_PRELINE = 3,
    ROTATE_FOR_LINE = 4,
    ROTATE_FINIAL = 5,
    REACH_STATION = 6,
    PRUNE_PATH = 7
  } control_mode_ = ROTATE_FOR_PRELINE;

 private:
    void tfHandle();
    int isCollisionImminent(const geometry_msgs::PoseStamped& robot_pose,
                            const double & linear_vel, const double & angular_vel,
                            const proto::AnchorBox anchor_boxes);
    int isCollisionImminent ();
    bool setControlAngle(const double target_yaw, const proto::RobotState& robot_state, proto::ControlCommand& control_command);
    bool setControlAngle(const double target_yaw, const double current_yaw, proto::ControlCommand& control_command);
    bool setControl(const proto::Path& path, const proto::RobotState& robot_state, const double& target_distance,
                    proto::ControlCommand* control_command, bool reverse = false);
    geometry2::Vec3d transOrientation2euler(const geometry_msgs::Quaternion orientation);
    void setZeroControlCommand(geometry_msgs::Twist& cmd_vel);
    void setZeroControlComannd(proto::ControlCommand* control_command);
    float previewDistSchedule(const proto::PreviewPoint& preview_point, float cmd);
    bool generateAreaPolygon(const proto::Vector3f& robot_state,
                             geometry_msgs::Polygon& robot_polygon);
    void findCornerPoint(const proto::Vector3f& robot_state, float x, float y, geometry_msgs::Point32 *p3f);
    bool inCollision(const geometry_msgs::PolygonStamped& polygon);
    void costmapCallback(const nav_msgs::OccupancyGridConstPtr map);
    void replanHandle();
    bool setReplan(const int conncet_begin, const int connect_end,
                   const proto::Polyline& local_path);
    bool clearCostmapService();
    nav_msgs::Path resamplePath(const nav_msgs::Path& input_path, double desired_spacing);

    const proto::ConfigControlLateral config_;
    proto::ConfigControlRotation config_rotation_control_;
    int hz_ = 10;
    const float wheel_base_ = 0.95f;
    const float wheel_distance_ = 0.45f;
    const double angle_slew_rate_ = 1.0f / hz_;
    const double speed_slew_rate_ = 0.5f / hz_;
    const float lateral_acc_limit_ = 0.4f;
    const float KBlockFlag_ = 0.01f;
    double steering_angle_setpoint_ = 0.0f, speed_setpoint_ = 0.0f;
    double last_speed_ = 0;
    struct DiffCfg {
        double max_vel_x = 0;
        double max_vel_x_backwards = 0;
        double max_vel_theta = 0;
        double acc_lim_x = 0;
        double dec_lim_x = 0;
        double acc_lim_theta = 0;
        double min_vel_x = 0;
    };
    struct SimpleCarCfg {
        bool use_angular_vel = false;
        double wheelbase = 0;
        double max_vel_x = 0;
        double max_vel_x_backwards = 0;
        double max_steering_angle = 0;
        double max_steering_rate = 0;
        double acc_lim_x = 0;
        double dec_lim_x = 0;
        double acc_lim_theta = 0;
    };
    struct RobotBoxCfg {
        double front_left_x;
        double front_left_y;
        double front_right_x;
        double front_right_y;
        double rear_right_x;
        double rear_right_y;
        double rear_left_x;
        double rear_left_y;
    };
    struct AvoidanceCfg {
         int collision_check_box_sizes = 0;
         int box_interval = 0;
         bool use_clear_costmap;
         double clear_costmap_wait_time;
         bool use_recovery;
         bool use_eliminate_shocks;
         bool move_to_pose;
         float dt;
         float forward_distance;
         float min_step_distance;
    };
    struct PathFollowCfg {
        std::string global_frame = "";
        std::string robot_frame = "";
        bool use_avoidance = false;
        DiffCfg diff_cfg;
        SimpleCarCfg simple_car_cfg;
        RobotBoxCfg robot_box;
        AvoidanceCfg avoidance_cfg;
        double dis_tolerance = 0.1;
        double theta_tolerance = 0.2;
        int look_path_ahead = 200;
        float min_look_ahead_dis = 10;
        float look_ahea_dis_ratio = 50.0f;
        double extend_path_dis = 0.0;
        double padding_footprint = 0.0;
    } path_follow_cfg_;

    double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2) {
        return sqrt(pow(p2.pose.position.x - p1.pose.position.x, 2) +
                    pow(p2.pose.position.y - p1.pose.position.y, 2) +
                    pow(p2.pose.position.z - p1.pose.position.z, 2));
    }

    geometry_msgs::PoseStamped interpolate(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2, double ratio) {
        geometry_msgs::PoseStamped interpolated_pose;
        // Linear interpolation
        interpolated_pose.pose.position.x = (1 - ratio) * p1.pose.position.x + ratio * p2.pose.position.x;
        interpolated_pose.pose.position.y = (1 - ratio) * p1.pose.position.y + ratio * p2.pose.position.y;
        interpolated_pose.pose.position.z = (1 - ratio) * p1.pose.position.z + ratio * p2.pose.position.z;

        // Here we are assuming that the orientation does not matter. If it does, we should also interpolate the orientation.
        interpolated_pose.pose.orientation = p2.pose.orientation;

        return interpolated_pose;
    }

    std::vector<float> speed_buffer_;
    std::unique_ptr<PathMatcher> path_matcher_ptr_;
    std::unique_ptr<ControllerPid2Dof> controller_yaw_pid_ptr_;
    std::unique_ptr<planner::path::CoordinateConvert> coordinate_convert_ptr_;
    std::unique_ptr<navit_planner::PathGslSpline> path_spline_ptr_;
    boost::thread* tf_handle_thread_ptr_;
    boost::thread* replan_thread_ptr_;
    std::shared_ptr<navit_costmap::Costmap2DROS> costmap_ros_;
    navit_costmap::Costmap2DROS* costmap_ros_ptr_;
    tf2_ros::Buffer* tf_;
    proto::Path slow_down_path_, smooth_global_path_, init_global_path_;
    proto::RobotState robot_state_, robot_state_in_map_;
    std::vector<std::pair<proto::Vector3f, proto::Vector3f>> poses_list_;
    std::vector<proto::PathPoint> path_point_vec_;

    std::list<proto::PathPoint> path_point_list_;
    navit_costmap::Costmap2D* costmap_;
    std::shared_ptr<navit_costmap::Costmap2D> costmap_ptr_;
    float distance_ = 100, linear_vel_ = 0.0f;
    float anchor_point_yaw_ = 0.0f, target_yaw_ = 0.0f;
    geometry_msgs::PoseStamped final_pose_;
    geometry_msgs::PolygonStamped emergency_polygon_, cushion_polygon_, replan_polygon_;
    //AstarSearch
    planner::AstarSearch astar_search_;

    geometry_msgs::Pose replan_start_, replan_goal_, init_replan_goal_;
    int replan_goal_index_ = 0;
    bool initialized_ = false, goal_reached_ = false, collisioned_ = false, replan_flag_ = false, replan_success_ = false;
    uint32_t replan_count_ = 12;
    int replan_current_index_ = 0;
    geometry2::Vec2f taget_pose_, current_pose_;

    geometry_msgs::PoseStamped current_pose_stamped_;

    ros::Subscriber cost_map_sub_, odom_sub_;
    ros::Publisher debug_pub_, collision_check_poses_pub_, ompl_path_pub_, split_jount_path_pub_;
    ros::ServiceClient clear_costmap_srv_;
    TrajectoryViewer trajectory_viewer_;

    bool should_rotate_ = false;
    bool replanHandle_done_ = true;
    bool first_hit_ = true;
    bool set_replan_ = false;
    bool tf_received_ = false;
    bool replan_attemped_failed_ = false;
    bool failed_flag_ = false;
    bool set_control_first_hit_ = true;
    bool robot_blocked_ = false;
    float round_corner_v_ = 0.0f;
    std::string plan_frame_ = "map";
    tf::StampedTransform global_to_target_transform_;
    std::mutex global_variable_mutex_;
    geometry_msgs::Twist cmd_vel_;
    tf2_ros::Buffer* tf_buffer_;
    ros::Publisher footprint_marker_array_pub_;
    nav2_smac_planner::SmacPlannerHybrid smac_planner_;

    //DISALLOW_COPY_AND_ASSIGN(ControllerPathFollow);
};
}  // namespace control
