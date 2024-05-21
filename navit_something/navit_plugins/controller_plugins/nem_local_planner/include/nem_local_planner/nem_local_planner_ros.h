#ifndef NEM_LOCAL_PLANNER_ROS_H_
#define NEM_LOCAL_PLANNER_ROS_H_

#include "nem_local_planner/line_iterator.h"
#include "nem_local_planner/local_supporter.h"

#include "nem_global_planner/cedge.h"
#include "nem_global_planner/cnode.h"
#include "nem_global_planner/common.h"
#include "nem_global_planner/json_util.h"
#include "nem_global_planner/pose.h"
#include "nem_global_planner/xform.h"

#include <teb_local_planner/costmap_model.h>
#include <navit_costmap/costmap_2d_ros.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <jsoncpp/json/json.h>
#include <memory.h>
#include <navit_core/base_controller.h>
#include <nem_local_planner/pid_control.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf2_ros/buffer.h>

#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>
#include <actionlib/client/simple_action_client.h>
#include <sound_play/SoundRequestAction.h>


namespace nem_local_planner {

class NemLocalPlannerROS : public navit_core::Controller{
    enum {
        GO_INIT,
        GO_DIST,
        GO_TURN,
        GO_STOP,
        GO_ACTION,
        GO_ACTION_TURN,
        GO_ACTION_TURN_OPP,
        GO_DIST_FINE,
        GO_TURN_FINE,
        GO_ADJUST_Y,
        GO_WAIT,
        GO_FAIL
    };

   public:
    NemLocalPlannerROS();

    ~NemLocalPlannerROS();

    // navit_core interface

    geometry_msgs::Twist computeVelocityCommands(const geometry_msgs::PoseStamped& current_pose,
                                                 const geometry_msgs::Twist& current_vel)
    {
        geometry_msgs::Twist cmd_vel;
        if (!computeVelocityCommands(cmd_vel) ) {
            cmd_vel.linear.z = 0.01;
            ROS_WARN("failed to compute velocity");
        }
        return cmd_vel;
    }

    void initialize(const std::string& name,
                    const std::shared_ptr<tf2_ros::Buffer>& tf,
                    const std::shared_ptr<navit_costmap::Costmap2DROS>& costmap_ros) 
    {
        initialize(name, tf.get(), costmap_ros.get());
    }

    bool setPlan(const nav_msgs::Path& plan) override
    {
        return setPlan(plan.poses);
    }
        
    bool setSpeedLimit(const double& speed_limit) override {return false;}

    void initialize(std::string name, tf2_ros::Buffer *tf, navit_costmap::Costmap2DROS *costmap_ros);

    bool setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan);

    bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

    uint32_t computeVelocityCommands(const geometry_msgs::PoseStamped &pose,
                                     const geometry_msgs::TwistStamped &velocity, 
                                     geometry_msgs::TwistStamped &cmd_vel,
                                     std::string &message);

    bool isGoalReached() override;

    bool initNemData(const std::string &node_file_path, const std::string &edge_file_path) override;

    bool isTrajectoryFeasible(teb_local_planner::CostmapModel *costmap_model,
                              const std::vector<geometry_msgs::Point> &footprint_spec, double inscribed_radius,
                              double circumscribed_radius, int look_ahead_idx, double res);

    /*工具型函数*/
    void generatePath(geometry_msgs::PoseStamped &robot_pose, geometry_msgs::PoseStamped &goal,
                      std::vector<geometry_msgs::PoseStamped> &plan, int len, double res);

    void generatePath(geometry_msgs::PoseStamped &start, geometry_msgs::PoseStamped &goal,
                      std::vector<geometry_msgs::PoseStamped> &plan, double res);

    double getDistance(geometry_msgs::PoseStamped &p1, geometry_msgs::PoseStamped &p2);

    void PoseReceived_amcl_lp(const geometry_msgs::PoseWithCovarianceStampedConstPtr pose);

    void PoseReceived(const geometry_msgs::PoseStampedConstPtr pose);

    void ForwardObsStateCallback(const std_msgs::BoolConstPtr &state);

    void BackObsStateCallback(const std_msgs::BoolConstPtr &state);

    void LongCorridorStateCallback(const std_msgs::String::ConstPtr& state);

    void LaserAvoidSwitchStateCallback(const std_msgs::BoolConstPtr &state);

    void CalGoDirection(int ret, CEdge edge, int *direction);

    void GenerateGoDistanceNodes(vector<CNode>::iterator itSrc, vector<CNode>::iterator itDes);

    bool RobotTurn(float target, float threshold);

    bool RobotGo(geometry_msgs::PoseStamped robot_pose, vector<CNode>::iterator target_node, float target_sloap,
                 float min_cross_product = 0.0, bool bDelay = true);

    bool CalculateDistVel(const geometry_msgs::PoseStamped &pose, const geometry_msgs::PoseStamped &targetPoint,
                          float &vx, float &vy, float &vr, float fMinCrossProduct, float fTargetSlope,
                          xform::Vector2<double> goDirectionV);

    bool isRobotGoReached(const geometry_msgs::PoseStamped &pose, const geometry_msgs::PoseStamped &targetPoint,
                          float threshold);

    void PublishCmdVel(float x, float y, float r, bool delay = false);

    bool CalculateDistVel(const geometry_msgs::PoseStamped &pose, const geometry_msgs::PoseStamped &targetPoint,
                          float &vx, float &vy, float &vr, float fMinCrossProduct, float fTargetSlope,
                          xform::Vector2<double> goDirectionV, float _minVx, float _maxVx, float _minVy, float _maxVy,
                          float _minVr, float _maxVr, float vyReversed);

    /*状态机函数*/
    bool GoInit();

    bool GoTurn();

    bool GoDistance();

    bool GoAction();

    bool GoActionTurn();

    bool GoActionTurnOpp();

    bool GoDistanceFine();

    bool GoTurnFine();

    bool GoAdjustY();

    bool GoWait();

    bool GoStop();

    bool GoFail();

    /*补充global对应的初始化函数*/
    bool Init(const std::string &nodeFilePath, const std::string &edgeFilePath);

    bool Init(std::istream &inNodes, std::istream &inEdges);

    bool ReadFromJson(const std::istream &node_stream, Json::Value &val);

    void ClearPathMap();

    int FindEdgeByNodes(const CNode &nodeA, const CNode &nodeB, CEdge &edge);

    // @brief 根据机器人位置获取当前所处的边，并判断所在边的两个顶点是否存在目标点（说明获取的边是正确的）
    // @return 获取的边索引号
    bool GetEdgeForProperty(float x, float y, CEdge &edge, vector<CNode>::iterator targetNode);

    CNode GetNode(unsigned int id);

    CEdge GetEdge(unsigned int id);

    bool ExistNode(CNode::IdType id);

    int FindEdgeByPoints(Point &pointA, Point &pointB, CEdge &edge);

    int GetEdgeLane(CNode::IdType idA, CNode::IdType idB);

    bool GetAvoidance(CNode::IdType idA, CNode::IdType idB);

    bool GetAntiDrop(CNode::IdType idA, CNode::IdType idB);

    float GetMaxVx(CNode::IdType idA, CNode::IdType idB);

    /*可视化发布型函数*/
    std::vector<geometry_msgs::Point> makeFootprintFromRadius(double radius);

    void pubFootprint(ros::Publisher pub, geometry_msgs::PoseStamped robot_pose, double radius);

    void transformFootprint(double x, double y, double theta, const std::vector<geometry_msgs::Point> &footprint_spec,
                            geometry_msgs::PolygonStamped &oriented_footprint);
                    
    void SoundPlay(const int command, const int vol, const std::string file);

    xform::Vector2<double> transformGlobalDeltaPoseToLocal(const double yaw, const double deltaPoseX, const double deltaPoseY);

   private:
    // M100底盘电机最小速度响应补偿量
    const float real_vr_compensate_ = 0.018;
    bool   stricter_on_line_judgement_;
    double points_on_one_line_;
    double min_confidence_;
    double min_vx_, max_vx_;
    double min_vy_, max_vy_;
    double min_vr_, max_vr_;
    double angle_threshold_;
    double fine_angle_threshold_;
    double fine_adjust_dist_;
    double threshold_compensation_;
    double max_speed_;
    double go_dist_gap_;
    int navi_step_;
    vector<CNode>::iterator original_node_;
    vector<CNode>::iterator target_node_;
    vector<Point> go_distance_nodes_;
    vector<Point>::iterator go_distance_node_it_;
    vector<CNode> aim_path_nodes_;
    xform::Vector2<double> y_adjust_target_;

    //    Pose robot_pose_;
    geometry_msgs::Twist cmd_vel_helper_;
    bool sim_mode_;
    bool goal_reached_;
    bool goal_failed_;
    float target_slope_;
    float vy_reversed_;
    bool initialize_;
    bool forward_obs_;
    bool back_obs_;
    bool long_corridor_;
    bool adjust_y_direction_;
    bool obsavoid_sound_play_;
    bool sound_play_flag_;
    int oscillation_num_;

    xform::Vector2<double> m_vecN_;
    string edge_file_path_local_;
    string node_file_path_local_;

    boost::recursive_mutex poseMutex_lp;

    actionlib::SimpleActionClient<sound_play::SoundRequestAction> sound_play_client_;

    ros::Subscriber robot_pose_sub_;
    ros::Subscriber robot_sim_pose_sub_;
    ros::Subscriber forward_obs_state_sub_;
    ros::Subscriber back_obs_state_sub_;
    ros::Subscriber long_corridor_sub_;
    ros::Subscriber laser_avoid_switch_sub_;
    ros::Publisher feasible_footprint_pub_;
    ros::Publisher local_plan_pub_;
    ros::Publisher trajectory_offset_pub_;
    ros::Publisher navi_error_pub_;
    ros::Publisher laser_obs_state_pub_;

    /*补充global对应的点边信息*/
    vector<CNode> nodes_local_;
    vector<CEdge> edges_local_;
    map<CNode::IdType, CNode::IdType> node_id_to_index_;
    map<CNode::IdType, CNode::IdType> edge_id_to_index_;
    CNode::IdType node_max_id_;
    CNode::IdType edge_max_id_;
    std::recursive_mutex m_mtx_local;
    json_util::JsonUtil json_util_;

    std::vector<geometry_msgs::Point> footprint_spec_;
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    double robot_inscribed_radius_;     //!< The radius of the inscribed circle of the robot (collision possible)
    double robot_circumscribed_radius;  //!< The radius of the circumscribed circle of the robot
    navit_costmap::Costmap2DROS *costmap_ros_;
    boost::shared_ptr<teb_local_planner::CostmapModel> costmap_model_;
    navit_costmap::Costmap2D *costmap_;

    pid_control::PidControl pid_control_;
    bool use_pid_;
    double Kp_;
    double Ki_;
    double Kd_;
    double max_angle_z_;
    double last_cmd_;
    double head_dist_;
    double acceleration_;
    double angular_acceleration_;
    double wheelbase_;
    int jump_node_num_;

    bool avoidance_;
    bool avoidance_edge_;
    bool anti_drop_edge_;
    int avoidance_num_;
    double avoidance_res_;
    bool fake_reached_;
    // 机器人行驶方向的全局变量
    int fix_direction_;

    ros::Time go_turn_fine_start_;
    ros::Time go_turn_fine_end_;
    ros::Time go_turn_start_;
    ros::Time go_turn_end_;
    ros::Time go_dist_start_;
    ros::Time go_dist_end_;
};
};  // namespace nem_local_planner

#endif  // NEM_LOCAL_PLANNER_ROS_H_
