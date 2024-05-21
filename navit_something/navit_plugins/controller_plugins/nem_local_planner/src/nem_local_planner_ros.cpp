#include <nav_msgs/Path.h>
#include <nem_local_planner/nem_local_planner_ros.h>
#include <pluginlib/class_list_macros.h>
#include <stdio.h>
#include <time.h>

#include <algorithm>
#include <cstdio>
#include <map>

PLUGINLIB_EXPORT_CLASS(nem_local_planner::NemLocalPlannerROS, navit_core::Controller)
namespace {
void VersionInfo()
{
    ROS_INFO("\n");
    ROS_INFO("---M100导航控制运行信息---");
	ROS_INFO("Navigation node start.");
	ROS_INFO("Version:1.0.0");
	ROS_INFO("Coding date:2022-10-18");
    ROS_INFO("修改内容: 进入长走廊最大速度设置为0.4m/s");
	ROS_INFO("Build date:%s  %s\n", __DATE__, __TIME__);
}
}

namespace nem_local_planner {

const static double GO_NEAR_DIST          = 0.1;
const static double GO_DISTANCE_X         = 0.647;
const static double GO_DISTANCE_Y         = 0.6;
const static double GO_DISTANCE_MIN       = 0.05;
const static double GO_DISTANCE_MIN_X     = 0.24;
const static double GO_DISTANCE_MIN_Y     = 0.042;
const static double GO_DISTANCE_MIN_R     = 0.02;
const static double GO_TURN_X             = 0.9;
const static double GO_TURN_Y             = 1.5;
const static double GO_TURN_R             = 0.2;
const static double LONG_CORRIDOR_MAX_VX  = 0.4;

NemLocalPlannerROS::NemLocalPlannerROS() : sound_play_client_("sound_play", true) {
    initialize_          = false;
    max_speed_           = 0.0;
    goal_reached_        = false;
    goal_failed_         = false;
    target_slope_        = 0.0;
    fake_reached_        = false;
    forward_obs_         = false;
    back_obs_            = false;
    adjust_y_direction_  = false;
    obsavoid_sound_play_ = false;
    sound_play_flag_     = false;
    long_corridor_       = false;
    fix_direction_       = 0;
    oscillation_num_     = 0;
    VersionInfo();
}

NemLocalPlannerROS::~NemLocalPlannerROS() {}

void NemLocalPlannerROS::initialize(std::string name, tf2_ros::Buffer *tf, navit_costmap::Costmap2DROS *costmap_ros) {
    ros::NodeHandle private_nh("~/" + name);
    ros::NodeHandle nh;

    costmap_ros_         = NULL;
    initialize_          = false;
    max_speed_           = 0.0;
    goal_reached_        = false;
    goal_failed_         = false;
    obsavoid_sound_play_ = false;
    sound_play_flag_     = false;
    target_slope_        = 0.0;
    last_cmd_            = 0.0;
    jump_node_num_       = 0;
    sim_mode_            = false;
    std::string pose_topic;
    pose_topic = "pose";
    navi_step_ = GO_INIT;

    private_nh.param("minConfidence", min_confidence_, 0.1);  //低于阈值就停止运算，发v=0
    private_nh.param("minVx", min_vx_, 0.0);
    private_nh.param("maxVx", max_vx_, 0.8);
    private_nh.param("minVy", min_vy_, 0.02);
    private_nh.param("maxVy", max_vy_, 0.1);
    private_nh.param("minVr", min_vr_, 0.035);
    private_nh.param("maxVr", max_vr_, 0.5);
    private_nh.param("stricterOnLineJudgement", stricter_on_line_judgement_, false);
    private_nh.param("pointOnOneLine", points_on_one_line_, 0.03);
    private_nh.param("usePid", use_pid_, true);
    private_nh.param("Kp", Kp_, 6.0);
    private_nh.param("Ki", Ki_, 0.0);
    private_nh.param("Kd", Kd_, 16.0);
    private_nh.param("max_angle_z", max_angle_z_, 16.0);
    private_nh.param("angleThreshold", angle_threshold_, 0.02);
    private_nh.param("fineAngleThreshold", fine_angle_threshold_, 0.02);
    private_nh.param("fineAdjustDist", fine_adjust_dist_, 0.02);
    private_nh.param("goDistGap", go_dist_gap_, 5.0);
    private_nh.param("avoidance", avoidance_, true);
    private_nh.param("head_dist", head_dist_, 0.3);
    private_nh.param("acceleration", acceleration_, 0.3);
    private_nh.param("angular_acceleration", angular_acceleration_, 0.3);
    private_nh.param("wheelbase", wheelbase_, 0.61);
    private_nh.param("pose_topic", pose_topic, pose_topic);
    private_nh.param("threshold_compensation", threshold_compensation_, 0.05);
    private_nh.param("avoidance_num", avoidance_num_, 8);
    private_nh.param("avoidance_res", avoidance_res_, 0.05);
    nh.param("sim_mode", sim_mode_, false);
    ROS_INFO("[nem_local_planner]: sim_mode=%d, minConfidence=%f, minVx=%f, maxVx=%f, minVy=%f, maxVy=%f, minVr=%f, "
             "maxVr=%f, usePid=%d, Kp=%f, Ki=%f, Kd=%f, angleThreshold=%f, fineAngleThreshold=%f, fineAdjustDist=%f, "
             "goDistGap=%f, avoidance=%d, acceleration=%f, angular_acceleration=%f, threshold_compensation=%f, "
             "avoidance_num=%d, avoidance_res=%f, stricter_on_line_judgement_=%d, points_on_one_line=%f",
             sim_mode_, min_confidence_, min_vx_, max_vx_, min_vy_, max_vy_, min_vr_, max_vr_, use_pid_, Kp_, Ki_, Kd_,
             angle_threshold_, fine_angle_threshold_, fine_adjust_dist_, go_dist_gap_, avoidance_, acceleration_,
             angular_acceleration_, threshold_compensation_, avoidance_num_, avoidance_res_, stricter_on_line_judgement_, points_on_one_line_);

    // if (sim_mode)
    //     robot_sim_pose_sub_ = nh.subscribe("amcl_pose", 1, &NemLocalPlannerROS::PoseReceived_amcl_lp, this);
    // else
    // robot_pose_sub_ = nh.subscribe(pose_topic, 1, &NemLocalPlannerROS::PoseReceived, this);
    forward_obs_state_sub_   = nh.subscribe("forward_obs", 1, &NemLocalPlannerROS::ForwardObsStateCallback, this);
    back_obs_state_sub_      = nh.subscribe("back_obs",    1, &NemLocalPlannerROS::BackObsStateCallback, this);
    long_corridor_sub_       = nh.subscribe("long_corridor_detection", 1, &NemLocalPlannerROS::LongCorridorStateCallback, this);
    laser_avoid_switch_sub_  = nh.subscribe<std_msgs::Bool>("laser_avoid", 1, &NemLocalPlannerROS::LaserAvoidSwitchStateCallback, this);
    feasible_footprint_pub_  = private_nh.advertise<geometry_msgs::PolygonStamped>("not_feasible_footprint", 1);
    local_plan_pub_          = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
    trajectory_offset_pub_   = nh.advertise<std_msgs::Float64>("error_value", 1);
    navi_error_pub_          = nh.advertise<std_msgs::Int32>("navi_error", 1, true);
    laser_obs_state_pub_     = nh.advertise<std_msgs::Bool>("front_depth_camera/laser_obsavoid_state", 1, true);

    costmap_ros_    = costmap_ros;
    footprint_spec_ = costmap_ros_->getRobotFootprint();
    navit_costmap::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);
    costmap_       = costmap_ros_->getCostmap();  // locking should be done in MoveBase.
    costmap_model_ = boost::make_shared<teb_local_planner::CostmapModel>(*costmap_);
}

bool NemLocalPlannerROS::initNemData(const std::string &node_file_path, const std::string &edge_file_path) {
    if (!Init(node_file_path, edge_file_path)) {
        initialize_ = false;
        ROS_ERROR("[nem_local_planner]: Initialize nem_local_planner failed");
        std_msgs::Int32 error_code;
        error_code.data = 211;
        navi_error_pub_.publish(error_code);
        return false;
    }
    initialize_ = true;
    return true;
}

bool NemLocalPlannerROS::Init(const std::string &nodeFilePath, const std::string &edgeFilePath) {
    std::lock_guard<std::recursive_mutex> lck(m_mtx_local);
    ifstream finNode(nodeFilePath.c_str(), std::ios::in);
    ifstream finEdge(edgeFilePath.c_str(), std::ios::in);
    bool bRet = Init(finNode, finEdge);
    finNode.close();
    finEdge.close();
    return bRet;
}

bool NemLocalPlannerROS::Init(std::istream &inNodes, std::istream &inEdges) {
    ClearPathMap();
    if (!json_util_.LoadNodeStream(inNodes, nodes_local_, node_max_id_, node_id_to_index_)) {
        ROS_ERROR("[nem_local_planner]: Load node failed");
        return false;
    }
    if (!json_util_.LoadEdgeStream(inEdges, nodes_local_, edges_local_, edge_max_id_, node_id_to_index_,
                                   edge_id_to_index_)) {
        ROS_ERROR("[nem_local_planner]: Load edge failed");
        return false;
    }
    ROS_INFO("[nem_local_planner]: Print node and edge information");
    for (int i = 0; i < nodes_local_.size(); i++) nodes_local_[i].PrintNodeInfo();
    for (int i = 0; i < edges_local_.size(); i++) edges_local_[i].PrintEdgeInfo();
    ROS_INFO("[nem_local_planner]: nodes numbers = %lld, edges number = %lld \n", nodes_local_.size(),
             edges_local_.size());
    return true;
}

void NemLocalPlannerROS::ClearPathMap() {
    node_max_id_ = edge_max_id_ = 0;
    nodes_local_.clear();
    edges_local_.clear();
    node_id_to_index_.clear();
    edge_id_to_index_.clear();
}

bool NemLocalPlannerROS::ReadFromJson(const std::istream &node_stream, Json::Value &val) {
    std::ostringstream sin;
    sin << node_stream.rdbuf();
    std::string str = sin.str();
    Json::Reader reader;
    return reader.parse(str, val);
}

int NemLocalPlannerROS::FindEdgeByNodes(const CNode &nodeA, const CNode &nodeB, CEdge &edge) {
    std::lock_guard<std::recursive_mutex> lck(m_mtx_local);
    int ret = 0;
    for (int i = 0; i < edges_local_.size(); i++) {
        if (edges_local_[i].nodeA_ == nodeA.id && edges_local_[i].nodeB_ == nodeB.id &&
            edges_local_[i].available_ != 0) {
            ret  = 1;
            edge = edges_local_[i];
            break;
        }
        if (edges_local_[i].nodeA_ == nodeB.id && edges_local_[i].nodeB_ == nodeA.id &&
            edges_local_[i].available_ != 0) {
            ret  = 2;
            edge = edges_local_[i];
            break;
        }
    }
    return ret;
}

int NemLocalPlannerROS::GetEdgeLane(CNode::IdType idA, CNode::IdType idB) {
    for (int i = 0; i < edges_local_.size(); i++) {
        if (edges_local_[i].nodeA_ == idA && edges_local_[i].nodeB_ == idB && edges_local_[i].available_ != 0) {
            return static_cast<int>(edges_local_[i].single_lane_);
        }
        if (edges_local_[i].nodeA_ == idB && edges_local_[i].nodeB_ == idA && edges_local_[i].available_ != 0) {
            return -static_cast<int>(edges_local_[i].single_lane_);
        }
    }
    return 0;
}

bool NemLocalPlannerROS::GetAvoidance(CNode::IdType idA, CNode::IdType idB) {
    for (int i = 0; i < edges_local_.size(); i++) {
        if (edges_local_[i].nodeA_ == idA && edges_local_[i].nodeB_ == idB && edges_local_[i].avoidance_ == 1) {
            return true;
        } else if (edges_local_[i].nodeA_ == idA && edges_local_[i].nodeB_ == idB && edges_local_[i].avoidance_ == 0) {
            return false;
        }
        if (edges_local_[i].nodeA_ == idB && edges_local_[i].nodeB_ == idA && edges_local_[i].avoidance_ == 1) {
            return true;
        } else if (edges_local_[i].nodeA_ == idB && edges_local_[i].nodeB_ == idA && edges_local_[i].avoidance_ == 0) {
            return false;
        }
    }
    return true;
}

bool NemLocalPlannerROS::GetAntiDrop(CNode::IdType idA, CNode::IdType idB) {
    for (int i = 0; i < edges_local_.size(); i++) {
        if (edges_local_[i].nodeA_ == idA && edges_local_[i].nodeB_ == idB && edges_local_[i].anti_drop_ == 1) {
            return true;
        } else if (edges_local_[i].nodeA_ == idA && edges_local_[i].nodeB_ == idB && edges_local_[i].anti_drop_ == 0) {
            return false;
        }
        if (edges_local_[i].nodeA_ == idB && edges_local_[i].nodeB_ == idA && edges_local_[i].anti_drop_ == 1) {
            return true;
        } else if (edges_local_[i].nodeA_ == idB && edges_local_[i].nodeB_ == idA && edges_local_[i].anti_drop_ == 0) {
            return false;
        }
    }
    return true;
}

float NemLocalPlannerROS::GetMaxVx(CNode::IdType idA, CNode::IdType idB) {
    for (int i = 0; i < edges_local_.size(); i++) {
        if (edges_local_[i].nodeA_ == idA && edges_local_[i].nodeB_ == idB) {
            return edges_local_[i].max_speed_;
        }
        if (edges_local_[i].nodeA_ == idB && edges_local_[i].nodeB_ == idA) {
            return edges_local_[i].max_speed_;
        }
    }
    return max_vx_;
}

bool NemLocalPlannerROS::ExistNode(CNode::IdType id) {
    std::lock_guard<std::recursive_mutex> lck(m_mtx_local);
    if (node_id_to_index_.count(id) <= 0) {
        return false;
    }
    return true;
}

int NemLocalPlannerROS::FindEdgeByPoints(Point &pointA, Point &pointB, CEdge &edge) {
    std::lock_guard<std::recursive_mutex> lck(m_mtx_local);
    int ret       = 0;
    double dSpeed = 100;
    for (int i = 0; i < edges_local_.size(); i++) {
        if (edges_local_[i].available_ == 0) {
            continue;
        }

        CNode &a = nodes_local_[node_id_to_index_[edges_local_[i].nodeA_]];
        CNode &b = nodes_local_[node_id_to_index_[edges_local_[i].nodeB_]];

        Point pointA1, pointB1;
        pointA1.x = a.x;
        pointA1.y = a.y;
        pointB1.x = b.x;
        pointB1.y = b.y;
        if (IsPointsInLine(pointA, pointB, pointB1) && IsPointsInLine(pointA, pointB, pointA1)) {
            if (dSpeed > edges_local_[i].max_speed_) {
                if (((pointA1.x > pointA.x && pointA1.x < pointB.x) || (pointA1.y > pointA.y && pointA1.y < pointB.y) ||
                     (pointA1.x < pointA.x && pointA1.x > pointB.x) ||
                     (pointA1.y < pointA.y && pointA1.y > pointB.y)) ||
                    ((pointB1.x > pointA.x && pointB1.x < pointB.x) || (pointB1.y > pointA.y && pointB1.y < pointB.y) ||
                     (pointB1.x < pointA.x && pointB1.x > pointB.x) ||
                     (pointB1.y < pointA.y && pointB1.y > pointB.y)) ||
                    ((pointA.x > pointA1.x && pointA.x < pointB1.x) || (pointA.y > pointA1.y && pointA.y < pointB1.y) ||
                     (pointA.x < pointA1.x && pointA.x > pointB1.x) ||
                     (pointA.y < pointA1.y && pointA.y > pointB1.y)) ||
                    ((pointB.x > pointA1.x && pointB.x < pointB1.x) || (pointB.y > pointA1.y && pointB.y < pointB1.y) ||
                     (pointB.x < pointA1.x && pointB.x > pointB1.x) ||
                     (pointB.y < pointA1.y && pointB.y > pointB1.y))) {
                    dSpeed = edges_local_[i].max_speed_;
                    edge   = edges_local_[i];
                    ret    = 1;
                }
            }
        }
    }
    return ret;
}

bool NemLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan) {
    ROS_INFO("[nem_local_planner]: Receive a new plan\n");
    navi_step_    = GO_INIT;
    goal_reached_ = false;
    goal_failed_  = false;
    aim_path_nodes_.clear();
    jump_node_num_ = 0;
    last_cmd_      = 0.0;
    std::string plan_id;
    for (int i = 0; i < orig_global_plan.size(); i++) {
        if (orig_global_plan[i].pose.position.z < -1.0)
            continue;
        CNode thisPoint;
        thisPoint.x  = orig_global_plan[i].pose.position.x;
        thisPoint.y  = orig_global_plan[i].pose.position.y;
        thisPoint.id = orig_global_plan[i].pose.position.z;
        thisPoint.r  = tf::getYaw(orig_global_plan[i].pose.orientation);
        aim_path_nodes_.push_back(thisPoint);
        if (i == 0)
            plan_id += std::to_string(-1) + " -> ";
        else if (i == orig_global_plan.size() - 1)
            plan_id += std::to_string(thisPoint.id);
        else
            plan_id += std::to_string(thisPoint.id) + " -> ";
    }
    original_node_ = aim_path_nodes_.begin();
    target_node_   = original_node_;
    global_plan_   = orig_global_plan;
    pid_control_.reSet("reset");
    ROS_INFO_STREAM("[nem_local_planner]: Global plan id = " << plan_id);
    return true;
}

bool NemLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
    std::string dummy_message;
    geometry_msgs::PoseStamped dummy_pose;
    geometry_msgs::TwistStamped dummy_velocity, cmd_vel_stamped;
    uint32_t outcome = computeVelocityCommands(dummy_pose, dummy_velocity, cmd_vel_stamped, dummy_message);
    cmd_vel          = cmd_vel_stamped.twist;
    return outcome == 0;
}

uint32_t NemLocalPlannerROS::computeVelocityCommands(const geometry_msgs::PoseStamped &pose,
                                                     const geometry_msgs::TwistStamped &velocity,
                                                     geometry_msgs::TwistStamped &cmd_vel, std::string &message) {
    if (!initialize_) {
        ROS_ERROR("[nem_local_planner]: nem_local_planner has not been initialized, please call initialize() before "
                  "using this planner");
        message = "nem_local_planner has not been initialized";
        std_msgs::Int32 error_code;
        error_code.data = 211;
        navi_error_pub_.publish(error_code);
        return -100;
    }

    cmd_vel_helper_.linear.x  = 0.0;
    cmd_vel_helper_.linear.y  = 0.0;
    cmd_vel_helper_.angular.z = 0.0;

    switch (navi_step_) {
        case GO_INIT:
            GoInit();
            break;
        case GO_TURN:
            GoTurn();
            break;
        case GO_DIST:
            GoDistance();
            break;
        case GO_ACTION:
            GoAction();
            break;
        case GO_ACTION_TURN:
            GoActionTurn();
            break;
        case GO_ACTION_TURN_OPP:
            GoActionTurnOpp();
            break;
        case GO_DIST_FINE:
            GoDistanceFine();
            break;
        case GO_TURN_FINE:
            GoTurnFine();
            break;
        case GO_ADJUST_Y:
            GoAdjustY();
            break;
        case GO_WAIT:
            GoWait();
            break;
        case GO_STOP:
            GoStop();
            break;
        case GO_FAIL:
            GoFail();
            break;
    }

    if (goal_reached_) {
        cmd_vel.twist.linear.x  = 0.0;
        cmd_vel.twist.linear.y  = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        jump_node_num_          = 0;
        return 0;
    }
    if (goal_failed_) {
        cmd_vel.twist.linear.x  = 0.0;
        cmd_vel.twist.linear.y  = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        jump_node_num_          = 0;
        
        std_msgs::Int32 error_code;
        error_code.data = 214;
        navi_error_pub_.publish(error_code);
        return -1;
    }
    if (avoidance_ && avoidance_edge_) {
        bool feasible = isTrajectoryFeasible(costmap_model_.get(), footprint_spec_, robot_inscribed_radius_,
                                             robot_circumscribed_radius, avoidance_num_, avoidance_res_);
        if (!feasible) {
            cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
            // now we reset everything to start again with the initialization of new trajectories.
            //            aim_path_nodes_.clear();
            ROS_WARN_THROTTLE(1, "[nem_local_planner]: trajectory is not feasible. Resetting planner...");
            message = "nem_local_planner trajectory is not feasible";
            std_msgs::Int32 error_code;
            error_code.data = 212;
            navi_error_pub_.publish(error_code);

            if(!obsavoid_sound_play_) {
                obsavoid_sound_play_ = true;
                sound_play_flag_     = false;
                SoundPlay(2, 1, "navigation_avoid.wav");
                std_msgs::Bool msg;
                msg.data = true;
                laser_obs_state_pub_.publish(msg);
            } 
            return -101;
        } else if(obsavoid_sound_play_) {
            obsavoid_sound_play_ = false;
            sound_play_flag_     = false;
            SoundPlay(0, 1, "navigation_avoid.wav");  
            std_msgs::Bool msg;
            msg.data = false;
            laser_obs_state_pub_.publish(msg); 
        }
    } else if(!sound_play_flag_) {
        sound_play_flag_ = true;
        SoundPlay(0, 1, "navigation_avoid.wav");
        std_msgs::Bool msg;
        msg.data = false;
        laser_obs_state_pub_.publish(msg); 
    }
    
    if((cmd_vel_helper_.linear.x > 0 && forward_obs_) || (cmd_vel_helper_.linear.x < 0 && back_obs_)) {
        cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
        // now we reset everything to start again with the initialization of new trajectories.
        //            aim_path_nodes_.clear();
        ROS_WARN_THROTTLE(1, "[nem_local_planner]: Vison obs is triggering . Please verify...");
        message = "nem_local_planner trajectory is not feasible";
        std_msgs::Int32 error_code;
        error_code.data = 213;
        navi_error_pub_.publish(error_code);
        return -101;
    }
    cmd_vel.twist.linear.x  = cmd_vel_helper_.linear.x;
    cmd_vel.twist.linear.y  = cmd_vel_helper_.linear.y;
    cmd_vel.twist.angular.z = cmd_vel_helper_.angular.z;
    return 0;
}

bool NemLocalPlannerROS::isTrajectoryFeasible(teb_local_planner::CostmapModel *costmap_model,
                                              const std::vector<geometry_msgs::Point> &footprint_spec,
                                              double inscribed_radius, double circumscribed_radius, int look_ahead_idx,
                                              double res) {
    std::vector<geometry_msgs::PoseStamped> local_plan;
    geometry_msgs::PoseStamped goal;
    goal.pose.position.x = target_node_->x;
    goal.pose.position.y = target_node_->y;
    geometry_msgs::PoseStamped robot_pose;
    costmap_ros_->getRobotPose(robot_pose);
    generatePath(robot_pose, goal, local_plan, look_ahead_idx, res);

    if (look_ahead_idx < 0 || look_ahead_idx >= local_plan.size())
        look_ahead_idx = local_plan.size() - 1;

    for (int i = 0; i <= look_ahead_idx; ++i) {
        if (costmap_model->footprintCost(local_plan[i].pose.position.x, local_plan[i].pose.position.y, 0.0,
                                         footprint_spec, inscribed_radius, circumscribed_radius) == -1) {
            pubFootprint(feasible_footprint_pub_, local_plan[i], inscribed_radius);
            return false;
        }
        // Checks if the distance between two poses is higher than the robot radius or the orientation diff is bigger
        // than the specified threshold and interpolates in that case. (if obstacles are pushing two consecutive poses
        // away, the center between two consecutive poses might coincide with the obstacle ;-)!
        if (i < look_ahead_idx) {
            double delta_dist = getDistance(local_plan[i + 1], local_plan[i]);
            if (delta_dist > inscribed_radius) {
                int n_additional_samples                     = std::ceil(delta_dist / inscribed_radius) - 1;
                geometry_msgs::PoseStamped intermediate_pose = local_plan[i];
                for (int step = 0; step < n_additional_samples; ++step) {
                    intermediate_pose.pose.position.x =
                        intermediate_pose.pose.position.x + delta_dist / (n_additional_samples + 1.0);
                    intermediate_pose.pose.position.y =
                        intermediate_pose.pose.position.y + delta_dist / (n_additional_samples + 1.0);
                    if (costmap_model->footprintCost(intermediate_pose.pose.position.x,
                                                     intermediate_pose.pose.position.y, 0.0, footprint_spec,
                                                     inscribed_radius, circumscribed_radius) == -1) {
                        pubFootprint(feasible_footprint_pub_, local_plan[i], inscribed_radius);
                        return false;
                    }
                }
            }
        }
    }
    return true;
}

void NemLocalPlannerROS::generatePath(geometry_msgs::PoseStamped &start, geometry_msgs::PoseStamped &goal,
                                      std::vector<geometry_msgs::PoseStamped> &plan, double res) {
    plan.clear();
    int num = 0;
    if (getDistance(start, goal) <= res) {
        plan.push_back(start);
        plan.push_back(goal);
        return;
    }
    for (nem_local_planner::LineIterator line(start.pose.position.x, start.pose.position.y, goal.pose.position.x,
                                              goal.pose.position.y, res);
         line.isValid(); line.advance()) {
        geometry_msgs::PoseStamped iterPoint;
        iterPoint.header.stamp    = ros::Time::now();
        iterPoint.header.frame_id = "map";
        iterPoint.header.seq      = num;
        iterPoint.pose.position.x = line.getX();
        iterPoint.pose.position.y = line.getY();
        iterPoint.pose.position.z = 0.0;
        plan.push_back(iterPoint);
        num++;
    }
    plan.push_back(goal);
}

void NemLocalPlannerROS::generatePath(geometry_msgs::PoseStamped &robot_pose, geometry_msgs::PoseStamped &goal,
                                      std::vector<geometry_msgs::PoseStamped> &plan, int len, double res) {
    plan.clear();
    int num = 0;
    nav_msgs::Path gui_path;
    gui_path.poses.clear();
    gui_path.header.frame_id = "map";
    gui_path.header.stamp    = ros::Time::now();
    if (getDistance(robot_pose, goal) <= res) {
        plan.push_back(robot_pose);
        plan.push_back(goal);
        return;
    }
    for (nem_local_planner::LineIterator line(robot_pose.pose.position.x, robot_pose.pose.position.y,
                                              goal.pose.position.x, goal.pose.position.y, res);
         line.isValid(); line.advance()) {
        if (num > len)
            break;
        geometry_msgs::PoseStamped iterPoint;
        iterPoint.header.stamp    = ros::Time::now();
        iterPoint.header.frame_id = "map";
        iterPoint.header.seq      = num;
        iterPoint.pose.position.x = line.getX();
        iterPoint.pose.position.y = line.getY();
        iterPoint.pose.position.z = 0.0;
        iterPoint.pose.orientation =
            robot_pose.pose
                .orientation;  //由于点边模式始终存在原地旋转的动作，故在做碰撞检测时默认机器人的footprint为一个圆
        plan.push_back(iterPoint);
        gui_path.poses.push_back(iterPoint);
        num++;
    }
    local_plan_pub_.publish(gui_path);
}

std::vector<geometry_msgs::Point> NemLocalPlannerROS::makeFootprintFromRadius(double radius) {
    std::vector<geometry_msgs::Point> points;
    // Loop over 16 angles around a circle making a point each time
    int N = 16;
    geometry_msgs::Point pt;
    for (int i = 0; i < N; ++i) {
        double angle = i * 2 * M_PI / N;
        pt.x         = cos(angle) * radius;
        pt.y         = sin(angle) * radius;
        points.push_back(pt);
    }
    return points;
}

void NemLocalPlannerROS::transformFootprint(double x, double y, double theta,
                                            const std::vector<geometry_msgs::Point> &footprint_spec,
                                            geometry_msgs::PolygonStamped &oriented_footprint) {
    // build the oriented footprint at a given location
    oriented_footprint.polygon.points.clear();
    double cos_th = cos(theta);
    double sin_th = sin(theta);
    for (unsigned int i = 0; i < footprint_spec.size(); ++i) {
        geometry_msgs::Point32 new_pt;
        new_pt.x = x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
        new_pt.y = y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
        oriented_footprint.polygon.points.push_back(new_pt);
    }
}

void NemLocalPlannerROS::pubFootprint(ros::Publisher pub, geometry_msgs::PoseStamped robot_pose, double radius) {
    geometry_msgs::PolygonStamped footprint;
    footprint.header.frame_id = "map";
    footprint.header.stamp    = ros::Time::now();
    double yaw                = tf::getYaw(robot_pose.pose.orientation);
    transformFootprint(robot_pose.pose.position.x, robot_pose.pose.position.y, yaw, makeFootprintFromRadius(radius),
                       footprint);
    pub.publish(footprint);
}

double NemLocalPlannerROS::getDistance(geometry_msgs::PoseStamped &p1, geometry_msgs::PoseStamped &p2) {
    return sqrt(pow(p2.pose.position.x - p1.pose.position.x, 2) + pow(p2.pose.position.y - p1.pose.position.y, 2));
}

void NemLocalPlannerROS::CalGoDirection(int ret, CEdge edge, int *direction) {
    if (ret == 1) {
        if (edge.single_lane_ == CEdge::DIRECTION_FIXED::DIRECTION_FIXED_POSITIVE) {
            *direction = 1;
        } else if (edge.single_lane_ == CEdge::DIRECTION_FIXED::DIRECTION_FIXED_NEGATIVE) {
            *direction = -1;
        } else if (edge.single_lane_ == CEdge::DIRECTION_FIXED::DIRECTION_FIXED_NONE) {
            *direction = 0;
        }
    }
    if (ret == 2) {
        if (edge.single_lane_ == CEdge::DIRECTION_FIXED::DIRECTION_FIXED_POSITIVE) {
            *direction = -1;
        } else if (edge.single_lane_ == CEdge::DIRECTION_FIXED::DIRECTION_FIXED_NEGATIVE) {
            *direction = 1;
        } else if (edge.single_lane_ == CEdge::DIRECTION_FIXED::DIRECTION_FIXED_NONE) {
            *direction = 0;
        }
    }
}

bool NemLocalPlannerROS::GoInit() {
    geometry_msgs::PoseStamped current_pose;
    costmap_ros_->getRobotPose(current_pose);
    if (global_plan_.size() == 2) {
        // 0.若规划轨迹仅两点，机器人当前点和目标点，且终点在起点附近，即机器人已处于目标点附近
        // 1.若此时机器人朝向并未朝向目标点，则逻辑上为了到达目标点，机器人会旋转至目标点的方向，再进行运动从而到达目标点。此时若在股道内，则存在旋转碰撞股道的风险
        // 2.故若机器人已处于目标点附近，则判断到位的距离阈值为fine_adjust_dist_ + threshold_compensation_，将到位阈值其适当放大，防止旋转碰撞
        if (getDistance(global_plan_.front(), global_plan_.back()) <= fine_adjust_dist_ + threshold_compensation_) {
            ROS_INFO("[nem_local_planner][GoInit]: Robot is already in target goal, and dist = %f. Enter GoTrunFine to "
                    "turn for %f rad \n",
                    getDistance(global_plan_.front(), global_plan_.back()),
                    target_node_->r - tf::getYaw(current_pose.pose.orientation));
            // 指向最终目标点，保证到位旋转角度的获取
            target_node_++;
            go_turn_fine_start_ = ros::Time::now();
            navi_step_          = GO_TURN_FINE;
            return true;
        }
    }

    //若目标点为轨迹序列的最后一点且不需要进行Y方向的调整，则到位(调整Y方向误差的地方，边属性一定要设置成非防跌落)
    if (target_node_->id == aim_path_nodes_.back().id && !adjust_y_direction_) {
        ROS_INFO("[nem_local_planner][GoInit]: Node %lld Goal Reached, enter GoTurnFine. \n", target_node_->id);
        go_turn_fine_start_ = ros::Time::now();
        navi_step_          = GO_TURN_FINE;
        return true;
    } else if (target_node_->id == aim_path_nodes_.back().id && adjust_y_direction_) {
        adjust_y_direction_ = false;
        navi_step_          = GO_TURN;
        ROS_INFO("[nem_local_planner][GoInit]: Adjust y dis drive complete, GoTurn");
        return true;
    }

    //更新起点终点（跳点）,第一次移动前，不进行更新起点终点（跳点）
    if (jump_node_num_ > 0) {
        original_node_ = target_node_;
        target_node_++;
    } else {
        target_node_ = original_node_;
        target_node_++;
    }
    jump_node_num_++;

    /*判断是否需要延长target node，即跳点行驶*/
    // a = original_node_, b = target_node_, c = pose
    Point a, b, c;
    a.x = original_node_->x;
    a.y = original_node_->y;
    //默认下一次导航为非单向边,避障
    int nFixDirection = 0;
    avoidance_edge_   = true;
    anti_drop_edge_   = true;

    int last_ret;
    bool  last_avoid;
    bool  last_anti_drop;
    float last_max_speed;
    // 0.若路径点数小于等于2时，即路径点为机器人当前位置和目标点，不进行跳点操作
    if (aim_path_nodes_.size() > 2) {
        last_avoid     = GetAvoidance((target_node_ - 1)->id, target_node_->id);
        last_anti_drop = GetAntiDrop((target_node_ - 1)->id, target_node_->id);
        last_ret       = GetEdgeLane((target_node_ - 1)->id, target_node_->id);
        last_max_speed = GetMaxVx((target_node_ - 1)->id, target_node_->id);
        // 1.遍历路径点,并获取target node和target node++之间的边属性(避障边和单向边)
        for (vector<CNode>::iterator it = target_node_; it != aim_path_nodes_.end() - 1; it++) {
            Point m, n;
            m.x = it->x;
            m.y = it->y;
            n.x = (it + 1)->x;
            n.y = (it + 1)->y;
            CNode::IdType idA, idB;
            idA             = it->id;
            idB             = (it + 1)->id;
            avoidance_edge_ = GetAvoidance(idA, idB);
            anti_drop_edge_ = GetAntiDrop(idA, idB);
            nFixDirection   = GetEdgeLane(idA, idB);
            max_speed_      = GetMaxVx(idA, idB);
            // 2.判断是否需要跳点，即相邻的边为同向边、避障需求相同、最大速度相同，且三点形成的面积小于阈值，若成立，则将最后一点作为目标点
            if(stricter_on_line_judgement_) {
                if (IsPointOnLineStricter(a, m, n, points_on_one_line_) && (avoidance_edge_ == last_avoid) && (max_speed_ == last_max_speed) &&
                    ((nFixDirection == last_ret) ||
                    nFixDirection == static_cast<int>(CEdge::DIRECTION_FIXED::DIRECTION_FIXED_NONE) ||
                    last_ret == static_cast<int>(CEdge::DIRECTION_FIXED::DIRECTION_FIXED_NONE))) {
                    // 如果起始点、目标点、目标点+1三点在一条直线上，则目标点后移一位
                    last_ret       = nFixDirection;
                    last_avoid     = avoidance_edge_;
                    last_anti_drop = anti_drop_edge_;
                    last_max_speed = max_speed_;
                    target_node_   = it + 1;
                    ROS_WARN("[nem_local_planner][GoInit]: Node %ld %ld %ld on one-line, target jump to %ld", original_node_->id, it->id, (it + 1)->id,
                            target_node_->id);
                } 
                else 
                    break;
            } else {
                if (IsPointsInLine(a, m, n) && (avoidance_edge_ == last_avoid) && (max_speed_ == last_max_speed) &&
                    ((nFixDirection == last_ret) ||
                    nFixDirection == static_cast<int>(CEdge::DIRECTION_FIXED::DIRECTION_FIXED_NONE) ||
                    last_ret == static_cast<int>(CEdge::DIRECTION_FIXED::DIRECTION_FIXED_NONE))) {
                    last_ret       = nFixDirection;
                    last_avoid     = avoidance_edge_;
                    last_anti_drop = anti_drop_edge_;
                    last_max_speed = max_speed_;
                    target_node_   = it + 1;
                } else {
                    break;
                }
            }
        }
        nFixDirection   = last_ret;
        avoidance_edge_ = last_avoid;
        anti_drop_edge_ = last_anti_drop;
        max_speed_      = last_max_speed;
    }

    CEdge tempEdge;
    // 如果起始点为-1，则根据其所在边和目标点赋值对应的边属性
    if (original_node_->id == -1 &&
        GetEdgeForProperty(current_pose.pose.position.x, current_pose.pose.position.y, tempEdge, target_node_) ) {
        if(GetDistance(original_node_, original_node_+1) <= 0.5) {
            // -1位置开始行驶不获取当前边属性，防止在边上进行掉头
            nFixDirection   = 0;
            avoidance_edge_ = tempEdge.avoidance_;
            anti_drop_edge_ = tempEdge.anti_drop_;
            max_speed_      = tempEdge.max_speed_;
            ROS_INFO("[nem_local_planner][GoInit]: Robot start at -1 position, direction = %d, avoidance = %d, antoDrop = "
                    "%d, maxSpeed = %f", nFixDirection, avoidance_edge_, anti_drop_edge_, max_speed_);
        } else {
            nFixDirection   = tempEdge.single_lane_;
            avoidance_edge_ = tempEdge.avoidance_;
            anti_drop_edge_ = tempEdge.anti_drop_;
            max_speed_      = tempEdge.max_speed_;
            ROS_WARN("[nem_local_planner][GoInit]: Robot start at -1 position, direction = %d, avoidance = %d, antoDrop = "
                    "%d, maxSpeed = %f", nFixDirection, avoidance_edge_, anti_drop_edge_, max_speed_);
        }
    }

    b.x            = target_node_->x;
    b.y            = target_node_->y;
    c.x            = current_pose.pose.position.x;
    c.y            = current_pose.pose.position.y;
    fix_direction_ = nFixDirection;
    // 最大速度以导航配置为准
    max_speed_     = max_vx_;
    ROS_INFO("\n [nem_local_planner][GoInit]: Current plan: %lld -> %lld, direction = %d, avoidance = %d, antoDrop = %d, max_speed = %f"
             "\n start node x = %f, y = %f, r = %f"
             "\n target node x = %f, y = %f, r = %f \n",
             original_node_->id, target_node_->id, nFixDirection, avoidance_edge_, anti_drop_edge_, max_speed_,
             current_pose.pose.position.x, current_pose.pose.position.y, tf::getYaw(current_pose.pose.orientation),
             target_node_->x, target_node_->y, target_node_->r);
    if (!ExistNode(target_node_->id)) {
        ROS_ERROR("[nem_local_planner][GoInit]: target node not exist node id:%lld. \n", target_node_->id);
        navi_step_ = GO_FAIL;
        return false;
    }

    xform::Vector2<double> vec(b.x - a.x, b.y - a.y);
    m_vecN_ = vec.normalize();

    // 若当前位置已在当前目标点附近，则将目标点改为下一点,减少一次导航到点的操作,防止在股道内旋转碰撞
    // 全局规划中机器人IsOnNode阈值为0.1m，即当超过0.1m时即认为机器人是在边上，在边上时目标点为远距离的点，这里的条件便不满足
    if (std::hypot(b.x - c.x, b.y - c.y) <= fine_adjust_dist_ + threshold_compensation_) {    
        navi_step_ = GO_INIT;
        ROS_INFO("[nem_local_planner][GoInit]: Its closed enough between robot pose and next node %lld, enter GoInit again",
            target_node_->id);
        return true;
    }
    //判断并当前边的边属性中朝向角度
    if (nFixDirection == CEdge::DIRECTION_FIXED::DIRECTION_FIXED_NEGATIVE) {
        target_slope_ = SlopeLine(b, a);
    } else if (nFixDirection == CEdge::DIRECTION_FIXED::DIRECTION_FIXED_POSITIVE) {
        target_slope_ = SlopeLine(a, b);
        ROS_ERROR("target_slope_ = %f", target_slope_);
    } else if (abs(nFixDirection) == CEdge::DIRECTION_FIXED::DIRECTION_FIXED_FOREWORD) {
        target_slope_ = SlopeLine(a, b);
    } else {
        xform::Matrix2x2<double> matrixRotation(xform::Vector2<double>(cos(tf::getYaw(current_pose.pose.orientation)),
                                                                       -sin(tf::getYaw(current_pose.pose.orientation))),
                                                xform::Vector2<double>(sin(tf::getYaw(current_pose.pose.orientation)),
                                                                       cos(tf::getYaw(current_pose.pose.orientation))));
        xform::Vector2<double> vecGlobalTarget(target_node_->x - current_pose.pose.position.x,
                                               target_node_->y - current_pose.pose.position.y);
        xform::Vector2<double> vecLocalTarget = matrixRotation * vecGlobalTarget;
        // 目标点在前面
        if (vecLocalTarget.x > 0) {
            target_slope_ = SlopeLine(c, b);  // a尾b头
        } else {
            target_slope_ = SlopeLine(b, c);
        }
    }

    // 启动时判断目标点朝向，选择是否GoTurn
    double angle_diff = fabs(xform::normalize(target_slope_ - tf::getYaw(current_pose.pose.orientation)));
    if (angle_diff < angle_threshold_) {
        navi_step_     = GO_DIST;
        go_dist_start_ = ros::Time::now();
        // GenerateGoDistanceNodes(original_node_, target_node_);
        ROS_INFO("[nem_local_planner][GoInit]: GoInit over. Angle diff is %f short than threshold %f. Enter GoDist for "
                 "%fm. \n",
                 angle_diff, angle_threshold_,
                 sqrt(pow(target_node_->x - current_pose.pose.position.x, 2) +
                      pow(target_node_->y - current_pose.pose.position.y, 2)));
    } else {
        navi_step_     = GO_TURN;
        go_turn_start_ = ros::Time::now();
        ROS_INFO("[nem_local_planner][GoInit]: GoInit over. angle_diff is %f higher than threshold %f, need to turn "
                 "first. Enter GoTurn. \n",
                 angle_diff, angle_threshold_);
    }

    return true;
}

bool NemLocalPlannerROS::GoTurn() {
    geometry_msgs::PoseStamped current_pose;
    costmap_ros_->getRobotPose(current_pose);
    if (RobotTurn(target_slope_, angle_threshold_)) {
        sleep(1);
        navi_step_     = GO_DIST;
        go_dist_start_ = ros::Time::now();
        // GenerateGoDistanceNodes(original_node_, target_node_);
        go_turn_end_            = ros::Time::now();
        ros::Duration time_cost = go_turn_end_ - go_turn_start_;
        ROS_INFO("[nem_local_planner][GoTurn]: GoTurn over, current yaw = %f, target yaw = %f. Time cost = %f. Enter "
                 "GoDist for %f m\n",
                 tf::getYaw(current_pose.pose.orientation), target_slope_, time_cost.toSec(),
                 sqrt(pow(target_node_->x - current_pose.pose.position.x, 2) +
                      pow(target_node_->y - current_pose.pose.position.y, 2)));
    }
    return true;
}

bool NemLocalPlannerROS::RobotTurn(float target, float threshold) {
    geometry_msgs::PoseStamped current_pose;
    costmap_ros_->getRobotPose(current_pose);
    float theta      = xform::normalize(target - tf::getYaw(current_pose.pose.orientation));
    float angle_diff = fabs(theta);

    ROS_DEBUG_THROTTLE(1,"[nem_local_planner][RobotTurn]: Robot trun trun trun!");
    ROS_DEBUG("[nem_local_planner][RobotTurn]: Robot trun %f", theta);
    if (angle_diff < threshold) {
        return true;
    }
    float speed;
    if (use_pid_)
        //由于在加速阶段和匀速阶段只需给最大速度即可，故只需在减速阶段计算实时速度，减速阶段为s = a * t^2 / 2
        //将导航过程中速度计算方式为由加速度和实时与终点的距离来约束，公式为vx = (2 * a)^0.5 * S^0.5，是由S = 1/2 * a *
        // t^2和vx = a * t消元得到 由于y = x^0.5在x属于（0,1）区间内斜率过大，导致无法精确停车，故将0.5放大至0.75
        speed = theta / angle_diff * pow(angle_diff, 0.75) * pow(2 * angular_acceleration_, 0.5);
    else
        speed = theta / angle_diff * pow(angle_diff, GO_TURN_X) * GO_TURN_Y;
    ROS_DEBUG("[nem_local_planner][RobotTurn]: Robot speed %f", speed);

    if (fabs(speed) > max_vr_) {
        speed = sign(speed) * max_vr_;
    } else if (fabs(speed) < min_vr_) {
        speed = sign(speed) * min_vr_;
    }
    // 原地旋转时的电机响应速度补偿
    if(0.0 != speed && !sim_mode_) {
        if(speed > 0.0)
            speed = speed + real_vr_compensate_;
        else
            speed = speed - real_vr_compensate_;
    }
    PublishCmdVel(0.0, 0.0, speed, true);

    return false;
}

bool NemLocalPlannerROS::GoDistance() {
    geometry_msgs::PoseStamped current_pose;
    costmap_ros_->getRobotPose(current_pose);

    double pose_yaw = tf::getYaw(current_pose.pose.orientation);
    xform::Vector2<double> globalTarget(target_node_->x - current_pose.pose.position.x,
                                        target_node_->y - current_pose.pose.position.y);
    xform::Matrix2x2<double> matrixRotation(xform::Vector2<double>(cos(pose_yaw), -sin(pose_yaw)),
                                            xform::Vector2<double>(sin(pose_yaw), cos(pose_yaw)));
    xform::Vector2<double> localTarget = matrixRotation * globalTarget;

    if (RobotGo(current_pose, target_node_, target_slope_, fine_adjust_dist_, true)) {
        go_dist_end_            = ros::Time::now();
        ros::Duration time_cost = go_dist_end_ - go_dist_start_;
        float dis               = sqrt(pow(target_node_->x - current_pose.pose.position.x, 2) +
                         pow(target_node_->y - current_pose.pose.position.y, 2));

        ROS_INFO("[nem_local_planner][GoDist]: Current antiDrop = %d", anti_drop_edge_);

        /*直线行驶后到位误差打印（转换到机器人局部坐标）*/
        if(target_node_->id == aim_path_nodes_.back().id) {
            geometry_msgs::PoseStamped targetPoint;
            targetPoint.pose.position.x = target_node_->x;
            targetPoint.pose.position.y = target_node_->y;
            tf::Transform local_pose_T, target_T, map_to_local_T;
            tf::Vector3 local_origin, target_origin;
            double local_yaw, target_yaw;

            local_origin = {current_pose.pose.position.x, current_pose.pose.position.y, 0};
            local_yaw    = tf::getYaw(current_pose.pose.orientation);
            local_pose_T.setOrigin(local_origin);
            local_pose_T.setRotation(tf::createQuaternionFromYaw(local_yaw));  

            CNode errorPose;
            errorPose.x = aim_path_nodes_.back().x - current_pose.pose.position.x;
            errorPose.y = aim_path_nodes_.back().y - current_pose.pose.position.y;
            errorPose.r = aim_path_nodes_.back().r - tf::getYaw(current_pose.pose.orientation);

            target_origin = {aim_path_nodes_.back().x, aim_path_nodes_.back().y, 0};
            target_yaw    = aim_path_nodes_.back().r ;
            target_T.setOrigin(target_origin);
            target_T.setRotation(tf::createQuaternionFromYaw(target_yaw)); 

            map_to_local_T = local_pose_T.inverse() * target_T;

            ROS_INFO("[nem_local_planner][GoDist]: Nem local planner control over \n");
            ROS_INFO("[nem_local_planner][GoDist]: Current pose: %f, %f, %f",
                        current_pose.pose.position.x, current_pose.pose.position.y, tf::getYaw(current_pose.pose.orientation));
            ROS_INFO("[nem_local_planner][GoDist]: Goal   Pose : %f, %f, %f",
                        aim_path_nodes_.back().x, aim_path_nodes_.back().y, aim_path_nodes_.back().r);
            ROS_INFO("[nem_local_planner][GoDist]: Position error: %f, %f, %f",
                        fabs(map_to_local_T.getOrigin().x()), fabs(map_to_local_T.getOrigin().y()), fabs(tf::getYaw(map_to_local_T.getRotation())));
        }
        // 只对当前规划的最后一个目标点进行Y方向调整，Y方向调整时的前进或后退不判断Y方向误差
        if (target_node_->id == aim_path_nodes_.back().id && !adjust_y_direction_ && fabs(localTarget.y) > 0.02 &&
            !anti_drop_edge_) {
            navi_step_          = GO_ADJUST_Y;
            adjust_y_direction_ = true;
            ROS_INFO("[nem_local_planner][GoDist]: GoDist over, but contins error in Y %f", localTarget.y);
        } else {
            navi_step_ = GO_INIT;
            ROS_INFO("[nem_local_planner][GoDist]: GoDist over. Goal dist = %f m, tolerance = %f m. Time cost = %f. "
                     "Enter GoInit. \n",
                     dis, fine_adjust_dist_, time_cost.toSec());
        }
        if (fake_reached_) {
            navi_step_ = GO_FAIL;
            ROS_ERROR("[nem_local_planner][GoDist]: Did not reached real Goal, Enter GoFail");
        }
        fake_reached_ = false;
    }
    return true;
}

bool NemLocalPlannerROS::GoTurnFine() {
    geometry_msgs::PoseStamped current_pose;
    costmap_ros_->getRobotPose(current_pose);
    if (RobotTurn(target_node_->r, fine_angle_threshold_)) {
        navi_step_              = GO_STOP;
        go_turn_fine_end_       = ros::Time::now();
        ros::Duration time_cost = go_turn_fine_end_ - go_turn_fine_start_;
        ROS_INFO("[nem_local_planner][GoTurnFine]: GoTurnFine over, current yaw = %f, target yaw = %f. Time cost = %f. "
                 "Enter GoStop. \n",
                 tf::getYaw(current_pose.pose.orientation), target_node_->r, time_cost.toSec());
    }
    return true;
}

bool NemLocalPlannerROS::GoAdjustY() {
    geometry_msgs::PoseStamped current_pose;
    costmap_ros_->getRobotPose(current_pose);
    ROS_INFO("[nem_local_planner][GoAdjustY]: Y_Adjust, target is: %ld, %f, %f", target_node_->id, target_node_->x,
             target_node_->y);
    navi_step_      = GO_DIST;
    double pose_yaw = tf::getYaw(current_pose.pose.orientation);

    xform::Vector2<double> globalTarget(target_node_->x - current_pose.pose.position.x,
                                        target_node_->y - current_pose.pose.position.y);
    xform::Matrix2x2<double> matrixRotation(xform::Vector2<double>(cos(pose_yaw), -sin(pose_yaw)),
                                            xform::Vector2<double>(sin(pose_yaw), cos(pose_yaw)));
    xform::Matrix2x2<double> matrixRotationInverse(xform::Vector2<double>(cos(pose_yaw), sin(pose_yaw)),
                                                   xform::Vector2<double>(-sin(pose_yaw), cos(pose_yaw)));
    xform::Vector2<double> localTarget = matrixRotation * globalTarget;

    double yAdjustDistance = 0;
    xform::Vector2<double> nextLocalTarget(0.0, 0.0);

    // 计算局部坐标下机器人实际要后退或前进的距离(“从哪来回哪去”进行前进和后退，保证安全)
    if (fix_direction_ == 1) {
        nextLocalTarget.x = -fabs(localTarget.y) / tan(0.174533);
        nextLocalTarget.y = 0.0;
        ROS_INFO("[nem_local_planner][GoAdjustY]: FixDirecton = 1, back!! %f", nextLocalTarget.x);
        if (localTarget.y > 0) {
            target_slope_ = pose_yaw + 0.174533;
        } else {
            target_slope_ = pose_yaw - 0.174533;
        }
        ROS_INFO("[nem_local_planner][GoAdjustY]: Turn!! %f", target_slope_);
    } else if (fix_direction_ == -1) {
        nextLocalTarget.x = fabs(localTarget.y) / tan(0.174533);
        nextLocalTarget.y = 0.0;
        ROS_INFO("[nem_local_planner][GoAdjustY]: FixDirecton = -1, forword!! %f", nextLocalTarget.x);
        if (localTarget.y > 0) {
            target_slope_ = pose_yaw - 0.174533;
        } else {
            target_slope_ = pose_yaw + 0.174533;
        }
        ROS_INFO("[nem_local_planner][GoAdjustY]: Turn!! %f", target_slope_);
    } else {
        nextLocalTarget.x = -fabs(localTarget.y) / tan(0.174533);
        nextLocalTarget.y = 0.0;
        ROS_INFO("[nem_local_planner][GoAdjustY]: FixDirecton = 0, back!! %f", nextLocalTarget.x);
        if (localTarget.y > 0) {
            target_slope_ = pose_yaw + 0.174533;
        } else {
            target_slope_ = pose_yaw - 0.174533;
        }
        ROS_INFO("[nem_local_planner][GoAdjustY]: Turn!! %f", target_slope_);
    }
    // 将局部坐标下的行走距离转换到全局坐标下
    xform::Vector2<double> nextGlobalTarget = matrixRotationInverse * nextLocalTarget;
    // 将全局坐标下的行走距离加到当前机器人的位姿下，得到要行驶的目标点
    y_adjust_target_.x = nextGlobalTarget.x + current_pose.pose.position.x;
    y_adjust_target_.y = nextGlobalTarget.y + current_pose.pose.position.y;

    ROS_INFO("[nem_local_planner][GoAdjustY]: Y adjust dis local  is %f, %f", nextLocalTarget.x, nextLocalTarget.y);
    ROS_INFO("[nem_local_planner][GoAdjustY]: Y adjust dis global is %f, %f", nextGlobalTarget.x, nextGlobalTarget.y);
    ROS_INFO("[nem_local_planner][GoAdjustY]: Y adjust targrt global is %f, %f", y_adjust_target_.x,
             y_adjust_target_.y);

    return true;
}

bool NemLocalPlannerROS::RobotGo(geometry_msgs::PoseStamped robot_pose, vector<CNode>::iterator target_node,
                                 float target_sloap, float min_cross_product, bool bDelay) {
    geometry_msgs::PoseStamped targetPoint;
    if (adjust_y_direction_) {
        targetPoint.pose.position.x = y_adjust_target_.x;
        targetPoint.pose.position.y = y_adjust_target_.y;
    } else {
        targetPoint.pose.position.x = target_node->x;
        targetPoint.pose.position.y = target_node->y;
    }
    float vx = 0.0;
    float vy = 0.0;
    float vr = 0.0;

    if (CalculateDistVel(robot_pose, targetPoint, vx, vy, vr, min_cross_product, target_sloap, m_vecN_)) {
        pid_control_.reSet("keep_plot");
        if (isRobotGoReached(robot_pose, targetPoint, min_cross_product+threshold_compensation_)) {
            return true;
        } else {
            fake_reached_ = true;
            ROS_ERROR("Local path control end, didn't reach Goal node, Failure!");
            return true;
        }
    }
    PublishCmdVel(vx, vy, vr, bDelay);
    return false;
}

bool NemLocalPlannerROS::CalculateDistVel(const geometry_msgs::PoseStamped &pose,
                                          const geometry_msgs::PoseStamped &targetPoint, float &vx, float &vy,
                                          float &vr, float fMinCrossProduct, float fTargetSlope,
                                          xform::Vector2<double> goDirectionV) {
    bool bRetVal = CalculateDistVel(pose, targetPoint, vx, vy, vr, fMinCrossProduct, fTargetSlope, goDirectionV,
                                    min_vx_, max_speed_, min_vy_, max_vy_, min_vr_, max_vr_, vy_reversed_);
    return bRetVal;
}

bool NemLocalPlannerROS::CalculateDistVel(const geometry_msgs::PoseStamped &pose,
                                          const geometry_msgs::PoseStamped &targetPoint, float &vx, float &vy,
                                          float &vr, float fMinCrossProduct, float fTargetSlope,
                                          xform::Vector2<double> goDirectionV, float _minVx, float _maxVx, float _minVy,
                                          float _maxVy, float _minVr, float _maxVr, float vyReversed) {
    double pose_yaw = tf::getYaw(pose.pose.orientation);
    xform::Vector2<double> vecDis(targetPoint.pose.position.x - pose.pose.position.x,
                                  targetPoint.pose.position.y - pose.pose.position.y);
    double fCrossPruduct = vecDis * goDirectionV;
    xform::Matrix2x2<double> matrixRotation(xform::Vector2<double>(cos(pose_yaw), -sin(pose_yaw)),
                                            xform::Vector2<double>(sin(pose_yaw), cos(pose_yaw)));
    xform::Vector2<double> vecGlobalTarget(targetPoint.pose.position.x - pose.pose.position.x,
                                           targetPoint.pose.position.y - pose.pose.position.y);
    xform::Vector2<double> vecLocalTarget = matrixRotation * vecGlobalTarget;
    if (use_pid_) {
        if (isRobotGoReached(pose, targetPoint, fMinCrossProduct)) {
            last_cmd_ = 0.0;
            return true;
        } else {
            ROS_DEBUG_THROTTLE(1,"[nem_local_planner][RobotGo]: Robot go go go!");
            //由于在加速阶段和匀速阶段只需给最大速度即可，故只需在减速阶段计算实时速度，减速阶段为s = a * t^2 / 2
            //将导航过程中速度计算方式为由加速度和实时与终点的距离来约束，公式为vx = (2 * a)^0.5 * S^0.5，是由S = 1/2 *
            // a * t^2和vx = a * t消元得到 由于y = x^0.5在x属于（0,1）区间内斜率过大，导致无法精确停车，故将0.5放大至0.65
            vx = sign(vecLocalTarget.x) * (pow(fabs(vecLocalTarget.x), 0.65) * pow(2 * acceleration_, 0.5));
            if(!long_corridor_) {
                if (fabs(vx) > fabs(_maxVx)) {
                    vx = sign(vx) * fabs(_maxVx);
                } else if (fabs(vx) < _minVx) {
                    vx = sign(vx) * _minVx;
                }
            }
            else {
                if (fabs(vx) > fabs(LONG_CORRIDOR_MAX_VX)) {
                    vx = sign(vx) * LONG_CORRIDOR_MAX_VX;
                } else if (fabs(vx) < _minVx) {
                    vx = sign(vx) * _minVx;
                }
            }
        }
        if (vx * last_cmd_ < 0.00000000) {
            ROS_WARN("[nem_local_planner][RobotGo]: Oscillation!");
            oscillation_num_++;
            if (oscillation_num_ == 9) {
                ROS_WARN("[nem_local_planner][RobotGo]: Oscillation and stop!");
                last_cmd_        = 0.0;
                oscillation_num_ = 0;
                return true;
            }
        }

        std::vector<geometry_msgs::PoseStamped> local_path;
        geometry_msgs::PoseStamped origin_pose, target_pose, local_goal_pose;
        target_pose.pose.position.x = target_node_->x;
        target_pose.pose.position.y = target_node_->y;
        origin_pose.pose.position.x = original_node_->x;
        origin_pose.pose.position.y = original_node_->y;

        /************建立当前起点和终点的局部轨迹并差值，差值的距离即为跟踪的前视距离*****************/
        generatePath(origin_pose, target_pose, local_path, head_dist_);
        //将局部轨迹、当前位姿以及当前速度输入进pid控制器进行误差函数建立
        pid_control_.setPathAndPose(local_path, pose, vx, fabs(vecLocalTarget.x));

        // 采用PID根据误差函数的结果来控制行驶过程中当前转向速度
        vr = pid_control_.getPidControl(Kp_, Ki_, Kd_, max_angle_z_);
        // 非仿真环境根据机器人实际电机响应速度进行角速度补偿
        if(0.0 != vr && !sim_mode_) {
            if(vr > 0.0)
                vr = vr + real_vr_compensate_;
            else
                vr = vr - real_vr_compensate_;
        }

        //发布可视化数据
        float dis_point_to_line;
        dis_point_to_line = GetDisPointToLine(pose, origin_pose, target_pose);
        std_msgs::Float64 dis;
        dis.data = dis_point_to_line;
        trajectory_offset_pub_.publish(dis);
        last_cmd_ = vx;
    } else {
        if (fabs(fCrossPruduct) < fMinCrossProduct || fabs(vecLocalTarget.x) < 0.015) {
            ROS_INFO("[nem_local_planner][RobotGo]: Reached! projected dist = %f, projected dist threshold = %f; "
                     "heading dist = %f, heading dist threshold = %f",
                     fabs(fCrossPruduct), fMinCrossProduct, fabs(vecLocalTarget.x), 0.015);
            return true;
        } else {
            vx = sign(vecLocalTarget.x) * (pow(fabs(vecLocalTarget.x), GO_DISTANCE_X) * GO_DISTANCE_Y);
            if (fabs(vx) > fabs(_maxVx)) {
                vx = sign(vx) * fabs(_maxVx);
            } else if (fabs(vx) < _minVx) {
                vx = sign(vx) * _minVx;
            }
        }
        if (fabs(fCrossPruduct) > GO_NEAR_DIST) {
            float theta      = xform::normalize(fTargetSlope - tf::getYaw(pose.pose.orientation));
            float angle_diff = fabs(theta);
            if (angle_diff > GO_TURN_R - 0.1) {
                vr = theta / angle_diff * pow(angle_diff, GO_TURN_X) * GO_TURN_Y * vyReversed;

                if (fabs(vr) > _maxVr) {
                    vr = sign(vr) * _maxVr;
                } else if (fabs(vr) < _minVr) {
                    vr = sign(vr) * _minVr;
                }
            }
        } else
            vr = 0;
    }
    //    last_cmd_ = 0.0;
    return false;
}

bool NemLocalPlannerROS::GoStop() {
    geometry_msgs::PoseStamped current_pose;
    costmap_ros_->getRobotPose(current_pose);

    geometry_msgs::PoseStamped targetPoint;
    targetPoint.pose.position.x = target_node_->x;
    targetPoint.pose.position.y = target_node_->y;
    /*到位旋转后的到位pose误差（转换到机器人局部坐标系下）*/
    tf::Transform local_pose_T, target_T, map_to_local_T;
    tf::Vector3 local_origin, target_origin;
    double local_yaw, target_yaw;

    local_origin = {current_pose.pose.position.x, current_pose.pose.position.y, 0};
    local_yaw    = tf::getYaw(current_pose.pose.orientation);
    local_pose_T.setOrigin(local_origin);
    local_pose_T.setRotation(tf::createQuaternionFromYaw(local_yaw));  

    // 为了防止异常情况下的机器人假到位
    if (isRobotGoReached(current_pose, targetPoint, fine_adjust_dist_ + threshold_compensation_)) {
        goal_reached_       = true;
        adjust_y_direction_ = false;
        jump_node_num_ = 0;
        CNode errorPose;
        errorPose.x = aim_path_nodes_.back().x - current_pose.pose.position.x;
        errorPose.y = aim_path_nodes_.back().y - current_pose.pose.position.y;
        errorPose.r = aim_path_nodes_.back().r - tf::getYaw(current_pose.pose.orientation);

        target_origin = {aim_path_nodes_.back().x, aim_path_nodes_.back().y, 0};
        target_yaw    = aim_path_nodes_.back().r ;
        target_T.setOrigin(target_origin);
        target_T.setRotation(tf::createQuaternionFromYaw(target_yaw)); 

        map_to_local_T = local_pose_T.inverse() * target_T;

        ROS_INFO("[nem_local_planner][GoStop]: Nem local planner control over \n");
        ROS_INFO("[nem_local_planner][GoStop]: Current pose: %f, %f, %f",
                 current_pose.pose.position.x, current_pose.pose.position.y, tf::getYaw(current_pose.pose.orientation));
        ROS_INFO("[nem_local_planner][GoStop]: Goal   Pose : %f, %f, %f",
                 aim_path_nodes_.back().x, aim_path_nodes_.back().y, aim_path_nodes_.back().r);
        ROS_INFO("[nem_local_planner][GoStop]: Position error: %f, %f, %f",
                 fabs(map_to_local_T.getOrigin().x()), fabs(map_to_local_T.getOrigin().y()), fabs(tf::getYaw(map_to_local_T.getRotation())));
        aim_path_nodes_.clear();
    } else {
        navi_step_ = GO_FAIL;
        ROS_ERROR("[nem_local_planner][GoStop]: Did not reached real Goal, Enter GoFail");
    }
}

bool NemLocalPlannerROS::GoFail() {
    goal_reached_       = false;
    goal_failed_        = true;
    adjust_y_direction_ = false;
    aim_path_nodes_.clear();
    jump_node_num_ = 0;
    ROS_ERROR_THROTTLE(1, "[nem_local_planner][GoFail]: Local planer control failed \n");
}

void NemLocalPlannerROS::GenerateGoDistanceNodes(vector<CNode>::iterator itSrc, vector<CNode>::iterator itDes) {
    go_distance_nodes_.clear();
    double dDis = sqrt((itSrc->x - itDes->x) * (itSrc->x - itDes->x) + (itSrc->y - itDes->y) * (itSrc->y - itDes->y));
    int nRate   = dDis / go_dist_gap_;
    for (int i = 1; i <= nRate; i++) {
        Point a;
        a.x = itSrc->x + go_dist_gap_ / dDis * (itDes->x - itSrc->x) * i;
        a.y = itSrc->y + go_dist_gap_ / dDis * (itDes->y - itSrc->y) * i;
        go_distance_nodes_.push_back(a);
    }
    Point a;
    a.x = itDes->x;
    a.y = itDes->y;
    go_distance_nodes_.push_back(a);
    go_distance_node_it_ = go_distance_nodes_.begin();

    do {
        geometry_msgs::PoseStamped current_pose;
        costmap_ros_->getRobotPose(current_pose);
        Point a, b;
        a.x = current_pose.pose.position.x;
        a.y = current_pose.pose.position.y;
        b.x = go_distance_node_it_->x;
        b.y = go_distance_node_it_->y;
        CEdge edge;
        max_speed_ = fabs(max_vx_);
        if (FindEdgeByPoints(a, b, edge) != 0) {
            if (max_speed_ > fabs(edge.max_speed_) && fabs(edge.max_speed_) > 0) {
                max_speed_ = fabs(edge.max_speed_);
            }
        }
    } while (0);
}

bool NemLocalPlannerROS::isRobotGoReached(const geometry_msgs::PoseStamped &pose,
                                          const geometry_msgs::PoseStamped &targetPoint, float threshold) {
    geometry_msgs::PoseStamped origin, target;
    origin.pose.position.x = original_node_->x;
    origin.pose.position.y = original_node_->y;
    // target.pose.position.x = target_node_->x;
    // target.pose.position.y = target_node_->y;
    target.pose.position.x = targetPoint.pose.position.x;
    target.pose.position.y = targetPoint.pose.position.y;
    double eDist           = sqrt(pow(pose.pose.position.x - targetPoint.pose.position.x, 2) +
                         pow(pose.pose.position.y - targetPoint.pose.position.y, 2));
    double pDist           = GetProDist(origin, target, pose);
    bool eReached          = eDist <= threshold;  //以欧式距离进行到位阈值
    bool pReached          = pDist <= threshold - 0.005;

    if (eReached || pReached) {
        if (!eReached)
            ROS_WARN("[nem_local_planner][RobotGo]: Reached for projection dist, and euclidean dist = %f\n", eDist);
        return true;
    }
    return false;
}

bool NemLocalPlannerROS::isGoalReached() { return goal_reached_; }

// void NemLocalPlannerROS::PoseReceived(const geometry_msgs::PoseStampedConstPtr pose) {
//    ROS_INFO("I hear Pose");
//    robot_pose_.x          = pose->pose.position.x;
//    robot_pose_.y          = pose->pose.position.y;
//    robot_pose_.r          = tf::getYaw(pose->pose.orientation);
//    robot_pose_.confidence = pose->pose.position.z;
//}

// void NemLocalPlannerROS::PoseReceived_amcl_lp(const geometry_msgs::PoseWithCovarianceStampedConstPtr pose) {
//    ROS_INFO("I hear amclPose");
//    robot_pose_.x = pose->pose.pose.position.x;
//    robot_pose_.y = pose->pose.pose.position.y;
//    robot_pose_.r = tf::getYaw(pose->pose.pose.orientation);
//}

void NemLocalPlannerROS::PublishCmdVel(float x, float y, float r, bool delay) {
    cmd_vel_helper_.linear.x  = x;
    cmd_vel_helper_.linear.y  = y;
    cmd_vel_helper_.angular.z = r;
}

bool NemLocalPlannerROS::GetEdgeForProperty(float x, float y, CEdge &edge, vector<CNode>::iterator targetNode) {
    float min = std::numeric_limits<float>::max();
    for (int i = 0; i < edges_local_.size(); i++) {
        if (edges_local_[i].available_ == 0)
            continue;
        CNode a = GetNode(edges_local_[i].nodeA_);
        CNode b = GetNode(edges_local_[i].nodeB_);

        float cross_a = (x - a.x) * (b.x - a.x) + (y - a.y) * (b.y - a.y);
        float cross_b = (x - b.x) * (a.x - b.x) + (y - b.y) * (a.y - b.y);
        // point not at line
        if (!(cross_a >= 0 && cross_b >= 0)) {
            continue;
        }

        // distance of A and B
        float dd = ((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));

        // calculate the pendicular point at line
        float rate = cross_a / dd;
        float px   = a.x + rate * (b.x - a.x);
        float py   = a.y + rate * (b.y - a.y);

        // get distance to the line
        float dt = sqrt((px - x) * (px - x) + (py - y) * (py - y));

        if (dt <= 0.05 && dt < min) {
            min  = dt;
            edge = edges_local_[i];
        }
        if ((edge.nodeA_ != targetNode->id) && (edge.nodeB_ != targetNode->id)) {
            continue;
        } else {
            ROS_INFO("[nem_local_planner][GoInit]: Get target node is %ld, edge's ID is %ld, %ld", targetNode->id, edge.nodeA_, edge.nodeB_);
            return true;
        }
    }
}

CNode NemLocalPlannerROS::GetNode(unsigned int id) {
    CNode temp;
    if (!node_id_to_index_.count(id)) {
        ROS_ERROR("Error,node:%d is not exist!", id);
        return temp;
    } else
        return nodes_local_[node_id_to_index_[id]];
}

CEdge NemLocalPlannerROS::GetEdge(unsigned int id) {
    if (!edge_id_to_index_.count(id)) {
        ROS_ERROR("Error,node is not exist!");
        return edges_local_[edges_local_.size()];
    } else
        return edges_local_[edge_id_to_index_[id]];
}

void NemLocalPlannerROS::ForwardObsStateCallback(const std_msgs::BoolConstPtr& state) {
    if (state->data) {
        forward_obs_ = true;
        ROS_WARN_THROTTLE(1, "Triggering forward vision obs!!!");
    }else {
        forward_obs_ = false;
    }
}

void NemLocalPlannerROS::BackObsStateCallback(const std_msgs::BoolConstPtr& state) {
    if (state->data) {
        back_obs_ = true;
        ROS_WARN_THROTTLE(1, "Triggering back vision obs!!!");
    }else {
        back_obs_ = false;
    }
}

void NemLocalPlannerROS::LongCorridorStateCallback(const std_msgs::String::ConstPtr& state) {
    if(state->data.find("on_by_reflector") == 0)
        long_corridor_ = true;
    else if(state->data.find("off_by_reflector") == 0)
        long_corridor_ = false;
}

void NemLocalPlannerROS::LaserAvoidSwitchStateCallback(const std_msgs::BoolConstPtr& state) {
    if(state->data) {
        avoidance_ = true;
        ROS_WARN_THROTTLE(1, "Laser avoidance ON!!!");
    } else {
        avoidance_ = false;
        ROS_WARN_THROTTLE(1, "Laser avoidance OFF!!!");
    }
}

void NemLocalPlannerROS::SoundPlay(const int command, const int vol, const std::string file) {
    sound_play::SoundRequestActionGoal action_goal;
    sound_play::SoundRequestGoal goal = action_goal.goal;
    goal.sound_request.sound          = -2;       
    goal.sound_request.command        = command;  // 0 stop 1 once 2 start
    goal.sound_request.volume         = vol;
    std::string path                  = "wav/" + file;
    goal.sound_request.arg            = path;
    goal.sound_request.arg2           = "sound_files";
    sound_play_client_.sendGoal(goal);
}

xform::Vector2<double> NemLocalPlannerROS::transformGlobalDeltaPoseToLocal(const double yaw, const double deltaPoseX, const double deltaPoseY) {
    xform::Vector2<double> globalDeltaTarget(deltaPoseX, deltaPoseY);
    xform::Matrix2x2<double> matrixRotation(xform::Vector2<double>(cos(yaw), -sin(yaw)),
                                            xform::Vector2<double>(sin(yaw), cos(yaw)));
    xform::Vector2<double> localDeltaTarget = matrixRotation * globalDeltaTarget;
    return localDeltaTarget;
}


}  // namespace nem_local_planner
