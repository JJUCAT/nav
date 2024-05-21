/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *********************************************************************/
#ifndef NAV_MOVE_BASE_ACTION_H_
#define NAV_MOVE_BASE_ACTION_H_

#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf/tf.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <navit_costmap/costmap_2d.h>
#include <navit_costmap/costmap_2d_ros.h>
#include <navit_core/base_global_planner.h>
#include <navit_core/base_controller.h>
#include <navit_move_base/base_recovery.h>
#include <navit_move_base/navi_support.h>

// actions
#include <sound_play/SoundRequestAction.h>
#include <navit_msgs/MoveBaseAction.h>

// msgs
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

// service
#include <nav_msgs/GetPlan.h>
#include <navit_move_base/MoveBaseLoadPlugin.h>
#include <navit_move_base/LoadNemData.h>
#include <navit_move_base/MoveBaseConfig.h>
#include <navit_move_base/CheckCurrentPlugin.h>

#include <topic_tools/MuxSelect.h>
#include <pluginlib/class_loader.hpp>

#include <dynamic_reconfigure/server.h>

namespace navit_move_base {
// typedefs to help us out with the action server so that we don't hace to type so much
typedef actionlib::SimpleActionServer<navit_msgs::MoveBaseAction> MoveBaseActionServer;
typedef actionlib::SimpleActionClient<navit_msgs::MoveBaseAction> MoveBaseActionClient;

enum MoveBaseState { PLANNING, CONTROLLING, CLEARING, FAILING };

enum RecoveryTrigger { PLANNING_R, CONTROLLING_R, OSCILLATION_R };

/**
 * @class MoveBase
 * @brief A class that uses the actionlib::ActionServer interface that moves the robot base to a goal location.
 */
class MoveBase {
   public:
    /**
     * @brief  Constructor for the actions
     * @param name The name of the action
     * @param tf A reference to a TransformListener
     */
    MoveBase(const std::shared_ptr<tf2_ros::Buffer>& tf);

    /**
     * @brief  Destructor - Cleans up
     */
    virtual ~MoveBase();

    /**
     * @brief  Performs a control cycle
     * @param goal A reference to the goal to pursue
     * @return True if processing of the goal is done, false otherwise
     */
    bool executeCycle(geometry_msgs::PoseStamped &goal);

   private:
    /**
     * @brief  A service call that clears the costmaps of obstacles
     * @param req The service request
     * @param resp The service response
     * @return True if the service call succeeds, false otherwise
     */
    bool clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

    /**
     * @brief  A service call that can be made when the action is inactive that will return a plan
     * @param  req The goal request
     * @param  resp The plan request
     * @return True if planning succeeded, false otherwise
     */
    bool planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp);

    /**
     * @brief  Make a new global plan
     * @param  goal The goal to plan to
     * @param  plan Will be filled in with the plan made by the planner
     * @return  True if planning succeeds, false otherwise
     */
    //bool makePlan(const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan);
    
    bool makePlan(const geometry_msgs::PoseStamped &goal, nav_msgs::Path &plan);

    /**
     * @brief  Load the recovery behaviors for the navigation stack from the parameter server
     * @param node The ros::NodeHandle to be used for loading parameters
     * @return True if the recovery behaviors were loaded successfully, false otherwise
     */
    bool loadRecoveryBehaviors(ros::NodeHandle node);

    /**
     * @brief  Loads the default recovery behaviors for the navigation stack
     */
    void loadDefaultRecoveryBehaviors();

    /**
     * @brief  Clears obstacles within a window around the robot
     * @param size_x The x size of the window
     * @param size_y The y size of the window
     */
    void clearCostmapWindows(double size_x, double size_y);

    /**
     * @brief  Publishes a velocity command of zero to the base
     */
    void publishZeroVelocity();

    /**
     * @brief  Reset the state of the move_base action and send a zero velocity command to the base
     */
    void resetState();

    bool initPlannerPlugins(boost::shared_ptr<navit_core::GlobalPlanner> &planner, const std::string &global_planner, const std::string &node_file_path = "/home/robot/.ros/nodes.json", const std::string &edge_file_path = "/home/robot/.ros/edges.json");

    bool initControllerPlugins(boost::shared_ptr<navit_core::Controller> &controller, const std::string &local_planner, const std::string &node_file_path = "/home/robot/.ros/nodes.json", const std::string &edge_file_path = "/home/robot/.ros/edges.json");

    bool clearLocalCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

    void movebaseClearLocalCostmap();

    bool loadNemDataService(navit_move_base::LoadNemDataRequest &req, navit_move_base::LoadNemDataResponse &res);

    bool plannerSwitchService(navit_move_base::MoveBaseLoadPluginRequest &req, navit_move_base::MoveBaseLoadPluginResponse &res);

    bool controllerSwitchService(navit_move_base::MoveBaseLoadPluginRequest &req, navit_move_base::MoveBaseLoadPluginResponse &res);

    bool checkCurrentPluginService(navit_move_base::CheckCurrentPluginRequest &req, navit_move_base::CheckCurrentPluginResponse &res);

    void goalCB(const geometry_msgs::PoseStamped::ConstPtr &goal);

    void poseCB(const geometry_msgs::PoseStamped msg);

    void naviErrorCB(const std_msgs::Int32::ConstPtr &data);

    void planThread();

    void executeCb(const navit_msgs::MoveBaseGoalConstPtr &move_base_goal);

    bool isQuaternionValid(const geometry_msgs::Quaternion &q);

    bool getRobotPose(geometry_msgs::PoseStamped &global_pose, navit_costmap::Costmap2DROS *costmap);

//    bool checkGoalAttribute(const geometry_msgs::PoseStamped &goal, std::string &aborted_content);

    double distance(const geometry_msgs::PoseStamped &p1, const geometry_msgs::PoseStamped &p2);

    geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped &goal_pose_msg);

    bool reLoadJson(const std::string &node_file_path, const std::string &edge_file_path);

    /**
     * @brief This is used to wake the planner at periodic intervals.
     */
    void wakePlanner(const ros::TimerEvent &event);

    void SoundPlay(const int command, const int vol, const std::string file);

    bool switchPluginsToRoundTheObs();

    bool switchToNemPlugins();

    actionlib::SimpleActionClient<sound_play::SoundRequestAction> sound_play_client_;

    std::shared_ptr<tf2_ros::Buffer> tf_;

    MoveBaseActionServer *as_;
    MoveBaseActionClient move_base_client_;

    boost::shared_ptr<navit_core::Controller> tc_;
    //navit_costmap::Costmap2DROS *planner_costmap_ros_, *controller_costmap_ros_;
    std::shared_ptr<navit_costmap::Costmap2DROS> planner_costmap_ros_, controller_costmap_ros_;

    boost::shared_ptr<navit_core::GlobalPlanner> planner_;
    std::string robot_base_frame_, global_frame_;

    std::vector<boost::shared_ptr<navit_move_base::recovery::RecoveryBehavior>> recovery_behaviors_, replan_recovery_behaviors_;
    std::vector<std::string> recovery_behavior_names_, replan_recovery_behavior_names_;
    unsigned int recovery_index_, replan_recovery_index_ = 0;

    geometry_msgs::PoseStamped global_pose_;
    double planner_frequency_, controller_frequency_, inscribed_radius_, circumscribed_radius_;
    double planner_patience_, controller_patience_;
    int32_t max_planning_retries_;
    uint32_t planning_retries_;
    double conservative_reset_dist_, neutral_reset_dist_, aggressive_reset_dist_, clearing_radius_;
    ros::Publisher current_goal_pub_, vel_pub_, action_goal_pub_, recovery_status_pub_, navi_state_pub_,
        navi_error_pub_;
    ros::Subscriber goal_sub_, pose_sub_;
    ros::Subscriber navi_error_sub_;
    ros::ServiceServer make_plan_srv_, clear_costmaps_srv_, planner_switch_srv_, controller_switch_srv_, clear_local_costmaps_srv_,
        update_file_srv_, check_plugin_srv_;
    ros::ServiceClient clear_local_costmap_client_, planner_switch_client_, controller_switch_client_, notice_update_files_srv_;
    bool shutdown_costmaps_, clearing_rotation_allowed_, recovery_behavior_enabled_;
    bool make_plan_clear_costmap_, make_plan_add_unreachable_goal_;
    double oscillation_timeout_, oscillation_distance_;

    MoveBaseState state_;
    RecoveryTrigger recovery_trigger_;

    ros::Time last_valid_plan_, last_valid_control_, last_oscillation_reset_, last_rotate_recovery_;
    geometry_msgs::PoseStamped oscillation_pose_;
    pluginlib::ClassLoader<navit_core::GlobalPlanner> bgp_loader_;
    pluginlib::ClassLoader<navit_core::Controller> blp_loader_;
    pluginlib::ClassLoader<navit_move_base::recovery::RecoveryBehavior> recovery_loader_;

    // set up plan triple buffer
    //std::vector<geometry_msgs::PoseStamped> *planner_plan_;
    //std::vector<geometry_msgs::PoseStamped> *latest_plan_;
    //std::vector<geometry_msgs::PoseStamped> *controller_plan_;
    nav_msgs::Path *planner_plan_;
    nav_msgs::Path *latest_plan_;
    nav_msgs::Path *controller_plan_;

    // set up the planner's thread
    bool runPlanner_;
    boost::recursive_mutex planner_mutex_;
    boost::condition_variable_any planner_cond_;
    geometry_msgs::PoseStamped planner_goal_;
    boost::thread *planner_thread_;

    boost::recursive_mutex configuration_mutex_;
    dynamic_reconfigure::Server<navit_move_base::MoveBaseConfig> *dsrv_;

    void reconfigureCB(navit_move_base::MoveBaseConfig &config, uint32_t level);

    navit_move_base::MoveBaseConfig last_config_;
    navit_move_base::MoveBaseConfig default_config_;
    bool setup_, p_freq_change_, c_freq_change_;
    bool new_global_plan_;
    bool idle_state_;
    bool get_load_nem_data_cmd_;
    bool use_avoidance_ = false;
    bool clear_full_map_ = false;
    bool sim_mode_;
    float confidence_;
    double available_confidence_;
    double wait_for_confidence_;
    bool begin_wait_;
    ros::Time last_wait_confidence_;

    // for navi state pub
    int last_data_;

    // for node and edge file path
    std::string node_file_path_local_;
    std::string edge_file_path_local_;

    // current plugin name
    std::string planner_plugin_;
    std::string controller_plugin_;
    std::string current_planner_plugin_;
    std::string current_controller_plugin_;

    // for different plugin name
    std::string primary_global_planner_;
    std::string secondary_global_planner_;
    std::string primary_local_planner_;
    std::string secondary_local_planner_;
    std::string dock_local_planner_;
    std_msgs::Int32 error_code_;

    navi_support::NaviSupport navi_support_;

};
};  // namespace navit_move_base
#endif
