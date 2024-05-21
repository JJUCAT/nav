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
*         Mike Phillips (put the planner in its own thread)
*********************************************************************/
#include <navit_move_base/navit_move_base.h>
#include <navit_msgs/RecoveryStatus.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nav_msgs/Path.h>
#include <std_msgs/Int8.h>

namespace navit_move_base {

    MoveBase::MoveBase(const std::shared_ptr<tf2_ros::Buffer> &tf) :
            tf_(tf),
            as_(NULL),
            planner_costmap_ros_(NULL), controller_costmap_ros_(NULL),
            bgp_loader_("navit_core", "navit_core::GlobalPlanner"),
            blp_loader_("navit_core", "navit_core::Controller"),
            recovery_loader_("navit_move_base", "navit_move_base::recovery::RecoveryBehavior"),
            planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL),last_data_(-1), begin_wait_(true), confidence_(0.0),
            runPlanner_(false), setup_(false), p_freq_change_(false), c_freq_change_(false), new_global_plan_(false), get_load_nem_data_cmd_(false),
            idle_state_(true),sound_play_client_("sound_play", true), move_base_client_("move_base", false) {

        as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", boost::bind(&MoveBase::executeCb, this, _1),
                                       false);

        ros::NodeHandle private_nh("~");
        ros::NodeHandle nh;

        recovery_trigger_ = PLANNING_R;

        //get some parameters that will be global to the move base node
        private_nh.param("primary_global_planner", primary_global_planner_, std::string("nem_global_planner/NemGlobalPlannerROS"));
        private_nh.param("secondary_global_planner", secondary_global_planner_, std::string("global_planner/GlobalPlanner"));
        private_nh.param("primary_local_planner", primary_local_planner_, std::string("nem_local_planner/NemLocalPlannerROS"));
        private_nh.param("secondary_local_planner", secondary_local_planner_, std::string("teb_local_planner/TebLocalPlannerROS"));
        private_nh.param("dock_local_planner", dock_local_planner_, std::string("m100_dock_local_planner/M100DockLocalPlannerROS"));
        private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
        private_nh.param("global_costmap/global_frame", global_frame_, std::string("map"));
        private_nh.param("planner_frequency", planner_frequency_, 0.0);
        private_nh.param("controller_frequency", controller_frequency_, 20.0);
        private_nh.param("planner_patience", planner_patience_, 5.0);
        private_nh.param("controller_patience", controller_patience_, 15.0);
        private_nh.param("max_planning_retries", max_planning_retries_, -1);  // disabled by default

        private_nh.param("oscillation_timeout", oscillation_timeout_, 0.0);
        private_nh.param("oscillation_distance", oscillation_distance_, 0.5);

        private_nh.param("sim_mode", sim_mode_, false);
        private_nh.param("available_confidence", available_confidence_, 0.5);
        private_nh.param("wait_for_confidence", wait_for_confidence_, 30.0);

        // parameters of make_plan service
        private_nh.param("make_plan_clear_costmap", make_plan_clear_costmap_, true);
        private_nh.param("make_plan_add_unreachable_goal", make_plan_add_unreachable_goal_, true);

        private_nh.param("nem_node_data_default_path", node_file_path_local_, std::string("/home/robot/.ros/sync_data/actual_nodes.json"));
        private_nh.param("nem_edge_data_default_path", edge_file_path_local_, std::string("/home/robot/.ros/sync_data/actual_edges.json"));

        private_nh.param("use_avoidance", use_avoidance_, false);
        //set up plan triple buffer
        //planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
        //latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
        //controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();
        planner_plan_ = new nav_msgs::Path();
        latest_plan_ = new nav_msgs::Path();
        controller_plan_ = new nav_msgs::Path();

        //set up the planner's thread
        planner_thread_ = new boost::thread(boost::bind(&MoveBase::planThread, this));

        //for commanding the base
        vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0);

        ros::NodeHandle action_nh("move_base");
        action_goal_pub_ = action_nh.advertise<navit_msgs::MoveBaseActionGoal>("goal", 1);
        recovery_status_pub_ = action_nh.advertise<navit_msgs::RecoveryStatus>("recovery_status", 1);

        //we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
        //they won't get any useful information back about its status, but this is useful for tools
        //like nav_view and rviz
        if(!sim_mode_)
            pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("pose", 1, &MoveBase::poseCB, this);
        ros::NodeHandle simple_nh("move_base_simple");
        goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1,
                                                                    boost::bind(&MoveBase::goalCB, this, _1));

        navi_error_sub_ = nh.subscribe<std_msgs::Int32>("navi_error", 1,
                                                                    boost::bind(&MoveBase::naviErrorCB, this, _1));

        navi_error_pub_ = nh.advertise<std_msgs::Int32>("navi_error", false, 1);
        navi_state_pub_ = nh.advertise<std_msgs::String>("navi_state", false, 1);
        //we'll assume the radius of the robot to be consistent with what's specified for the costmaps
        private_nh.param("local_costmap/inscribed_radius", inscribed_radius_, 0.325);
        private_nh.param("local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);
        private_nh.param("clearing_radius", clearing_radius_, circumscribed_radius_);
        private_nh.param("conservative_reset_dist", conservative_reset_dist_, 3.0);
        private_nh.param("neutral_reset_dist", neutral_reset_dist_, 1.5);
        private_nh.param("aggressive_reset_dist", aggressive_reset_dist_, 1.1);
        private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);
        private_nh.param("clearing_rotation_allowed", clearing_rotation_allowed_, true);
        private_nh.param("recovery_behavior_enabled", recovery_behavior_enabled_, true);

        //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
        //planner_costmap_ros_ = new navit_costmap::Costmap2DROS("global_costmap", tf_);
        planner_costmap_ros_ = std::make_shared<navit_costmap::Costmap2DROS>("global_costmap", *tf_, private_nh);
        planner_costmap_ros_->pause();

        //create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
        //controller_costmap_ros_ = new navit_costmap::Costmap2DROS("local_costmap", tf_);
        controller_costmap_ros_ = std::make_shared<navit_costmap::Costmap2DROS>("local_costmap", *tf_, private_nh);
        controller_costmap_ros_->pause();

        planner_plugin_ = primary_global_planner_;
        controller_plugin_ = primary_local_planner_;

        //update node and edge file service
        update_file_srv_ = nh.advertiseService("load_nem_data", &MoveBase::loadNemDataService, this);

        //若当前插件需要点边文件，则检查点边文件是否存在于目标路径下，否则挂起movebase直到文件出现在目标路径下为止
        if(planner_plugin_ == "nem_global_planner/NemGlobalPlannerROS" || controller_plugin_ == "nem_local_planner/NemLocalPlannerROS"){
            ros::Rate r(1.0);
            while(!get_load_nem_data_cmd_){
                std_msgs::Int32 error;
                error.data = 311;
                navi_error_pub_.publish(error);
                ros::spinOnce();
                ROS_WARN_ONCE("[move_base]: Wait for load nem data cmd or nem data load failed!");
                r.sleep();
            }
            ROS_INFO("[move_base]: Receive load nem data cmd, and nem data load success");
        } else
            ROS_INFO("[move_base]: Current plugin without nem");
        if(planner_plugin_ != "nem_global_planner/NemGlobalPlannerROS"){
            if (!initPlannerPlugins(planner_, planner_plugin_))
                exit(1);
            planner_costmap_ros_->start();
        }

        if(controller_plugin_ != "nem_local_planner/NemLocalPlannerROS"){
            if (!initControllerPlugins(tc_, controller_plugin_))
                exit(1);
            controller_costmap_ros_->start();
        }

        //advertise a service for getting a plan
        make_plan_srv_ = private_nh.advertiseService("make_plan", &MoveBase::planService, this);

        //advertise a service for clearing the costmaps
        clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps", &MoveBase::clearCostmapsService, this);
        clear_local_costmaps_srv_ = private_nh.advertiseService("clear_local_costmaps", &MoveBase::clearLocalCostmapsService, this);

        //switch planner plugins service
        planner_switch_srv_ = private_nh.advertiseService("planner_switch", &MoveBase::plannerSwitchService, this);

        check_plugin_srv_   = private_nh.advertiseService("check_current_plugin", &MoveBase::checkCurrentPluginService, this);

        //switch controller plugins service
        controller_switch_srv_ = private_nh.advertiseService("controller_switch", &MoveBase::controllerSwitchService,
                                                             this);
        
        clear_local_costmap_client_ = private_nh.serviceClient<std_srvs::Empty>("clear_local_costmaps");

        planner_switch_client_      = private_nh.serviceClient<navit_move_base::MoveBaseLoadPlugin>("planner_switch");

        controller_switch_client_   = private_nh.serviceClient<navit_move_base::MoveBaseLoadPlugin>("controller_switch");

        notice_update_files_srv_ = private_nh.serviceClient<navit_move_base::LoadNemData>("notice_update_nemdata");
        //if we shutdown our costmaps when we're deactivated... we'll do that now
        if (shutdown_costmaps_) {
            ROS_DEBUG_NAMED("move_base", "Stopping costmaps initially");
            planner_costmap_ros_->stop();
            controller_costmap_ros_->stop();
        }

        //load any user specified recovery behaviors, and if that fails load the defaults
        if (!loadRecoveryBehaviors(private_nh)) {
            loadDefaultRecoveryBehaviors();
        }

        //initially, we'll need to make a plan
        state_ = PLANNING;

        //we'll start executing recovery behaviors at the beginning of our list
        recovery_index_ = 0;

        //we're all set up now so we can start the action server
        as_->start();
        std_msgs::String error_code;
        error_code.data = "[move_base]: Navigation Standby";
        navi_state_pub_.publish(error_code);
        dsrv_ = new dynamic_reconfigure::Server<navit_move_base::MoveBaseConfig>(ros::NodeHandle("~"));
        dynamic_reconfigure::Server<navit_move_base::MoveBaseConfig>::CallbackType cb = boost::bind(&MoveBase::reconfigureCB,
                                                                                              this, _1, _2);
        dsrv_->setCallback(cb);
        current_planner_plugin_ = planner_plugin_;
        current_controller_plugin_ = controller_plugin_;
        ROS_INFO("[move_base]: Navigation Standby, current planner plugin = %s, controller plugin = %s", planner_plugin_.c_str(), controller_plugin_.c_str());
    }

    void MoveBase::reconfigureCB(navit_move_base::MoveBaseConfig &config, uint32_t level) {
        boost::recursive_mutex::scoped_lock l(configuration_mutex_);

        //The first time we're called, we just want to make sure we have the
        //original configuration
        if (!setup_) {
            last_config_ = config;
            default_config_ = config;
            setup_ = true;
            return;
        }

        if (config.restore_defaults) {
            config = default_config_;
            //if someone sets restore defaults on the parameter server, prevent looping
            config.restore_defaults = false;
        }

        if (planner_frequency_ != config.planner_frequency) {
            planner_frequency_ = config.planner_frequency;
            p_freq_change_ = true;
        }

        if (controller_frequency_ != config.controller_frequency) {
            controller_frequency_ = config.controller_frequency;
            c_freq_change_ = true;
        }

        planner_patience_ = config.planner_patience;
        controller_patience_ = config.controller_patience;
        max_planning_retries_ = config.max_planning_retries;
        conservative_reset_dist_ = config.conservative_reset_dist;

        recovery_behavior_enabled_ = config.recovery_behavior_enabled;
        clearing_rotation_allowed_ = config.clearing_rotation_allowed;
        shutdown_costmaps_ = config.shutdown_costmaps;

        oscillation_timeout_ = config.oscillation_timeout;
        oscillation_distance_ = config.oscillation_distance;
        if (config.base_global_planner != last_config_.base_global_planner) {
            boost::shared_ptr<navit_core::GlobalPlanner> old_planner = planner_;
            //initialize the global planner
            ROS_INFO("Loading global planner %s", config.base_global_planner.c_str());
            try {
                planner_ = bgp_loader_.createInstance(config.base_global_planner);

                // wait for the current planner to finish planning
                boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);

                // Clean up before initializing the new planner
                planner_plan_->poses.clear();
                latest_plan_->poses.clear();
                controller_plan_->poses.clear();
                resetState();
                planner_->initialize(bgp_loader_.getName(config.base_global_planner), planner_costmap_ros_);

                lock.unlock();
            } catch (const pluginlib::PluginlibException &ex) {
                ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s", config.base_global_planner.c_str(), ex.what());
                planner_ = old_planner;
                config.base_global_planner = last_config_.base_global_planner;
            }
        }

        if (config.base_local_planner != last_config_.base_local_planner) {
            boost::shared_ptr<navit_core::Controller> old_planner = tc_;
            //create a local planner
            try {
                tc_ = blp_loader_.createInstance(config.base_local_planner);
                // Clean up before initializing the new planner
                planner_plan_->poses.clear();
                latest_plan_->poses.clear();
                //controller_plan_->poses.clear();
                resetState();
                //tc_->initialize(blp_loader_.getName(config.base_local_planner), &tf_, controller_costmap_ros_);
                tc_->initialize(blp_loader_.getName(config.base_local_planner), tf_, controller_costmap_ros_);
            } catch (const pluginlib::PluginlibException &ex) {
                ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s", config.base_local_planner.c_str(), ex.what());
                tc_ = old_planner;
                config.base_local_planner = last_config_.base_local_planner;
            }
        }

        make_plan_clear_costmap_ = config.make_plan_clear_costmap;
        make_plan_add_unreachable_goal_ = config.make_plan_add_unreachable_goal;

        last_config_ = config;
    }

    bool MoveBase::initPlannerPlugins(boost::shared_ptr<navit_core::GlobalPlanner> &planner, const std::string &global_planner,
                                const std::string &node_file_path, const std::string &edge_file_path) {
        boost::shared_ptr<navit_core::GlobalPlanner> old_planner = planner_;
        //initialize the global planner
        try {
            planner_ = bgp_loader_.createInstance(global_planner);
            boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
            // Clean up before initializing the new planner
            planner_plan_->poses.clear();
            latest_plan_->poses.clear();
            controller_plan_->poses.clear();
            resetState();
            planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
            planner_plugin_ = global_planner;
            if(planner_plugin_ == "nem_global_planner/NemGlobalPlannerROS"){
                if(!planner_->initNemData(node_file_path, edge_file_path))
                    return false;
            }
        } catch (const pluginlib::PluginlibException &ex) {
            ROS_FATAL(
                    "Failed to switch the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s",
                    global_planner.c_str(), ex.what());
            planner_ = old_planner;
            return false;
        }
        return true;
    }

    bool MoveBase::initControllerPlugins(boost::shared_ptr<navit_core::Controller> &controller,const std::string &local_planner,
                                        const std::string &node_file_path, const std::string &edge_file_path) {
        boost::shared_ptr<navit_core::Controller> old_controller = tc_;
        //create a local planner
        try {
            tc_ = blp_loader_.createInstance(local_planner);
            boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
            // Clean up before initializing the new planner
            planner_plan_->poses.clear();
            latest_plan_->poses.clear();
            controller_plan_->poses.clear();
            resetState();
            //tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
            tc_->initialize(blp_loader_.getName(local_planner), tf_, controller_costmap_ros_);
            controller_plugin_ = local_planner;
            if(controller_plugin_ == "nem_local_planner/NemLocalPlannerROS"){
                if(!tc_->initNemData(node_file_path, edge_file_path))
                    return false;
            }
        } catch (const pluginlib::PluginlibException &ex) {
            ROS_FATAL(
                    "Failed to switch the %s controller, are you sure it is properly registered and that the containing library is built? Exception: %s",
                    local_planner.c_str(), ex.what());
            tc_ = old_controller;
            return false;
        }
        return true;
    }

    void MoveBase::poseCB(const geometry_msgs::PoseStamped msg) {
        confidence_ = msg.pose.position.z;
    }

    void MoveBase::goalCB(const geometry_msgs::PoseStamped::ConstPtr &goal) {
        ROS_DEBUG_NAMED("move_base",
                        "In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
        navit_msgs::MoveBaseActionGoal action_goal;
        action_goal.header.stamp = ros::Time::now();
        action_goal.goal.target_pose = *goal;

        action_goal_pub_.publish(action_goal);
    }

    void MoveBase::naviErrorCB(const std_msgs::Int32::ConstPtr &data){
        if (data->data != last_data_) {
            error_code_.data = data->data;
            std_msgs::String error_code;
            switch (data->data) {
                default:
                    error_code.data = "导航错误码异常";
                    ROS_ERROR("[move base]: navi status error!");
                    break;
                case 111:
                    error_code.data = "[nem_global_planner]: Node or edge file error";
                    break;
                case 112:
                    error_code.data = "[nem_global_planner]: Goal is not on any node or edge. Without goal id";
                    break;
                case 113:
                    error_code.data = "[nem_global_planner]: Goal is not on any node or edge. With goal id";
                    break;
                case 114:
                    error_code.data = "[nem_global_planner]: Aim path is empty, maybe there is not any available edges";
                    break;
                case 115:
                    error_code.data = "[nem_global_planner]: There is only one target node and it is too faraway that can not reached!";
                    break;
                case 116:
                    error_code.data = "[nem_global_planner]: The distance between two points is too long when getting the path";
                    break;
                case 117:
                    error_code.data = "[nem_global_planner]: Although the robot is on th edge with only one target node, but it is too faraway";
                    break;
                case 118:
                    error_code.data = "[nem_global_planner]: Path is empty, this should not be happened";
                    break;
                case 119:
                    error_code.data =
                        "[nem_global_planner]: The path node is less than two, this should not be happened";
                    break;
                case 120:
                    error_code.data = "[nem_global_planner]: Now the robot is not on any node or path";
                    break;
                case 211:
                    error_code.data = "[nem_local_planner]: Node or edge file error";
                    break;
                case 212:
                    error_code.data = "[nem_local_planner]: Avoiding obstacles";
                    break;
                case 213:
                    error_code.data = "[nem_local_planner]: Avoiding vision obstacles";
                case 214:
                    error_code.data = "[nem_local_planner]: Robot stop fail";
                case 311:
                    error_code.data = "[move_base]: Wait for load nem data cmd!";
                    break;
                case 312:
                    error_code.data = "[move_base]: Planner plugin is error";
                    break;
                case 313:
                    error_code.data = "[move_base]: Global costmap is error";
                    break;
                case 314:
                    error_code.data = "[move_base]: Planner makePlan call failed";
                    break;
                case 315:
                    error_code.data = "[move_base]: Confidence is too low";
                    break;
            }
            navi_state_pub_.publish(error_code);
            ROS_INFO("current state = %d", data->data);
            ROS_INFO("last state = %d", last_data_);
        }
        last_data_ = data->data;
    }

    void MoveBase::clearCostmapWindows(double size_x, double size_y) {
        geometry_msgs::PoseStamped global_pose;

        //clear the planner's costmap
        getRobotPose(global_pose, planner_costmap_ros_.get());

        std::vector<geometry_msgs::Point> clear_poly;
        double x = global_pose.pose.position.x;
        double y = global_pose.pose.position.y;
        geometry_msgs::Point pt;

        pt.x = x - size_x / 2;
        pt.y = y - size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x + size_x / 2;
        pt.y = y - size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x + size_x / 2;
        pt.y = y + size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x - size_x / 2;
        pt.y = y + size_y / 2;
        clear_poly.push_back(pt);

        planner_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, navit_costmap::FREE_SPACE);

        //clear the controller's costmap
        getRobotPose(global_pose, controller_costmap_ros_.get());

        clear_poly.clear();
        x = global_pose.pose.position.x;
        y = global_pose.pose.position.y;

        pt.x = x - size_x / 2;
        pt.y = y - size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x + size_x / 2;
        pt.y = y - size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x + size_x / 2;
        pt.y = y + size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x - size_x / 2;
        pt.y = y + size_y / 2;
        clear_poly.push_back(pt);

        controller_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, navit_costmap::FREE_SPACE);
    }

    bool MoveBase::clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
        //clear the costmaps
        boost::unique_lock<navit_costmap::Costmap2D::mutex_t> lock_controller(
                *(controller_costmap_ros_->getCostmap()->getMutex()));
        controller_costmap_ros_->resetLayers();

        boost::unique_lock<navit_costmap::Costmap2D::mutex_t> lock_planner(
                *(planner_costmap_ros_->getCostmap()->getMutex()));
        planner_costmap_ros_->resetLayers();
        return true;
    }

    bool MoveBase::clearLocalCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
        //clear the costmaps
        boost::unique_lock<navit_costmap::Costmap2D::mutex_t> lock_controller(
                *(controller_costmap_ros_->getCostmap()->getMutex()));
        controller_costmap_ros_->resetLayers();
        return true;
    }

    bool MoveBase::loadNemDataService(navit_move_base::LoadNemDataRequest &req, navit_move_base::LoadNemDataResponse &res){
        ROS_INFO("[move_base]: Receive load nem data cmd,\n node file path = %s,\n edge file path = %s", req.node_file_path.c_str(), req.edge_file_path.c_str());
        const std::string node_file_name = req.node_file_path;
        const std::string edge_file_name = req.edge_file_path;

        node_file_path_local_ = node_file_name;
        edge_file_path_local_ = edge_file_name;

        if (navi_support_.FileExist(node_file_name) && navi_support_.FileExist(edge_file_name)) {
            ROS_INFO("[move_base]: Nem data is in target path");
            if(!reLoadJson(node_file_name, edge_file_name)){
                ROS_ERROR("Load nem data failed! 点边文件加载失败");
                res.success = false;
                res.message = "Load nem data failed! 点边文件加载失败";
                return true;
            }
            navit_move_base::LoadNemData srv;
            srv.request.node_file_path = node_file_name;
            srv.request.edge_file_path = edge_file_name;
            if (!notice_update_files_srv_.call(srv)) {
                ROS_ERROR("Router node Load nem data failed! 点边文件加载失败");
                res.success = false;
                res.message = "Load nem data failed! 点边文件加载失败";
            }
            res.success = true;
            res.message = "Load nem data success! 点边文件加载成功";
            get_load_nem_data_cmd_ = true;
            navi_support_.GetNemVisualData(node_file_name, edge_file_name);
            return true;
        } else {
            res.success = false;
            res.message = "Nem data maybe is empty! 点边文件中存在一个两个为空现象";
            ROS_ERROR("[move_base]: Node path = %s, edge path = %s. But nem data maybe is empty! 点边文件中存在一个或两个为空现象", node_file_name.c_str(), edge_file_name.c_str());
            return true;
        }
    }

    bool MoveBase::reLoadJson(const std::string &node_file_path, const std::string &edge_file_path){
        if (!initPlannerPlugins(planner_, "nem_global_planner/NemGlobalPlannerROS", node_file_path, edge_file_path)){
            ROS_ERROR("[move_base]: Init nem global planner failed");
            return false;
        }
        planner_costmap_ros_->start();
        if (!initControllerPlugins(tc_, "nem_local_planner/NemLocalPlannerROS", node_file_path, edge_file_path)){
            ROS_ERROR("[move_base]: Init nem local planner failed");
            return false;
        }
        controller_costmap_ros_->start();
        return true;
    }

    bool MoveBase::checkCurrentPluginService(navit_move_base::CheckCurrentPluginRequest &req,
                                             navit_move_base::CheckCurrentPluginResponse &res) {
        if(req.check_current_plugin){
            res.global_planner_plugin   = current_planner_plugin_;
            res.local_controller_plugin = current_controller_plugin_;
        }
        return true;
    }

    bool MoveBase::plannerSwitchService(navit_move_base::MoveBaseLoadPluginRequest &req, navit_move_base::MoveBaseLoadPluginResponse &res) {
        ROS_INFO("[move_base]: Receive switch planner plugin cmd, cmd.plugin name = %s", req.plugin_name.c_str());
        std::string plugin_name;
        if(req.plugin_name == "primary_global_planner"){
            plugin_name = primary_global_planner_;
        } else if(req.plugin_name == "secondary_global_planner"){
            plugin_name = secondary_global_planner_;
        } else
            plugin_name = req.plugin_name;
        ROS_INFO("[move_base]: Current target planner plugin name = %s", plugin_name.c_str());
        if (idle_state_) {
            if (initPlannerPlugins(planner_, plugin_name, node_file_path_local_, edge_file_path_local_)) {
                ROS_INFO("[move_base]: Switch planner plugin success, current planner plugin = %s, controller plugin = %s", planner_plugin_.c_str(), controller_plugin_.c_str());
                res.error_code = 0;
                res.message = "planner switch success!";
                current_planner_plugin_ = plugin_name;
                return true;
            } else {
                ROS_ERROR("[move_base]: Switch planner plugin failed, current planner plugin = %s, controller plugin = %s", planner_plugin_.c_str(), controller_plugin_.c_str());
                res.error_code = -1;
                res.message = "planner switch failed!";
                return false;
            }
        } else {
            ROS_WARN("[move_base]: Movebase is busy now, can not switch planner plugin");
            res.error_code = 1;
            res.message = "now move base is working, cannot switch plugins!";
            return false;
        }
    }

    bool MoveBase::controllerSwitchService(navit_move_base::MoveBaseLoadPluginRequest &req, navit_move_base::MoveBaseLoadPluginResponse &res) {
        ROS_INFO("[move_base]: Receive switch controller plugin cmd, cmd.plugin name = %s", req.plugin_name.c_str());
        std::string plugin_name;
        if(req.plugin_name == "primary_local_planner"){
            plugin_name = primary_local_planner_;
        } else if(req.plugin_name == "secondary_local_planner"){
            plugin_name = secondary_local_planner_;
        } else if(req.plugin_name == "dock_local_planner"){
            plugin_name = dock_local_planner_;
        } else
            plugin_name = req.plugin_name;
        ROS_INFO("[move_base]: Current target controller plugin name = %s", plugin_name.c_str());
        if (idle_state_) {

            if (initControllerPlugins(tc_, plugin_name, node_file_path_local_, edge_file_path_local_)) {
                ROS_INFO("[move_base]: Switch controller plugin success, current planner plugin = %s, controller plugin = %s", planner_plugin_.c_str(), controller_plugin_.c_str());
                res.error_code = 0;
                res.message = "controller switch success!";
                current_controller_plugin_ = plugin_name;
                return true;
            } else {
                ROS_ERROR("[move_base]: Switch controller plugin failed, current planner plugin = %s, controller plugin = %s", planner_plugin_.c_str(), controller_plugin_.c_str());
                res.error_code = -1;
                res.message = "controller switch failed!";
                return false;
            }
        } else {
            ROS_WARN("[move_base]: Movebase is busy now, can not switch controller plugin");
            res.error_code = 1;
            res.message = "now move base is working, cannot switch plugins!";
            return false;
        }

    }

    bool MoveBase::planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp) {
        ROS_INFO("[move_base]: Receive makePlan service");
        ROS_INFO("[move_base][planService]: \n start.frame_id = %s \n start.x = %f \n start.y = %f \n start.z = %f \n start.yaw = %f \n"
                 "\n"
                 "goal.frame_id = %s \n goal.x = %f \n goal.y = %f \n goal.z = %f \n goal.yaw = %f",
                 req.goal.header.frame_id.c_str(), req.goal.pose.position.x, req.goal.pose.position.y, req.goal.pose.position.z,
                 tf::getYaw(req.goal.pose.orientation));
        if (as_->isActive()) {
            std_msgs::Int32 error;
            error.data = 312;
            navi_error_pub_.publish(error);
            ROS_ERROR("[move_base]: Planner plugin is error");
            return false;
        }
        //make sure we have a costmap for our planner
        if (planner_costmap_ros_ == NULL) {
            std_msgs::Int32 error;
            error.data = 313;
            navi_error_pub_.publish(error);
            ROS_ERROR("[move_base]: Global costmap is error");
            return false;
        }

        geometry_msgs::PoseStamped start;
        //if the user does not specify a start pose, identified by an empty frame id, then use the robot's pose
//        if (req.start.header.frame_id.empty()) {
//            geometry_msgs::PoseStamped global_pose;
//            if (!getRobotPose(global_pose, planner_costmap_ros_)) {
//                ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
//                return false;
//            }
//            start = global_pose;
//        } else {
//            start = req.start;
//        }
        start = req.start;
        if (make_plan_clear_costmap_) {
            //update the copy of the costmap the planner uses
            clearCostmapWindows(2 * clearing_radius_, 2 * clearing_radius_);
        }

        //first try to make a plan to the exact desired goal
        //std::vector<geometry_msgs::PoseStamped> global_plan;
        nav_msgs::Path global_plan = planner_->makePlan(start, req.goal);
        if (global_plan.poses.empty()) {
            std_msgs::Int32 error;
            error.data = 314;
            navi_error_pub_.publish(error);
            ROS_ERROR("[move_base]: Planner makePlan call failed");
            return false;
        }

        //copy the plan into a message to send out
        resp.plan.poses.resize(global_plan.poses.size());
        for (unsigned int i = 0; i < global_plan.poses.size(); ++i) {
            resp.plan.poses[i] = global_plan.poses[i];
        }
        ROS_INFO("[move_base]: Planner service call success");
        return true;
    }

    MoveBase::~MoveBase() {
        recovery_behaviors_.clear();

        delete dsrv_;

        if (as_ != NULL)
            delete as_;

        if (planner_costmap_ros_ != NULL)
            //delete planner_costmap_ros_;
            planner_costmap_ros_.reset();

        if (controller_costmap_ros_ != NULL)
            //delete controller_costmap_ros_;
            controller_costmap_ros_.reset();

        planner_thread_->interrupt();
        planner_thread_->join();

        delete planner_thread_;

        delete planner_plan_;
        delete latest_plan_;
        delete controller_plan_;

        planner_.reset();
        tc_.reset();
    }


    void MoveBase::movebaseClearLocalCostmap()
    {
        std_srvs::Empty msg;
        clear_local_costmap_client_.call(msg);
        ROS_INFO("[move_base]: Clear local costmap");
    }

    bool MoveBase::makePlan(const geometry_msgs::PoseStamped &goal, nav_msgs::Path &plan) {
        boost::unique_lock<navit_costmap::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));

        //make sure to set the plan to be empty initially
        plan.poses.clear();

        //since this gets called on handle activate
        if (planner_costmap_ros_ == NULL) {
            ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
            return false;
        }

        //get the starting pose of the robot
        geometry_msgs::PoseStamped global_pose;
        if (!getRobotPose(global_pose, planner_costmap_ros_.get())) {
            ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
            return false;
        }

        const geometry_msgs::PoseStamped &start = global_pose;
        plan = planner_->makePlan(start, goal);

        //if the planner fails or returns a zero length plan, planning failed
        if ( plan.poses.empty()) {
            //ROS_DEBUG_NAMED("move_base","Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
            return false;
        }

        return true;
    }

    void MoveBase::publishZeroVelocity() {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        vel_pub_.publish(cmd_vel);
    }

    bool MoveBase::isQuaternionValid(const geometry_msgs::Quaternion &q) {
        //first we need to check if the quaternion has nan's or infs
        if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)) {
            ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
            return false;
        }

        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

        //next, we need to check if the length of the quaternion is close to zero
        if (tf_q.length2() < 1e-6) {
            ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
            return false;
        }

        //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
        tf_q.normalize();

        tf2::Vector3 up(0, 0, 1);

        double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

        if (fabs(dot - 1) > 1e-3) {
            ROS_ERROR(
                    "Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
            return false;
        }

        return true;
    }

    geometry_msgs::PoseStamped MoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped &goal_pose_msg) {
        std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
        geometry_msgs::PoseStamped goal_pose, global_pose;
        goal_pose = goal_pose_msg;

        //just get the latest available transform... for accuracy they should send
        //goals in the frame of the planner
        goal_pose.header.stamp = ros::Time();

        try {
            tf_->transform(goal_pose_msg, global_pose, global_frame);
            global_pose.header.seq = goal_pose_msg.header.seq;
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
                     goal_pose.header.frame_id.c_str(), global_frame.c_str(), ex.what());
            return goal_pose_msg;
        }

        return global_pose;
    }

    void MoveBase::wakePlanner(const ros::TimerEvent &event) {
        // we have slept long enough for rate
        planner_cond_.notify_one();
    }

    void MoveBase::planThread() {
        ROS_DEBUG_NAMED("move_base_plan_thread", "Starting planner thread...");
        ros::NodeHandle n;
        ros::Timer timer;
        ros::Rate r(10);
        bool wait_for_wake = false;
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        while (n.ok()) {
            //check if we should run the planner (the mutex is locked)
            while (wait_for_wake || !runPlanner_) {
                //if we should not be running the planner then suspend this thread
                ROS_DEBUG_NAMED("move_base_plan_thread", "Planner thread is suspending");
                planner_cond_.wait(lock);
                wait_for_wake = false;
            }
            ros::Time start_time = ros::Time::now();

            //time to plan! get a copy of the goal and unlock the mutex
            geometry_msgs::PoseStamped temp_goal = planner_goal_;
            lock.unlock();
            ROS_DEBUG_NAMED("move_base_plan_thread", "Planning...");

            //run planner
            planner_plan_->poses.clear();
            bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);

            if (gotPlan) {
                ROS_DEBUG_NAMED("move_base_plan_thread", "Got Plan with %zu points!", planner_plan_->poses.size());
                //pointer swap the plans under mutex (the controller will pull from latest_plan_)
                //std::vector<geometry_msgs::PoseStamped> *temp_plan = planner_plan_;
                nav_msgs::Path *temp_plan = planner_plan_;

                lock.lock();
                planner_plan_ = latest_plan_;
                latest_plan_ = temp_plan;
                last_valid_plan_ = ros::Time::now();
                planning_retries_ = 0;
                new_global_plan_ = true;

                ROS_DEBUG_NAMED("move_base_plan_thread", "Generated a plan from the base_global_planner");

                //make sure we only start the controller if we still haven't reached the goal
                if (runPlanner_)
                    state_ = CONTROLLING;
                if (planner_frequency_ <= 0)
                    runPlanner_ = false;
                lock.unlock();
            }
                //if we didn't get a plan and we are in the planning state (the robot isn't moving)
            else if (state_ == PLANNING) {
                ROS_DEBUG_NAMED("move_base_plan_thread", "No Plan...");
                // 规划时间冗余默认1.0s
                ros::Time attempt_end = last_valid_plan_ + ros::Duration(planner_patience_);

                //check if we've tried to make a plan for over our time limit or our maximum number of retries
                //issue #496: we stop planning when one of the conditions is true, but if max_planning_retries_
                //is negative (the default), it is just ignored and we have the same behavior as ever
                lock.lock();
                planning_retries_++;
                // max_planning_retries_ 默认为-1，-1表示无限次
                if (runPlanner_ &&
                    (ros::Time::now() > attempt_end || planning_retries_ > uint32_t(max_planning_retries_))) {
                    // 全局规划失败后进入失败状态（不进行Clear）
                    state_ = FAILING;
                    runPlanner_ = false;  // proper solution for issue #523
                    publishZeroVelocity();
                    recovery_trigger_ = PLANNING_R;
                } else {
                    //当规划频率为0时，即只规划一次，则在规划失败时进行sleep，防止在while中高频打印规划失败的日志 （默认为0.0）
                    if(planner_frequency_ <= 0)
                        r.sleep();
                }

                lock.unlock();
            }

            //take the mutex for the next iteration
            lock.lock();

            //setup sleep interface if needed
            if (planner_frequency_ > 0) {
                ros::Duration sleep_time = (start_time + ros::Duration(1.0 / planner_frequency_)) - ros::Time::now();
                if (sleep_time > ros::Duration(0.0)) {
                    wait_for_wake = true;
                    timer = n.createTimer(sleep_time, &MoveBase::wakePlanner, this);
                }
            }
        }
    }

//    bool MoveBase::checkGoalAttribute(const geometry_msgs::PoseStamped &goal, std::string &aborted_content) {
//        if (goal.pose.position.z > 0) {
//            if (initControllerPlugins(tc_, free_navi_controller_)) {
//                ROS_WARN("[move_base]: Switch to free navigation");
//                return true;
//            } else {
//                ROS_ERROR("[move_base]: Switch free navigation failed!");
//                aborted_content = "Aborting on goal because it was sent with an invalid controller";
//                return false;
//            }
//        } else if (goal.pose.position.z < 0) {
//            if (initControllerPlugins(tc_, point_to_point_navi_controller_)) {
//                ROS_WARN("[move_base]: Switch to point-to-point navigation");
//                return true;
//            } else {
//                ROS_ERROR("[move_base]: Switch point-to-point navigation failed!");
//                aborted_content = "Aborting on goal because it was sent with an invalid controller";
//                return false;
//            }
//        } else {
//            ROS_WARN("[move_base]: Not receive commond of switch planner ");
//            return true;
//        }
//    }

    void MoveBase::executeCb(const navit_msgs::MoveBaseGoalConstPtr &move_base_goal) {
        ros::Time start_time = ros::Time::now();
        ROS_INFO("[move_base]: Receive a goal from move_base action\n goal.x = %f\n goal.y = %f\n goal.z = %f\n goal.theta = %f",
            move_base_goal->target_pose.pose.position.x, move_base_goal->target_pose.pose.position.y, move_base_goal->target_pose.pose.position.z,
                 tf::getYaw(move_base_goal->target_pose.pose.orientation));
        idle_state_ = false;
        //forbid receive new goal if robot in avoidance abs atate
        // if (current_controller_plugin_ == secondary_local_planner_ && ) {
        //     as_->setAborted(navit_msgs::MoveBaseResult(),
        //                     "Robot in avoidance abs state, We cannot make global plan.");
        // }
        // 接受到新的目标点后先清理局部costmap，防止误避障
        movebaseClearLocalCostmap();
        if (!isQuaternionValid(move_base_goal->target_pose.pose.orientation)) {
            as_->setAborted(navit_msgs::MoveBaseResult(),
                            "Aborting on goal because it was sent with an invalid quaternion");
            idle_state_ = true;
            std_msgs::String error_code;
            error_code.data = "[move_base]: Navigation Standby";
            navi_state_pub_.publish(error_code);
            return;
        }

        geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_base_goal->target_pose);

//        std::string aborted_content;
//        if (!checkGoalAttribute(goal, aborted_content)) {
//            as_->setAborted(move_base_msgs::MoveBaseResult(), aborted_content);
//            idle_state_ = true;
//            return;
//        }
        publishZeroVelocity();
        //we have a goal so start the planner
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        planner_goal_ = goal;
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

        current_goal_pub_.publish(goal);

        ros::Rate r(controller_frequency_);
        if (shutdown_costmaps_) {
            ROS_DEBUG_NAMED("move_base", "Starting up costmaps that were shut down previously");
            planner_costmap_ros_->start();
            controller_costmap_ros_->start();
        }

        //we want to make sure that we reset the last time we had a valid plan and control
        last_valid_control_ = ros::Time::now();
        last_valid_plan_ = ros::Time::now();
        last_oscillation_reset_ = ros::Time::now();
        last_wait_confidence_ = ros::Time::now();
        last_rotate_recovery_ = ros::Time::now();


        planning_retries_ = 0;

        ros::NodeHandle n;
        while (n.ok()) {
            ros::spinOnce();
            if (!sim_mode_ && confidence_ < available_confidence_) {
                publishZeroVelocity();
                if(begin_wait_)
                {
                    last_wait_confidence_ = ros::Time::now();
                    begin_wait_ = false;
                }
                ros::Time attempt_end = last_wait_confidence_ + ros::Duration(wait_for_confidence_);
                if (ros::Time::now() > attempt_end) {
                    //TODO: pub the navigation failed status to task
                    runPlanner_ = false;
                    as_->setAborted(navit_msgs::MoveBaseResult(),
                                    "Failed to find a valid control. Because the confidence is too low");
                    ROS_ERROR("Wait for confidence time out!");
                    begin_wait_ = true;
                    idle_state_ = true;
                    return;
                }
                std_msgs::Int32 msg;
                msg.data = 315;
                navi_error_pub_.publish(msg);
                ROS_WARN_THROTTLE(1, "[move_base]: Now, the confidence(%f) is too low, and waiting for localization recovery",
                         confidence_);
                r.sleep();
                continue;
            }
            begin_wait_ = true;

            if (c_freq_change_) {
                ROS_INFO("Setting controller frequency to %.2f", controller_frequency_);
                r = ros::Rate(controller_frequency_);
                c_freq_change_ = false;
            }
            if (as_->isPreemptRequested()) {
                if (as_->isNewGoalAvailable()) {
                    start_time = ros::Time::now();
                    ROS_INFO("[move_base]: Receive a preempt requested goal from move_base action\n goal.x = %f\n goal.y = %f\n goal.z = %f\n goal.theta = %f",
                             move_base_goal->target_pose.pose.position.x, move_base_goal->target_pose.pose.position.y, move_base_goal->target_pose.pose.position.z,
                             tf::getYaw(move_base_goal->target_pose.pose.orientation));
                    //if we're active and a new goal is available, we'll accept it, but we won't shut anything down
                    navit_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();
                    if (!isQuaternionValid(new_goal.target_pose.pose.orientation)) {
                        as_->setAborted(navit_msgs::MoveBaseResult(),
                                        "Aborting on goal because it was sent with an invalid quaternion");
                        idle_state_ = true;
                        std_msgs::String error_code;
                        error_code.data = "[move_base]: Navigation Standby";
                        navi_state_pub_.publish(error_code);
                        return;
                    }
                    goal = goalToGlobalFrame(new_goal.target_pose);
//                    std::string aborted_content;
//                    if (!checkGoalAttribute(goal, aborted_content)) {
//                        as_->setAborted(move_base_msgs::MoveBaseResult(), aborted_content);
//                        idle_state_ = true;
//                        return;
//                    }
                    //we'll make sure that we reset our state for the next execution cycle
                    recovery_index_ = 0;
                    state_ = PLANNING;

                    //we have a new goal so make sure the planner is awake
                    lock.lock();
                    planner_goal_ = goal;
                    runPlanner_ = true;
                    planner_cond_.notify_one();
                    lock.unlock();

                    //publish the goal point to the visualizer
                    ROS_DEBUG_NAMED("move_base", "move_base has received a goal of x: %.2f, y: %.2f",
                                    goal.pose.position.x, goal.pose.position.y);
                    current_goal_pub_.publish(goal);

                    //make sure to reset our timeouts and counters
                    last_valid_control_ = ros::Time::now();
                    last_valid_plan_ = ros::Time::now();
                    last_oscillation_reset_ = ros::Time::now();
                    planning_retries_ = 0;
                } else {
                    ROS_INFO("[move_base]: Navigation action was stopped");
                    //if we've been preempted explicitly we need to shut things down
                    resetState();

                    //notify the ActionServer that we've successfully preempted
                    ROS_DEBUG_NAMED("move_base", "Move base preempting the current goal");
                    as_->setPreempted();

                    //we'll actually return from execute after preempting
                    idle_state_ = true;
                    return;
                }
            }

            //we also want to check if we've changed global frames because we need to transform our goal pose
            if (goal.header.frame_id != planner_costmap_ros_->getGlobalFrameID()) {
                goal = goalToGlobalFrame(goal);

                //we want to go back to the planning state for the next execution cycle
                recovery_index_ = 0;
                state_ = PLANNING;

                //we have a new goal so make sure the planner is awake
                lock.lock();
                planner_goal_ = goal;
                runPlanner_ = true;
                planner_cond_.notify_one();
                lock.unlock();

                //publish the goal point to the visualizer
                ROS_DEBUG_NAMED("move_base",
                                "The global frame for move_base has changed, new frame: %s, new goal position x: %.2f, y: %.2f",
                                goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y);
                current_goal_pub_.publish(goal);

                //make sure to reset our timeouts and counters
                last_valid_control_ = ros::Time::now();
                last_valid_plan_ = ros::Time::now();
                last_oscillation_reset_ = ros::Time::now();
                planning_retries_ = 0;
            }

            //for timing that gives real time even in simulation
            ros::WallTime start = ros::WallTime::now();

            //the real work on pursuing a goal is done here
            bool done = executeCycle(goal);

            //if we're done, then we'll return from execute
            if (done) {
                idle_state_ = true;
                ros::Duration time_cost = ros::Time::now() - start_time;
                ROS_INFO("[move_base]: time cost = %f s", time_cost.toSec());
                return;
            }

            //check if execution of the goal has completed in some way

            ros::WallDuration t_diff = ros::WallTime::now() - start;
            ROS_DEBUG_NAMED("move_base", "Full control cycle time: %.9f\n", t_diff.toSec());

            r.sleep();
            //make sure to sleep for the remainder of our cycle time
            if (r.cycleTime() > ros::Duration(1 / controller_frequency_) && state_ == CONTROLLING)
                ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds",
                         controller_frequency_, r.cycleTime().toSec());
        }

        //wake up the planner thread so that it can exit cleanly
        lock.lock();
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

        //if the node is killed then we'll abort and return
        as_->setAborted(navit_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
        idle_state_ = true;
        std_msgs::String error_code;
        error_code.data = "[move_base]: Navigation Standby";
        navi_state_pub_.publish(error_code);
        return;
    }

    double MoveBase::distance(const geometry_msgs::PoseStamped &p1, const geometry_msgs::PoseStamped &p2) {
        return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
    }

    bool MoveBase::executeCycle(geometry_msgs::PoseStamped &goal) {
        boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
        //we need to be able to publish velocity commands
        geometry_msgs::Twist cmd_vel;

        //update feedback to correspond to our curent position
        geometry_msgs::PoseStamped global_pose;
        getRobotPose(global_pose, planner_costmap_ros_.get());
        const geometry_msgs::PoseStamped &current_position = global_pose;

        //push the feedback out
        navit_msgs::MoveBaseFeedback feedback;
        feedback.base_position = current_position;
        as_->publishFeedback(feedback);

        //check to see if we've moved far enough to reset our oscillation timeout
        if (distance(current_position, oscillation_pose_) >= oscillation_distance_) {
            last_oscillation_reset_ = ros::Time::now();
            oscillation_pose_ = current_position;

            //if our last recovery was caused by oscillation, we want to reset the recovery index
            if (recovery_trigger_ == OSCILLATION_R)
                recovery_index_ = 0;
        }

        //check that the observation buffers for the costmap are current, we don't want to drive blind
        if (!controller_costmap_ros_->isCurrent()) {
            ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",
                     ros::this_node::getName().c_str());
            publishZeroVelocity();
            return false;
        }

        //if we have a new plan then grab it and give it to the controller
        if (new_global_plan_) {
            //make sure to set the new plan flag to false
            new_global_plan_ = false;

            ROS_DEBUG_NAMED("move_base", "Got a new plan...swap pointers");

            //do a pointer swap under mutex
            //std::vector<geometry_msgs::PoseStamped> *temp_plan = controller_plan_;
            nav_msgs::Path *temp_plan = controller_plan_;

            boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
            controller_plan_ = latest_plan_;
            latest_plan_ = temp_plan;
            lock.unlock();
            ROS_DEBUG_NAMED("move_base", "pointers swapped!");

            if (!tc_->setPlan(*controller_plan_)) {
                //ABORT and SHUTDOWN COSTMAPS
                ROS_ERROR("Failed to pass global plan to the controller, aborting.");
                resetState();

                //disable the planner thread
                lock.lock();
                runPlanner_ = false;
                lock.unlock();

                as_->setAborted(navit_msgs::MoveBaseResult(), "Failed to pass global plan to the controller.");
                std_msgs::String error_code;
                error_code.data = "[move_base]: Navigation Standby";
                navi_state_pub_.publish(error_code);
                return true;
            }

            //make sure to reset recovery_index_ since we were able to find a valid plan
            if (recovery_trigger_ == PLANNING_R)
                recovery_index_ = 0;
        }
        //the move_base state machine, handles the control logic for navigation
        switch (state_) {
            //if we are in a planning state, then we'll attempt to make a plan
            case PLANNING: {
                boost::recursive_mutex::scoped_lock lock(planner_mutex_);
                runPlanner_ = true;
                planner_cond_.notify_one();
            }
                ROS_DEBUG_NAMED("move_base", "Waiting for plan, in the planning state.");
                break;

                //if we're controlling, we'll attempt to find valid velocity commands
            case CONTROLLING:
                ROS_DEBUG_NAMED("move_base", "In controlling state.");

                //check to see if we've reached our goal
                if (tc_->isGoalReached()) {
                    ROS_DEBUG_NAMED("move_base", "Goal reached!");
                    resetState();

                    //disable the planner thread
                    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
                    runPlanner_ = false;
                    lock.unlock();

                    as_->setSucceeded(navit_msgs::MoveBaseResult(), "Goal reached.");
                    std_msgs::String error_code;
                    error_code.data = "[move_base]: Navigation Standby";
                    navi_state_pub_.publish(error_code);
                    /*到位后切回点边模式*/

                    idle_state_ = true;

                    geometry_msgs::PoseStamped lastGoal;
                    lastGoal = controller_plan_->poses.back();

                    if(controller_plugin_ != "m100_dock_local_planner/M100DockLocalPlannerROS") {
                        if(switchToNemPlugins()) {
                            navit_msgs::MoveBaseGoal actionGoal;
                            actionGoal.target_pose.header.frame_id    = "map";
                            actionGoal.target_pose.header.stamp       = ros::Time::now();
                            actionGoal.target_pose.pose.position.x    = lastGoal.pose.position.x;
                            actionGoal.target_pose.pose.position.y    = lastGoal.pose.position.y;
                            actionGoal.target_pose.pose.position.z    = lastGoal.pose.position.z;
                            actionGoal.target_pose.pose.orientation.x = lastGoal.pose.orientation.x;
                            actionGoal.target_pose.pose.orientation.y = lastGoal.pose.orientation.y;
                            actionGoal.target_pose.pose.orientation.z = lastGoal.pose.orientation.z;
                            actionGoal.target_pose.pose.orientation.w = lastGoal.pose.orientation.w;
                            move_base_client_.sendGoal(actionGoal);

                            ROS_INFO("[move_base]: Current plugins are NEM");
                        } else {
                            ROS_INFO("[move_base]: No need to switch plugins to NEM");
                        }
                    } else {
                        ROS_INFO("[move_base]: Dock status. No need to switch plugins to NEM");
                    }

                    /*到位后切回点边模式*/
                    ROS_INFO("[move_base]: Navigation reached success! \n");

                    return true;
                }

                //check for an oscillation condition
                if (oscillation_timeout_ > 0.0 &&
                    last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now()) {
                    publishZeroVelocity();
                    state_ = CLEARING;
                    recovery_trigger_ = OSCILLATION_R;
                }
                {
                    boost::unique_lock<navit_costmap::Costmap2D::mutex_t> lock(
                            *(controller_costmap_ros_->getCostmap()->getMutex()));

                    geometry_msgs::PoseStamped dummy_pose;
                    geometry_msgs::Twist dummy_vel;
                    cmd_vel = tc_->computeVelocityCommands(dummy_pose, dummy_vel);
                    // if (tc_->computeVelocityCommands(cmd_vel)) {
                    if (cmd_vel.linear.z == 0) {
                        ROS_DEBUG_NAMED("move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                                        cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
                        last_valid_control_ = ros::Time::now();
                        //make sure that we send the velocity command to the base
                        vel_pub_.publish(cmd_vel);
                        if(cmd_vel.linear.x!=0 || cmd_vel.linear.y!=0 && cmd_vel.angular.z!=0) {
                            if (recovery_trigger_ == CONTROLLING_R) {
                                recovery_index_ = 0;
                                replan_recovery_index_ = 0;
                                }
                        }
                    } else {
                        ROS_DEBUG_NAMED("move_base", "The local planner could not find a valid plan.");
                        ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);

                        //check if we've tried to find a valid control for longer than our time limit or nem_local_planner control failed

                        if ((ros::Time::now() > attempt_end || 214 == error_code_.data || 212 == error_code_.data) && (current_controller_plugin_ == primary_local_planner_)) {
                            //we'll move into our obstacle clearing mode
                            publishZeroVelocity();
                            state_ = CLEARING;
                            recovery_trigger_ = CONTROLLING_R;
                            return false;
                        }

                        if (current_controller_plugin_ == secondary_local_planner_ && (last_rotate_recovery_ + ros::Duration(3) < ros::Time::now())) {
                            publishZeroVelocity();
                            state_ = CLEARING;
                            recovery_trigger_ = CONTROLLING_R;
                        }

                        //TODO:由于速度计算失败后重规划会使nem local planner从头计算速度，即computevelocitycommands函数返回true，
                        // 故无法形成连贯的速度计算失败场景，导致controller_patience_失效，在此将以下代码注释，不再进行重规划
//                        else {
//                            //otherwise, if we can't find a valid control, we'll go back to planning
//                            last_valid_plan_ = ros::Time::now();
//                            planning_retries_ = 0;
//                            state_ = PLANNING;
//                            publishZeroVelocity();
//
//                            //enable the planner thread in case it isn't running on a clock
//                            boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
//                            runPlanner_ = true;
//                            planner_cond_.notify_one();
//                            lock.unlock();
//                        }
                    }
                }

                break;

                //we'll try to clear out space with any user-provided recovery behaviors
            case CLEARING:
                ROS_DEBUG_NAMED("move_base", "In clearing/recovery state");
                //we'll invoke whatever recovery behavior we're currently on if they're enabled
                if (recovery_behavior_enabled_ && recovery_index_ < recovery_behaviors_.size()) {
                    // ROS_DEBUG_NAMED("move_base_recovery", "Executing behavior %u of %zu", recovery_index_ + 1,
                    //                 recovery_behaviors_.size());
                    ROS_INFO("Executing behavior %u of %zu", recovery_index_ + 1, recovery_behaviors_.size());

                    navit_msgs::RecoveryStatus msg;
                    msg.pose_stamped = current_position;
                    msg.current_recovery_number = recovery_index_;
                    msg.total_number_of_recoveries = recovery_behaviors_.size();
                    msg.recovery_behavior_name = recovery_behavior_names_[recovery_index_];

                    recovery_status_pub_.publish(msg);

                    recovery_behaviors_[recovery_index_]->runBehavior();

                    //we at least want to give the robot some time to stop oscillating after executing the behavior
                    last_oscillation_reset_ = ros::Time::now();

                    //we'll check if the recovery behavior actually worked
                    ROS_DEBUG_NAMED("move_base_recovery", "Going back to planning state");
                    last_valid_plan_ = ros::Time::now();
                    planning_retries_ = 0;
                    // 恢复后会默认进入CONTROLLING状态，继续之前的行走（PLANNING会导致之前局部导航失败后继续行走）
                    state_ = CONTROLLING;
                    //update the index of the next recovery behavior that we'll try
                    recovery_index_++;
                    // sleep(1.0);
                } else if(recovery_behavior_enabled_ && replan_recovery_index_ < replan_recovery_behaviors_.size() && current_controller_plugin_ == secondary_local_planner_) {
                    ROS_INFO("Executing replan recovery behavior %u of %zu", replan_recovery_index_ + 1, replan_recovery_behaviors_.size());
                    //if (last_rotate_recovery_ + ros::Duration(2) < ros::Time::now()) {
                        navit_msgs::RecoveryStatus msg;
                        msg.pose_stamped = current_position;
                        msg.current_recovery_number = replan_recovery_index_;
                        msg.total_number_of_recoveries = replan_recovery_behaviors_.size();
                        msg.recovery_behavior_name = replan_recovery_behavior_names_[replan_recovery_index_];

                        recovery_status_pub_.publish(msg);

                        replan_recovery_behaviors_[replan_recovery_index_]->runBehavior();

                        //we at least want to give the robot some time to stop oscillating after executing the behavior



                        //we'll check if the recovery behavior actually worked
                        ROS_DEBUG_NAMED("move base recovery", "Going back to controll state");
                        last_valid_plan_ = ros::Time::now();
                        planning_retries_ = 0;
                        // 恢复后会默认进入CONTROLLING状态，继续之前的行走（PLANNING会导致之前局部导航失败后继续行走）
                        state_ = CONTROLLING;
                        //update the index of the next recovery behavior that we'll try
                        replan_recovery_index_++;

                        last_rotate_recovery_ = ros::Time::now();
                    //}
                    sleep(1.0);
                } else {
                    ROS_DEBUG_NAMED("move_base_recovery",
                                    "All recovery behaviors have failed, locking the planner and disabling it.");
                    //disable the planner thread
                    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
                    runPlanner_ = false;
                    lock.unlock();

                    ROS_DEBUG_NAMED("move_base_recovery", "Something should abort after this.");

                    if (recovery_trigger_ == CONTROLLING_R) {
                        SoundPlay(0, 1, "navigation_avoid.wav");
                        ROS_ERROR(
                                "Aborting because a valid control could not be found. Even after executing all recovery behaviors");
                        // 控制失败后采用绕障插件
                        // as_->setAborted(move_base_msgs::MoveBaseResult(),
                        //                 "Failed to find a valid control. Even after executing recovery behaviors.");

                        /*点边模式控制失败切换自主导航*/
                        idle_state_ = true;
                        if(planner_plugin_ == "nem_global_planner/NemGlobalPlannerROS" && controller_plugin_ == "nem_local_planner/NemLocalPlannerROS" && use_avoidance_) {
                            geometry_msgs::PoseStamped lastGoal;
                            lastGoal = controller_plan_->poses.back();
                            if (switchPluginsToRoundTheObs()) {
                                navit_msgs::MoveBaseGoal actionGoal;
                                actionGoal.target_pose.header.frame_id    = "map";
                                actionGoal.target_pose.header.stamp       = ros::Time::now();
                                actionGoal.target_pose.pose.position.x    = lastGoal.pose.position.x;
                                actionGoal.target_pose.pose.position.y    = lastGoal.pose.position.y;
                                actionGoal.target_pose.pose.position.z    = lastGoal.pose.position.z;
                                actionGoal.target_pose.pose.orientation.x = lastGoal.pose.orientation.x;
                                actionGoal.target_pose.pose.orientation.y = lastGoal.pose.orientation.y;
                                actionGoal.target_pose.pose.orientation.z = lastGoal.pose.orientation.z;
                                actionGoal.target_pose.pose.orientation.w = lastGoal.pose.orientation.w;
                                move_base_client_.sendGoal(actionGoal);
                                as_->setPreempted();
                            } else {
                                as_->setAborted(navit_msgs::MoveBaseResult(),
                                                "Failed to switch plugins after trigger obs avoidance. Beacuse current plugins are not NEM plugins.");
                            }
                        } else {
                             as_->setAborted(navit_msgs::MoveBaseResult(),
                                             "Failed to find a valid control. Even after executing recovery behaviors.");
                        }

                        /*点边模式控制失败切换自主导航*/

                    } else if (recovery_trigger_ == PLANNING_R) {

                        ROS_ERROR(
                                "Aborting because a valid plan could not be found. Even after executing all recovery behaviors");
                        as_->setAborted(navit_msgs::MoveBaseResult(),
                                        "Failed to find a valid plan. Even after executing recovery behaviors.");
                    } else if (recovery_trigger_ == OSCILLATION_R) {
                        ROS_ERROR(
                                "Aborting because the robot appears to be oscillating over and over. Even after executing all recovery behaviors");
                        as_->setAborted(navit_msgs::MoveBaseResult(),
                                        "Robot is oscillating. Even after executing recovery behaviors.");
                    }
                    std_msgs::String error_code;
                    error_code.data = "[move_base]: Navigation Standby";
                    navi_state_pub_.publish(error_code);
                    resetState();
                    return true;
                }
                break;
            case FAILING:
                {
                    ROS_DEBUG_NAMED("move_base", "In failing state");
                    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
                    runPlanner_ = false;
                    lock.unlock();

                    ROS_DEBUG_NAMED("move_base_recovery", "Something should abort after this.");

                    if (recovery_trigger_ == CONTROLLING_R) {
                        ROS_ERROR(
                                "Aborting because a valid control could not be found. Even after executing all recovery behaviors");
                        as_->setAborted(navit_msgs::MoveBaseResult(),
                                        "Failed to find a valid control. Even after executing recovery behaviors.");
                    } else if (recovery_trigger_ == PLANNING_R) {
                        ROS_ERROR(
                                "Aborting because a valid plan could not be found. Even after executing all recovery behaviors");
                        as_->setAborted(navit_msgs::MoveBaseResult(),
                                        "Failed to find a valid plan. Even after executing recovery behaviors.");
                    } else if (recovery_trigger_ == OSCILLATION_R) {
                        ROS_ERROR(
                                "Aborting because the robot appears to be oscillating over and over. Even after executing all recovery behaviors");
                        as_->setAborted(navit_msgs::MoveBaseResult(),
                                        "Robot is oscillating. Even after executing recovery behaviors.");
                    }
                    std_msgs::String error_code;
                    error_code.data = "[move_base]: Navigation Standby";
                    navi_state_pub_.publish(error_code);
                    resetState();
                    return true;
                }
                break;
            default:
                ROS_ERROR("This case should never be reached, something is wrong, aborting");
                resetState();
                //disable the planner thread
                boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
                runPlanner_ = false;
                lock.unlock();
                as_->setAborted(navit_msgs::MoveBaseResult(),
                                "Reached a case that should not be hit in move_base. This is a bug, please report it.");
                std_msgs::String error_code;
                error_code.data = "[move_base]: Navigation Standby";
                navi_state_pub_.publish(error_code);
                return true;
        }

        //we aren't done yet
        return false;
    }

    bool MoveBase::loadRecoveryBehaviors(ros::NodeHandle node) {
        XmlRpc::XmlRpcValue behavior_list;
        if (node.getParam("recovery_behaviors", behavior_list)) {
            if (behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
                for (int i = 0; i < behavior_list.size(); ++i) {
                    if (behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
                        if (behavior_list[i].hasMember("name") && behavior_list[i].hasMember("type")) {
                            //check for recovery behaviors with the same name
                            for (int j = i + 1; j < behavior_list.size(); j++) {
                                if (behavior_list[j].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
                                    if (behavior_list[j].hasMember("name") && behavior_list[j].hasMember("type")) {
                                        std::string name_i = behavior_list[i]["name"];
                                        std::string name_j = behavior_list[j]["name"];
                                        if (name_i == name_j) {
                                            ROS_ERROR(
                                                    "A recovery behavior with the name %s already exists, this is not allowed. Using the default recovery behaviors instead.",
                                                    name_i.c_str());
                                            return false;
                                        }
                                    }
                                }
                            }
                        } else {
                            ROS_ERROR(
                                    "Recovery behaviors must have a name and a type and this does not. Using the default recovery behaviors instead.");
                            return false;
                        }
                    } else {
                        ROS_ERROR(
                                "Recovery behaviors must be specified as maps, but they are XmlRpcType %d. We'll use the default recovery behaviors instead.",
                                behavior_list[i].getType());
                        return false;
                    }
                }

                //if we've made it to this point, we know that the list is legal so we'll create all the recovery behaviors
                for (int i = 0; i < behavior_list.size(); ++i) {
                    try {
                        //check if a non fully qualified name has potentially been passed in
                        if (!recovery_loader_.isClassAvailable(behavior_list[i]["type"])) {
                            std::vector<std::string> classes = recovery_loader_.getDeclaredClasses();
                            for (unsigned int i = 0; i < classes.size(); ++i) {
                                if (behavior_list[i]["type"] == recovery_loader_.getName(classes[i])) {
                                    //if we've found a match... we'll get the fully qualified name and break out of the loop
                                    ROS_WARN(
                                            "Recovery behavior specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                                            std::string(behavior_list[i]["type"]).c_str(), classes[i].c_str());
                                    behavior_list[i]["type"] = classes[i];
                                    break;
                                }
                            }
                        }

                        boost::shared_ptr<navit_move_base::recovery::RecoveryBehavior> behavior(
                                recovery_loader_.createInstance(behavior_list[i]["type"]));

                        //shouldn't be possible, but it won't hurt to check
                        if (behavior.get() == NULL) {
                            ROS_ERROR(
                                    "The ClassLoader returned a null pointer without throwing an exception. This should not happen");
                            return false;
                        }

                        //initialize the recovery behavior with its name
                        //behavior->initialize(behavior_list[i]["name"], &tf_, planner_costmap_ros_,
                        behavior->initialize(behavior_list[i]["name"], tf_.get(), planner_costmap_ros_.get(),
                                             controller_costmap_ros_.get());
                        recovery_behavior_names_.push_back(behavior_list[i]["name"]);
                        recovery_behaviors_.push_back(behavior);
                    }
                    catch (pluginlib::PluginlibException &ex) {
                        ROS_ERROR("Failed to load a plugin. Using default recovery behaviors. Error: %s", ex.what());
                        return false;
                    }
                }
            } else {
                ROS_ERROR(
                        "The recovery behavior specification must be a list, but is of XmlRpcType %d. We'll use the default recovery behaviors instead.",
                        behavior_list.getType());
                return false;
            }
        } else {
            //if no recovery_behaviors are specified, we'll just load the defaults
            return false;
        }

        //if we've made it here... we've constructed a recovery behavior list successfully
        return true;
    }

    //we'll load our default recovery behaviors here
    void MoveBase::loadDefaultRecoveryBehaviors() {
        recovery_behaviors_.clear();
        try {
            //we need to set some parameters based on what's been passed in to us to maintain backwards compatibility
            ros::NodeHandle n("~");
            n.setParam("conservative_reset/reset_distance", conservative_reset_dist_);
            n.setParam("neutral_reset/reset_distance", neutral_reset_dist_);
            n.setParam("aggressive_reset/reset_distance", aggressive_reset_dist_);
            //first, we'll load a recovery behavior to clear the costmap
            boost::shared_ptr<navit_move_base::recovery::RecoveryBehavior> cons_clear(
                    recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
            cons_clear->initialize("conservative_reset", tf_.get(), planner_costmap_ros_.get(), controller_costmap_ros_.get());
            recovery_behavior_names_.push_back("conservative_reset");
            recovery_behaviors_.push_back(cons_clear);

            //next, we'll load a recovery behavior that will do an neutral reset of the costmap
            boost::shared_ptr<navit_move_base::recovery::RecoveryBehavior> neu_clear(
                    recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
            neu_clear->initialize("neutral_reset", tf_.get(), planner_costmap_ros_.get(), controller_costmap_ros_.get());
            recovery_behavior_names_.push_back("neutral_reset");
            recovery_behaviors_.push_back(neu_clear);

            //next, we'll load a recovery behavior that will do an aggressive reset of the costmap
            boost::shared_ptr<navit_move_base::recovery::RecoveryBehavior> ags_clear(
                    recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
            ags_clear->initialize("aggressive_reset", tf_.get(), planner_costmap_ros_.get(), controller_costmap_ros_.get());
            recovery_behavior_names_.push_back("aggressive_reset");
            recovery_behaviors_.push_back(ags_clear);

            //next, we'll load rotate and clear recovery for replan
            if (clearing_rotation_allowed_) {
                boost::shared_ptr<navit_move_base::recovery::RecoveryBehavior> full_map_clear_first(
                    recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
                full_map_clear_first->initialize("full_map_clear_first", tf_.get(), planner_costmap_ros_.get(), controller_costmap_ros_.get());
                replan_recovery_behavior_names_.push_back("full_map_clear_first");
                replan_recovery_behaviors_.push_back(full_map_clear_first);

                boost::shared_ptr<navit_move_base::recovery::RecoveryBehavior> rotate_first(
                    recovery_loader_.createInstance("rotate_recovery/RotateRecovery"));
                rotate_first->initialize("clockwise_rotate_recovery_first", tf_.get(), planner_costmap_ros_.get(), controller_costmap_ros_.get());
                replan_recovery_behavior_names_.push_back("clockwise_rotate_recovery_first");
                replan_recovery_behaviors_.push_back(rotate_first);

                boost::shared_ptr<navit_move_base::recovery::RecoveryBehavior> full_map_clear_second(
                    recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
                full_map_clear_second->initialize("full_map_clear_second", tf_.get(), planner_costmap_ros_.get(), controller_costmap_ros_.get());
                replan_recovery_behavior_names_.push_back("full_map_clear_second");
                replan_recovery_behaviors_.push_back(full_map_clear_second);

                boost::shared_ptr<navit_move_base::recovery::RecoveryBehavior> rotate_second(
                    recovery_loader_.createInstance("rotate_recovery/RotateRecovery"));

                rotate_second->initialize("clockwise_rotate_recovery_second", tf_.get(), planner_costmap_ros_.get(), controller_costmap_ros_.get());
                replan_recovery_behavior_names_.push_back("clockwise_rotate_recovery_second");
                replan_recovery_behaviors_.push_back(rotate_second);

                boost::shared_ptr<navit_move_base::recovery::RecoveryBehavior> full_map_clear_third(
                    recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
                full_map_clear_third->initialize("full_map_clear_third", tf_.get(), planner_costmap_ros_.get(), controller_costmap_ros_.get());
                replan_recovery_behavior_names_.push_back("full_map_clear_third");
                replan_recovery_behaviors_.push_back(full_map_clear_third);

                boost::shared_ptr<navit_move_base::recovery::RecoveryBehavior> rotate_third(
                    recovery_loader_.createInstance("rotate_recovery/RotateRecovery"));

                rotate_third->initialize("counterclockwise_rotate_recovery_first", tf_.get(), planner_costmap_ros_.get(), controller_costmap_ros_.get());
                replan_recovery_behavior_names_.push_back("counterclockwise_rotate_recovery_first");
                replan_recovery_behaviors_.push_back(rotate_third);

                boost::shared_ptr<navit_move_base::recovery::RecoveryBehavior> full_map_clear_forth(
                    recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
                full_map_clear_forth->initialize("full_map_clear_forth", tf_.get(), planner_costmap_ros_.get(), controller_costmap_ros_.get());
                replan_recovery_behavior_names_.push_back("full_map_clear_forth");
                replan_recovery_behaviors_.push_back(full_map_clear_forth);


                boost::shared_ptr<navit_move_base::recovery::RecoveryBehavior> rotate_forth(
                    recovery_loader_.createInstance("rotate_recovery/RotateRecovery"));

                rotate_forth->initialize("counterclockwise_rotate_recovery_second", tf_.get(), planner_costmap_ros_.get(), controller_costmap_ros_.get());
                replan_recovery_behavior_names_.push_back("counterclockwise_rotate_recovery_second");
                replan_recovery_behaviors_.push_back(rotate_forth);
            }
            //we'll rotate in-place one more time
            // if (clearing_rotation_allowed_) {
            //     recovery_behaviors_.push_back(rotate);
            //     recovery_behavior_names_.push_back("rotate_recovery");
            // }
        }
        catch (pluginlib::PluginlibException &ex) {
            ROS_FATAL("Failed to load a plugin. This should not happen on default recovery behaviors. Error: %s", ex.what());
        }

        return;
    }

    void MoveBase::resetState() {
        // Disable the planner thread
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        runPlanner_ = false;
        lock.unlock();

        // Reset statemachine
        state_ = PLANNING;
        recovery_index_ = 0;
        recovery_trigger_ = PLANNING_R;
        publishZeroVelocity();

        //if we shutdown our costmaps when we're deactivated... we'll do that now
        if (shutdown_costmaps_) {
            ROS_DEBUG_NAMED("move_base", "Stopping costmaps");
            planner_costmap_ros_->stop();
            controller_costmap_ros_->stop();
        }
    }

    bool MoveBase::getRobotPose(geometry_msgs::PoseStamped &global_pose, navit_costmap::Costmap2DROS *costmap) {
        tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
        geometry_msgs::PoseStamped robot_pose;
        tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
        robot_pose.header.frame_id = robot_base_frame_;
        robot_pose.header.stamp = ros::Time(); // latest available
        ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

        // get robot pose on the given costmap frame
        try {
            tf_->transform(robot_pose, global_pose, costmap->getGlobalFrameID());
        }
        catch (tf2::LookupException &ex) {
            ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf2::ConnectivityException &ex) {
            ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf2::ExtrapolationException &ex) {
            ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
            return false;
        }

        // check if global_pose time stamp is within costmap transform tolerance
        if (current_time.toSec() - global_pose.header.stamp.toSec() > costmap->getTransformTolerance()) {
            ROS_WARN_THROTTLE(1.0, "Transform timeout for %s. " \
                        "Current time: %.4f, pose stamp: %.4f, tolerance: %.4f", costmap->getName().c_str(),
                              current_time.toSec(), global_pose.header.stamp.toSec(), costmap->getTransformTolerance());
            return false;
        }

        return true;
    }
    void MoveBase::SoundPlay(const int command, const int vol, const std::string file) {
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

    bool MoveBase::switchPluginsToRoundTheObs() {
        if(planner_plugin_ == "nem_global_planner/NemGlobalPlannerROS" && controller_plugin_ == "nem_local_planner/NemLocalPlannerROS") {
            navit_move_base::MoveBaseLoadPlugin srv;
            // srv.request.plugin_name = "primary_global_planner";
            // planner_switch_client_.call(srv);

            srv.request.plugin_name = "secondary_local_planner";
            controller_switch_client_.call(srv);
            return true;
        } else {
            ROS_ERROR("[move_base]: Current plugin is not NEM plugins!");
            return false;
        }
    }

    bool MoveBase::switchToNemPlugins() {
        if(planner_plugin_ != "nem_global_planner/NemGlobalPlannerROS" || controller_plugin_ != "nem_local_planner/NemLocalPlannerROS") {
            navit_move_base::MoveBaseLoadPlugin srv;
            // srv.request.plugin_name = "primary_global_planner";
            // planner_switch_client_.call(srv);

            srv.request.plugin_name = "primary_local_planner";
            controller_switch_client_.call(srv);
            return true;
        } else  {
            ROS_WARN("[move_base]: Current plugins have already been NEM plugins!");
            return false;
        }
    }

};
