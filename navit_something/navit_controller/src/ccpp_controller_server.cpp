#include <navit_controller/ccpp_controller_server.h>

namespace navit_controller {

    CCPPControllerServer::CCPPControllerServer(std::shared_ptr<tf2_ros::Buffer>& tf_buffer, ros::NodeHandle& nh):
        nh_(nh),
        controller_loader_("navit_core", "navit_core::Controller"),
        controller_plugin_manager_("navit_ccpp_controller",
                                    boost::bind(&CCPPControllerServer::loadControllerPlugin, this, _1),
                                    boost::bind(&CCPPControllerServer::initControllerPlugin, this, _1, _2),
                                    nh_),

        as_(nh_, "/follow_coverage_path",
            boost::bind(&CCPPControllerServer::followCoveragePath, this, _1), false)
    {
        tf_ = std::move(tf_buffer);
        tf2_ros::TransformListener tfListener(*tf_);
        ros::NodeHandle pnh("~");

        nh_.param("control_frequency", config_.control_frequency, config_.control_frequency);
        nh_.param("control_wait_timeout", config_.control_wait_timeout, config_.control_wait_timeout);
        // nh_.param("global_frame", config_.global_frame, config_.global_frame);
        nh_.param("robot_frame", config_.robot_frame, config_.robot_frame);
        nh_.param("extract_length", config_.extract_length, config_.extract_length);
        nh_.param("cmd_vel_topic_name", config_.cmd_vel_topic_name, config_.cmd_vel_topic_name);
        nh_.param("clear_costmap_srv_name", config_.clear_costmap_srv_name, config_.clear_costmap_srv_name);

        odom_sub_ = nh_.subscribe("/odom", 1, &CCPPControllerServer::odomCallback, this);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(config_.cmd_vel_topic_name, 1);
        plan_pub_ = nh_.advertise<nav_msgs::Path>("following_coverage_path",1);
        cleaned_pub_ = nh_.advertise<nav_msgs::Path>("cleaned_path", 1);
        local_path_pub_ = nh_.advertise<nav_msgs::Path>("local_path", 1);
        local_origon_path_pub_ = nh_.advertise<nav_msgs::Path>("local_origon_path", 1);

        costmap_ = std::make_shared<navit_costmap::Costmap2DROS>("controller_costmap", *tf_, nh_);
        config_.global_frame = costmap_->getGlobalFrameID();

        clear_costmap_srv_ = nh_.advertiseService( config_.clear_costmap_srv_name, &CCPPControllerServer::clearCostmapService, this);
        costmap_->pause();

        controller_plugin_manager_.loadPlugins();

        try
        {
            default_controller_ptr_ = controller_loader_.createInstance("teb_local_planner/TebLocalPlannerROS");
            default_controller_ptr_->initialize("default_ccpp_controller", tf_, costmap_);
        } catch(const pluginlib::PluginlibException& ex)
        {
            ROS_FATAL("Failed to create defualt controller, Exception:%s", ex.what());
        }

        goal_canceled_ = false;
        costmap_->start();
        // close cleaned costmap layer, use path to clean costmap
        // cleaned_layer_ = std::make_shared<navit_costmap::CleanedLayer>();
        // cleaned_layer_->onInitialize(pnh);

        controller_server_helper_ptr_ = std::make_shared<ControllerServerHelper>();
        controller_server_helper_ptr_->init(costmap_);

        tf_handle_thread_ptr_ = new boost::thread(boost::bind(&CCPPControllerServer::tfHandleThread, this));
        as_.start();
    }

    ControllerPtr CCPPControllerServer::loadControllerPlugin(const std::string& plugin)
    {
       ControllerPtr controller_ptr;
       try
       {
            controller_ptr = controller_loader_.createInstance(plugin);
            std::string controller_name = controller_loader_.getName(plugin);
            ROS_DEBUG_STREAM("Controller: "<< plugin <<
                             "with name: " << controller_name << " is loaded." );
       }
       catch(const pluginlib::PluginlibException ex)
       {
            ROS_WARN_STREAM("Failed to load "<< plugin << " with "<< ex.what());
       }
       return controller_ptr;
    }

    bool CCPPControllerServer::initControllerPlugin(const std::string& plugin,
                                                const ControllerPtr& controller_ptr)
    {
        controller_ptr->initialize(plugin, tf_, costmap_);
        return true;
    }

    void CCPPControllerServer::odomCallback(const nav_msgs::OdometryConstPtr& odom_msg)
    {
       current_pose_.header = odom_msg->header;
       current_pose_.pose = odom_msg->pose.pose;
       current_vel_ = odom_msg->twist.twist;
    }

    void CCPPControllerServer::followCoveragePath(const FcpActionGoal& action_goal)
    {
        // cleaned_layer_->setPolygonMap(action_goal->coverage_area, action_goal->holes);
        // cleaned_layer_->mapUpdateLoop(10.0);
        FcpActionResult result;
        goal_canceled_ = false;

        std::string controller_name = action_goal->plugin_name;
        if (controller_name != "" ) {
            if (controller_plugin_manager_.hasPlugin(controller_name))
            {
                controller_ptr_ = controller_plugin_manager_.getPlugin(controller_name);
                ROS_DEBUG_STREAM("controller plugin "<< controller_name << " will be used");
            }
            else
            {
                ROS_WARN_STREAM("controller plugin "<< controller_name << " is not avaible");
                result.error_code = FcpActionResult::INVALID_CONTROLLER;
                result.error_msg = "the controller assigned to follow the path is not avaible";
                as_.setAborted(result,"the controller is not valid");
                return;
            }
        }
        else
        {
            controller_ptr_ = default_controller_ptr_;
            ROS_WARN("Requested controller plugin is empty! use default TEB for this task!");
        }

        if (!setPathToFollow(action_goal->path))
        {
            ROS_WARN_STREAM("failed to set path to controller");
            result.error_code = FcpActionResult::INVALID_PATH;
            result.error_msg = "the path assigned to follow is not vaild";
            as_.setAborted(result, "the path is not valid");
            return;
        }

        ros::Rate loop_rate(config_.control_frequency);
        last_valid_cmd_time_ = ros::Time::now();

        ControllerStats controller_stats;
        controller_stats.setControllerName(controller_name);

        // wait for costmap back online
        ros::Rate r(100);
        costmap_->resume();
        while (!costmap_->isCurrent()) r.sleep();

        while (ros::ok()&&!goal_canceled_)
        {
            updateGlobalPath();
            controller_stats.tic();
            computeAndPublishVelocity();
            controller_stats.toc();
            controller_stats.setCmdVel(current_vel_);

            if (controller_ptr_->isGoalReached())
            {
                ROS_INFO("Goal reached!");
                controller_stats.setSuccess();
                controller_stats.printStats();
                break;
            }

            loop_rate.sleep();

            controller_stats.printStats();

            if (loop_rate.cycleTime() > ros::Duration(1/config_.control_frequency) )
            {
                ROS_WARN("Controller missed its loop rate of %.4f hz, it took %.4f",
                        config_.control_frequency,
                        loop_rate.cycleTime().toSec());
            }
        }
        // make sure the robot is stopped
        publishZeroVelocity();

        // save some cpu
        costmap_->pause();

        if(goal_canceled_)
        {
            ROS_ERROR("Action goal is canceled!");
            // set goal_canceled_ to false for next goal
            goal_canceled_=false;
            return;
        }
        // Succeeded
        result.error_code = FcpActionResult::OK;
        result.error_msg = "Follow path done! Goal is reached";
        as_.setSucceeded(result,"Goal reached");
    }

    void CCPPControllerServer::publishZeroVelocity()
    {
        geometry_msgs::Twist zero_vel;
        cmd_vel_pub_.publish(zero_vel);
    }

    void CCPPControllerServer::computeAndPublishVelocity()
    {
        FcpActionFeedback feedback;
        FcpActionResult result;
        geometry_msgs::Twist cmd_vel;
        geometry_msgs::PoseStamped robot_pose;

        bool valid_cmd_vel = true;

        costmap_->getRobotPose(robot_pose);

        nav_msgs::Path path;
        if (start_index_ > original_path_.poses.size() - 1) {
            start_index_ = original_path_.poses.size() - 1;
        }

        uint16_t end_index = start_index_ + extract_index_;
        if (end_index > original_path_.poses.size() - 1) {
            end_index = original_path_.poses.size() - 1;
        }

        if (!controller_server_helper_ptr_->extractFromGlobalPath(start_index_, end_index, original_path_, path)) {
            ROS_WARN("extractFromGlobalPath failed");
            result.error_code = FcpActionResult::INVALID_PATH;
            result.error_msg = "the path assigned to follow is not vaild";
            as_.setAborted(result, "the path is not valid");
            return;
        }
        controller_server_helper_ptr_->visualizePath(path, local_origon_path_pub_);
        nav_msgs::Path path_in_global;
        if (!controller_server_helper_ptr_->doTransform(path, transform_stamped_, path_in_global, config_.global_frame)) {
            ROS_WARN("doTransform failed");
            result.error_code = FcpActionResult::INVALID_PATH;
            result.error_msg = "the path assigned to follow is not vaild";
            return;
        }
        // find nearest index on path
        uint16_t index_in_local;
        if (!controller_server_helper_ptr_->findNearestIndexOnPath(robot_pose, path_in_global, index_in_local)) {
            ROS_WARN("findNearestIndexOnPath failed");
            result.error_code = FcpActionResult::INVALID_PATH;
            result.error_msg = "the path assigned to follow is not vaild";
            as_.setAborted(result, "the path is not valid");
            return;
        }
        while (index_in_local > 0 && index_in_local < path_in_global.poses.size() - 1) {
            // TODO(CZK): vector erase first element is very expensive
            path_in_global.poses.erase(path_in_global.poses.begin());
            start_index_++;
            index_in_local --;
        }

        controller_server_helper_ptr_->visualizePath(path_in_global, local_path_pub_);

        if (!controller_ptr_->setPlan(path_in_global))
        {
            ROS_WARN("Failed to set plan to controller plugin");
            return;
        }

        try
        {
           cmd_vel = controller_ptr_->computeVelocityCommands(current_pose_, current_vel_);
        }

        catch (const std::runtime_error& e)
        {
           valid_cmd_vel = false;
           // if control patience is 0, which means robot stops whenever
           // controller plugin failed to compute valid control
           // stop the robot
           if ( config_.control_wait_timeout == 0 )
           {
               publishZeroVelocity();
               result.error_code = FcpActionResult::NO_VALID_CONTROL;
               result.error_msg = "no valid control can be computed";
               as_.setAborted(result, "no valid control");
               goal_canceled_ = true;
               return;
           }
           // failed to compute velocity under current settings or environment
           ROS_WARN("Controller Server: failed to compute velocity, %s", e.what());

           // stop the robot for safety
           publishZeroVelocity();

           // if stopped for a while, cancel the goal
           if((ros::Time::now()).toSec() - last_valid_cmd_time_.toSec() > config_.control_wait_timeout)
           {
               result.error_code = FcpActionResult::PATIENCE_EXCEEDED;
               result.error_msg = "Controller patience exceeded!";
               as_.setAborted(result,"the cmd_vel is not valid, and wait timeout");
		       goal_canceled_ = true;
		       return;
           }
        }

        if (valid_cmd_vel) last_valid_cmd_time_ = ros::Time::now();

        double percent;
        controller_server_helper_ptr_->getCompletionPercentage(*tf_, robot_pose, original_path_ ,current_path_.poses, completion_percentage_);
        nav_msgs::Path cleaned_paths;
        nav_msgs::Path uncleaned_paths;

        controller_server_helper_ptr_->taskPath(robot_pose, cleaned_paths, uncleaned_paths);

        cleaned_pub_.publish(uncleaned_paths);

        feedback.distance_to_goal = distance_to_goal_ * (1.0 - completion_percentage_/ 100.0);
        feedback.current_vel = cmd_vel.linear.x;
        feedback.completion_percentage = completion_percentage_;
        // feedback.cleaned_paths = cleaned_paths;
        // feedback.uncleaned_paths = uncleaned_paths;

        ROS_DEBUG("feedback.current_vel:%lf,feedback.completion_percentage:%lf,feedback.distance_to_goal=%lf \n",
        feedback.current_vel,feedback.completion_percentage,feedback.distance_to_goal);

        as_.publishFeedback(feedback);
        cmd_vel_pub_.publish(cmd_vel);
        // visualize the path
        plan_pub_.publish(current_path_);
    }

    void CCPPControllerServer::updateGlobalPath()
    {
        // reset();
        if (!as_.isPreemptRequested())
        {
            return;
        }

        if (!as_.isNewGoalAvailable())
        {
            ROS_WARN(" canceled received but do nothing ");
            goal_canceled_ = true;
            return;
        }

        FcpActionResult result;
        auto goal = as_.acceptNewGoal();
        std::string controller_name = goal->plugin_name;

        // sanity check
        if (controller_name.empty())
        {
            ROS_WARN_STREAM("controller plugin name is empty");
            result.error_code = FcpActionResult::INVALID_CONTROLLER;
            result.error_msg = "the controller assigned to follow the path is not avaible";
            as_.setAborted(result, "controller assigned is not avaible");
            return;
        }

        if (!controller_plugin_manager_.hasPlugin(controller_name))
        {
            ROS_WARN_STREAM("controller plugin "<< controller_name << " is not avaible");
            result.error_code = FcpActionResult::INVALID_CONTROLLER;
            result.error_msg = "the controller assigned to follow the path is not avaible";
            as_.setAborted(result, "controller assigned is not avaible");
            return;
        }

        if (!setPathToFollow(goal->path))
        {
            ROS_WARN_STREAM("failed to set path to controller");
            result.error_code = FcpActionResult::INVALID_PATH;
            result.error_msg = "the path assigned to follow is not vaild";
            as_.setAborted(result, "failed to set path to controller");
            return;
        }

        controller_ptr_ = controller_plugin_manager_.getPlugin(controller_name);
        goal_canceled_ = false;
    }


    bool CCPPControllerServer::setPathToFollow(const nav_msgs::Path& path)
    {
        reset();
        // coverage path points size must have at least 3 points.
        if (path.poses.size() < 3)
        {
            ROS_WARN("Cannot follow empty path");
            return false;
        }
        // resample and check path
        float dis_nabor_points = std::sqrt(std::pow(path.poses[0].pose.position.x - path.poses[1].pose.position.x, 2) +
                                           std::pow(path.poses[0].pose.position.y - path.poses[1].pose.position.y, 2));

        extract_index_ = std::ceil(config_.extract_length / dis_nabor_points);

        current_path_ = path;
        original_path_ = path;
        action_plan_frame_id_ = path.header.frame_id;

        distance_to_goal_ = controller_server_helper_ptr_->getPathLength(current_path_);
        controller_server_helper_ptr_->setGlobalPath(path);

        return true;
    }

    void CCPPControllerServer::tfHandleThread()
    {
        while (ros::ok())
        {
            usleep(1000);
            try {
               transform_stamped_ = tf_->lookupTransform(config_.global_frame, action_plan_frame_id_, ros::Time(0));
            } catch (tf2::TransformException &ex) {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
        }
    }
}
