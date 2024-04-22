#include <navit_controller/controller_server.h>
#include "ros/duration.h"

namespace navit_controller {

    ControllerServer::ControllerServer(std::shared_ptr<tf2_ros::Buffer>& tf_buffer, ros::NodeHandle& nh):
        nh_(nh),
        controller_loader_("navit_core", "navit_core::Controller"),
        controller_plugin_manager_("nav_controller",
                    boost::bind(&ControllerServer::loadControllerPlugin, this, _1),
                    boost::bind(&ControllerServer::initControllerPlugin, this, _1, _2),
                    nh_),
        as_(nh_, "/follow_path",
            boost::bind(&ControllerServer::followPath, this, _1), false)
    {
        tf_ = std::move(tf_buffer);

        ros::NodeHandle pnh("~");
        pnh.param("control_frequency", config_.control_frequency, config_.control_frequency);
        pnh.param("control_wait_timeout", config_.control_wait_timeout, config_.control_wait_timeout);
        pnh.param("enable_local_planning", config_.enable_local_planning, config_.enable_local_planning);
        pnh.param("local_plan_frequency", config_.local_plan_frequency, config_.local_plan_frequency);
        pnh.param("blocked_timeout", config_.blocked_timeout, config_.blocked_timeout);

        pnh.param("min_step_distance", config_.min_step_distance, 0.05);
        pnh.param("forward_distance", config_.forward_distance, 2.0);
        pnh.param("padding_footprint", config_.padding_footprint, 0.0);
        pnh.param("dt", config_.dt, 2.0);

        ROS_INFO("min_step_distance: %.2f", config_.min_step_distance);
        ROS_INFO("forward_distance: %.2f", config_.forward_distance);
        ROS_INFO("padding_footprint: %.2f", config_.padding_footprint);
        ROS_INFO("dt: %.2f", config_.dt);
        setupDynamicReconfigure(pnh);

        odom_sub_ = nh_.subscribe("/odom", 1, &ControllerServer::odomCallback, this);

        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        plan_pub_ = nh_.advertise<nav_msgs::Path>("/following_path",1);
        footprint_marker_array_pub_ = nh.advertise<visualization_msgs::MarkerArray>("footprint_marker_array", 8);

        costmap_ = std::make_shared<navit_costmap::Costmap2DROS>("controller_costmap", *tf_, nh_);
        collision_checker_.reset(new navit_collision_checker::FootprintCollisionChecker<navit_costmap::Costmap2D*>(costmap_->getCostmap()));

        costmap_->start();
        clear_costmap_srv_ = nh_.advertiseService("/clear_controller_costmap", &ControllerServer::clearCostmapService, this);
        toggle_avoidance_ = nh_.advertiseService("/toggle_avoidance", &ControllerServer::toggleAvoidanceService, this);
        lp_ = std::make_shared<local_planner::LocalPlanner>(costmap_, tf_);
        lp_thread_ = std::make_shared<std::thread>(
            std::mem_fun(&ControllerServer::LocalPlan), this);
        ThreadSuspend();
        lp_->SetHz(config_.local_plan_frequency);

        controller_plugin_manager_.loadPlugins();
        try
        {
            //TODO(chenzongkui): make this default controller as Controller
            //Server option
            default_controller_ptr_ = controller_loader_.createInstance("teb_local_planner/TebLocalPlannerROS");
            default_controller_ptr_->initialize("default_controller", tf_, costmap_);
        } catch(const pluginlib::PluginlibException& ex)
        {
            ROS_FATAL("Failed to create defualt controller, Exception:%s", ex.what());
        }

        goal_canceled_=false;

        // start action server
        as_.start();

        costmap_->pause();
    }

    ControllerPtr ControllerServer::loadControllerPlugin(const std::string& plugin)
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

    bool ControllerServer::initControllerPlugin(const std::string& plugin,
                                                const ControllerPtr& controller_ptr)
    {
        controller_ptr->initialize(plugin, tf_, costmap_);
        return true;
    }

    void ControllerServer::setupDynamicReconfigure(ros::NodeHandle& nh)
    {
      dsrv_ = new dynamic_reconfigure::Server<navit_controller::NavitControllerReconfigureConfig>(nh);
      dynamic_reconfigure::Server<navit_controller::NavitControllerReconfigureConfig>::CallbackType cb =
          [this](auto& config, auto level){ reconfigureCB(config, level); };
      dsrv_->setCallback(cb);
    }

    void ControllerServer::reconfigureCB(navit_controller::NavitControllerReconfigureConfig &config, uint32_t level)
    {
      config_.control_wait_timeout = config.control_wait_timeout;
      config_.enable_local_planning = config.enable_local_planning;
      config_.local_plan_frequency = config.local_plan_frequency;
      config_.blocked_timeout = config.blocked_timeout;
      ROS_INFO("Reconfigure Request: control_wait_timeout: %.2f, enable_local_planning: %d, local_plan_frequency: %.2f, blocked_timeout: %.2f",
               config_.control_wait_timeout, config_.enable_local_planning, config_.local_plan_frequency, config_.blocked_timeout);
    }

    void ControllerServer::odomCallback(const nav_msgs::OdometryConstPtr& odom_msg)
    {
       current_pose_.header = odom_msg->header;
       current_pose_.pose = odom_msg->pose.pose;
       current_vel_ = odom_msg->twist.twist;
    }

    void ControllerServer::followPath(const ActionGoal& action_goal)
    {
        ROS_INFO("[CS]: Received new goal.");
        ActionResult result;
        goal_canceled_=false;

        std::string controller_name = action_goal->controller_plugin;
        if (controller_name != "" )
        {
            if (controller_plugin_manager_.hasPlugin(controller_name))
            {
                controller_ptr_ = controller_plugin_manager_.getPlugin(controller_name);
                ROS_DEBUG_STREAM("controller plugin "<< controller_name << " will be used");
            }
            else
            {
                ROS_WARN_STREAM("controller plugin "<< controller_name << " is not avaible");
                result.error_code = ActionResult::INVALID_CONTROLLER;
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

        nav_msgs::Path resampled_path = navit_common::resamplePath(action_goal->path, 0.1);

        if (resampled_path.poses.empty() || resampled_path.poses.size() < 2)
        {
            result.error_code = ActionResult::OK;
            result.error_msg = "Follow path done! Goal is reached";
            as_.setSucceeded(result,"Goal reached");
            return;
        }

        UpdateWorkingScene();

        if (config_.enable_local_planning)
        {
            ThreadSuspend();
            lp_->SetPlan(resampled_path);
            localplan_get_plan_ = true;
            ThreadResume();
            ros::Rate r(100);
            for (int i = 0; i < 50; i ++) r.sleep();
            ROS_WARN("[CL] lp set plan finished.");
        }
        else
        {
            if (!setPathToFollow(resampled_path))
            {
                ROS_WARN_STREAM("failed to set path to controller");
                result.error_code = ActionResult::INVALID_PATH;
                result.error_msg = "the path assigned to follow is not vaild";
                as_.setAborted(result, "the path is not valid");
                return;
            }
        }

        //TODO ros params
        ros::Rate loop_rate(config_.control_frequency);
        last_valid_cmd_time_ = ros::Time::now();
        blocked_time_ = ros::Time::now();
        ControllerStats controller_stats;
        controller_stats.setControllerName(controller_name);

        // wait for costmap back online
        ros::Rate r(100);
        costmap_->resume();
        while (!costmap_->isCurrent()) r.sleep();

        while (ros::ok() && !goal_canceled_)
        {
            updateGlobalPath();
            // updateLocalPath(); // 改为线程内执行了
            controller_stats.tic();
            computeAndPublishVelocity();
            controller_stats.toc();
            controller_stats.setCmdVel(current_vel_);

            bool check_local_finished = true;
            if (config_.enable_local_planning) {
              check_local_finished = lp_->IsFinished();
            }
            if (controller_ptr_->isGoalReached() && check_local_finished)
            {
                ROS_INFO("Goal reached!");
                controller_stats.setSuccess();
                controller_stats.printStats();
                if (config_.enable_local_planning) ThreadSuspend(); // 控制器退出执行需要挂起线程
                break;
            }

            loop_rate.sleep();

            controller_stats.printStats();

            if ( loop_rate.cycleTime() > ros::Duration(1/config_.control_frequency) )
            {
                ROS_WARN("Controller missed its loop rate of %.4f hz, it took %.4f",
                        config_.control_frequency,
                        loop_rate.cycleTime().toSec());
            }

        }
        if (config_.enable_local_planning) {
          ThreadSuspend(); // 控制器退出执行需要挂起线程
          localplan_get_plan_ = false;
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
        result.error_code = ActionResult::OK;
        result.error_msg = "Follow path done! Goal is reached";
        as_.setSucceeded(result,"Goal reached");
    }

    void ControllerServer::publishZeroVelocity()
    {
        geometry_msgs::Twist zero_vel;
        cmd_vel_pub_.publish(zero_vel);
    }

    void ControllerServer::computeAndPublishVelocity()
    {
        ActionFeedback feedback;
        ActionResult result;
        geometry_msgs::Twist cmd_vel;
        bool valid_cmd_vel = true;
        bool is_blocked = false;
        // TODO(fuhaiming): test is needed for this try/catch
        try
        {
          if (!lp_->Wait())
            cmd_vel = controller_ptr_->computeVelocityCommands(current_pose_, current_vel_);
          else
            publishZeroVelocity();
        }
        catch (const std::runtime_error& e)
        {
           valid_cmd_vel = false;

           // if control patience is 0, which means robot stops whenever
           // controller plugin failed to compute valid control
           // stop the robot
           if ( config_.control_wait_timeout == 0)
           {
               publishZeroVelocity();
               result.error_code = ActionResult::NO_VALID_CONTROL;
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
              result.error_code = ActionResult::PATIENCE_EXCEEDED;
              result.error_msg = "Controller patience exceeded!";
              as_.setAborted(result,"the cmd_vel is not valid, and wait timeout");
              goal_canceled_=true;
              return;
           }
        }

        is_blocked = IsControllerBlocking(cmd_vel.linear.z, kCtrlerBlockingMarker_);
        if (is_blocked) {
          cmd_vel.linear.x = 0.0;
          cmd_vel.angular.z = 0.0;
        }

        if (lp_->Block() || IsControllerBlocked(config_.blocked_timeout)) {
          ROS_WARN_THROTTLE(1, "Controller Server: Obstacle blocked.");
          result.error_code = ActionResult::OBSTACLE_BLOCKED;
          result.error_msg = "Controller blocked!";
          as_.setAborted(result,"Controller blocked with obstacle");
          goal_canceled_ = true;
          return;
        }

        if (valid_cmd_vel) last_valid_cmd_time_ = ros::Time::now();
        if (!is_blocked) blocked_time_ = ros::Time::now();

        geometry_msgs::PoseStamped robot_pose;
        costmap_->getRobotPose(robot_pose);
        getCompletionPercentage(*tf_, robot_pose, current_path_.poses, completion_percentage_);

        feedback.distance_to_goal = distance_to_goal_ * (1.0 - completion_percentage_/ 100.0);
        feedback.current_vel = cmd_vel.linear.x;
        feedback.completion_percentage = completion_percentage_;
        ROS_DEBUG("feedback.current_vel:%lf,feedback.completion_percentage:%lf,feedback.distance_to_goal=%lf \n",
        feedback.current_vel,feedback.completion_percentage,feedback.distance_to_goal);

        // TODO(fuhaiming): test if progress checker works properly
        // failed to make progress for 30 feedbacks
        // if (!progressChecker(feedback))
        // {
        //     result.error_code = ActionResult::FAILED_TO_MAKE_PROGRESS;
        //     result.error_msg = "Failed to make progress for 30 feedbacks";
        //     as_.setAborted(result, "Failed to make progress for 30 feedbacks");
        //     goal_canceled_ = true;
        //     return;
        // }

        as_.publishFeedback(feedback);
        cmd_vel_pub_.publish(cmd_vel);
        // visualize the path
        plan_pub_.publish(current_path_);
    }

    //TODO: unit test is needed for this function
    bool ControllerServer::getCompletionPercentage(const tf2_ros::Buffer& tf,
                                                   const geometry_msgs::PoseStamped& global_pose,
                                                   std::vector<geometry_msgs::PoseStamped>& global_plan,
                                                   double& completion_percentage)
    {
        if (global_plan.empty()) return true;

        try
        {
          // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
          geometry_msgs::TransformStamped global_to_plan_transform = tf.lookupTransform(global_plan.front().header.frame_id, global_pose.header.frame_id, ros::Time(0));
          geometry_msgs::PoseStamped robot;
          tf2::doTransform(global_pose, robot, global_to_plan_transform);

          double dist_thresh_sq = 0.1;

          // iterate plan until a pose close the robot is found
          std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
          std::vector<geometry_msgs::PoseStamped>::iterator erase_end = it;
          while (it != global_plan.end())
          {
            double dx = robot.pose.position.x - it->pose.position.x;
            double dy = robot.pose.position.y - it->pose.position.y;
            double dist_sq = dx * dx + dy * dy;
            if (dist_sq < dist_thresh_sq)
            {
               erase_end = it;
               break;
            }
            ++it;
          }
          if (erase_end == global_plan.end())
            return false;

          if (erase_end != global_plan.begin())
            global_plan.erase(global_plan.begin(), erase_end);

          completion_percentage= ( 1.0 - (double) global_plan.size() / (double) original_path_.poses.size() )* 100.0;

        //   auto distance_to_goal = distance_to_goal_ * (1.0 - completion_percentage / 100.0);
        }
        catch (const tf2::TransformException& ex)
        {
          ROS_DEBUG("Cannot prune path since no transform is available: %s\n", ex.what());
          return false;
        }
        return true;
    }


    void ControllerServer::updateGlobalPath()
    {
        if (!as_.isPreemptRequested())
        {
            return;
        }

        if (!as_.isNewGoalAvailable())
        {
            ROS_WARN(" canceled received but do nothing ");
            goal_canceled_=true;
            return;
        }

        ROS_WARN("[CS] Preempt requested, cancel current goal and accept new goal.");
        ActionResult result;
        auto goal = as_.acceptNewGoal();
        std::string controller_name = goal->controller_plugin;

        // sanity check
        if (controller_name.empty())
        {
            ROS_WARN_STREAM("controller plugin name is empty");
            result.error_code = ActionResult::INVALID_CONTROLLER;
            result.error_msg = "the controller assigned to follow the path is not avaible";
            as_.setAborted(result, "controller assigned is not avaible");
            return;
        }

        if ( !controller_plugin_manager_.hasPlugin(controller_name))
        {
            ROS_WARN_STREAM("controller plugin "<< controller_name << " is not avaible");
            result.error_code = ActionResult::INVALID_CONTROLLER;
            result.error_msg = "the controller assigned to follow the path is not avaible";
            as_.setAborted(result, "controller assigned is not avaible");
            return;
        }

        if (config_.enable_local_planning)
        {
            ThreadSuspend();
            ROS_INFO("[CS] Set new plan to local planner");
            lp_->SetPlan(goal->path);
            ThreadResume();
        }
        else
        {
            if (!setPathToFollow(goal->path))
            {
                ROS_WARN_STREAM("failed to set path to controller");
                result.error_code = ActionResult::INVALID_PATH;
                result.error_msg = "the path assigned to follow is not vaild";
                as_.setAborted(result, "failed to set path to controller");
                return;
            }
        }
        controller_ptr_ = controller_plugin_manager_.getPlugin(controller_name);
        goal_canceled_ = false;
    }

    void ControllerServer::updateLocalPath()
    {
      if (!config_.enable_local_planning) return;
      if (!localplan_get_plan_) return;

      double st = ros::Time::now().toSec();
      nav_msgs::Path local_plan;
      if (!lp_->GetLocalPlan(local_plan))
      {
        // ROS_WARN("[CL] Cannot find local path");
      } else {
        ROS_WARN("[CL] find local plan (%lu) speed time %f s.", local_plan.poses.size(), ros::Time::now().toSec()-st);
        lp_->Pass();
        if (!setPathToFollow(local_plan))
        {
          ROS_WARN_STREAM("failed to set path to controller");
        }
      }
    }

    bool ControllerServer::setPathToFollow(const nav_msgs::Path& path)
    {
       std::unique_lock<std::mutex> lock(set_plan_mutex_);
       if (path.poses.empty())
       {
            ROS_WARN("Cannot follow empty path");
            return false;
       }

       if ( !controller_ptr_->setPlan(path) )
       {
            ROS_WARN("Failed to set plan to controller plugin");
            return false;
       }

       auto end_pose = path.poses.back();
       end_pose.header.frame_id = path.header.frame_id;
       ROS_DEBUG("end pose of current path to follow is (%.2f, %.2f)", end_pose.pose.position.x, end_pose.pose.position.y);

       end_pose_ = end_pose;
       current_path_ = path;
       original_path_= path;

       distance_to_goal_ = getPathLength(current_path_);
       ROS_DEBUG("distance to goal is %.2f meters \n", distance_to_goal_);

       return true;
    }

    double ControllerServer::getPathLength(const nav_msgs::Path& path)
    {
       if (path.poses.size() == 0 ) return 0.0;
       double diff_x, diff_y, distance_to_goal = 0.0;
       for(auto it = path.poses.begin(); it != path.poses.end()-1; it++)
       {
          diff_x = it->pose.position.x - (it + 1)->pose.position.x;
          diff_y = it->pose.position.y - (it + 1)->pose.position.y;
          distance_to_goal += std::sqrt(diff_x * diff_x + diff_y * diff_y);
       }
       return distance_to_goal;
    }

    bool ControllerServer::progressChecker(const ActionFeedback& feedback)
    {
        ActionFeedback new_feedback = feedback;

        // TODO(xuemantian): remove this magic number 30 here
        if (feedback_window_.size() > 30 )
        {
            feedback_window_.pop_front();
            feedback_window_.push_back(new_feedback);
        }
        else
        {
            feedback_window_.push_back(new_feedback);
            return true;
        }

        // check average completion_percentage for progress
        double completion_sum = 0.0;
        for(auto f : feedback_window_)
            completion_sum += f.completion_percentage;
        double completion = completion_sum / feedback_window_.size();

        return true;
    }

    bool ControllerServer::IsControllerBlocking(const double cmd, const double blocking_cmd)
    {
      return predictCollision(config_.forward_distance, config_.dt, config_.control_frequency);
      // double epsilon = 0.001;
      // if (fabs(cmd - blocking_cmd) <= epsilon) {
      //   ROS_WARN_THROTTLE(2, "[CS] Controller check blocking ...");
      //   return true;
      // }
      // return false;
    }

    bool ControllerServer::IsControllerBlocked(const double timeout) const
    {
      ros::Duration to = ros::Duration(timeout);
      if(ros::Time::now() - blocked_time_ > to) {
        ROS_ERROR("[CS] controller check blocked !");
        return true;
      }
      return false;
    }
    bool ControllerServer::toggleAvoidanceService(navit_msgs::ToggleAvoidance::Request& req,
                                                   navit_msgs::ToggleAvoidance::Response& res) {
      ROS_INFO_STREAM("Received toggle request, target mode is " << (req.use_avoidance ? "ON" : "OFF"));
      config_.enable_local_planning = req.use_avoidance;

      if (config_.enable_local_planning == req.use_avoidance) {
        ROS_INFO("Avoidance mode is already set to the requested state.");

        if (config_.enable_local_planning) {
          lp_->Terminate();
          lp_->Pass();
          ThreadResume();
        } else {
          // lp_->Terminate();
          // lp_->Pass();
          ThreadSuspend();
        }
        res.success = true;
      }

      return true;
    }
    // -------------------- local planner --------------------

    void ControllerServer::LocalPlan()
    {
      ros::Rate r(config_.local_plan_frequency);
      while (ros::ok()) {
        {
          std::unique_lock<std::mutex> lock(th_mutex_);
          while (suspend_) {
            lp_cond_.wait(lock);
          }
        }

        updateLocalPath();
        r.sleep();
      }
      lp_thread_->join();
    }

    void ControllerServer::ThreadSuspend() {
      std::unique_lock<std::mutex> lock(th_mutex_);
      suspend_ = true;
    }

    void ControllerServer::ThreadResume() {
      {
        std::unique_lock<std::mutex> lock(th_mutex_);
        suspend_ = false;
      }
      lp_cond_.notify_one();
    }

    // -------------------- collision checker ----------
  bool ControllerServer::predictCollision(const double forward_distance, const double forward_time, const double controller_frequency)
  {
    geometry_msgs::PoseStamped check_pose;
    geometry_msgs::PoseStamped current_pose_on_global_frame;

    static bool collisioned = false;
    geometry_msgs::PoseStamped robot_pose;
    costmap_->getRobotPose(robot_pose);
    if(robot_pose.header.frame_id==" "){
      robot_pose.header.frame_id=="odom";
    }

    try {
      tf_->transform(robot_pose, current_pose_on_global_frame, costmap_->getGlobalFrameID());
      ROS_INFO_STREAM_ONCE("Success to transform from " << robot_pose.header.frame_id
                                                        << " to "
                                                        << costmap_->getGlobalFrameID()
                                                        << " frame.");
    } catch (tf2::TransformException &ex) {
      ROS_WARN_STREAM("Failed to transform from " << robot_pose.header.frame_id << " to " << costmap_->getGlobalFrameID());
    }

    check_pose = current_pose_on_global_frame;
    // TODO(czk): accurate controller frequency needed test
    double dt = 1.0/controller_frequency, d = 0;
    // set minimum checkers number to 2
    int N = std::max(static_cast<int>(std::ceil(forward_time/dt)), 2);

    static double diff_x = 0.0, diff_yaw = 0.0;

    if (!collisioned) {
        diff_x = current_vel_.linear.x * dt;
        diff_yaw = current_vel_.angular.z * dt;
    }

    if (diff_x < config_.min_step_distance) {
      diff_x = config_.min_step_distance;
    }

    double diff_y = current_vel_.linear.y * dt;
    double heading = tf2::getYaw(robot_pose.pose.orientation);

    geometry_msgs::PoseArray collision_check_poses;
    collision_check_poses.header = check_pose.header;

    visualization_msgs::MarkerArray footprint_marker_array;
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = costmap_->getGlobalFrameID();;
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.ns = "anchor_boxes";
    delete_marker.id = 0;
    delete_marker.type = visualization_msgs::Marker::LINE_STRIP;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    footprint_marker_array.markers.push_back(delete_marker);
    footprint_marker_array_pub_.publish(footprint_marker_array);

    for (int n = 0; n < N; n++) {
      geometry_msgs::PoseStamped next_pose = check_pose;
      double yaw = fabs(diff_yaw) * ((current_vel_.linear.x * current_vel_.angular.z) > 0 ? 1 : -1);
      heading += yaw;
      next_pose.pose.position.x += diff_x * std::cos(heading) - diff_y * std::sin(heading);
      next_pose.pose.position.y += diff_x * std::sin(heading) + diff_y * std::cos(heading);
      tf2::Quaternion q;
      q.setRPY(0, 0, heading);
      next_pose.pose.orientation = tf2::toMsg(q);

      if (collisionCheckPose(next_pose)) {
        ROS_WARN_THROTTLE(1.0, "Path follow detects collision at   (%.2f, %.2f)!", check_pose.pose.position.x, check_pose.pose.position.y);
        collisioned = true;
        return true;
      }
      collisioned = false;
      d += std::hypot(diff_x, diff_y);
      if (d > forward_distance) {
        ROS_DEBUG("d %lf ,forward_distance is  %f", d, forward_distance);
        break;
      }

      check_pose = next_pose;
      collision_check_poses.poses.push_back(check_pose.pose);

      std::vector<geometry_msgs::Point> footprint_points;
      footprint_points =  costmap_->getRobotFootprint();
      padFootprint(footprint_points, config_.padding_footprint);
      // 膨胀footprint_points
      visualization_msgs::Marker footprint_marker;
      footprint_marker.header.frame_id = costmap_->getGlobalFrameID();
      footprint_marker.header.stamp = ros::Time::now();
      footprint_marker.ns = "footprint";
      footprint_marker.id = n;
      footprint_marker.type = visualization_msgs::Marker::LINE_STRIP;

      footprint_marker.color.r = 1.0;
      footprint_marker.color.g = 0.0;
      footprint_marker.color.b = 0.0;
      footprint_marker.color.a = 1.0;

      footprint_marker.pose = check_pose.pose;

      footprint_marker.scale.x = 0.1;

      for (const auto& point : footprint_points) {
          footprint_marker.points.push_back(point);
      }
      footprint_marker.points.push_back(footprint_points.front());

      footprint_marker_array.markers.push_back(footprint_marker);
    }

    footprint_marker_array_pub_.publish(footprint_marker_array);
    return false;
  }

  bool ControllerServer::collisionCheckPose(const geometry_msgs::PoseStamped& check_pose) {
      if (check_pose.header.frame_id != costmap_->getGlobalFrameID()) {
          ROS_WARN("please use pose in %s frame", (costmap_->getGlobalFrameID()).c_str());
          return true;
      }

      double yaw = tf2::getYaw(check_pose.pose.orientation);
      double x   = check_pose.pose.position.x;
      double y   = check_pose.pose.position.y;

      auto map_mutex = costmap_->getCostmap()->getMutex();
      boost::recursive_mutex::scoped_lock lock(*map_mutex);
      if (collision_checker_ == nullptr)
      {
          ROS_WARN("collision checker is not initilized");
      }
      std::vector<geometry_msgs::Point> footprint_points = costmap_->getRobotFootprint();
      padFootprint(footprint_points, config_.padding_footprint);

      if (collision_checker_->footprintCostAtPose(x, y, yaw, footprint_points) >= navit_costmap::LETHAL_OBSTACLE ) {
          ROS_WARN_THROTTLE(1.0, "pose in collision detected,cost=%lf \n",collision_checker_->footprintCostAtPose(x, y, yaw, costmap_->getRobotFootprint()));
          return true;
      }

      return false;
  }

  // -------------------- working scene --------------------

  void ControllerServer::UpdateWorkingScene()
  {
    ros::NodeHandle pnh("~");
    pnh.param("working_scene", config_.working_scene, config_.working_scene);

    int lp_collision_detect_level = 0;
    switch (config_.working_scene) {
      case kFollowPath: // 沿线全覆盖或者手绘的沿线轨迹
        ROS_INFO("[CS] working scene ==> follow path");
        break;
      case kFollowEdge: // 沿墙用于录制的地图边界
        ROS_INFO("[CS] working scene ==> follow edge");
        lp_collision_detect_level = 1;
        break;
      case kFollowPathWithoutAvoidance: // 无避障沿线（停障），用于一些窄道、小径等场景
        ROS_INFO("[CS] working scene ==> follow path without avoidance");
        break;
      default:
        ROS_ERROR("[CS] unknown working scene, working scene code %d", config_.working_scene);
        break;
    }

    lp_->SetCollisionDetectLevel(lp_collision_detect_level);
  }



}
