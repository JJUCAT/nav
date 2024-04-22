
#include <navit_planner/planner_server.h>
#include "nav_msgs/Path.h"
namespace navit_planner {

    PlannerServer::PlannerServer(std::shared_ptr<tf2_ros::Buffer>& tf_buffer, ros::NodeHandle& nh):
        nh_(nh),
        planner_plugin_manager_("nav_planner",
                          boost::bind(&PlannerServer::loadPlannerPlugins, this,_1),
                          boost::bind(&PlannerServer::initPlannerPlugins, this,_1, _2),
                          nh_),
        planner_loader_("navit_core","navit_core::GlobalPlanner"),
        as_(nullptr)
    {
        tf_buffer_ = tf_buffer;
        as_ = std::make_shared<ActionServer>(nh_, "/compute_path",
                boost::bind(&PlannerServer::computePath, this, _1), false);
        psc_as_ = std::make_shared<PSC_ActionServer>(nh_, "/plan_security_checking",
                boost::bind(&PlannerServer::PlanSecurityChecking, this, _1), false);

        costmap_ = std::make_shared<navit_costmap::Costmap2DROS>("planner_costmap", *tf_buffer_, nh_);
        costmap_->start();

        planner_plugin_manager_.loadPlugins();
        
        try
        {
           default_planner_ = planner_loader_.createInstance("global_planner/GlobalPlanner");
           default_planner_->initialize("default_planner", costmap_);
        } catch(const pluginlib::PluginlibException& ex)
        {
            ROS_FATAL("Failed to create defualt planner, Exception:%s", ex.what());
        }

        plan_pub_ = nh_.advertise<nav_msgs::Path>("/planned_path",100);
        clear_costmaps_srv_ = nh_.advertiseService("/clear_planner_costmap",&PlannerServer::clearCostmapService, this);

        as_->start();
        psc_as_->start();

        // collision checker
        collision_checker_.reset(new navit_collision_checker::FootprintCollisionChecker<
                navit_costmap::Costmap2D*>(costmap_->getCostmap()));
        collision_checker_srv_ = nh_.advertiseService("pose_collision_check_planner_costmap",
                                                      &PlannerServer::collisionCheckService, this);

        costmap_->pause();
    }

    PlannerPtr PlannerServer::loadPlannerPlugins(const std::string& plugin)
    {
        PlannerPtr planner_ptr;
        try
        {
            planner_ptr = planner_loader_.createInstance(plugin);
            std::string planner_name = planner_loader_.getName(plugin);
            ROS_DEBUG_STREAM("planner plugin " << plugin  << 
                             "with name " << planner_name << " loaded");
        }
        catch (const pluginlib::PluginlibException& ex)
        {
            ROS_WARN_STREAM("Failed to load " << plugin << 
                            " with " << ex.what());
        }

        return planner_ptr;
    }

    bool PlannerServer::initPlannerPlugins(const std::string& plugin,
                                           const PlannerPtr& planner_ptr)
    {
        planner_ptr->initialize(plugin, costmap_);
        return true;
    }

    void PlannerServer::computePath(const ActionGoal& action_goal)
    {
        ros::Time planning_start, planning_end;
        nav_msgs::Path planned_path;
        ActionResult result;
        result.path_found = false;
        bool report = false;

        // resume costmap update
        costmap_->resume();
        waitForCostmap();

        geometry_msgs::PoseStamped start, goal;
        start.header.stamp = ros::Time::now();
        goal.header.stamp = ros::Time::now();
        if (!getStartPose(action_goal, start))
        {
            result.error_code = ActionResult::START_OUTSIDE_MAP;
            result.error_msg = "Start pose outside the map";
            goto label_exit;
        }

        goal = action_goal->goal;

        if (!transformPosesToGlobalFrame(start, goal))
        {
            result.error_code = ActionResult::TF_ERROR;
            result.error_msg = "Failed to transform poses to global frame";
            goto label_exit;
        }
        
        if (action_goal->planner_plugin != "" && 
            !planner_plugin_manager_.hasPlugin(action_goal->planner_plugin))
        {
            result.error_code = ActionResult::INVALID_PLANNER;
            result.error_msg = "Invalid planner";
            goto label_exit;
        }

        if(collisionCheck(start))
        {
            result.error_code = ActionResult::START_OCCUPIED;
            result.error_msg = "Start is occupied";
            std::cout << "Start is occupied" << std::endl;
            goto label_exit;
        }

        if (collisionCheck(goal))
        {
            result.error_code = ActionResult::GOAL_OCCUPIED;
            result.error_msg = "Goal is occupied";
            std::cout << "Goal is occupied" << std::endl;
            goto label_exit;
        }

        planning_start = ros::Time::now();

        planner_stats_.setActionGoal(action_goal);
        planner_stats_.setStartTime();

    try
    {
        planned_path = getPlan(start, goal, action_goal->planner_plugin);
    }
    catch(const std::runtime_error & e)
    {
        ROS_INFO("exceeded maximum iterations or exceeded planner time limit");
        result.error_code = ActionResult::NO_VALID_PATH;
        result.error_msg = "No path is found";
        report = true;
        goto label_exit;
    }

        planning_end = ros::Time::now();
        planner_stats_.setEndTime();
        planner_stats_.setResult(planned_path);
        planner_stats_.printStats();

        if (planned_path.poses.size() == 0)
        {
            result.error_code = ActionResult::NO_VALID_PATH;
            result.error_msg = "No path is found";
            report = true;
            goto label_exit;
        }

        planned_path.header.frame_id = costmap_->getGlobalFrameID();
        planned_path.header.stamp = ros::Time::now();
        plan_pub_.publish(planned_path);

        result.error_code = ActionResult::OK;
        result.error_msg = "OK";
        result.path = planned_path;
        result.planning_used_time.data = planning_end - planning_start;
        result.path_found = true;
        report = true;

    label_exit:
        if (report) {
          as_->setSucceeded(result);
        } else {
          as_->setAborted(result, result.error_msg);
        }

        // stop update costmap
        costmap_->pause();
    }

    nav_msgs::Path 
    PlannerServer::getPlan(const geometry_msgs::PoseStamped& start,
                           const geometry_msgs::PoseStamped& goal,
                           const std::string& plugin)
    {
        if (plugin == "")
        {
            ROS_WARN("Requested planner plugin is empty! use A* for this planning task!");
            planner_ = default_planner_;
        }
        else
        {
            planner_ = planner_plugin_manager_.getPlugin(plugin);
        }
        nav_msgs::Path navit_plan = planner_->makePlan(start, goal);
        nav_msgs::Path plan;
        plan.header = navit_plan.header;
        plan.poses = navit_plan.poses;
        return plan;
    }

    bool PlannerServer::transformPosesToGlobalFrame(geometry_msgs::PoseStamped& start,
                                                    geometry_msgs::PoseStamped& goal) 
    {
        std::string global_frame = costmap_->getGlobalFrameID();
        try
        {
            //auto tf_start = tf_buffer_.lookupTransform(global_frame,
            //                                           start.header.frame_id,
            //                                           ros::Time(0));
            //start.header.frame_id = global_frame;

            //tf2::doTransform(start, start, tf_start);
            tf_buffer_->transform(start, start, global_frame);
        }
        catch (const tf2::TransformException& ex)
        {
           ROS_WARN_STREAM("Failed to transform from " << start.header.frame_id 
                           << " to " << global_frame );
           return false;
        }

        try
        {
            //auto tf_goal  = tf_buffer_.lookupTransform(global_frame,
            //                                           goal.header.frame_id,
            //                                           ros::Time(0));
            //goal.header.frame_id = global_frame;
            //tf2::doTransform(goal, goal, tf_goal);
            tf_buffer_->transform(goal, goal, global_frame);
        }
        catch (const tf2::TransformException& ex)
        {
            ROS_WARN_STREAM("Failed to transform from " << goal.header.frame_id
                           << " to " << global_frame ); 
            return false;
        }
        return true;
    }

    void PlannerServer::waitForCostmap() 
    {
        ros::Rate r(100);
        while (!costmap_->isCurrent())
        {
            r.sleep();
        }
    }

    bool PlannerServer::getStartPose(const ActionGoal& action_goal,
                                     geometry_msgs::PoseStamped& start) 
    {
        if (action_goal->use_start)
        {
            start = action_goal->start;

            double wx = start.pose.position.x;
            double wy = start.pose.position.y;
            unsigned int start_x_i, start_y_i;
            
            if(!costmap_->getCostmap()->worldToMap(wx, wy, start_x_i, start_y_i))
            {
                ROS_WARN("PlannerServer: start pose is off the planner costmap! Planning will be failed!");
                return false;
            }

            return true;
        }
        else if (!costmap_->getRobotPose(start))
        {
           return false;
        }

        return true;
    }

    // -------------------- path security checking 路径碰撞检测 action --------------------

    void PlannerServer::PlanSecurityChecking(const PSC_ActionGoal& action_goal) {
      nav_msgs::Path plan = action_goal->plan;
      double check_distance = action_goal->check_distance;
      uint32_t start = action_goal->start;
      ROS_INFO("[PSC] path size is %lu, frame is %s, check distance %f, start %u",
        plan.poses.size(), plan.header.frame_id.c_str(), check_distance, start);

      PSC_ActionResult result;
      result.collised = false;

      if (plan.header.frame_id != costmap_->getGlobalFrameID()) {
        ROS_WARN("[PSC] plan frame id %s is mismatching with costmap frame id %s.",
          plan.header.frame_id.c_str(), costmap_->getGlobalFrameID().c_str());
        try {
          auto plan_to_map_transform = tf_buffer_->lookupTransform(
            costmap_->getGlobalFrameID(), plan.header.frame_id, ros::Time(0));
          for (auto& p : plan.poses) 
            tf2::doTransform(p, p, plan_to_map_transform);
        } catch (const tf2::TransformException& ex) {
          ROS_ERROR("[PSC] tf listen plan -> map failed, %s", ex.what());
          psc_as_->setAborted(result, "tf transform failed.");
        }
      }

      // resume costmap update
      costmap_->resume();
      waitForCostmap();

      double dist = 0.0;
      double collised_cost = 253.0f;
      for (uint32_t i = start, j = start; i < plan.poses.size(); j = i, i ++) {
        dist += std::hypot(plan.poses.at(i).pose.position.y - plan.poses.at(j).pose.position.y,
                           plan.poses.at(i).pose.position.x - plan.poses.at(j).pose.position.x);
        if (collisionCheck(plan.poses.at(i), collised_cost))
          result.collised_indexes.push_back(i);
        if (dist >= check_distance) break;
      }
      result.collised = !result.collised_indexes.empty();
      ROS_INFO("[PSC] plan check %s, collised size %lu",
        result.collised?"collised":"safe", result.collised_indexes.size());

      costmap_->pause();
      psc_as_->setSucceeded(result);
    }

}
