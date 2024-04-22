#include <navit_planner/smooth_path_server.h>

namespace navit_planner {

    SmoothPathServer::SmoothPathServer(ros::NodeHandle& nh, const std::string& param_ns):
        nh_(nh),
        smooth_path_planner_plugin_manager_("smooth_path_planner",
                          boost::bind(&SmoothPathServer::loadPlannerPlugins, this,_1),
                          boost::bind(&SmoothPathServer::initPlannerPlugins, this,_1,_2),
                          nh_),
        smooth_path_planner_plugin_loader_("navit_core","navit_core::SmoothPathPlanner"),
        smooth_path_as_(nullptr)
    {
        smooth_path_as_ = std::make_shared<SmoothPathActionServer>(nh_, "/smooth_path",
                boost::bind(&SmoothPathServer::smoothPath, this, _1), false);

        smooth_path_planner_plugin_manager_.loadPlugins();

        smooth_path_as_->start();
    }

    SmoothPathPlannerPtr SmoothPathServer::loadPlannerPlugins(const std::string& plugin)
    {
        SmoothPathPlannerPtr smooth_path_planner_ptr;
        try
        {
            smooth_path_planner_ptr = smooth_path_planner_plugin_loader_.createInstance(plugin);

            std::string planner_name = smooth_path_planner_plugin_loader_.getName(plugin);

            ROS_DEBUG_STREAM("Smooth path planner plugin " << plugin  <<
                             "with name " << planner_name << " loaded");
        }
        catch (const pluginlib::PluginlibException& ex)
        {
            ROS_WARN_STREAM("Failed to load " << plugin <<
                            " with " << ex.what());
        }

        return smooth_path_planner_ptr;
    }

    bool SmoothPathServer::initPlannerPlugins(const std::string& plugin,
                                              const SmoothPathPlannerPtr& smooth_path_planner_ptr)
    {
        smooth_path_planner_ptr->initialize(plugin);
        return true;
    }

    void SmoothPathServer::smoothPath(const SmoothPathActionGoal& action_goal)
    {
        navit_msgs::SmoothPathResult planner_result;

        nav_msgs::Path smooth_path;

        smooth_path_planner_ = smooth_path_planner_plugin_manager_.getPlugin(action_goal->smooth_path_plugin);

        if (!smooth_path_planner_->makePlan(action_goal->rough_path, smooth_path))
        {
            planner_result.smooth_success = false;

            smooth_path_as_->setAborted(planner_result, "Failed to smooth path, Please use origon path.");

            ROS_ERROR("Failed to smooth path.");

            return;
        }

        planner_result.smooth_success = true;

        planner_result.smooth_path = smooth_path;

        smooth_path_as_->setSucceeded(planner_result);
    }
} // namespace navit_planner
