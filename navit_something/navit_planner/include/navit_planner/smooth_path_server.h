#ifndef SMOOTH_PATH_SERVER_H
#define SMOOTH_PATH_SERVER_H

#include <navit_core/base_global_planner.h>
#include <navit_core/abstract_plugin_manager.h>

#include <navit_msgs/SmoothPathAction.h>

#include <navit_msgs/SmoothPathGoal.h>

#include <nav_msgs/OccupancyGrid.h>

#include <pluginlib/class_loader.hpp>

#include <actionlib/server/simple_action_server.h>

namespace navit_planner {
    using SmoothPathPlannerPtr = navit_core::SmoothPathPlanner::Ptr;
    using SmoothPathActionType = navit_msgs::SmoothPathAction;
    using SmoothPathActionGoal = navit_msgs::SmoothPathGoalConstPtr;
    using SmoothPathActionResult = navit_msgs::SmoothPathResult;
    using SmoothPathActionServer = actionlib::SimpleActionServer<SmoothPathActionType>;


    class SmoothPathServer
    {
        public:
            using SmoothPathPlanner = navit_core::SmoothPathPlanner;
            using SmoothPathPlannerLoader = pluginlib::ClassLoader<SmoothPathPlanner>;
            using SmoothPathPlannerPluginManager = navit_core::AbstractPluginManager<SmoothPathPlanner>;

            SmoothPathServer(ros::NodeHandle& nh, const std::string& param_ns = "~");

            ~SmoothPathServer()
            {
                smooth_path_planner_plugin_manager_.clearPlugins();

                smooth_path_planner_.reset();
            }


            bool getPlan(const nav_msgs::OccupancyGrid& map,
                         const nav_msgs::Path& edge,
                         const std::string& plugin_name,
                         nav_msgs::Path& planned_coverage_path,
                         nav_msgs::Path& planned_wall_path);

        protected:
            ros::NodeHandle nh_;

            void smoothPath(const SmoothPathActionGoal& goal);

            SmoothPathPlannerPtr smooth_path_planner_ = nullptr;

            SmoothPathPlannerPluginManager smooth_path_planner_plugin_manager_;

            SmoothPathPlannerLoader smooth_path_planner_plugin_loader_;

            SmoothPathPlannerPtr loadPlannerPlugins(const std::string& plugin);

            bool initPlannerPlugins(const std::string& plugin,
                                    const SmoothPathPlannerPtr& smooth_path_planner_ptr);

            std::shared_ptr<SmoothPathActionServer> smooth_path_as_;
    };

}
#endif //SMOOTH_PATH_SERVER_H

