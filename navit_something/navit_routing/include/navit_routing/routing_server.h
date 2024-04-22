#ifndef ROUTING_SERVER_H
#define ROUTING_SERVER_H

#include <ros/ros.h>

#include <navit_core/base_graph_search.h>
#include <navit_core/abstract_plugin_manager.h>

#include <navit_msgs/RoutingAction.h>

#include <actionlib/server/simple_action_server.h>
#include <pluginlib/class_loader.hpp>

namespace navit_routing {

using Router = navit_core::BaseTspSearch;
using RouterPtr = navit_core::BaseTspSearch::Ptr;
using ActionType = navit_msgs::RoutingAction;
using ActionGoal = navit_msgs::RoutingGoalConstPtr;
using ActionResult = navit_msgs::RoutingResult;
using ActionFeedback = navit_msgs::RoutingFeedback;
using ActionServer = actionlib::SimpleActionServer<ActionType>;

class RoutingServer
{
    public:
        using RouterLoader = pluginlib::ClassLoader<Router>;
        using RouterPluginManager = navit_core::AbstractPluginManager<Router>;

        RoutingServer(ros::NodeHandle& nh);

        ~RoutingServer(){
            router_plugin_manager_.clearPlugins();
            router_ptr_.reset();
        }

    protected:
        ros::NodeHandle nh_;
        // load plugins
        RouterPtr loadRouterPlugin(const std::string& plugin);
        bool initRouterPlugin(const std::string& plugin,
                              const RouterPtr& router_ptr);
        void graphSearch(const ActionGoal& action_goal);
        std::vector<int64_t>  getRoute(const int64_t& start,
                                            const int64_t& goal,
                                            const std::vector<int64_t>& via_points,
                                            const navit_core::Graph &graph,
                                            const std::string& plugin);
        RouterPtr router_ptr_;
        RouterLoader router_loader_;

        RouterPluginManager router_plugin_manager_;

        std::shared_ptr<ActionServer> as_;
};
}
#endif
