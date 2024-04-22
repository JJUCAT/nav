#include <navit_routing/routing_server.h>

namespace navit_routing {
    RoutingServer::RoutingServer(ros::NodeHandle& nh):
        nh_(nh),
        router_plugin_manager_("navit_routing",
                          boost::bind(&RoutingServer::loadRouterPlugin, this,_1),
                          boost::bind(&RoutingServer::initRouterPlugin, this,_1, _2),
                          nh_),
        router_loader_("navit_core","navit_core::BaseTspSearch"),
        as_(nullptr)
    {
        as_ = std::make_shared<ActionServer>(nh_, "/routing",
                boost::bind(&RoutingServer::graphSearch, this, _1), false);
        router_plugin_manager_.loadPlugins();

        as_->start();
    }

    RouterPtr RoutingServer::loadRouterPlugin(const std::string& plugin)
    {
       RouterPtr route_ptr;
       try
       {
            route_ptr = router_loader_.createInstance(plugin);
            std::string route_name = router_loader_.getName(plugin);
            ROS_DEBUG_STREAM("Router: "<< plugin <<
                             "with name: " << route_name << " is loaded." );
       }
       catch(const pluginlib::PluginlibException ex)
       {
            ROS_WARN_STREAM("Failed to load "<< plugin << " with "<< ex.what());
       }
       return route_ptr;
    }

    bool RoutingServer::initRouterPlugin(const std::string& plugin,
                                         const RouterPtr& router_ptr)
    {
        navit_core::Graph::Ptr graph;
        router_ptr->initialize(plugin, graph);
        return true;
    }

    void RoutingServer::graphSearch(const ActionGoal& action_goal)
    {
        ActionResult result;
        std::vector<int64_t> route_result;
        
        
        navit_msgs::GraphPointsConnection graph_points_connection;
        graph_points_connection = action_goal->map;
        navit_core::Graph graph;
        for (int i = 0; i < graph_points_connection.predecessors.size(); ++i) {
            graph.add_edge(i,
                           graph_points_connection.predecessors[i].from, 
                           graph_points_connection.predecessors[i].to, 
                           graph_points_connection.predecessors[i].cost);
        }

        // route_result = getRoute(action_goal->waypoints, "tsp");
        route_result = getRoute(action_goal->start, 
                                action_goal->goal, 
                                action_goal->via_points, 
                                graph, 
                                "tsp");
        
        if (route_result.size() != 0) {
            for (int i = 0; i < route_result.size(); ++i) {
                result.route.push_back(route_result[i]);
            }
            as_->setSucceeded(result, "Find route successfully.");
        } else {
            as_->setSucceeded(result, "Find route failed, check the waypoints if exists or not.");
        }
    }

    std::vector<int64_t> RoutingServer::getRoute(
                            const int64_t& start,
                            const int64_t& goal,
                            const std::vector<int64_t>& via_points,
                            const navit_core::Graph &graph,
                            const std::string& plugin)

    {
        std::vector<int64_t> result_route;
        router_ptr_ = router_plugin_manager_.getPlugin(plugin);
        if(!router_ptr_->search(start, goal, via_points, graph, result_route)) {
            ROS_ERROR("Cannot make route plan.");
        }

        return result_route;
    }
}
