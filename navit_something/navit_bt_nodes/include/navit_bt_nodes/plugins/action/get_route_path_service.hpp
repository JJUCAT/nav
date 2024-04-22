//
// Created by fan on 23-2-2.
//

#ifndef NAVIT_BT_NODES_GET_ROUTE_PATH_SERVICE_HPP
#define NAVIT_BT_NODES_GET_ROUTE_PATH_SERVICE_HPP

#include <navit_msgs/GetMapRoutePath.h>
#include "navit_bt_nodes/bt_service_node.h"

namespace navit_bt_nodes
{
class GetRoutePathService : public BT::RosServiceNode<navit_msgs::GetMapRoutePath>
{
public:
  GetRoutePathService(ros::NodeHandle& handle, const std::string& node_name, const BT::NodeConfiguration& conf)
    : RosServiceNode(handle, node_name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::InputPort<std::vector<std::string>>("route", "The route such as LM1, LM2, LM3, LM9"),
        BT::InputPort<double>("distance_interval", "The distance interval to gen path."),
        BT::OutputPort<std::string>("start", "The start station of the route."),
        BT::OutputPort<std::string>("end", "The end station of the route."),
        BT::OutputPort<nav_msgs::Path>("route_path", "The path of the route."),
    });
  }

  bool prepareRequest() override
  {
    std::vector<std::string> route;
    getInput("route", route);
    getInput("distance_interval", request_.distance_interval);

    if (!route.size())
    {
      return false;
    }

    request_.waypoints = route;
    setOutput("start", route.front());
    setOutput("end", route.back());
    return true;
  }

  BT::NodeStatus onResponse(const ResponseType& rep) override
  {
    setOutput("route_path", rep.path);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace navit_bt_nodes
#endif  // NAVIT_BT_NODES_GET_ROUTE_PATH_SERVICE_HPP
