//
// Created by fan on 23-8-15.
//

#ifndef NAVIT_BT_NODES_GET_AREA_EDGE_SERVICE_H
#define NAVIT_BT_NODES_GET_AREA_EDGE_SERVICE_H

#include <navit_msgs/GetMapAreaEdge.h>
#include "navit_bt_nodes/bt_service_node.h"
#include <boost/algorithm/string.hpp>

namespace navit_bt_nodes
{
class GetAreaEdgeService : public BT::RosServiceNode<navit_msgs::GetMapAreaEdge>
{
public:
  GetAreaEdgeService(ros::NodeHandle& handle, const std::string& node_name, const BT::NodeConfiguration& conf)
    : RosServiceNode(handle, node_name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::InputPort<std::string>("area", "The area such as A1"),
        BT::OutputPort<nav_msgs::Path>("edge_path", "The edge of the area."),
    });
  }

  bool prepareRequest() override
  {
    getInput("area", request_.area_name);
    return true;
  }

  BT::NodeStatus onResponse(const ResponseType& rep) override
  {
    nav_msgs::Path out_path = rep.path;
    setOutput("edge_path", out_path);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace navit_bt_nodes

#endif  // NAVIT_BT_NODES_GET_AREA_EDGE_SERVICE_H
