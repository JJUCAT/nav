//
// Created by fan on 23-2-2.
//

#ifndef NAVIT_BT_NODES_GET_STATION_POSE_SERVICE_HPP
#define NAVIT_BT_NODES_GET_STATION_POSE_SERVICE_HPP

#include <navit_msgs/GetMapStationPose.h>
#include "navit_bt_nodes/bt_service_node.h"
#include <geometry_msgs/PoseStamped.h>

namespace navit_bt_nodes
{
class GetStationPoseService : public BT::RosServiceNode<navit_msgs::GetMapStationPose>
{
public:
  GetStationPoseService(ros::NodeHandle& handle, const std::string& node_name, const BT::NodeConfiguration& conf)
    : RosServiceNode(handle, node_name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::InputPort<std::string>("station", "station id."),
        BT::OutputPort<geometry_msgs::PoseStamped>("station_pose", "The pose of the station id"),
    });
  }

  bool prepareRequest() override
  {
    getInput("station", request_.station_name);
    return true;
  }

  BT::NodeStatus onResponse(const ResponseType& rep) override
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose = rep.pose;
    setOutput("station_pose", pose);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace navit_bt_nodes
#endif  // NAVIT_BT_NODES_GET_STATION_POSE_SERVICE_HPP
