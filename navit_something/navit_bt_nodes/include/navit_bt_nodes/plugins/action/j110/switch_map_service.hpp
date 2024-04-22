#ifndef NAVIT_BT_NODES__PLUGINS__ACTION__SWITCH_MAP_SERVICE_HPP_
#define NAVIT_BT_NODES__PLUGINS__ACTION__SWITCH_MAP_SERVICE_HPP_

#include <std_srvs/Empty.h>
#include "navit_bt_nodes/bt_service_node.h"
#include <navit_msgs/SwitchTaskMap.h>
namespace navit_bt_nodes
{

class SwitchMapService : public BT::RosServiceNode<navit_msgs::SwitchTaskMap>
{
public:
  SwitchMapService(ros::NodeHandle& handle, const std::string& node_name, const BT::NodeConfiguration& conf)
    : RosServiceNode(handle, node_name, conf)
  {
  }
  static BT::PortsList providedPorts() {
    return providedBasicPorts({
      BT::InputPort<std::string>("switch_map_name", "the map needed switch")
    });
  }

  bool prepareRequest() override {
    if (!getInput<std::string>("switch_map_name")) {
      ROS_ERROR("switch_map_name is not specified");
      return false;
    }

    request_.map_name = getInput<std::string>("switch_map_name").value();
    return true;
  }

  BT::NodeStatus onResponse(const ResponseType& rep) override {
    if (rep.success == false) {
        ROS_ERROR("Failed to switch map");
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace navit_bt_nodes

#endif  // NAVIT_BT_NODES__PLUGINS__ACTION__SWITCH_MAP_SERVICE_HPP_
