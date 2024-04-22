
#ifndef NAVIT_BT_NODES_PLUGINS_ACTION_GET_CURRENT_CONTROLLER_PLUGIN_SERVICE_HPP_
#define NAVIT_BT_NODES_PLUGINS_ACTION_GET_CURRENT_CONTROLLER_PLUGIN_SERVICE_HPP_

#include <navit_move_base/LoadNemData.h>
#include "navit_bt_nodes/bt_service_node.h"

namespace navit_bt_nodes
{
class GetControllerPluginService : public BT::RosServiceNode<navit_move_base::LoadNemData>
{
public:
  GetControllerPluginService(ros::NodeHandle& handle, const std::string& node_name, const BT::NodeConfiguration& conf)
    : RosServiceNode(handle, node_name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  bool prepareRequest() override
  {
    return true;
  }

  BT::NodeStatus onResponse(const ResponseType& rep) override
  {
    return BT::NodeStatus::SUCCESS;
  }

  bool serviceCallback(navit_move_base::LoadNemData::Request& req, navit_move_base::LoadNemData::Response& res)
  {
    ROS_INFO("GetControllerPluginService serviceCallback Info: %s", res.message.c_str());
    return true;
  }
};
}  // namespace navit_bt_nodes

#endif  // NAVIT_BT_NODES_PLUGINS_ACTION_GET_CURRENT_CONTROLLER_PLUGIN_SERVICE_HPP_
