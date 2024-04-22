#ifndef NAVIT_BT_NODES__PLUGINS__ACTION__LOAD_POLYGON_JSON_SERVICE_HPP_
#define NAVIT_BT_NODES__PLUGINS__ACTION__LOAD_POLYGON_JSON_SERVICE_HPP_

#include "navit_msgs/LoadPolygonJson.h"
#include "navit_bt_nodes/bt_service_node.h"
#include <fstream>
namespace navit_bt_nodes
{

class LoadPolygonJsonService : public BT::RosServiceNode<navit_msgs::LoadPolygonJson>
{
public:
  LoadPolygonJsonService(ros::NodeHandle& handle, const std::string& node_name, const BT::NodeConfiguration& conf)
    : RosServiceNode(handle, node_name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>("json_file_path", "json file path.")
    });
  }

  bool prepareRequest() override
  {
    auto json_data = getInput<std::string>("json_file_path");
    if (!json_data)
    {
      return false;
    }
    this->request_.polygon_json_path = *json_data;
    return true;
  }

  BT::NodeStatus onResponse(const ResponseType& rep) override
  {
    if (rep.success)
    {
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return BT::NodeStatus::FAILURE;
    }
  }
};

}  // namespace navit_bt_nodes

#endif  // NAVIT_BT_NODES__PLUGINS__ACTION__LOAD_POLYGON_JSON_SERVICE_HPP_