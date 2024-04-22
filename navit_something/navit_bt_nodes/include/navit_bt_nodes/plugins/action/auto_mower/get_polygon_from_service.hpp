#ifndef NAVIT_BT_NODES__PLUGINS__ACTION__GET_SELECTION_SERVICE_HPP_
#define NAVIT_BT_NODES__PLUGINS__ACTION__GET_SELECTION_SERVICE_HPP_

#include <geometry_msgs/PointStamped.h>
#include <jsoncpp/json/json.h>
#include <fstream>

#include "navit_utils/GetSelection.h"
#include "navit_bt_nodes/bt_service_node.h"

namespace navit_bt_nodes
{

class GetSelectionPolygonService : public BT::RosServiceNode<navit_utils::GetSelection>
{
public:
  GetSelectionPolygonService(ros::NodeHandle& handle, const std::string& node_name, const BT::NodeConfiguration& conf)
    : RosServiceNode(handle, node_name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      //BT::InputPort<std::string>("polygon_server_name", "Name of the polygon to get from the server"),
      BT::InputPort<bool>("is_saved", "do you need to get a saved polygon from the server?"),
      BT::InputPort<std::string>("saved_path", "path to the saved polygon file"),
      BT::OutputPort<std::vector<geometry_msgs::PointStamped>>("polygon", "get polygon points from rviz"),
    });
  }

  bool prepareRequest() override
  {
    return true;
  }

  BT::NodeStatus onResponse(const ResponseType& rep) override
  {
    Json::Value json;
    for (const auto& point : rep.selection)
    {
      // Json::Value point_json;
      // point_json["x"] = point.point.x;
      // point_json["y"] = point.point.y;
      // json["polygon"].append(point_json);
    }

    // Convert the JSON object to a string and set it as the output port value.
    Json::StreamWriterBuilder writer;
    auto json_str = Json::writeString(writer, json);

    std::string saved_path;
    if (!getInput("saved_path", saved_path))
    {
      return BT::NodeStatus::FAILURE;
    }

    // Save JSON string to a file.
    std::ofstream file(saved_path);
    if (!file)
    {
      // Error opening the file.
      return BT::NodeStatus::FAILURE;
    }

    file << json_str;
    file.close();

    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace navit_bt_nodes

#endif  // NAVIT_BT_NODES__PLUGINS__ACTION__GET_SELECTION_SERVICE_HPP_