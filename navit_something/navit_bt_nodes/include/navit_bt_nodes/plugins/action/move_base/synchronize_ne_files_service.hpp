
#ifndef NAVIT_BT_NODES__PLUGINS__ACTION__SYNCHRONIZE_NE_FILES_SERVICE_HPP_
#define NAVIT_BT_NODES__PLUGINS__ACTION__SYNCHRONIZE_NE_FILES_SERVICE_HPP_

#include <navit_move_base/LoadNemData.h>
#include "navit_bt_nodes/bt_service_node.h"

namespace navit_bt_nodes
{
class SynchronizeNeFilesService : public BT::RosServiceNode<navit_move_base::LoadNemData>
{
public:
  SynchronizeNeFilesService(ros::NodeHandle& handle, const std::string& node_name, const BT::NodeConfiguration& conf)
    : RosServiceNode(handle, node_name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::InputPort<std::string>("nodes_path", "nodes directory path"),
        BT::InputPort<std::string>("edges_path", "edges directory path"),
    });
  }

  bool prepareRequest() override
  {
    BT::Result get_node_file_path_result = getInput<std::string>("nodes_path", request_.node_file_path);
    BT::Result get_edges_file_path_result = getInput<std::string>("edges_path", request_.edge_file_path);
    
    // getInput("nodes_path", request_.node_file_path);
    // getInput("edges_path", request_.edge_file_path);

    if ((!get_node_file_path_result) || (!get_edges_file_path_result))
    {
      ROS_ERROR("Failed to get input [data]: %s", get_node_file_path_result.error().c_str());
      ROS_ERROR("Failed to get input [data]: %s", get_edges_file_path_result.error().c_str());
      return false;
    }
    return true;
  }

  BT::NodeStatus onResponse(const ResponseType& rep) override
  {
    return BT::NodeStatus::SUCCESS;
  }

  bool serviceCallback(navit_move_base::LoadNemData::Request& req, navit_move_base::LoadNemData::Response& res)
  {
    return true;
  }
};
}  // namespace navit_bt_nodes

#endif  // NAVIT_BT_NODES__PLUGINS__ACTION__SYNCHRONIZE_NE_FILES_SERVICE_HPP_