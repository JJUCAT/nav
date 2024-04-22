#ifndef CLEAR_MAP_ACTION_HPP_
#define CLEAR_MAP_ACTION_HPP_

#include <string>
#include <google/protobuf/util/json_util.h>
#include <navit_common/log.h>
#include <tf/tf.h>
#include "navit_bt_nodes/bt_action_node.h"
#include "navit_bt_nodes/bt_conversions.hpp"
#include <navit_bt_nodes/bt_exception.hpp>
#include "ros/node_handle.h"
#include <std_srvs/Empty.h>

namespace navit_bt_nodes
{

class ClearMapAction : public BT::ActionNodeBase
{
 public:

    ClearMapAction(const std::string& xml_tag_name, const BT::NodeConfiguration& conf)
      : BT::ActionNodeBase(xml_tag_name, conf)
    {
    }

    static BT::PortsList providedPorts()
    {
      return {
        BT::InputPort<std::string>("map_name", "map_name"),
      };
    }

 private:

    void halt() override {}

    BT::NodeStatus tick() override {
      std::string map_name;
      LoadArg(map_name);

      std::string server_map = "/clear_"+map_name;
      ROS_INFO("[BT][%s] server name [%s]", node_name_, server_map.c_str());

      ros::NodeHandle pnh("~");
      ros::ServiceClient clear_cli_ = pnh.serviceClient<std_srvs::Empty>(server_map.c_str());

      double timeout = 3.0;
      ros::service::waitForService(server_map.c_str(), timeout);
      
      std_srvs::Empty empty_msg;
      clear_cli_.call(empty_msg);
      usleep(500000);
      return BT::NodeStatus::SUCCESS;
    };

    void LoadArg(std::string& map_name)
    {
      if (!getInput<std::string>("map_name", map_name)) {
        std::string msg("missing arg [map_name]");
        ROS_ERROR("[BT][%s] %s", node_name_, msg.c_str());
        throw(BT_Exception(msg));
      }
      ROS_INFO("[BT][%s] clear map [%s]", node_name_, map_name.c_str());
    }

 private:
    const char* node_name_ = "clear_map_action";

}; // class ClearMapAction

}  // namespace navit_bt_nodes

#endif  // CLEAR_MAP_ACTION_HPP_
