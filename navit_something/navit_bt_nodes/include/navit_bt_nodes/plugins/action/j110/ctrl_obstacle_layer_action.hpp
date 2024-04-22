#ifndef CTRL_OBSTACLE_LAYER_ACTION_HPP_
#define CTRL_OBSTACLE_LAYER_ACTION_HPP_

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

class CtrlObstacleLayerAction : public BT::ActionNodeBase
{
 public:

    CtrlObstacleLayerAction(const std::string& xml_tag_name, const BT::NodeConfiguration& conf)
      : BT::ActionNodeBase(xml_tag_name, conf)
    {
    }

    static BT::PortsList providedPorts()
    {
      return {
        BT::InputPort<std::string>("map_name", "map_name"),
        BT::InputPort<bool>("enable", "enable or disenable"),
      };
    }

 private:

    void halt() override {}

    BT::NodeStatus tick() override {
      std::string map_name;
      bool enable;
      LoadArg(map_name, enable);

      std::string operation {"rosrun dynamic_reconfigure dynparam set "};
      std::string obs_node = {map_name+"/obstacle_layer "};
      std::string param = "enabled ";
      std::string value {enable?"true":"false"};
      std::string obs_cmd = operation+obs_node+param+value;
      ROS_INFO("[BT][%s] sys cmd [%s]", node_name_, obs_cmd.c_str());
      system(obs_cmd.c_str());

      std::string inf_node = {map_name+"/inflation_layer "};      
      std::string inf_cmd = operation+inf_node+param+value;
      ROS_INFO("[BT][%s] sys cmd [%s]", node_name_, inf_cmd.c_str());
      system(inf_cmd.c_str());

      return BT::NodeStatus::SUCCESS;
    };

    void LoadArg(std::string& map_name, bool& enable)
    {
      if (!getInput<std::string>("map_name", map_name)) {
        std::string msg("missing arg [map_name]");
        ROS_ERROR("[BT][%s] %s", node_name_, msg.c_str());
        throw(BT_Exception(msg));
      }
      ROS_INFO("[BT][%s] control map [%s] obstacle layer", node_name_, map_name.c_str());

      if (!getInput<bool>("enable", enable)) {
        std::string msg("missing arg [enable]");
        ROS_ERROR("[BT][%s] %s", node_name_, msg.c_str());
        throw(BT_Exception(msg));
      }
      ROS_INFO("[BT][%s] enable is %s", node_name_, enable?"true":"false");
    }

 private:
    const char* node_name_ = "ctrl_obstacle_layer_action";

}; // class CtrlObstacleLayerAction

}  // namespace navit_bt_nodes

#endif  // CTRL_OBSTACLE_LAYER_ACTION_HPP_
