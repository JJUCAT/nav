#ifndef MOVE_BACK_ACTION_HPP_
#define MOVE_BACK_ACTION_HPP_

#include "navit_bt_nodes/bt_action_node.h"
#include <nav_msgs/Path.h>
#include <navit_msgs/BackUpAction.h>
#include <memory>

namespace navit_bt_nodes
{

class MoveBackAction : public BT::ActionNodeBase
{
 public:

    MoveBackAction(const std::string& xml_tag_name, const BT::NodeConfiguration& conf)
      : BT::ActionNodeBase(xml_tag_name, conf)
    {
      ros::NodeHandle nh;
      client_ = std::make_shared<actionlib::SimpleActionClient<navit_msgs::BackUpAction>>(nh, "backup", true);
    }

    static BT::PortsList providedPorts()
    {
      return {
        BT::InputPort<double>("back", "back"),
        BT::InputPort<double>("speed", "speed"),
      };
    }

 private:
    enum state_t
    {
      kIdle,
      kRunning,
      kSucceed,
      kFailed,
    };

    void LoadArg(double& back, double& speed);

    void halt() override;

    void ResultCallback(const actionlib::SimpleClientGoalState &state,
      const navit_msgs::BackUpResultConstPtr &result);

    void ActiveCallback();

    void FeedbackCallback(const navit_msgs::BackUpFeedbackConstPtr &feedback);

    BT::NodeStatus tick() override;

 private:

  const char* node_name_ = "move_back_action";
  std::shared_ptr<actionlib::SimpleActionClient<navit_msgs::BackUpAction>> client_;
  state_t state_ = kIdle;

}; // class MoveBackAction

}  // namespace navit_bt_nodes

#endif  // MOVE_BACK_ACTION_HPP_
