#ifndef EXCAPE_ACTION_HPP_
#define EXCAPE_ACTION_HPP_

#include "navit_bt_nodes/bt_action_node.h"
#include <nav_msgs/Path.h>
#include <navit_msgs/ExcapeAction.h>
#include <memory>

namespace navit_bt_nodes
{

class ExcapeAction : public BT::ActionNodeBase
{
 public:

    ExcapeAction(const std::string& xml_tag_name, const BT::NodeConfiguration& conf)
      : BT::ActionNodeBase(xml_tag_name, conf)
    {
      ros::NodeHandle nh;
      client_ = std::make_shared<actionlib::SimpleActionClient<navit_msgs::ExcapeAction>>(nh, "excape", true);
    }

    static BT::PortsList providedPorts()
    {
      return {
        BT::InputPort<double>("timeout", "timeout"),
        BT::InputPort<double>("free", "free"),
        BT::InputPort<double>("v", "v"),
        BT::InputPort<double>("w", "w"),
        BT::InputPort<double>("r", "r"),
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

    void LoadArg(double& timeout, double& free, double& v, double& w, double& r);

    void halt() override;

    void ResultCallback(const actionlib::SimpleClientGoalState &state,
      const navit_msgs::ExcapeResultConstPtr &result);

    void ActiveCallback();

    void FeedbackCallback(const navit_msgs::ExcapeFeedbackConstPtr &feedback);

    BT::NodeStatus tick() override;

 private:

  const char* node_name_ = "excape_action";
  std::shared_ptr<actionlib::SimpleActionClient<navit_msgs::ExcapeAction>> client_;
  state_t state_ = kIdle;

}; // class ExcapeAction

}  // namespace navit_bt_nodes

#endif  // EXCAPE_ACTION_HPP_
