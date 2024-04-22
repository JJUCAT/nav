#ifndef GET_POSITION_IN_FRAME_H
#define GET_POSITION_IN_FRAME_H

#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>
#include <navit_common/log.h>

#include "behaviortree_cpp_v3/decorator_node.h"

namespace navit_bt_nodes
{

class GetPositionInFrame : public BT::DecoratorNode
{
public:
    GetPositionInFrame(const std::string &name, const BT::NodeConfiguration &config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

} // namespace navit_bt_nodes

#endif // GET_POSITION_IN_FRAME_H