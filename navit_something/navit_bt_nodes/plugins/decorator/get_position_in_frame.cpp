#include "navit_bt_nodes/plugins/decorator/get_position_in_frame.hpp"


namespace navit_bt_nodes
{

GetPositionInFrame::GetPositionInFrame(const std::string &name, const BT::NodeConfiguration &config)
    : BT::DecoratorNode(name, config), tf_listener_(tf_buffer_)
{
}

BT::PortsList GetPositionInFrame::providedPorts()
{
    return {BT::InputPort<std::string>("frame_id"),
            BT::InputPort<std::string>("expand_dis"),
            BT::OutputPort<geometry_msgs::PoseStamped>("current_pos")};
}

BT::NodeStatus GetPositionInFrame::tick()
{
    ros::Time current_time = ros::Time::now();
    auto frame_id_result = getInput<std::string>("frame_id");
    if (!frame_id_result)
    {
        NAVIT_ROS_ERROR_STREAM("frame_id is not specified");
        return BT::NodeStatus::FAILURE;
    }
    float expand_dis;
    getInput<float>("expand_dis", expand_dis);

    std::string frame_id = frame_id_result.value();

    geometry_msgs::PoseStamped robot_pose_in_frame;
    try
    {
        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped = tf_buffer_.lookupTransform(frame_id, "base_link", ros::Time(0));
        geometry_msgs::PoseStamped robot_pose;
        robot_pose.header.frame_id = "base_link";
        robot_pose.pose.orientation.w = 1.0;
        tf2::doTransform(robot_pose, robot_pose_in_frame, transform_stamped);
    }
    catch (tf2::TransformException &ex)
    {
        printf("%s",ex.what());
        return BT::NodeStatus::FAILURE;
    }
    tf::Quaternion quat;
    tf::quaternionMsgToTF(robot_pose_in_frame.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    robot_pose_in_frame.pose.position.x = robot_pose_in_frame.pose.position.x + expand_dis * cos(yaw);
    robot_pose_in_frame.pose.position.x = robot_pose_in_frame.pose.position.x + expand_dis * sin(yaw);

    setOutput("current_pos", robot_pose_in_frame);
    //
    return child_node_->executeTick();
}

} // namespace navit_bt_nodes

// Register GetPositionInFrame node
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder = [](const std::string &name, const BT::NodeConfiguration &config) {
        return std::make_unique<navit_bt_nodes::GetPositionInFrame>(name, config);
    };
    factory.registerBuilder<navit_bt_nodes::GetPositionInFrame>("GetPositionInFrame", builder);
}