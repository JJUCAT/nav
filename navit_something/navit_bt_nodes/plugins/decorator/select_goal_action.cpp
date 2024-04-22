#include "navit_bt_nodes/plugins/decorator/select_goal_action.hpp"

namespace navit_bt_nodes
{

BT::PortsList SelectGoalAction::providedPorts()
{
    return {
        BT::InputPort<nav_msgs::Path>("reference_path", "Reference path"),
        BT::InputPort<geometry_msgs::Pose>("current_pos", "Current position of the agent"),
        BT::InputPort<double>("step_length", 1.0f, "step length to the next target point"),
        BT::OutputPort<geometry_msgs::PoseStamped>("target_point", "Selected target point")
    };
}

BT::NodeStatus SelectGoalAction::tick() {
    tick_count_++;
    // Retrieve the current position of the robot
    geometry_msgs::Pose current_pos;
    if (!getInput<geometry_msgs::Pose>("current_pos", current_pos)) {
        return BT::NodeStatus::FAILURE;
    }
    // Retrieve the reference path
    nav_msgs::Path reference_path;
    if (!getInput<nav_msgs::Path>("reference_path", reference_path)) {
        return BT::NodeStatus::FAILURE;
    }

    double step_length = 0.0;
    if (!getInput<double>("step_length", step_length)) {
        return BT::NodeStatus::FAILURE;
    }
    // update step_length
    step_length *= tick_count_; 
    // Find the closest point on the path
    double min_dist = std::numeric_limits<double>::max();
    nav_msgs::Path::_poses_type::iterator closest_pose_iter;
    
    if (reference_path.poses.size() < 5) {
        return BT::NodeStatus::FAILURE;
    }
    for (auto it = reference_path.poses.begin(); it != reference_path.poses.end(); ++it) {
        double dist = std::hypot(it->pose.position.x - current_pos.position.x,
                                it->pose.position.y - current_pos.position.y);
        if (dist < min_dist) {
            min_dist = dist;
            closest_pose_iter = it;
        }
    }

    geometry_msgs::Pose closest_pose = closest_pose_iter->pose;
    // Iterate from the closest_pose iterator to both sides until the path is completely traversed
    bool found_target_point = false;
    auto left_iter = closest_pose_iter;
    auto right_iter = closest_pose_iter;

    while (left_iter != reference_path.poses.begin() || right_iter != reference_path.poses.end()) {
        if (left_iter != reference_path.poses.begin()) {
            --left_iter;
            double dist = std::hypot(left_iter->pose.position.x - closest_pose.position.x, left_iter->pose.position.y - closest_pose.position.y);
            if (fabs(dist) > step_length) {
                goal_point_.pose = left_iter->pose;
                found_target_point = true;
                break;
            }
        }

        if (right_iter != reference_path.poses.end()) {
            ++right_iter;
            double dist = std::hypot(right_iter->pose.position.x - closest_pose.position.x, right_iter->pose.position.y - closest_pose.position.y);
            if (fabs(dist) > step_length) {
                goal_point_.pose = right_iter->pose;
                found_target_point = true;
                break;
            }
        }
    }

    // if (tick_count_ >= 10) {
    //     tick_count_ = 0;
    //     return BT::NodeStatus::FAILURE;
    // }

    if (!found_target_point && left_iter == reference_path.poses.begin() && right_iter == reference_path.poses.end()) {
        tick_count_ = 0;
        return BT::NodeStatus::FAILURE;
    }
    goal_point_.header.frame_id = "map";
    setOutput<geometry_msgs::PoseStamped>("target_point", goal_point_);
     return child_node_->executeTick();
    //return BT::NodeStatus::SUCCESS;
}
} //namespace navit_bt_nodes
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navit_bt_nodes::SelectGoalAction>("SelectNearestPose");
}
