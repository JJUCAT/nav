//
// Created by yjh on 23-2-24.
//
#include "path_follower/trajectory_view.h"
#include <nav_msgs/Path.h>

void TrajectoryViewer::Init() {
    ros::NodeHandle nh;

    smooth_path_pub_ = nh.advertise<nav_msgs::Path>("trajectory_controller_smooth_path", 8);

    prune_path_pub_ = nh.advertise<nav_msgs::Path>("/controller_cut_path", 8);

    anchor_point_pub_ = nh.advertise<visualization_msgs::Marker>("anchor_point", 8);

    emergency_bounding_box_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("emergency_bounding_box", 1);

    cushion_bounding_box_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("cushion_bounding_box", 1);

    replan_bounding_box_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("replan_bounding_box", 1);

    local_plan_pub_ = nh.advertise<nav_msgs::Path>("local_plan", 8);

    goal_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("goal_pose", 1);

    poses_lists_pub_ = nh.advertise<geometry_msgs::PoseArray>("anchor_poses", 1);

    local_smooth_path_pub_ = nh.advertise<nav_msgs::Path>("local_smooth_path", 1);

    polygons_pub_ = nh.advertise<visualization_msgs::MarkerArray>("polygons", 1);
}

// view global smooth path
void TrajectoryViewer::visualizePreSmoothPath(const proto::Path proto_path, const std::string frame) {
    nav_msgs::Path path;
    path.header.frame_id = frame;
    geometry_msgs::PoseStamped pose;
    for (int i = 0; i < proto_path.path_points().size(); ++i) {
        pose.pose.position.x = proto_path.path_points(i).position().x();
        pose.pose.position.y = proto_path.path_points(i).position().y();
        path.poses.push_back(pose);
    }
    smooth_path_pub_.publish(path);
}

void TrajectoryViewer::visualizeAnchorPoint(const geometry2::Vec2f vec2f_pose, const std::string frame) {
    visualization_msgs::Marker anchor_point_marker;

    anchor_point_marker.type = 7;
    anchor_point_marker.scale.x = 0.1;
    anchor_point_marker.scale.y = 0.1;
    anchor_point_marker.scale.z = 0.1;

    anchor_point_marker.color.r = 1.0f;
    anchor_point_marker.color.g = 0.0f;
    anchor_point_marker.color.b = 0.0f;
    anchor_point_marker.color.a = 1.0;

    anchor_point_marker.header.frame_id = frame;

    anchor_point_marker.pose.position.x = 0.0;
    anchor_point_marker.pose.position.y = 0.0;
    anchor_point_marker.pose.position.z = 0.0;

    anchor_point_marker.pose.orientation.x = 0.0;
    anchor_point_marker.pose.orientation.y = 0.0;
    anchor_point_marker.pose.orientation.z = 0.0;
    anchor_point_marker.pose.orientation.w = 1.0;

    geometry_msgs::Point point;

    point.x = vec2f_pose.x;
    point.y = vec2f_pose.y;
    point.z = 0.0;

    anchor_point_marker.points.push_back(point);
    anchor_point_pub_.publish(anchor_point_marker);
}

void TrajectoryViewer::visualizeLocalPlanGoal(const proto::Pose goal_pose, const std::string frame) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame;

    pose.pose.position.x = goal_pose.pose().x();
    pose.pose.position.y = goal_pose.pose().y();

    pose.pose.orientation.x = goal_pose.qua().x();
    pose.pose.orientation.y = goal_pose.qua().y();
    pose.pose.orientation.z = goal_pose.qua().z();
    pose.pose.orientation.w = goal_pose.qua().w();

    goal_pose_pub_.publish(pose);
}

void TrajectoryViewer::visualizeAnchorBoxes(geometry_msgs::PolygonStamped emergency_box,
                                            geometry_msgs::PolygonStamped cushion_box,
                                            geometry_msgs::PolygonStamped replan_box,
                                            const std::string frame) {
    emergency_box.header.frame_id = frame;
    emergency_bounding_box_pub_.publish(emergency_box);
    cushion_box.header.frame_id = frame;
    cushion_bounding_box_pub_.publish(cushion_box);
    replan_box.header.frame_id = frame;
    replan_bounding_box_pub_.publish(replan_box);
}

void TrajectoryViewer::visualizeAnchorBoxes(std::vector<geometry_msgs::PolygonStamped> polygons, const std::string frame) {
    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = frame;
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.ns = "anchor_boxes";
    delete_marker.id = 0;
    delete_marker.type = visualization_msgs::Marker::LINE_STRIP;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    polygons_pub_.publish(marker_array);

    marker_array.markers.clear();

    for (int i = 0; i < polygons.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame;
        marker.header.stamp = ros::Time::now();
        marker.ns = "anchor_boxes_" + std::to_string(i);
        marker.id = i; 
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        //marker.lifetime = ros::Duration(0.1);
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
       
        marker.pose.position.z = 0.0;
        for (int j = 0; j < polygons[i].polygon.points.size(); ++j) {

            geometry_msgs::Point point;
            point.x = polygons[i].polygon.points[j].x;
            point.y = polygons[i].polygon.points[j].y;
            point.z = 0.0;
            marker.points.push_back(point);
        }
        marker_array.markers.push_back(marker);
    }
    polygons_pub_.publish(marker_array);

}
void TrajectoryViewer::visualizeAnchorPoses(const proto::AnchorBox& anchor_boxes, const std::string frame) {
    geometry_msgs::PoseArray pose_array;
    pose_array.header.frame_id = frame;
    for (int i = 0; i < anchor_boxes.anchor_pose_size(); ++i) {
        geometry_msgs::Pose pose;
        pose.position.x = anchor_boxes.anchor_pose(i).pose().x();
        pose.position.y = anchor_boxes.anchor_pose(i).pose().y();

        pose.orientation.x = anchor_boxes.anchor_pose(i).qua().x();
        pose.orientation.y = anchor_boxes.anchor_pose(i).qua().y();
        pose.orientation.z = anchor_boxes.anchor_pose(i).qua().z();
        pose.orientation.w = anchor_boxes.anchor_pose(i).qua().w();

        pose_array.poses.push_back(pose);
    }
    poses_lists_pub_.publish(pose_array);
}

void TrajectoryViewer::visualizePruneGlobalPath(const proto::Path& path, const std::string frame) {
    nav_msgs::Path prune_path;
    prune_path.header.frame_id = frame;

    for (int i = 0; i < path.path_points_size(); ++i) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = path.path_points(i).position().x();
        pose.pose.position.y = path.path_points(i).position().y();
        prune_path.poses.push_back(pose);
    }
    prune_path_pub_.publish(prune_path);
}

void TrajectoryViewer::visualizeLocalPath(nav_msgs::Path& path, const std::string frame) {
    path.header.frame_id = frame;
    local_plan_pub_.publish(path);
}

void TrajectoryViewer::visualizeLocalSmoothPath(const proto::Path path_proto, const std::string frame) {
    nav_msgs::Path path_ros;
    geometry_msgs::PoseStamped pose;
    path_ros.header.frame_id = frame;
    for (int i = 0; i < path_proto.path_points().size(); ++i) {
        pose.pose.position.x = path_proto.path_points(i).position().x();
        pose.pose.position.y = path_proto.path_points(i).position().y();
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;
        path_ros.poses.push_back(pose);
    }
    local_smooth_path_pub_.publish(path_ros);
}