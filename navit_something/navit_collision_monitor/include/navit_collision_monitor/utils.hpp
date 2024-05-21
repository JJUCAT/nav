//
// Created by fan on 23-8-23.
//

#ifndef COLLISION_MONITOR_UTILS_HPP
#define COLLISION_MONITOR_UTILS_HPP

#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>

namespace collision_monitor_utils {

inline bool getTransform(const std::string &source_frame_id,
                         const std::string &target_frame_id,
                         const ros::Duration &transform_tolerance,
                         const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                         tf2::Transform &tf2_transform) {
  geometry_msgs::TransformStamped transform;
  tf2_transform.setIdentity(); // initialize by identical transform

  if (source_frame_id == target_frame_id) {
    // We are already in required frame
    return true;
  }

  try {
    // Obtaining the transform to get data from source to target frame
    transform = tf_buffer->lookupTransform(target_frame_id, source_frame_id,
                                           ros::Time(0), transform_tolerance);
  } catch (tf2::TransformException &e) {
    ROS_ERROR("Failed to get \"%s\"->\"%s\" frame transform: %s",
              source_frame_id.c_str(), target_frame_id.c_str(), e.what());
    return false;
  }

  // Convert TransformStamped to TF2 transform
  tf2::fromMsg(transform.transform, tf2_transform);
  return true;
}

inline bool getTransform(const std::string &source_frame_id,
                         const ros::Time &source_time,
                         const std::string &target_frame_id,
                         const ros::Time &target_time,
                         const std::string &fixed_frame_id,
                         const ros::Duration &transform_tolerance,
                         const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                         tf2::Transform &tf2_transform) {

  geometry_msgs::TransformStamped transform;
  tf2_transform.setIdentity(); // initialize by identical transform

  try {
    // Obtaining the transform to get data from source to target frame.
    // This also considers the time shift between source and target.
    transform = tf_buffer->lookupTransform(target_frame_id, target_time,
                                           source_frame_id, source_time,
                                           fixed_frame_id, transform_tolerance);
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("Failed to get \"%s\"->\"%s\" frame transform: %s",
              source_frame_id.c_str(), target_frame_id.c_str(), ex.what());
    return false;
  }

  // Convert TransformStamped to TF2 transform
  tf2::fromMsg(transform.transform, tf2_transform);
  return true;
}

inline bool validateTwist(const geometry_msgs::Twist &msg) {
  if (std::isinf(msg.linear.x) || std::isnan(msg.linear.x)) {
    return false;
  }

  if (std::isinf(msg.linear.y) || std::isnan(msg.linear.y)) {
    return false;
  }

  if (std::isinf(msg.linear.z) || std::isnan(msg.linear.z)) {
    return false;
  }

  if (std::isinf(msg.angular.x) || std::isnan(msg.angular.x)) {
    return false;
  }

  if (std::isinf(msg.angular.y) || std::isnan(msg.angular.y)) {
    return false;
  }

  if (std::isinf(msg.angular.z) || std::isnan(msg.angular.z)) {
    return false;
  }

  return true;
}

} // namespace collision_monitor_utils

#endif // COLLISION_MONITOR_UTILS_HPP
