#include "lkf.h"
#include <memory>
#include "tf2/LinearMath/Quaternion.h"

namespace navit_auto_dock {
namespace plugins {
  void PerceptionLKF::initialize(const std::string& name,
                                  const std::shared_ptr<tf2_ros::Buffer>& tf)
  {
    ros::NodeHandle pnh("~/" + name);
    tf_buffer_ = tf;

    pnh.param("x_estimate_noise", x_noise_.estimate_sigma, x_noise_.estimate_sigma);
    pnh.param("x_measure_noise", x_noise_.measure_sigma, x_noise_.measure_sigma);
    pnh.param("x_project_noise", x_noise_.project_sigma, x_noise_.project_sigma);
    pnh.param("y_estimate_noise", y_noise_.estimate_sigma, y_noise_.estimate_sigma);
    pnh.param("y_measure_noise", y_noise_.measure_sigma, y_noise_.measure_sigma);
    pnh.param("y_project_noise", y_noise_.project_sigma, y_noise_.project_sigma);
    pnh.param("yaw_estimate_noise", yaw_noise_.estimate_sigma, yaw_noise_.estimate_sigma);
    pnh.param("yaw_measure_noise", yaw_noise_.measure_sigma, yaw_noise_.measure_sigma);
    pnh.param("yaw_project_noise", yaw_noise_.project_sigma, yaw_noise_.project_sigma);

    x_filter_ = std::make_shared<navit_auto_dock::Static_LKF>();
    y_filter_ = std::make_shared<navit_auto_dock::Static_LKF>();
    yaw_filter_ = std::make_shared<navit_auto_dock::Static_LKF>();
    init();

    estimate_pose_pub_= pnh.advertise<geometry_msgs::PoseStamped>("estimate_pose",1);
    predict_pose_pub_= pnh.advertise<geometry_msgs::PoseStamped>("predict_pose",1);
  }

  void PerceptionLKF::reset()
  {
    filter_init_ = false;
    init();
  }

  void PerceptionLKF::update(geometry_msgs::PoseStamped& dock_pose,
                              const geometry_msgs::Twist& current_vel)
  {
    dock_pose = updateFilter(dock_pose);      
    predictFilter(current_vel);
  }
  
  void PerceptionLKF::init()
  {
    x_filter_->setNoise(x_noise_.estimate_sigma, x_noise_.measure_sigma, x_noise_.project_sigma);
    y_filter_->setNoise(y_noise_.estimate_sigma, y_noise_.measure_sigma, y_noise_.project_sigma);
    yaw_filter_->setNoise(yaw_noise_.estimate_sigma, yaw_noise_.measure_sigma, yaw_noise_.project_sigma);
  }

  geometry_msgs::PoseStamped PerceptionLKF::updateFilter(const geometry_msgs::PoseStamped& pose)
  {
    if (!filter_init_) {
      filter_init_ = true;
      x_filter_->init(pose.pose.position.x);
      y_filter_->init(pose.pose.position.y);
      yaw_filter_->init(tf2::getYaw(pose.pose.orientation));
    } else {
      x_filter_->update(pose.pose.position.x);
      y_filter_->update(pose.pose.position.y);
      yaw_filter_->update(tf2::getYaw(pose.pose.orientation));
    }
    ROS_INFO("[LKF] update [%f,%f,%f]", x_filter_->get(), y_filter_->get(), yaw_filter_->get());

    geometry_msgs::PoseStamped p = pose;
    pose_frame_ = p.header.frame_id;
    p.header.stamp = ros::Time::now();
    p.pose.position.x = x_filter_->get();
    p.pose.position.y = y_filter_->get();
    p.pose.position.z = 0;
    tf2::Quaternion q; q.setRPY(0, 0, yaw_filter_->get());
    p.pose.orientation = tf2::toMsg(q);
    estimate_pose_pub_.publish(p);
    return p;
  }

  void PerceptionLKF::predictFilter(const geometry_msgs::Twist& cmd_vel)
  {
    x_filter_->predict();
    y_filter_->predict();
    yaw_filter_->predict();
    ROS_INFO("[LKF] predict [%f,%f,%f]", x_filter_->get(), y_filter_->get(), yaw_filter_->get());

    geometry_msgs::PoseStamped p;
    p.header.frame_id = pose_frame_;
    p.header.stamp = ros::Time::now();
    p.pose.position.x = x_filter_->get();
    p.pose.position.y = y_filter_->get();
    p.pose.position.z = 0;
    tf2::Quaternion q; q.setRPY(0, 0, yaw_filter_->get());
    p.pose.orientation = tf2::toMsg(q);
    predict_pose_pub_.publish(p);
  }

}
}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(navit_auto_dock::plugins::PerceptionLKF, navit_auto_dock::plugins::ApproachDockFilter)

