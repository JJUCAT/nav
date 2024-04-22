#include "ekf.h"

namespace navit_auto_dock {
namespace plugins {
    void PerceptionEKF::initialize(const std::string& name,
                                   const std::shared_ptr<tf2_ros::Buffer>& tf)
    {
        ros::NodeHandle pnh("~/" + name);
        tf_buffer_ = tf;

        pnh.param("sigma_w",config_.sigma_w, config_.sigma_w);
        pnh.param("sigma_u",config_.sigma_u, config_.sigma_u);
        pnh.param("error_cov",config_.error_cov, config_.error_cov);
        pnh.param("dt", config_.dt, config_.dt);
        pnh.param("base_frame_id", config_.base_frame_id, config_.base_frame_id);
        measurement_pub_= pnh.advertise<geometry_msgs::PoseStamped>("ekf_measurement",1);
        auto filter_ = std::make_shared<cv::KalmanFilter>();
    }

    void PerceptionEKF::reset()
    {
       initKalmanFilter(); 
    }

    void PerceptionEKF::update(geometry_msgs::PoseStamped& dock_pose,
                               const geometry_msgs::Twist& current_vel)
    {
         predictKalmanFilter(current_vel);
         updateKalmanFilter(dock_pose);
    }
    
    void PerceptionEKF::initKalmanFilter()
    {
       auto& sigma_w = config_.sigma_w;
       auto& sigma_u = config_.sigma_u;
       auto& error_cov = config_.error_cov;
    
       ROS_INFO("sigma_w is [%f, %f, %f]", sigma_w[0],sigma_w[1],sigma_w[2]);
    
       filter_.reset(new cv::KalmanFilter);
       
       filter_->init(3,3,2);
    
       cv::Mat_<float> measurement(3,1);
       measurement.setTo(cv::Scalar(0));
       filter_->statePost.at<float>(0) = 0;
       filter_->statePost.at<float>(1) = 0;
       filter_->statePost.at<float>(2) = 0;
       filter_->transitionMatrix = (cv::Mat_<float>(3,3) << 1,0,0, 0,1,0, 0,0,1) ;
       filter_->measurementMatrix = (cv::Mat_<float>(3,3) << 1,0,0, 0,1,0, 0,0,1) ;
       filter_->controlMatrix = (cv::Mat_<float>(3,2) << 1,1,1,
                                                         1,1,1) ;
    
       // set initial covirance matrix
       filter_->processNoiseCov = (cv::Mat_<float>(3,3) << std::pow(sigma_w[0],2), 0, 0,
                                                            0, std::pow(sigma_w[1],2), 0, 
                                                            0, 0, std::pow(sigma_w[2],2));
    
       filter_->measurementNoiseCov = (cv::Mat_<float>(3,3) << std::pow(sigma_u[0],2), 0, 0,
                                                                0, std::pow(sigma_u[1],2), 0,
                                                                0, 0, std::pow(sigma_u[2], 2));
    
       filter_->errorCovPost = (cv::Mat_<float>(3,3) << error_cov[0], 0, 0,
                                                       0, error_cov[1], 0,
                                                       0, 0, error_cov[2]);
    }
    
    void PerceptionEKF::predictKalmanFilter(const geometry_msgs::Twist& cmd_vel)
    {
        double vx = cmd_vel.linear.x;
        double omega = cmd_vel.angular.z;
        double dt = config_.dt;
    
        double xKF = filter_->statePost.at<float>(0);
        double yKF = filter_->statePost.at<float>(1);
        double thetaKF = filter_->statePost.at<float>(2);
    
        filter_->transitionMatrix = (cv::Mat_<float>(3,3) << 1, 0, vx * std::sin(thetaKF)*dt,
                                                             0, 1, -vx *std::cos(thetaKF)*dt,
                                                             0, 0, 1 );
        filter_->controlMatrix = (cv::Mat_<float>(3,2) << -std::cos(thetaKF)*dt, 0,
                                                          -std::sin(thetaKF)*dt, 0,
                                                           0, -1);
        cv::Mat_<float> control(2,1);
        control.at<float>(0) = vx;
        control.at<float>(1) = omega;
        cv::Mat prediction = filter_->predict(control);
    
        filter_->statePre.at<float>(0) = xKF - vx * std::cos(thetaKF) * dt;
        filter_->statePre.at<float>(1) = yKF - vx * std::sin(thetaKF) * dt;
        filter_->statePre.at<float>(2) = thetaKF - omega * dt;
    }
    
    void PerceptionEKF::updateKalmanFilter(geometry_msgs::PoseStamped& pose)
    {
        // transform to base frame
        geometry_msgs::PoseStamped local_pose;
        local_pose.header.frame_id = config_.base_frame_id;
        std::string dock_pose_frame = pose.header.frame_id;
        try
        {
            tf_buffer_->transform(pose,local_pose, "base_link", ros::Duration(0.1));    
        }
        catch(tf2::TransformException& e)
        {
            ROS_WARN("failed to transform %s",e.what());
        }
        cv::Mat_<float> measurement(3,1);
        measurement.at<float>(0) = local_pose.pose.position.x;
        measurement.at<float>(1) = local_pose.pose.position.y;
        measurement.at<float>(2) = tf2::getYaw(local_pose.pose.orientation);
    
        cv::Mat estimate = filter_->correct(measurement);
        filter_->temp5.at<float>(0) = measurement.at<float>(0) - filter_->statePre.at<float>(0);
        filter_->temp5.at<float>(1) = measurement.at<float>(1) - filter_->statePre.at<float>(1);
        filter_->temp5.at<float>(2) = measurement.at<float>(2) - filter_->statePre.at<float>(2);
        filter_->statePost = filter_->statePre + filter_->gain * filter_->temp5;
    
        tf2::Quaternion q;
        geometry_msgs::Quaternion q_msg;
        q.setRPY(0,0,filter_->statePost.at<float>(2));
        q_msg = tf2::toMsg(q);
        local_pose.pose.position.x = filter_->statePost.at<float>(0);
        local_pose.pose.position.y = filter_->statePost.at<float>(1);
        local_pose.pose.orientation = q_msg;
        local_pose.header.stamp = ros::Time::now();
        measurement_pub_.publish(local_pose);
        // transform back to global frame
        try
        {
            tf_buffer_->transform(local_pose, pose, dock_pose_frame,ros::Duration(0.1));
        }
        catch ( tf2::TransformException& e)
        {
            ROS_WARN("failed to transform %s", e.what());
        }
        ROS_ERROR_STREAM("new measurement kf (" << pose.pose.position.x << ", "
                                                 << pose.pose.position.y << ", "
                                                 << tf2::getYaw(pose.pose.orientation) << ") ");
        pose.header.frame_id = dock_pose_frame;
        pose.header.stamp = ros::Time::now();
    }
}
}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(navit_auto_dock::plugins::PerceptionEKF, navit_auto_dock::plugins::ApproachDockFilter)

