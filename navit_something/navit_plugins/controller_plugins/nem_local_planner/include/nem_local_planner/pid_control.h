//
// Created by czh on 8/18/21.
//

#ifndef SRC_PID_FILTER_H
#define SRC_PID_FILTER_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>

namespace pid_control {
class PidControl {
   public:
    PidControl();

    virtual ~PidControl();

    float getPidControl(double Kp, double Ki, double Kd, double max);

    void setPathAndPose(const std::vector<geometry_msgs::PoseStamped> &path, const geometry_msgs::PoseStamped &pose, const double &vx, const double &left_dist);

    double getAngleDiff(const geometry_msgs::PoseStamped &pose, const geometry_msgs::PoseStamped &target);

    double purePursitFunctionCost(double distance, double theta);

    bool translationP(const geometry_msgs::PoseStamped &A, const geometry_msgs::PoseStamped &B, const geometry_msgs::PoseStamped &P);

    double stanleyFunctionCost(double d, double v, double theta, double k, double dist_k, double left_dis);

    void headPoseMarkerPub(const geometry_msgs::PoseStamped &head_pose);

    double getOffsetDist(const geometry_msgs::PoseStamped &P, const geometry_msgs::PoseStamped &A, const geometry_msgs::PoseStamped &B);

    void reSet(const std::string &cmd);

   private:
    ros::Publisher reset_pub_;
    ros::Publisher head_pose_pub_;
    double error_last_ = 0.0;
    double integral_   = 0.0;
    double angle_diff_ = 0.0;
    double head_dist_  = 0.0;
    double error_      = 0.0;
    double offset_dist_= 0.0;
};
}  // namespace pid_control

#endif  // SRC_PID_FILTER_H
