#ifndef PROJECT_POSE_H
#define PROJECT_POSE_H

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

struct Pose {
   public:
    float x, y, r, confidence;

    Pose() : x(0.0), y(0.0), r(0.0), confidence(-100) {}

    Pose(float _x, float _y, float _r, float _confidence) : x(_x), y(_y), r(_r), confidence(_confidence) {}

    Pose(const nav_msgs::Odometry &odom)
        : Pose(odom.pose.pose.position.x, odom.pose.pose.position.y, tf::getYaw(odom.pose.pose.orientation), 0) {}

    static Pose createInvalid();

    float distance(const Pose &pose) const;

    bool isValid() const;
};

#endif  // PROJECT_POSE_H
