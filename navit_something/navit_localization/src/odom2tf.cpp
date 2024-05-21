

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;

    // Set translation
    transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));

    // Set rotation
    q.setX(msg->pose.pose.orientation.x);
    q.setY(msg->pose.pose.orientation.y);
    q.setZ(msg->pose.pose.orientation.z);
    q.setW(msg->pose.pose.orientation.w);
    transform.setRotation(q);

    // Publish tf
    // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
    br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "odom", "base_link"));

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry_to_tf_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("odom", 5, odometryCallback);

    ros::spin();

    return 0;
}

