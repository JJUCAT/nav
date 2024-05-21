#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/TransformStamped.h"
#include <tf2/LinearMath/Transform.h>
#include "navit_localization/ros_filter_utilities.h"
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>

using namespace RobotLocalization;
using namespace RosFilterUtilities;

// TODO 需要根据当前map的经纬高设定出时位置，默认为0
tf2::Transform trans ;
tf2::Transform transform_odom;
bool have_raw = false;
bool have_filtered = false;

void odomfilterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{

    ROS_INFO("filter x %lf y %lf z %lf",msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    ROS_INFO("X Y Z W %lf %lf %lf %lf",msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z , msg->pose.pose.orientation.w);
    
    trans.setOrigin(tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z , msg->pose.pose.orientation.w);
    trans.setRotation(q);
    if(!have_filtered) have_filtered = true;

}

void odomrawCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    ROS_INFO("raw");
    transform_odom.setOrigin(tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z , msg->pose.pose.orientation.w);
    transform_odom.setRotation(q);
    if(!have_raw) have_raw = true;
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "odom_transform");
    ros::NodeHandle nh;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    tf2_ros::TransformBroadcaster tf_broadcaster;

    // tf2::Transform transform_odom;
    tf2::Transform trans_odom_ekf;
    geometry_msgs::TransformStamped transform_ekf_msg;

    ros::Rate rate(10.0);
    ros::Time time_now = ros::Time::now();

    ros::Subscriber odom_sub = nh.subscribe("/odometry/filtered", 1, odomfilterCallback);
    ros::Subscriber odomraw_sub = nh.subscribe("/odom",1,odomrawCallback);
    
    bool init_ok = false;

    while(ros::ok())
    {
        ros::spinOnce();
        if( !have_filtered || !have_raw) continue;
        time_now = ros::Time::now();    
        // tf2::fromMsg(tf_buffer.lookupTransform("odom", "base_link", ros::Time(),ros::Duration(1)).transform, transform_odom);

        // trans_odom_ekf.mult(trans,transform_odom.inverse());
        if(!init_ok)
        {

            // trans.setOrigin(tf2::Vector3(0, 0, 0));
            // // 初始化旋转为单位矩阵
            // tf2::Quaternion rotation;
            // rotation.setRPY(0, 0, 0);  // 将旋转角度设置为0
            // trans.setRotation(rotation);

            trans_odom_ekf = trans * transform_odom.inverse();

            // trans_odom_ekf =  transform_odom.inverse();

            // double odom_roll, odom_pitch, odom_yaw;
            // ROS_INFO("odom ekf trans: %lf %lf %lf ",trans.getOrigin().getX(),trans.getOrigin().getY(),trans.getOrigin().getZ());
            // tf2::Matrix3x3(trans.getRotation()).getRPY(odom_roll, odom_pitch, odom_yaw);
            // ROS_INFO("odom ekf rotate: %lf %lf %lf",odom_roll,odom_pitch, odom_yaw);


            // ROS_INFO("odom raw trans: %lf %lf %lf ",transform_odom.getOrigin().getX(),transform_odom.getOrigin().getY(),transform_odom.getOrigin().getZ());
            // tf2::Matrix3x3(transform_odom.getRotation()).getRPY(odom_roll, odom_pitch, odom_yaw);
            // ROS_INFO("odom raw rotate: %lf %lf %lf",odom_roll,odom_pitch, odom_yaw);

            // tf2::Matrix3x3(trans_odom_ekf.getRotation()).getRPY(odom_roll, odom_pitch, odom_yaw);

            // ROS_INFO("final trans: %lf %lf %lf ",trans_odom_ekf.getOrigin().getX(),trans_odom_ekf.getOrigin().getY(),trans_odom_ekf.getOrigin().getZ());
            // ROS_INFO("final rotate: %lf %lf %lf",odom_roll,odom_pitch, odom_yaw);
            
            transform_ekf_msg.transform = tf2::toMsg(trans_odom_ekf);
            transform_ekf_msg.header.frame_id = "map";
            transform_ekf_msg.child_frame_id = "odom";
            init_ok = true;

        }
        
        
        transform_ekf_msg.header.stamp = ros::Time::now();
        tf_broadcaster.sendTransform(transform_ekf_msg);

        
        rate.sleep();
    }


    return 0;
}

