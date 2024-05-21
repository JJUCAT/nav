// This node subscribes to the /pose topic and broadcasts the pose as a tf transform
// We can adjust the frame_id and child_frame_id to match our needs
// subscribe pose and tf to broadcast target frame

// http://wiki.ros.org/amcl
// http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF
/*
* we assume that the robot tf tree is as follows:
* map -> odom -> base_footprint -> base_link -> sensors_link
* Tmap2odom = Tmap2baselink * Tbaselink2basefootprint * Tbasefootprint2odom  
*/
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <rosgraph_msgs/Clock.h>
namespace navit_utils{
class ConvertPoseToTF
{
private:
    struct euler {
        double roll;
        double pitch;
        double yaw;
    } euler_;
    
    struct pose {
        double x;
        double y;
        double z;
    } pose_;

    void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void clockCallback(const rosgraph_msgs::Clock::ConstPtr& msg);
    inline void convertRosMsgToTf(const geometry_msgs::TransformStamped& transform, tf2::Transform& tf) {
        tf = tf2::Transform(tf2::Quaternion(transform.transform.rotation.x,
                                            transform.transform.rotation.y,
                                            transform.transform.rotation.z,
                                            transform.transform.rotation.w),
                            tf2::Vector3(transform.transform.translation.x,
                                         transform.transform.translation.y,
                                         transform.transform.translation.z));
    }

    inline void convertTfToRosmsg(const tf2::Transform& tf, geometry_msgs::TransformStamped& transform) {
        transform.transform.translation.x = tf.getOrigin().getX();
        transform.transform.translation.y = tf.getOrigin().getY();
        transform.transform.translation.z = tf.getOrigin().getZ();

        transform.transform.rotation.x = tf.getRotation().getX();
        transform.transform.rotation.y = tf.getRotation().getY();
        transform.transform.rotation.z = tf.getRotation().getZ();
        transform.transform.rotation.w = tf.getRotation().getW();
    }

    tf2_ros::Buffer tf_buffer_;
    boost::thread* tf_handle_thread_ptr_;
    ros::Time current_time_;
    ros::NodeHandle nh_;
    std::string global_frame_, local_frame_;
public:
    ConvertPoseToTF(/* args */);
    ~ConvertPoseToTF();
};

ConvertPoseToTF::ConvertPoseToTF() {
    ros::NodeHandle nh_;
    nh_.param("global_frame", global_frame_, std::string("map"));
    nh_.param("local_frame", local_frame_, std::string("odom"));
    
    //Get euler parameters for transform source frame to taget frame
    nh_.param("euler/roll", euler_.roll, 0.0);
    nh_.param("euler/pitch", euler_.pitch, 0.0);
    nh_.param("euler/yaw", euler_.yaw, 0.0);

    nh_.param("pose/x", pose_.x, 0.0);
    nh_.param("pose/y", pose_.y, 0.0);
    nh_.param("pose/z", pose_.z, 0.0);

    ros::Subscriber pose_sub = nh_.subscribe("/odometry/filtered", 10, &ConvertPoseToTF::poseCallback, this);
    ros::Subscriber tf_sub = nh_.subscribe("/clock", 10, &ConvertPoseToTF::clockCallback, this);
    tf2_ros::TransformListener tfListener(tf_buffer_);
    
    ros::spin();
}

ConvertPoseToTF::~ConvertPoseToTF() {}

void ConvertPoseToTF::clockCallback(const rosgraph_msgs::Clock::ConstPtr& msg) {
    current_time_ = msg->clock;
}

void ConvertPoseToTF::poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    ROS_INFO("ConvertPoseToTF::poseCallback");
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transform_stamped;

    transform_stamped.header.stamp = msg->header.stamp;
    transform_stamped.header.frame_id = "map";
    transform_stamped.child_frame_id = "odom";
    transform_stamped.transform.translation.x = msg->pose.pose.position.x;
    transform_stamped.transform.translation.y = msg->pose.pose.position.y;
    transform_stamped.transform.translation.z = 0.0;
    transform_stamped.transform.rotation = msg->pose.pose.orientation;

    // transform according to euler parameter
    tf2::Transform tf_init_base_footprint_to_base_link;
    convertRosMsgToTf(transform_stamped, tf_init_base_footprint_to_base_link);

    tf2::Transform tf_map_to_baselink;
    tf2::Transform tf_transform;
    tf_transform.setOrigin(tf2::Vector3(pose_.x, pose_.y, pose_.z));
    tf2::Quaternion q;
    q.setRPY(euler_.roll, euler_.pitch, euler_.yaw);
    tf_transform.setRotation(q);
    tf_map_to_baselink = tf_init_base_footprint_to_base_link * tf_transform;

    // get tf from odom to base_footprint
    geometry_msgs::TransformStamped odom_to_base_footprint;
    tf2::Transform tf_odom_to_base_footprint;

    try {
        odom_to_base_footprint = tf_buffer_.lookupTransform("odom", "base_footprint", ros::Time(0));
        convertRosMsgToTf(odom_to_base_footprint, tf_odom_to_base_footprint);
    } catch (tf2::TransformException &ex) {
        ROS_ERROR("Can not get tf from odom to base_footprint.");
    }
    // we try can get tf from base_footprint to base_link
    geometry_msgs::TransformStamped base_footprint_to_base_link;
    tf2::Transform tf_base_footprint_to_base_link;
    
    try {
        base_footprint_to_base_link = tf_buffer_.lookupTransform("base_footprint", "base_link", ros::Time(0));
        convertRosMsgToTf(base_footprint_to_base_link, tf_base_footprint_to_base_link);
    } catch (tf2::TransformException &ex) {
        ROS_ERROR("Can not get tf from base_footprint to base_link.");
    }

    geometry_msgs::TransformStamped map_to_odom;

    tf2::Transform tf_map_to_odom;

    tf_map_to_odom = tf_map_to_baselink * tf_odom_to_base_footprint.inverse() * tf_base_footprint_to_base_link;

    ROS_INFO("tf map to baselink x is %f, y is %f, z is %f", 
            tf_map_to_baselink.getOrigin().getX(), 
            tf_map_to_baselink.getOrigin().getY(), 
            tf_map_to_baselink.getOrigin().getZ());
    
    ROS_INFO("tf odom to base footprint x is %f, y is %f, z is %f", 
            tf_odom_to_base_footprint.getOrigin().getX(), 
            tf_odom_to_base_footprint.getOrigin().getY(), 
            tf_odom_to_base_footprint.getOrigin().getZ());

    // convert from tf to msg
    geometry_msgs::TransformStamped map_to_odom_transform_stamped;

    convertTfToRosmsg(tf_map_to_odom, map_to_odom_transform_stamped);

    map_to_odom_transform_stamped.header.stamp = msg->header.stamp;
    map_to_odom_transform_stamped.header.frame_id = "map";
    map_to_odom_transform_stamped.child_frame_id = "odom";

    br.sendTransform(map_to_odom_transform_stamped);
}
} // namespace navit_utils

int main(int argc, char** argv){
    ros::init(argc, argv, "pose_to_tf_node");
    navit_utils::ConvertPoseToTF convert_pose_to_tf;
    return 0;
}
