#include <dock_visual_perception/VisualPerception.h>
#include <tf/tf.h>
#include <ros/ros.h>

int main(int argc, char* argv[])
{
    dock_visual_perception::VisualPerception temp;
    ros::init(argc,argv,"get_pose_node");
    ros::start();

    tf::Quaternion one = tf::Quaternion::getIdentity();
    tf::Transform trans(one);
    ros::Time time_stamp = ros::Time();
    std::string frame_id = "/base_link",child_frame_id = "/camera_link"; 
    tf::StampedTransform CamToOutput(trans,time_stamp,frame_id,child_frame_id);
    // ros::Rate loop(30);
    std::shared_ptr<tf2_ros::Buffer> tf_buffer = std::make_shared<tf2_ros::Buffer>();
    temp.initialize("name",tf_buffer);
    // ros::Duration(1.0).sleep();
    // ros::spinOnce();
    while(ros::ok())
    {
        // loop.sleep();
        // ros::spinOnce();
        geometry_msgs::PoseStamped out;
        bool isok = temp.getPose(out);
        if(isok) std::cout<<"ok"<<temp.confidence_<<std::endl;
        else std::cout<<"not good"<<std::endl;
    }
    return 0;
}