#include <dummy_controller/dummy_controller.h>

namespace dummy_controller {

    void DummyController::initialize(const std::string& name,
                                     const std::shared_ptr<tf2_ros::Buffer>& tf,
                                     const std::shared_ptr<navit_costmap::Costmap2DROS>& costmap_ros)
    {
        tf_ = tf;
        costmap_ros_ = costmap_ros;
        ros::NodeHandle pnh("~/" + name);
        name_ = name;
        pnh.param("dummy_param", dummy_param_, 100);
        ROS_INFO_STREAM("Dummy controller param: "<< dummy_param_ << " loaded");


        dummy_counter_ = 0;
        ROS_INFO_STREAM("Dummy controller initalized");
    }

    

    geometry_msgs::Twist 
    DummyController::computeVelocityCommands(const geometry_msgs::PoseStamped& current_pose,
                            const geometry_msgs::Twist& current_vel)
    {
        ROS_DEBUG_STREAM(" " << name_ << " is following given path..");
        geometry_msgs::Twist cmd_vel;
        cmd_vel.angular.z = 0.1;
        dummy_counter_++;
        return cmd_vel;
    }

}

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(dummy_controller::DummyController, navit_core::Controller)
