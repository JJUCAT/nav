#include "lpf.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(navit_auto_dock::plugins::Lpf, navit_auto_dock::plugins::ApproachDockFilter)

namespace navit_auto_dock{
namespace plugins{

    void Lpf::initialize(const std::string& name,
                         const std::shared_ptr<tf2_ros::Buffer>&)
    {
        ros::NodeHandle pnh("~/" + name);
        std::vector<float> b={0.20657, 0.41314, 0.20657};
        std::vector<float> a={1.00000, -0.36953, 0.19582};
        pnh.getParam("low_pass_filter/b_array",b);
        pnh.getParam("low_pass_filter/a_array",a);
        linear_pose_filter_ = std::make_shared<LinearPoseFilter2D>(b,a);

        dock_found_ = false;
        ROS_INFO("Low pass filter initialized!");
    }

    void Lpf::reset()
    {
        linear_pose_filter_->reset();
        dock_found_ = false;
        ROS_INFO("Low pass filter reseted!");
    }

    void Lpf::update(geometry_msgs::PoseStamped& dock_pose,
                     const geometry_msgs::Twist&)
    {
        auto tmp_pose = dock_pose;
        if (!dock_found_)
        {
            linear_pose_filter_->setFilterState(tmp_pose.pose, tmp_pose.pose);
            dock_found_ = true;
        }

        dock_pose.pose = linear_pose_filter_->filter(tmp_pose.pose);
    }

}
}
