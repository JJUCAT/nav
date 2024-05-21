#include <dock_external_perception/external_perception.h>

namespace dock_external_perception{
void ExternalPerception::initialize(const std::string name,
                                    const std::shared_ptr<tf2_ros::Buffer>& tf)
{
    ros::NodeHandle nh, pnh("~/" + name);
    pnh.param("detection_topic_name", config_.detection_topic_name, config_.detection_topic_name);
    pose_sub_ = nh.subscribe(config_.detection_topic_name, 1, &ExternalPerception::dockDetectionCallback,this); 
}
};
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(dock_external_perception::ExternalPerception, navit_auto_dock::plugins::ApproachDockPerception)
