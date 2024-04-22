#include <navit_auto_dock/dock_calibration.h>
//#include <navit_auto_dock/approach_dock.h>

namespace navit_auto_dock {



CalibrateDock::CalibrateDock(const std::string& name, 
                           std::shared_ptr<tf2_ros::Buffer> &tf_buffer, ros::NodeHandle& nh):
      nh_(nh),
      tf_buffer_(tf_buffer),
      approach_dock_as_(nh_, "/approach_dock",
                        boost::bind(&CalibrateDock::executeCallback, this, _1),
                        false),
      approach_perception_pm_( name + "/perception",
                        boost::bind(&CalibrateDock::loadPerceptionPlugins, this, _1),
                        boost::bind(&CalibrateDock::initPerceptionPlugins, this, _1, _2), nh_),
      perception_loader_("navit_auto_dock", 
                         "navit_auto_dock::plugins::ApproachDockPerception")
{
  ros::NodeHandle pnh("/auto_dock/2d_lidar");
  pnh.param("yaw_offset", yaw_offset, 0.0f);

  // Load plugins
  approach_perception_pm_.loadPlugins();
  approach_dock_as_.start();
  ROS_INFO("navit_auto_dock_calibrate dock initialized!");
}

PerceptionPtr CalibrateDock::loadPerceptionPlugins(const std::string& plugin)
{
    PerceptionPtr perception_ptr;
    try
    {
        perception_ptr = perception_loader_.createInstance(plugin);
        std::string perception_name = perception_loader_.getName(plugin);
        ROS_DEBUG_STREAM("Approach dock perception plugin" << plugin << " loaded");
    }
    catch(const pluginlib::PluginlibException& ex)
    {
        ROS_WARN_STREAM("Failed to load " << plugin << " as approach dock controller plugin"
                << ex.what());
    }
    return perception_ptr;
}


bool CalibrateDock::initPerceptionPlugins(const std::string& plugin,
                                         const PerceptionPtr& perception_ptr)
{
    //TODO: validate if success
    perception_ptr->initialize(plugin, tf_buffer_);
    return true;
}

void CalibrateDock::executeCallback( const navit_msgs::ApproachDockGoalConstPtr &goal) 
{
  std::cout<<"enter CalibrateDock::executeCallback "<<std::endl;
  ros::Rate r(5.0);
  
  if (approach_perception_pm_.hasPlugin(goal->perception_plugin_name))
  {
    perception_ = approach_perception_pm_.getPlugin(goal->perception_plugin_name);
  }
  else
  {
    ROS_FATAL("Requested perception %s doesn't exist!", (goal->perception_plugin_name).c_str() );
    approach_dock_as_.setAborted(result_, "perception plugin dose not exist");
    return;
  }
  ROS_DEBUG_STREAM("perception plugin "<<goal->perception_plugin_name);
  geometry_msgs::PoseStamped expected_dock_pose;
  expected_dock_pose.header.frame_id = "odom";
  expected_dock_pose.header.stamp=ros::Time::now();
  expected_dock_pose.pose.position.x = 0;
  expected_dock_pose.pose.position.y = 0;
  expected_dock_pose.pose.orientation.x = 0.0;
  expected_dock_pose.pose.orientation.y = 0.0;
  expected_dock_pose.pose.orientation.z = 0.0;
  expected_dock_pose.pose.orientation.w = 0.0;

  // start perception
  perception_->start(expected_dock_pose);
  while (ros::ok()) {
   if(perception_->getPose(expected_dock_pose)){
    ROS_INFO("expected pose x %f y %f yaw %f", expected_dock_pose.pose.position.x,
                                          expected_dock_pose.pose.position.y,
                                         tf2::getYaw(expected_dock_pose.pose.orientation)*180.0/M_PI-yaw_offset);
    usleep(5e5);
   }
   else 
   {
   // ROS_INFO_STREAM_THROTTLE(3, "Perception plugin could not get Pose, waiting for next loop");
    ROS_INFO("Perception plugin could not get Pose, waiting for next loop");
    usleep(1e6);
   } 
}
  
}

} //namespace 
