#include <navit_recovery/recovery_server.h>

namespace navit_recovery {
    
    RecoveryServer::RecoveryServer(TFPtr& tf, ros::NodeHandle& nh):
        nh_(nh),
        tf_(tf),
        recovery_loader_("navit_core", "navit_core::Recovery"),
        recovery_manager_("nav_recovery",
                          boost::bind(&RecoveryServer::loadRecoveryPlugins, this, _1),
                          boost::bind(&RecoveryServer::initRecoveryPlugins, this, _1, _2),
                          nh_),
        costmap_topic_name_("/navit_controller_node/controller_costmap/costmap"),
        footprint_topic_name_("/navit_controller_node/controller_costmap/footprint"),
        global_frame_("odom"),
        robot_base_frame_("base_link"),
        transform_tolerance_(0.1)
    {
        nh_.param("costmap_topic_name", costmap_topic_name_, costmap_topic_name_);
        nh_.param("footprint_topic_name", footprint_topic_name_, footprint_topic_name_);

        auto costmap_sub = std::make_shared<CostmapSub>(nh_, costmap_topic_name_);
        auto footprint_sub = std::make_shared<FootprintSub>(nh_, footprint_topic_name_);
        collision_checker_ = std::make_shared<CollisionChecker>(
               costmap_sub,
               footprint_sub,
               tf_,
               "collision_checker",
               global_frame_,
               robot_base_frame_,
               transform_tolerance_ );
        map_ros_ = std::make_shared<navit_costmap::Costmap2DROS>("recovery_costmap", *tf_, nh_);
        map_ros_->start();

        recovery_manager_.loadPlugins();

        map_ros_->pause();
    }

    RecoveryPtr
    RecoveryServer::loadRecoveryPlugins(const std::string& plugin)
    {
        RecoveryPtr recovery_ptr;
        try
        {
           recovery_ptr = recovery_loader_.createInstance(plugin);
           std::string recovery_name = recovery_loader_.getName(plugin);
           ROS_INFO("recovery plugin %s with name %s loaded!", plugin.c_str(),recovery_name.c_str());
        }
        catch (const pluginlib::PluginlibException& ex)
        {
            ROS_WARN("failed to load %s, with %s", plugin.c_str(), ex.what());
        }

        return recovery_ptr;
    }

    bool
    RecoveryServer::initRecoveryPlugins(const std::string& plugin,
                                        const RecoveryPtr& recovery_ptr)
    {
        recovery_ptr->initialize(plugin, tf_, collision_checker_);
        recovery_ptr->SetExtraCostmap(map_ros_);
        return true;
    }





}
