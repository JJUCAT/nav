#ifndef NAVIT_RECOVERY__RECOVERY_SERVER_H
#define NAVIT_RECOVERY__RECOVERY_SERVER_H

#include <ros/ros.h>
#include <navit_core/abstract_plugin_manager.h>
#include <pluginlib/class_loader.h>
#include <navit_collision_checker/collision_checker.h>
#include <navit_costmap/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>

#include "recovery_action.h"

namespace navit_recovery {

using CollisionChecker = navit_collision_checker::CollisionChecker;
using CostmapSub = navit_collision_checker::CostmapSub;
using FootprintSub = navit_collision_checker::FootprintSub;
using CollisionCheckerPtr = std::shared_ptr<navit_collision_checker::CollisionChecker>;
using TFPtr = std::shared_ptr<tf2_ros::Buffer>;
using Recovery = navit_core::Recovery;
using RecoveryPtr = Recovery::Ptr; 
using RecoveryPluginLoader = pluginlib::ClassLoader<Recovery>;
using RecoveryPluginManager = navit_core::AbstractPluginManager<Recovery>;

    class RecoveryServer
    {
        public:
            RecoveryServer(TFPtr& tf, ros::NodeHandle& nh);

            ~RecoveryServer()
            {
                recovery_manager_.clearPlugins();

                recovery_ptr_.reset();
                collision_checker_.reset();
                tf_.reset();
            }

        protected:
            ros::NodeHandle nh_;

            CollisionCheckerPtr collision_checker_;
            TFPtr tf_;
            RecoveryPtr recovery_ptr_;
            RecoveryPluginLoader recovery_loader_;
            RecoveryPluginManager recovery_manager_;

            RecoveryPtr loadRecoveryPlugins(const std::string& plugin);
            bool initRecoveryPlugins(const std::string& plugin,
                                     const RecoveryPtr& recovery_ptr);

            std::string costmap_topic_name_;
            std::string footprint_topic_name_;
            std::string global_frame_;
            std::string robot_base_frame_;
            double transform_tolerance_;

            std::shared_ptr<navit_costmap::Costmap2DROS> map_ros_;
    };
}

#endif
