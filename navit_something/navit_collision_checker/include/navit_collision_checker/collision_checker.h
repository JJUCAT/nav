#ifndef COLLISION_CHECKER_H
#define COLLISION_CHECKER_H

#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>
#include <tf2_ros/buffer.h>
#include <navit_costmap/footprint.h>
#include <navit_costmap/costmap_2d.h>
#include <navit_costmap/costmap_2d_ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <navit_collision_checker/costmap_sub.h>
#include <navit_collision_checker/footprint_sub.h>
#include <navit_collision_checker/footprint_collision_checker.h>
#include "tf2/utils.h"

namespace navit_collision_checker {
    class CollisionChecker
    {
        public:

        CollisionChecker(
                const std::shared_ptr<CostmapSub>& costmap_sub,
                const std::shared_ptr<FootprintSub>& footprint_sub,
                const std::shared_ptr<tf2_ros::Buffer>& tf,
                std::string name = "collision_checker",
                std::string global_frame = "map",
                std::string robot_base_frame = "base_link",
                double transform_tolerance = 0.5);

        ~CollisionChecker() = default;

        double scorePose(const geometry_msgs::Pose2D& pose);

        bool isCollisionFree(const geometry_msgs::Pose2D& pose);
        bool isCollisionFree(const geometry_msgs::PoseStamped& pose);

        // assume in w.r.t. current base frame
        bool isCollisionFree(const nav_msgs::Path& path)
        {
            for(auto p : path.poses)
            {
                if(!isCollisionFree(p))
                    return false;
            }
            return true;
        }

        bool isCollisionFree(const geometry_msgs::PoseStamped p,
                             const geometry_msgs::Twist& cmd_vel,
                             const double predict_time_horizon,
                             const double predict_step = 0.1)
        {
            geometry_msgs::PoseStamped pose = p;
            double orient = tf2::getYaw(p.pose.orientation);
            int N = std::max(predict_time_horizon / predict_step / 2, 2.0);
            tf2::Quaternion q;
            for(int n=0; n < N; n++)
            {
                double diff_x = cmd_vel.linear.x * predict_step;
                double diff_y = cmd_vel.linear.y * predict_step;
                // assume in w.r.t. current base frame
                double heading = cmd_vel.angular.z * predict_step*n;
                pose.pose.position.x += diff_x * std::cos(heading) - diff_y * std::sin(heading);
                pose.pose.position.y += diff_x * std::sin(heading) + diff_y * std::cos(heading);
                q.setRPY(0.0, 0.0, heading+orient);
                tf2::convert(q, pose.pose.orientation);

                if (!isCollisionFree(pose))
                {
                    ROS_WARN("pose[%f, %f] is in collsion",pose.pose.position.x, pose.pose.position.y); 
                    return false;
                }
            }
            return true;
        }

        void SetExtraCostmap(const std::shared_ptr<navit_costmap::Costmap2DROS>& costmap,
                             const std::vector<geometry_msgs::Point>& footprint,
                             const unsigned char collised_value)
        {
          use_extra_costmap_ = true;
          extra_costmap_ = costmap;
          extra_footprint_.clear();
          for (auto p32 : footprint) {
            geometry_msgs::Point p; p.x = p32.x; p.y = p32.y;
            extra_footprint_.push_back(p);
          }
          extra_collised_value_ = collised_value;
        }

        protected:
        bool getCurrentPose(
                  geometry_msgs::PoseStamped & global_pose,
                  const std::shared_ptr<tf2_ros::Buffer>& tf_buffer, const std::string global_frame = "map",
                  const std::string robot_frame = "base_link", const double transform_timeout = 0.8);

        void unorientFootprint(const Footprint& orientated_footprint, Footprint& reset_footprint);

        Footprint getFootprint(const geometry_msgs::Pose2D& pose);

        std::string name_;
        std::string global_frame_;
        std::string robot_base_frame_;
        std::shared_ptr<CostmapSub> costmap_sub_;
        std::shared_ptr<FootprintSub> footprint_sub_;
        std::shared_ptr<tf2_ros::Buffer> tf_; double transform_tolerance_;

        FootprintCollisionChecker<std::shared_ptr<navit_costmap::Costmap2D>> footprint_collision_checker_;
        ros::Publisher footprint_pub_;
        std::shared_ptr<navit_costmap::Costmap2D> costmap_ptr_;
        bool use_extra_costmap_ = false;
        std::shared_ptr<navit_costmap::Costmap2DROS> extra_costmap_;
        Footprint extra_footprint_;
        unsigned char extra_collised_value_;
    };

}

#endif
