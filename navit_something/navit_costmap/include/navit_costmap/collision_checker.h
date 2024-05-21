#ifndef COLLISION_CHECKER_H
#define COLLISION_CHECKER_H

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>

#include <navit_costmap/footprint.h>
#include <navit_costmap/costmap_2d_ros.h>
#include <navit_costmap/footprint_collision_checker.h>

namespace navit_costmap {
    class CollisionChecker
    {
        public:

        CollisionChecker(const std::shared_ptr<Costmap2DROS>& costmap_ros_ptr);

        ~CollisionChecker()
        {
            if ( costmap_ros_ptr_ != nullptr )
                costmap_ros_ptr_.reset();
            if ( costmap_ptr_ != nullptr )
                costmap_ptr_.reset();
        }

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

        bool isCollisionFree( const geometry_msgs::Twist& cmd_vel,
                             const double predict_time_horizon,
                             const double predict_step = 0.1)
        {
            geometry_msgs::PoseStamped pose;
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
                q.setRPY(0.0, 0.0, heading);
                tf2::convert(q, pose.pose.orientation);

                if (!isCollisionFree(pose))
                {
                    ROS_WARN("pose[%f, %f] is in collsion",pose.pose.position.x, pose.pose.position.y); 
                    return false;
                }
            }
            return true;
        }


        protected:
        void unorientFootprint(const Footprint& orientated_footprint, Footprint& reset_footprint);

        Footprint getFootprint(const geometry_msgs::Pose2D& pose);

        FootprintCollisionChecker<std::shared_ptr<navit_costmap::Costmap2D>> footprint_collision_checker_;
        std::shared_ptr<Costmap2DROS> costmap_ros_ptr_;
        std::shared_ptr<Costmap2D> costmap_ptr_;
    };

}

#endif
