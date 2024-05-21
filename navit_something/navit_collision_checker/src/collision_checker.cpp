#include <navit_collision_checker/collision_checker.h>
#include <tf2/utils.h>
#include <navit_costmap/cost_values.h>
#include "navit_costmap/costmap_2d.h"

using namespace navit_costmap;

namespace navit_collision_checker {

    CollisionChecker::CollisionChecker(
                const std::shared_ptr<CostmapSub>& costmap_sub,
                const std::shared_ptr<FootprintSub>& footprint_sub,
                const std::shared_ptr<tf2_ros::Buffer>& tf,
                std::string name,
                std::string global_frame,
                std::string robot_base_frame,
                double transform_tolerance ):
        costmap_sub_(costmap_sub),
        footprint_sub_(footprint_sub),
        tf_(tf),
        name_(name),
        global_frame_(global_frame),
        robot_base_frame_(robot_base_frame),
        transform_tolerance_(transform_tolerance),
        footprint_collision_checker_(nullptr)
    {
        ros::NodeHandle nh;
        footprint_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("/collision_checker_footprint", 1);
        
    }

    bool
    CollisionChecker::isCollisionFree(
            const geometry_msgs::PoseStamped& pose)
    {
        geometry_msgs::Pose2D pose2d;
        pose2d.x = pose.pose.position.x;
        pose2d.y = pose.pose.position.y;
        pose2d.theta = tf2::getYaw(pose.pose.orientation);
        return isCollisionFree(pose2d);
    }

    bool 
    CollisionChecker::isCollisionFree(const geometry_msgs::Pose2D& pose)
    {
        try
        {
            double collised_value = 90;
            if (use_extra_costmap_) collised_value = static_cast<double>(extra_collised_value_);
            if (scorePose(pose) >= collised_value)
                return false;
            return true;
        }
        catch (...)
        {
            ROS_ERROR("failed to check pose socre");
            return false;
        }
    }

    double
    CollisionChecker::scorePose(const geometry_msgs::Pose2D& pose)
    {
        try
        {
          if (use_extra_costmap_) {
            if (costmap_ptr_.use_count()==0) {
              costmap_ptr_.reset(extra_costmap_->getCostmap());              
            }
          }
          else costmap_ptr_ = costmap_sub_->getCostmap();
          footprint_collision_checker_.setCostmap(costmap_ptr_);
        }
        catch (const std::runtime_error& ex)
        {
           ROS_ERROR("Exception: [%s]", ex.what()); 
        }

        return footprint_collision_checker_.footprintCost(getFootprint(pose));
    }


    Footprint CollisionChecker::getFootprint(const geometry_msgs::Pose2D & pose)
    {
      Footprint footprint_spec;
      if (!use_extra_costmap_) {
        std::lock_guard<std::mutex> lock(footprint_sub_->footprint_lock_);        
        if (!footprint_sub_->getFootprint(footprint_spec)) {
          ROS_ERROR("Current footprint not available.");
        }
      } else {
        footprint_spec = extra_footprint_;
      }

      Footprint footprint;
        //TODO: check if rectangular shaped footprint works
      //unorientFootprint(footprint, footprint_spec);
      transformFootprint(pose.x, pose.y, pose.theta, footprint_spec, footprint);

      geometry_msgs::PolygonStamped footprint_msg;
      footprint_msg.header.frame_id = "odom";
      footprint_msg.header.stamp = ros::Time::now();
      footprint_msg.polygon = toPolygon(footprint);

      footprint_pub_.publish(footprint_msg);

      return footprint;
    }

    void CollisionChecker::unorientFootprint(
      const std::vector<geometry_msgs::Point> & oriented_footprint,
      std::vector<geometry_msgs::Point> & reset_footprint)
    {
      geometry_msgs::PoseStamped current_pose;
      if (!getCurrentPose(
          current_pose, tf_, global_frame_, robot_base_frame_,
          transform_tolerance_))
      {
        ROS_ERROR("Robot pose unavailable.");
      }
    
      double x = current_pose.pose.position.x;
      double y = current_pose.pose.position.y;
      double theta = tf2::getYaw(current_pose.pose.orientation);
    
      Footprint temp;
      transformFootprint(-x, -y, 0, oriented_footprint, temp);
      transformFootprint(0, 0, -theta, temp, reset_footprint);
    }

    bool 
    CollisionChecker::getCurrentPose(
                  geometry_msgs::PoseStamped & global_pose,
                  const std::shared_ptr<tf2_ros::Buffer>& tf_buffer, const std::string global_frame,
                  const std::string robot_frame, const double transform_timeout )
    {
        tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
        global_pose.header.frame_id = robot_frame;
        global_pose.header.stamp = ros::Time::now();

        try
        {
            tf_buffer->transform(global_pose, global_pose, global_frame, ros::Duration(transform_timeout));
            ROS_INFO_THROTTLE(0.5,"transfomred global pose is (%.2f, %.2f)", global_pose.pose.position.x,
                                                                global_pose.pose.position.y);
            return true;
        }
        catch (tf2::TransformException& ex)
        {
            ROS_WARN_STREAM("failed to get current pose from " << global_frame << " with " << ex.what());
            return false;
        }
    }


}
