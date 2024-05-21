#include <navit_costmap/collision_checker.h>
#include <tf2/utils.h>

namespace navit_costmap {

    CollisionChecker::CollisionChecker(const std::shared_ptr<Costmap2DROS>& costmap_ros_ptr):
        costmap_ros_ptr_(costmap_ros_ptr)
    {
        costmap_ptr_.reset(costmap_ros_ptr_->getCostmap());
    }

    bool
    CollisionChecker::isCollisionFree( const geometry_msgs::PoseStamped& pose)
    {
        geometry_msgs::Pose2D pose2d;
        pose2d.x = pose.pose.position.x;
        pose2d.y = pose.pose.position.y;
        pose2d.theta = tf2::getYaw(pose.pose.orientation);
        return isCollisionFree(pose2d);
    }

    bool 
    CollisionChecker::isCollisionFree( const geometry_msgs::Pose2D& pose)
    {
        try
        {
            if (scorePose(pose) >= 90)
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
    CollisionChecker::scorePose( const geometry_msgs::Pose2D& pose)
    {
        try
        {
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
	  Footprint footprint_oriented = costmap_ros_ptr_->getRobotFootprint();
	
      //TODO: check if rectangular shaped footprint works
      Footprint footprint_spec;
	  unorientFootprint(footprint_oriented, footprint_spec);

	  Footprint footprint_at_pose;
	  transformFootprint(pose.x, pose.y, pose.theta, footprint_spec, footprint_at_pose);

	  return footprint_at_pose;
	}
                
    void CollisionChecker::unorientFootprint(const std::vector<geometry_msgs::Point> & oriented_footprint,
                                             std::vector<geometry_msgs::Point> & reset_footprint)
    {
      geometry_msgs::PoseStamped current_pose; 
      if ( !costmap_ros_ptr_->getRobotPose(current_pose) )
          ROS_FATAL("CollisionChecker failed to get current robot pose!");
    
      double x = current_pose.pose.position.x;
      double y = current_pose.pose.position.y;
      double theta = tf2::getYaw(current_pose.pose.orientation);
    
      transformFootprint(-x, -y, -theta, oriented_footprint, reset_footprint);
    }

}
