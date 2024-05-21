#ifndef M100_DOCK_LOCAL_PLANNER_ROS_H_
#define M100_DOCK_LOCAL_PLANNER_ROS_H_

#include <teb_local_planner/costmap_model.h>
#include <navit_costmap/costmap_2d_ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <navit_core/base_controller.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_ros/buffer.h>
#include <sstream>
#include <string>
#include <vector>

namespace m100_dock_local_planner {

class M100DockLocalPlannerROS : public navit_core::Controller{
   public:
    M100DockLocalPlannerROS();

    ~M100DockLocalPlannerROS();
    // Controller interface
    /*
        virtual geometry_msgs::Twist computeVelocityCommands(const geometry_msgs::PoseStamped& current_pose,
                                                             const geometry_msgs::Twist& current_vel) = 0;

        virtual bool computeVelocity(geometry_msgs::Twist& current_vel) { return true; };

        virtual void initialize(const std::string& name,
                                const std::shared_ptr<tf2_ros::Buffer>& tf,
                                const std::shared_ptr<navit_costmap::Costmap2DROS>& costmap_ros) = 0;


        virtual bool setPlan(const nav_msgs::Path& plan) = 0;
        
        virtual bool isGoalReached() = 0;

        virtual bool setSpeedLimit(const double& speed_limit) = 0;
    */

    geometry_msgs::Twist computeVelocityCommands(const geometry_msgs::PoseStamped& current_pose,
                                                 const geometry_msgs::Twist& current_vel)
    {
        geometry_msgs::Twist cmd_vel;
        if ( !computeVelocityCommands(cmd_vel) )
            ROS_WARN("Dock planner failed to compute velocity");
        return cmd_vel;
    }

    void initialize(const std::string& name,
                    const std::shared_ptr<tf2_ros::Buffer>& tf,
                    const std::shared_ptr<navit_costmap::Costmap2DROS>& costmap_ros)
    {
        initialize(name, tf.get(), costmap_ros.get());
    }
    bool setPlan(const nav_msgs::Path& plan) { return setPlan(plan.poses); } 
    bool setSpeedLimit(const double& speed_limit){ return false;} 

    void initialize(std::string name, tf2_ros::Buffer *tf, navit_costmap::Costmap2DROS *costmap_ros);

    bool setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan);

    bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

    bool isGoalReached();

    bool isInitialized() { return initialized_; }

    int sign(double x);

    double clipGoBack(double v, double v_max, double v_min);

    double clipGoHead(double v, double v_max, double v_min);

    double getDistance(geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2);

    int getDirection(geometry_msgs::PoseStamped robot_pose, geometry_msgs::PoseStamped goal_pose);

    void reSetState();

   private:
    navit_costmap::Costmap2DROS *costmap_ros_;
    tf2_ros::Buffer *tf_;
    std::string frame_id_;

    double kp_;
    double max_vel_;
    double min_vel_;
    double tolerance_;
    std::string global_frame_;
    bool initialized_;

    std::vector<geometry_msgs::PoseStamped> global_plan_;
    bool goal_reached_;
    double dis_old_;
};
};  // namespace m100_dock_local_planner

#endif  // M100_DOCK_LOCAL_PLANNER_ROS_H_
