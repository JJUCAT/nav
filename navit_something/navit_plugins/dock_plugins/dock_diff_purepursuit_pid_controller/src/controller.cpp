#include <dock_diff_purepursuit_pid_controller/controller.h>

#include <algorithm>
#include <cmath>
#include <list>
#include <vector>

#include <pluginlib/class_list_macros.h>

#define EPISON 1e-7

PLUGINLIB_EXPORT_CLASS(dock_diff_purepursuit_pid_controller::Controller,
                       navit_auto_dock::plugins::ApproachDockController)

namespace dock_diff_purepursuit_pid_controller {

void Controller::initialize(const std::string name,
                            const std::shared_ptr<tf2_ros::Buffer> &tf) {
  tf_buffer_ = tf;
  ros::NodeHandle pnh("~/" + name);

  // TODO(enhancement): add dynamic reconfig server for these


  pnh.param("half_length_robot", half_length_robot_, 0.45);
  pnh.param("ld", ld_, 0.15);
  pnh.param("kl", k_control_, 2.0);
  pnh.param("min_velocity", min_velocity_, 0.01);
  pnh.param("max_velocity", max_velocity_, 0.15);
  pnh.param("max_angular_velocity", max_angular_velocity_,0.5);
  pnh.param("beta", beta_, 0.2);
  pnh.param("lambda", lambda_, 2.0);

  // TODO: remove these frames
  // frames

  pnh.param("base_frame", local_frame_, std::string("base_link"));

  // tolerance

  pnh.param("xy_tolerance", dist_tolerance_, 0.01);
  pnh.param("yaw_tolerance", angle_tolerance_, 0.02);


}

bool Controller::setTarget(const geometry_msgs::PoseStamped &target) {
  geometry_msgs::PoseStamped pose = target;
  if (pose.header.frame_id != local_frame_) {
    // Transform target into base frame
    try {
      geometry_msgs::TransformStamped target_to_base_link;
      target_to_base_link =
          tf_buffer_->lookupTransform(local_frame_, // TODO: rosparam
                                      pose.header.frame_id, ros::Time(0));

      ROS_DEBUG_NAMED("diff_purepursuit controller",
                      "current pose before tf x %3.2f, y %3.2f",
                      pose.pose.position.x, pose.pose.position.y);
      // TODO: check the results
      tf2::doTransform(pose, pose, target_to_base_link);
      ROS_DEBUG_NAMED("diff_purepursuit controller",
                      "current pose after tf x %3.2f, y %3.2f",
                      pose.pose.position.x, pose.pose.position.y);
    } catch (tf2::TransformException const &ex) {
      ROS_WARN_STREAM_THROTTLE(1.0,
                               "Couldn't get transform from dock to base_link");
      return false;
    }
  }

  target_ = pose;
  return true;
}

bool Controller::isGoalReached() {
  double x_diff = target_.pose.position.x;
  double y_diff = target_.pose.position.y;
  double dist_error = std::sqrt(x_diff * x_diff + y_diff * y_diff);

  double yaw_error = std::abs(tf2::getYaw(target_.pose.orientation));

  ROS_WARN_STREAM("dist error is " << dist_error << "  yaw error is "
                                   << yaw_error);

  if (dist_error == 0.0 && yaw_error == 0.0)
  {
    ROS_WARN("Control error is zero, is target pose set?");
    return false;
  }

  if (dist_error < dist_tolerance_ && yaw_error < angle_tolerance_)
    return true;
  else
    return false;
}

bool Controller::computeVelocityCommands(
    const geometry_msgs::PoseStamped &current_pose,
    const geometry_msgs::Twist &current_vel, geometry_msgs::Twist &cmd_vel) {
    

    //KENIMATIC 运动学部分
    //goal in base
    geometry_msgs::PoseStamped pose = target_;
    ROS_DEBUG("-------------------------------------------------------------");
    KDL::Vector2 V_base_goal= KDL::Vector2(pose.pose.position.x,pose.pose.position.y);
    KDL::Rotation2 R_base_goal(tf2::getYaw(pose.pose.orientation));
    KDL::Frame2 Frame_base_goal(R_base_goal,V_base_goal);

    //base in goal
    KDL::Frame2 Frame_goal_base=Frame_base_goal.Inverse();

    double goal_x_inverse=Frame_goal_base.p.x();//x方向误差
    double goal_y_inverse=Frame_goal_base.p.y();//y方向误差
    double ahead_distance=fabs(goal_y_inverse)+ld_;//前视距离

    double theta_error=Frame_base_goal.M.GetRot();


    double dist_error = std::sqrt(goal_x_inverse * goal_x_inverse + goal_y_inverse * goal_y_inverse);
    if(dist_error<dist_tolerance_){//如果已经进入重点十分附近但是还差点角度没有转

      cmd_vel.angular.z =std::min(max_angular_velocity_, std::max(-max_angular_velocity_, theta_error));

    }else{//按照控制算法走
    
      //TODO 计算delta_error正负号的方式有待优化，找不到一个通用的公式？？
      double delta_error;
      if(goal_x_inverse>0){
        delta_error=asin(goal_y_inverse/ahead_distance);
      }else{
        delta_error=asin(-goal_y_inverse/ahead_distance);
      }

      double alpha=theta_error+delta_error; //车身x轴与（车中心与朝向点之前连线）的夹角


      ROS_DEBUG("goal_x_inverse and goal_y_inverse is (%.4f, %.4f)" , goal_x_inverse,goal_y_inverse);
      ROS_DEBUG("theta_error is (%.4f, %.4f)" , theta_error*180/M_PI,theta_error);
      ROS_DEBUG("delta_error is (%.4f, %.4f)" , delta_error*180/M_PI,  delta_error);
      ROS_DEBUG("alpha is (%.4f, %.4f)" , alpha*180/M_PI,  alpha);


      //DYNAMIC 动力学部分
    
      double vel_temp=std::max(min_velocity_, std::min(fabs(goal_x_inverse), max_velocity_));
      ROS_DEBUG("vel_temp : %.4f", vel_temp);

      // double heading_angle_=atan2(2*fabs(half_length_robot_)*sin(alpha),ahead_distance);//角度朝向
      double heading_angle_=atan2(2*fabs(half_length_robot_)*sin(alpha),fabs(vel_temp)*k_control_);//角度朝向

      //TODO 计算control_x_正负号的方式有待优化，找不到一个通用的公式？？
      double control_x_;
      if(goal_x_inverse>0){
        control_x_=-half_length_robot_;
      }else{
        control_x_=half_length_robot_;
      }

      //根据当前角度计算控制半径（车上的瞬心与中心的距离）
      double control_radius=control_x_/tan(heading_angle_);

      ROS_DEBUG("heading_angle_ is (%.4f, %.4f)" , heading_angle_*180/M_PI, heading_angle_);
      ROS_DEBUG("control_radius is %.4f", control_radius);

      double v_middle= goal_x_inverse<0? vel_temp:-vel_temp;
      ROS_DEBUG("v_middle : %.4f", v_middle);

      // 控制点的速度一直设成最大值，通过控制点的速度算出车中点的速度
      double omega_tmp=v_middle*sin(heading_angle_)/control_x_;
      ROS_DEBUG("omega_tmp : %.4f", omega_tmp);

      // Bound angular velocity 用最大最小速度夹一下
      double bounded_omega =std::min(max_angular_velocity_, std::max(-max_angular_velocity_, omega_tmp));
      //保证线速度和角速度是按照一定比例的
      double vel_final=bounded_omega*control_radius;


      ROS_DEBUG("vel_final : %.4f", vel_final);
      ROS_DEBUG("bounded_omega : %.4f", bounded_omega);
      cmd_vel.linear.x = vel_final;
      cmd_vel.angular.z = bounded_omega;

    }

  return true;
}

} // end of namespace
