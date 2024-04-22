#ifndef VELOCITY_SMOOTHER_H_
#define VELOCITY_SMOOTHER_H_

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Odometry.h>
#include <navit_velocity_smoother/paramsConfig.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <thread>

namespace navit_velocity_smoother {

class VelocitySmoother {
public:
  VelocitySmoother(const std::string &name, ros::NodeHandle &p_nh);

  VelocitySmoother() : shutdown_req(false), input_active(false){}

  virtual ~VelocitySmoother() {
    shutdown();
    if (worker_thread_ != nullptr)
      worker_thread_->join();
  }

  virtual bool init(ros::NodeHandle &nh) = 0;
  virtual void spin() = 0;
  void shutdown() { shutdown_req = true; };

protected:
  enum RobotFeedbackType {
    NONE,
    ODOMETRY,
    COMMANDS
  } robot_feedback; /**< What source to use as robot velocity feedback */

  std::string name_;
  ros::NodeHandle p_nh_;
  double steer_lim, steer_speed_lim;
  double speed_lim_vx, accel_lim_vx, decel_lim_vx, decel_lim_vx_safe;
  double speed_lim_vy, accel_lim_vy, decel_lim_vy, decel_lim_vy_safe;
  double speed_lim_w, accel_lim_w, decel_lim_w, decel_lim_w_safe;
  double decel_factor, decel_factor_safe;

  double frequency;

  std::shared_ptr<std::thread> worker_thread_;

  bool shutdown_req; /**< Shutdown requested by nodelet; kill worker thread */
  bool input_active;
  double cb_avg_time;
  std::vector<double> period_record; /**< Historic of latest periods between
                                        velocity commands */
  unsigned int
      pr_next; /**< Next position to fill in the periods record buffer */

  ros::Subscriber odometry_sub;    /**< Current velocity from odometry */
  ros::Subscriber current_vel_sub; /**< Current velocity from commands sent to
                                      the robot, not necessarily by this node */
  ros::Subscriber raw_in_vel_sub;  /**< Incoming raw velocity commands */
  ros::Publisher smooth_vel_pub;   /**< Outgoing smoothed velocity commands */
  ros::Time last_cb_time;

  inline double sign(double x) { return x < 0.0 ? -1.0 : +1.0; };

  inline double median(std::vector<double> values) {
    // Return the median element of an doubles vector
    nth_element(values.begin(), values.begin() + values.size() / 2,
                values.end());
    return values[values.size() / 2];
  };

  inline bool isZeroVel(geometry_msgs::Twist a) {
    return ((a.linear.x == 0.0) && (a.linear.y == 0.0) && (a.angular.z == 0.0));
  }

  inline bool isZeroVel(ackermann_msgs::AckermannDriveStamped a) {
    return ((a.drive.steering_angle == 0.0) && (a.drive.speed == 0.0));
  }

  std::shared_ptr<
      dynamic_reconfigure::Server<navit_velocity_smoother::paramsConfig>>
      dynamic_reconfigure_server;
  dynamic_reconfigure::Server<navit_velocity_smoother::paramsConfig>::
      CallbackType dynamic_reconfigure_callback;
  void reconfigCB(navit_velocity_smoother::paramsConfig &config,
                  uint32_t unused_level);
};

class TwistVelocitySmoother : public VelocitySmoother, public nodelet::Nodelet {
public:
  TwistVelocitySmoother(const std::string &name, ros::NodeHandle &p_nh)
      : VelocitySmoother(name, p_nh) {
    init(p_nh);
  }
  TwistVelocitySmoother() {}

  virtual void onInit();
  bool init(ros::NodeHandle &nh);
  void spin();

protected:
  geometry_msgs::Twist last_cmd_vel;
  geometry_msgs::Twist current_vel;
  geometry_msgs::Twist target_vel;

  ros::NodeHandle nh_;

  void velocityCB(const geometry_msgs::Twist::ConstPtr &msg);
  void robotVelCB(const geometry_msgs::Twist::ConstPtr &msg);
  void odometryCB(const nav_msgs::Odometry::ConstPtr &msg);
};

class AckermannVelocitySmoother : public VelocitySmoother,
                                  public nodelet::Nodelet {
public:
  AckermannVelocitySmoother(const std::string &name, ros::NodeHandle &p_nh)
      : VelocitySmoother(name, p_nh) {
    init(p_nh);
  }
  AckermannVelocitySmoother() {}
  virtual void onInit();
  bool init(ros::NodeHandle &nh);
  void spin();

protected:
  ros::NodeHandle nh_;
  ackermann_msgs::AckermannDriveStamped last_cmd_vel;
  ackermann_msgs::AckermannDriveStamped target_vel;

  void velocityCB(const ackermann_msgs::AckermannDriveStamped::ConstPtr &msg);
};

} // navit_velocity_smoother

#endif /* VELOCITY_SMOOTHER_H_ */
