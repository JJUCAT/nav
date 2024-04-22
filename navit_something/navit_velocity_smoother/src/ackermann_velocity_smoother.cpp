#include <pluginlib/class_list_macros.h>
#include <velocity_smoother.h>

#define PERIOD_RECORD_SIZE 5

PLUGINLIB_EXPORT_CLASS(navit_velocity_smoother::AckermannVelocitySmoother,
                       nodelet::Nodelet)

namespace navit_velocity_smoother {

void AckermannVelocitySmoother::onInit() {
  p_nh_ = getPrivateNodeHandle();
  nh_ = getNodeHandle();
  if (init(p_nh_))
    worker_thread_.reset(
        new std::thread(&AckermannVelocitySmoother::spin, this));
  else
    NODELET_ERROR("Ackermann velocity smoother initialization failed!");
}

void AckermannVelocitySmoother::velocityCB(
    const ackermann_msgs::AckermannDriveStamped::ConstPtr &msg) {
  // Estimate commands frequency; we do continuously as it can be very different
  // depending on the
  // publisher type, and we don't want to impose extra constraints to keep this
  // package flexible
  if (period_record.size() < PERIOD_RECORD_SIZE) {
    period_record.push_back((ros::Time::now() - last_cb_time).toSec());
  } else {
    period_record[pr_next] = (ros::Time::now() - last_cb_time).toSec();
  }

  pr_next++;
  pr_next %= period_record.size();
  last_cb_time = ros::Time::now();

  if (period_record.size() <= PERIOD_RECORD_SIZE / 2) {
    // wait until we have some values; make a reasonable assumption (10 Hz)
    // meanwhile
    cb_avg_time = 0.1;
  } else {
    // enough; recalculate with the latest input
    cb_avg_time = median(period_record);
  }

  input_active = true;

  // TODO: add steering angle_velocity maybe?
  target_vel.drive.steering_angle =
      msg->drive.steering_angle > 0.0
          ? std::min(msg->drive.steering_angle, (float)steer_lim)
          : std::max(msg->drive.steering_angle, -(float)steer_lim);
  target_vel.drive.speed =
      msg->drive.speed > 0.0 ? std::min(msg->drive.speed, (float)speed_lim_vx)
                             : std::max(msg->drive.speed, -(float)speed_lim_vx);
}

void AckermannVelocitySmoother::spin() {
  double period = 1.0 / frequency;
  ros::Rate spin_rate(frequency);

  double decel_vx;

  while (!shutdown_req && ros::ok()) {
    if ((input_active == true) && (cb_avg_time > 0.0) &&
        ((ros::Time::now() - last_cb_time).toSec() >
         std::min(3.0 * cb_avg_time, 0.5))) {
      // Velocity input no active anymore; normally last command is a
      // zero-velocity one, but reassure
      // this, just in case something went wrong with our input, or he just
      // forgot good manners...
      // Issue #2, extra check in case cb_avg_time is very big, for example with
      // several atomic commands
      // The cb_avg_time > 0 check is required to deal with low-rate simulated
      // time, that can make that
      // several messages arrive with the same time and so lead to a zero median
      input_active = false;
      if (isZeroVel(target_vel) == false) {
        NODELET_WARN_STREAM("Velocity Smoother : input got inactive leaving us "
                            "a non-zero target velocity ("
                            << target_vel.drive.steering_angle << ", "
                            << target_vel.drive.speed << ", "
                                                         "), zeroing...["
                            << name_ << "]");
        target_vel = ackermann_msgs::AckermannDriveStamped();
      }
    }

    // 如果没有检测到输入，认为进入紧急停止的状态，增加减速度上限
    if (input_active) {
      decel_vx = decel_lim_vx;
    } else {
      // increase decel factor because this is a safty case, no more commands
      // means we should stop as fast as it is safe
      decel_vx = decel_lim_vx_safe;
    }

    ackermann_msgs::AckermannDriveStampedPtr cmd_vel;

    if ((target_vel.drive.steering_angle !=
         last_cmd_vel.drive.steering_angle) ||
        (target_vel.drive.speed != last_cmd_vel.drive.speed)) {
      // Try to reach target velocity ensuring that we don't exceed the
      // acceleration limits
      cmd_vel.reset(new ackermann_msgs::AckermannDriveStamped(target_vel));

      double steer_inc, vx_inc, max_steer_inc, max_vx_inc;

      steer_inc =
          target_vel.drive.steering_angle - last_cmd_vel.drive.steering_angle;
      max_steer_inc = ((steer_inc * target_vel.drive.steering_angle > 0.0)
                           ? steer_speed_lim
                           : steer_speed_lim) *
                      period;

      vx_inc = target_vel.drive.speed - last_cmd_vel.drive.speed;
      max_vx_inc =
          ((vx_inc * target_vel.drive.speed > 0.0) ? accel_lim_vx : decel_vx) *
          period;

      if (std::abs(steer_inc) > max_steer_inc)
        cmd_vel->drive.steering_angle =
            last_cmd_vel.drive.steering_angle + sign(steer_inc) * max_steer_inc;

      if (std::abs(vx_inc) > max_vx_inc)
        cmd_vel->drive.speed =
            last_cmd_vel.drive.speed + sign(vx_inc) * max_vx_inc;

      cmd_vel->header.stamp = ros::Time::now();
      smooth_vel_pub.publish(cmd_vel);
      last_cmd_vel = *cmd_vel;
    } else if (input_active == true) {
      // We already reached target velocity; just keep resending last command
      // while input is active
      cmd_vel.reset(new ackermann_msgs::AckermannDriveStamped(last_cmd_vel));
      cmd_vel->header.stamp = ros::Time::now();
      smooth_vel_pub.publish(cmd_vel);
    }
    spin_rate.sleep();
  }
}

bool AckermannVelocitySmoother::init(ros::NodeHandle &nh) {
  // Dynamic Reconfigure
  dynamic_reconfigure_callback =
      boost::bind(&AckermannVelocitySmoother::reconfigCB, this, _1, _2);

  dynamic_reconfigure_server = std::make_shared<
      dynamic_reconfigure::Server<navit_velocity_smoother::paramsConfig>>(nh);
  dynamic_reconfigure_server->setCallback(dynamic_reconfigure_callback);

  // Optional parameters
  nh.param("ackermann/frequency", frequency, 20.0);
  nh.param("ackermann/decel_factor", decel_factor, 1.0);
  nh.param("ackermann/decel_factor_safe", decel_factor_safe, 1.0);

  // Mandatory parameters
  if (nh.getParam("ackermann/steer_lim", steer_lim) == false) {
    ROS_ERROR("Missing steering limit parameter(s)");
    return false;
  }

  if (nh.getParam("ackermann/speed_lim_vx", speed_lim_vx) == false) {
    ROS_ERROR("Missing velocity limit parameter(s)");
    return false;
  }

  if (nh.getParam("ackermann/accel_lim_vx", accel_lim_vx) == false) {
    ROS_ERROR("Missing acceleration limit parameter(s)");
    return false;
  }

  // Deceleration can be more aggressive, if necessary
  decel_lim_vx = decel_factor * accel_lim_vx;
  // In safety cases (no topic command anymore), deceleration should be very
  // aggressive
  decel_lim_vx_safe = decel_factor_safe * accel_lim_vx;

  // topic names
  std::string raw_cmd_vel = "raw_cmd_vel";
  std::string smooth_cmd_vel = "smooth_cmd_vel";
  nh.param("ackermann/topic_names/raw_cmd_vel", raw_cmd_vel, raw_cmd_vel);
  nh.param("ackermann/topic_names/smooth_cmd_vel", smooth_cmd_vel,
           smooth_cmd_vel);

  // Publishers and subscribers
  raw_in_vel_sub = nh_.subscribe(raw_cmd_vel, 1,
                                 &AckermannVelocitySmoother::velocityCB, this);
  smooth_vel_pub =
      nh_.advertise<ackermann_msgs::AckermannDriveStamped>(smooth_cmd_vel, 1);

  return true;
}
}
