#include <dynamic_reconfigure/server.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <velocity_smoother.h>

#define PERIOD_RECORD_SIZE 5

namespace navit_velocity_smoother {

VelocitySmoother::VelocitySmoother(const std::string &name,
                                   ros::NodeHandle &p_nh)
    : name_(name), p_nh_(p_nh), shutdown_req(false), input_active(false),
      pr_next(0), dynamic_reconfigure_server(nullptr), worker_thread_(nullptr) {
}

void VelocitySmoother::reconfigCB(navit_velocity_smoother::paramsConfig &config,
                                  uint32_t level) {
  ROS_INFO("Reconfigure request : %f %f %f %f %f %f %f %f", config.speed_lim_vx,
           config.speed_lim_vy, config.speed_lim_w, config.accel_lim_vx,
           config.accel_lim_vy, config.accel_lim_w, config.decel_factor,
           config.decel_factor_safe);

  steer_lim = config.steer_lim;
  steer_speed_lim = config.steer_speed_lim;
  speed_lim_vx = config.speed_lim_vx;
  speed_lim_vy = config.speed_lim_vy;
  speed_lim_w = config.speed_lim_w;
  accel_lim_vx = config.accel_lim_vx;
  accel_lim_vy = config.accel_lim_vy;
  accel_lim_w = config.accel_lim_w;
  decel_factor = config.decel_factor;
  decel_factor_safe = config.decel_factor_safe;
  decel_lim_vx = decel_factor * accel_lim_vx;
  decel_lim_vy = decel_factor * accel_lim_vy;
  decel_lim_w = decel_factor * accel_lim_w;
  decel_lim_vx_safe = decel_factor_safe * accel_lim_vx;
  decel_lim_vy_safe = decel_factor_safe * accel_lim_vy;
  decel_lim_w_safe = decel_factor_safe * accel_lim_w;
}
}
