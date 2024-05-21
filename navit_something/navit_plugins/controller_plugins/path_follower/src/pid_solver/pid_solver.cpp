/*
* @Author: czk
* @Date:   2022-10-03 10:01:53
* @Last Modified by:   chenzongkui
* @Last Modified time: 2023-02-08 13:41:47
*/
#include <pid_solver/pid_solver.h>

namespace control {

/// PID controller
ControllerPid::ControllerPid() : saturation_threshold_(0.0) {}

ControllerPid::ControllerPid(float kp, float ki, float kd, float saturation_threshold)
    : kp_(kp), ki_(ki), kd_(kd), previous_error_(0.0), integral_(0.0), first_hit_(true), saturation_threshold_(saturation_threshold) {}

float ControllerPid::setControl(float error, float dt) {
  float diff = 0.0;
  if (dt > 0) {
    if (first_hit_) {
      diff = 0.0;
      first_hit_ = false;
    } else {
      diff = (error - previous_error_) / dt;
    }

    previous_error_ = error;
    integral_ += error * dt;
  } else {
    // VFATAL("the time step is " << dt);
  }

  integral_ = clamp(integral_, -saturation_threshold_, saturation_threshold_);

  return error * kp_ + diff * kd_ + integral_ * ki_;
}

void ControllerPid::reset() {
  previous_error_ = 0.0;
  integral_ = 0.0;
  first_hit_ = true;
}

/// 2-DOF PID controller
ControllerPid2Dof::Parameters::Parameters() {}

ControllerPid2Dof::Parameters::Parameters(float kp, float ki, float kd, float dt)
    : kp_(kp), ki_(ki), kd_(kd), kt_(0.0), wp_(1.0), ad_(0.0) {
  if (dt > 0) {
    bd_ = kd / dt;
  } else {
    // VFATAL("the time step is " << dt);
  }
}

ControllerPid2Dof::Parameters::Parameters(float kp, float ki, float kd, float dt, float kt, float wp, float tau)
    : kp_(kp), ki_(ki), kd_(kd), kt_(kt), wp_(wp) {
  if (dt > 0) {
    ad_ = tau / (tau + dt);
    bd_ = kd / (tau + dt);
  } else {
    // VFATAL("the time step is " << dt);
  }
}

ControllerPid2Dof::ControllerPid2Dof() : saturation_threshold_(0.0) {}

ControllerPid2Dof::ControllerPid2Dof(float kp, float ki, float kd, float dt, float saturation_threshold)
    : param_(kp, ki, kd, dt),
      dt_(dt),
      previous_out_(0.0),
      diff_(0.0),
      integral_(0.0),
      first_hit_(true),
      saturation_threshold_(saturation_threshold) {}
ControllerPid2Dof::ControllerPid2Dof(float kp, float ki, float kd, float dt, float kt, float wp, float tau, float saturation_threshold)
    : param_(kp, ki, kd, dt, kt, wp, tau),
      dt_(dt),
      previous_out_(0.0),
      diff_(0.0),
      integral_(0.0),
      first_hit_(true),
      saturation_threshold_(saturation_threshold) {}

float ControllerPid2Dof::setControl(float ref, float out) { return setControl(ref, out, dt_); }

float ControllerPid2Dof::setControl(float ref, float out, float dt) {
  if (first_hit_) {
    diff_ = 0.0;
    first_hit_ = false;
  } else {
    diff_ = param_.ad_ * diff_ - param_.bd_ * (out - previous_out_);
  }

  float control = param_.kp_ * (param_.wp_ * ref - out) + integral_ + diff_;
  float sat_control = clamp(control, -saturation_threshold_, saturation_threshold_);

  integral_ = (param_.ki_ * (ref - out) + param_.kt_ * (sat_control - control)) * dt;

  previous_out_ = out;

  return sat_control;
}

void ControllerPid2Dof::reset() {
  previous_out_ = 0.0;
  diff_ = 0.0;
  integral_ = 0.0;
  first_hit_ = true;
}

}  // namespace control
