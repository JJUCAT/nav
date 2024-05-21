/*
* @Author: czk
* @Date:   2022-10-03 10:01:38
* @Last Modified by:   chenzongkui
* @Last Modified time: 2022-10-03 10:48:33
*/
#pragma once

#include "macros.h"
#include "math.h"

namespace control {

class ControllerPid {
 public:
  ControllerPid();

  ControllerPid(float kp, float ki, float kd, float saturation_threshold);

  ~ControllerPid() = default;

  float setControl(float error, float dt);

  void reset();

 private:
  /// PID parameters
  float kp_, ki_, kd_;

  float previous_error_;

  float integral_;

  /// to indicate first run
  bool first_hit_;

  /// the saturation threshold of integral
  const float saturation_threshold_;

  DISALLOW_COPY_AND_ASSIGN(ControllerPid);
};

class ControllerPid2Dof {
 public:
  struct Parameters {
    Parameters();

    Parameters(float kp, float ki, float kd, float dt);

    Parameters(float kp, float ki, float kd, float dt, float kt, float wp, float tau);

    /// PID parameters
    float kp_, ki_, kd_;

    /// Back-calculation feedback gain for integral anti windup
    float kt_;

    /// Propotional setpoint weightingcoeff [0,1]
    float wp_;

    /// Low pass filter coeffs for derivative
    float ad_, bd_;
  } param_;

  ControllerPid2Dof();

  ControllerPid2Dof(float kp, float ki, float kd, float dt, float saturation_threshold);

  ControllerPid2Dof(float kp, float ki, float kd, float dt, float kt, float wp, float tau, float saturation_threshold);

  ~ControllerPid2Dof() = default;

  float setControl(float ref, float out);

  float setControl(float ref, float out, float dt);

  void reset();

 private:
  float dt_;

  float previous_out_;

  float diff_;

  float integral_;

  /// to indicate first run
  bool first_hit_;

  /// the saturation threshold of integral
  float saturation_threshold_;

  DISALLOW_COPY_AND_ASSIGN(ControllerPid2Dof);
};

}  // namespace control
