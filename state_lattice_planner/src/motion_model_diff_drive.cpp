#include "trajectory_generator/motion_model_diff_drive.h"

MotionModelDiffDrive::MotionModelDiffDrive()
{
    // default setting
    MAX_YAWRATE = 0.8;
    MAX_D_YAWRATE = 2.0;
    MAX_ACCELERATION = 1.0;
    WHEEL_RADIUS = 0.125;
    TREAD = 0.5;
    MAX_WHEEL_ANGULAR_VELOCITY = 11.6;

    ratio = 0.35;
}

MotionModelDiffDrive::State::State(double _x, double _y, double _yaw, double _v, double _omega)
{
    x = _x;
    y = _y;
    yaw = _yaw;
    v = _v;
    omega = _omega;
}

MotionModelDiffDrive::VelocityParams::VelocityParams(void)
{

}

MotionModelDiffDrive::VelocityParams::VelocityParams(double _v0, double _a0, double _vt, double _vf, double _af)
{
    v0 = _v0;
    vt = _vt;
    vf = _vf;
    a0 = v0 < vt ? fabs(_a0) : -fabs(_a0);
    // std::cout << _a0 << ", " << a0 << std::endl;
    // a0 = _a0;
    af = vt > vf ? fabs(_af) : -fabs(_af);
    // std::cout << _af << ", " << af << std::endl;
    // af = _af;
    time = 0;
}

MotionModelDiffDrive::AngularVelocityParams::AngularVelocityParams(void)
{
    coefficients.resize(2);
    for(auto c : coefficients){
        c = Eigen::Vector4d::Zero();
    }
    coefficients_abc.resize(2);
    for(auto c : coefficients_abc){
        c = Eigen::Vector3d::Zero();
    }
    coefficients_ab.resize(2);
    for(auto c : coefficients_ab){
        c = Eigen::Vector2d::Zero();
    }
}

/**
  * @brief Constructor
  * @param[in] _k0 Initial angular velocity [rad/s] 初始点角速度
  * @param[in] _km Intermediate angular velocity [rad/s] 插值点角速度
  * @param[in] _kf Terminal angular velocity [rad/s] 终点角速度
  * @param[in] _sf Length of trajectory [m] 曲线的长度
  */
MotionModelDiffDrive::AngularVelocityParams::AngularVelocityParams(double _k0, double _km, double _kf, double _sf)
{
    k0 = _k0;
    km = _km;
    kf = _kf;
    sf = _sf;
    coefficients.resize(2);
    for(auto c : coefficients){
        c = Eigen::Vector4d::Zero();
    }
    coefficients_abc.resize(2);
    for(auto c : coefficients_abc){
        c = Eigen::Vector3d::Zero();
    }
    coefficients_ab.resize(2);
    for(auto c : coefficients_ab){
        c = Eigen::Vector2d::Zero();
    }
}

MotionModelDiffDrive::ControlParams::ControlParams(void)
{

}

MotionModelDiffDrive::ControlParams::ControlParams(const VelocityParams& _vel, const AngularVelocityParams& _omega)
{
    vel = _vel;
    omega = _omega;
}

MotionModelDiffDrive::Trajectory::Trajectory(void)
{

}

void MotionModelDiffDrive::set_param(const double max_yawrate, const double max_d_yawrate, const double max_acceleration, const double max_wheel_angular_velocity, const double wheel_radius, const double tread)
{
    MAX_YAWRATE = max_yawrate;
    MAX_D_YAWRATE = max_d_yawrate;
    MAX_ACCELERATION = max_acceleration;
    MAX_WHEEL_ANGULAR_VELOCITY = max_wheel_angular_velocity;
    WHEEL_RADIUS = wheel_radius;
    TREAD = TREAD;
}

/**
 * @brief 由当前状态向目标状态，在加速度，角速度约束下计算下个控制时间点的状态
 * @param  s                当前状态
 * @param  v                目标线速度
 * @param  omega            目标角速度
 * @param  dt               控制时间间隔
 * @param  output_s         下一个控制点的状态
 */
void MotionModelDiffDrive::update(const State& s, const double v, const double omega, const double dt, State& output_s)
{
    output_s.v = v;
    output_s.omega = omega;
    response_to_control_inputs(s, dt, output_s);

    output_s.x = s.x + output_s.v * dt * cos(s.yaw);
    output_s.y = s.y + output_s.v * dt * sin(s.yaw);
    output_s.yaw = s.yaw + output_s.omega * dt;
    if(output_s.yaw < -M_PI || output_s.yaw > M_PI){
        output_s.yaw = atan2(sin(output_s.yaw), cos(output_s.yaw));
    }
}

double MotionModelDiffDrive::calculate_linear_function(const double x, const Eigen::Vector2d& coeff)
{
    return coeff(0) * x + coeff(1);
}

double MotionModelDiffDrive::calculate_quadratic_function(const double x, const Eigen::Vector3d& coeff)
{
    return coeff(0) * x * x + coeff(1) * x + coeff(2);
}

double MotionModelDiffDrive::calculate_cubic_function(const double x, const Eigen::Vector4d& coeff)
{
    return coeff(0) * x * x * x + coeff(1) * x * x + coeff(2) * x + coeff(3);
}

/**
 * @brief  更新下一个控制节点的线速度，角速度
 *         线速度向目标速度调整，角速度向终点角速度调整
 * @param  state            当前控制节点状态
 * @param  dt               控制节点时间间隔
 * @param  output           下一个控制节点状态
 */
void MotionModelDiffDrive::response_to_control_inputs(const State& state, const double dt, State& output)
{
    double _dt = 1.0 / dt; // hz
    double k = state.omega;
    double _k = output.omega;
    double dk = (_k - k) * _dt;
    dk = std::max(std::min(dk, MAX_D_YAWRATE), -MAX_D_YAWRATE); // 角加速度

    _k = k + dk * dt; // 角速度
    output.omega = std::max(std::min(_k, MAX_YAWRATE), -MAX_YAWRATE);

    // adjust output.v 计算线速度
    control_speed(output, output);

    double v = state.v;
    double _v = output.v;
    double a = (_v - v) * _dt; // 加速度
    a = std::max(std::min(a, MAX_ACCELERATION), -MAX_ACCELERATION);
    output.v = v + a * dt;

    // additional omega limitation
    // double yawrate = output.v * output.omega;
    // if(fabs(yawrate) > MAX_YAWRATE){
    //     output.omega = MAX_YAWRATE / fabs(output.v) * (output.omega > 0 ? 1 : -1);
    // }
}

void MotionModelDiffDrive::control_speed(const State& state, State& _state)
{
    // speed control logic
    _state = state;
    // v = r * w
    // 不知道 TREAD 是什么，就不用它了
    // _state.v = WHEEL_RADIUS * std::min(
    //   fabs(_state.v) / WHEEL_RADIUS, 
    //   MAX_WHEEL_ANGULAR_VELOCITY - 0.5 * fabs(_state.omega) * TREAD / WHEEL_RADIUS) *
    //   (_state.v >= 0.0 ? 1 : -1);
    _state.v = WHEEL_RADIUS * fabs(_state.v) / WHEEL_RADIUS * (_state.v >= 0.0 ? 1 : -1);

    // double yawrate = _state.omega * _state.v;
    // if(fabs(yawrate) > MAX_YAWRATE){
    //     _state.v = MAX_YAWRATE / fabs(_state.omega) * (state.v > 0 ? 1 : -1);
    // }
}

double MotionModelDiffDrive::generate_trajectory(const double dt, const ControlParams& control_param, Trajectory& trajectory)
{
    // std::cout << "gen start" << std::endl;
    auto start = std::chrono::system_clock::now();
    AngularVelocityParams omega = control_param.omega;
    VelocityParams vel = control_param.vel;

    // lookup table 中的 sf 参数已经估计过大概的曲线长度
    // 根据当前的速度估计走完曲线需要多长时间
    vel.time = estimate_driving_time(control_param);
    // std::cout << "driving time: " << vel.time << "[s]" << std::endl;
    auto time = std::chrono::system_clock::now();
    double elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(time - start).count();
    // std::cout << "estimate time: " << elapsed_time << "[s]" << std::endl;

    // 获取各个时间控制点的线速度，以及该位置累计的距离
    // 提高速度达到目标速度（最大速度），获取各个控制点的速度和累计距离
    make_velocity_profile(dt, vel); 
    time = std::chrono::system_clock::now();
    elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(time - start).count();
    // std::cout << "v prof: " << elapsed_time << "[s]" << std::endl;
    // std::cout << vel.v0 << ", " << vel.vt << ", " << vel.vf << ", " << vel.time << ", " << omega.sf << ", " << std::endl;

    // omega.calculate_spline(ratio); // ratio = 0.5
    // omega.calculate_spline_abc(ratio); // ratio = 0.5
    omega.calculate_spline_ab(ratio); // ratio = 0.5
    const int N = s_profile.size(); // 生成控制点的速度，和累计的距离
    // std::cout << "s_profile size: " << N << std::endl;
    if(N == 0){
        return 0.0;
    }
    // omega.sf 是 control_param 状态表中路径曲线长度
    // sf_2 是准备插值的点在总曲线的长度位置
    double sf_2 = omega.sf * ratio; 

    State state(0, 0, 0, vel.v0, omega.k0);
    State state_(0, 0, 0, vel.v0, omega.k0);
    Eigen::Vector3d pose;
    pose << state.x, state.y, state.yaw;
    trajectory.trajectory.resize(N);
    trajectory.velocities.resize(N);
    trajectory.angular_velocities.resize(N);
    trajectory.trajectory[0] = pose;
    trajectory.velocities[0] = state.v;
    // trajectory.angular_velocities[0] = state.v * state.omega;
    trajectory.angular_velocities[0] = state.omega;

    // x 轴是曲线长度，y 轴是角速度，coefficients 是 ax^3 + bx^2 + cx + d 系数
    for(int i=1;i<N;i++) {
      double s = s_profile[i];
      double k = 0;
      if(s < sf_2){
        // k = calculate_cubic_function(s, omega.coefficients[0]); // 插值角速度前的系数
        // k = calculate_quadratic_function(s, omega.coefficients_abc[0]); // 插值角速度前的系数
        k = calculate_linear_function(s, omega.coefficients_ab[0]); // 插值角速度前的系数
      }else{
        // k = calculate_cubic_function(s, omega.coefficients[1]); // 插值角速度后的系数
        // k = calculate_quadratic_function(s, omega.coefficients_abc[1]); // 插值角速度前的系数
        k = calculate_linear_function(s, omega.coefficients_ab[1]); // 插值角速度前的系数
      }
      update(state, v_profile[i], k, dt, state_);
      state = state_;
      pose << state.x, state.y, state.yaw;
      trajectory.trajectory[i] = pose;
      trajectory.velocities[i] = state.v;
      // trajectory.angular_velocities[i] = state.v * state.omega;
      trajectory.angular_velocities[i] = state.omega;
    }
    time = std::chrono::system_clock::now();
    elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(time - start).count();
    // std::cout << "gen t: " << elapsed_time << "[s]" << std::endl;
    return s_profile.back();
}

void MotionModelDiffDrive::generate_last_state(
  const double dt, const double trajectory_length, const VelocityParams& _vel,
  const double k0, const double km, const double kf, Eigen::Vector3d& output)
{
  // std::cout << "generate_last_state start" << trajectory_length << std::endl;
    // std::cout << "--- generate last state ---" << std::endl;
    // std::cout << k0 << ", " << km << ", " << kf << ", " << trajectory_length << std::endl;
    AngularVelocityParams omega(k0, km, kf, trajectory_length);
    VelocityParams vel = _vel;

    vel.time = estimate_driving_time(ControlParams(vel, omega));

    make_velocity_profile(dt, vel);

    // omega.calculate_spline(ratio);
    // omega.calculate_spline_abc(ratio);
    omega.calculate_spline_ab(ratio);
    const int N = s_profile.size();
    double sf_2 = omega.sf * ratio;
    State state(0, 0, 0, vel.v0, omega.k0);
    output << state.x, state.y, state.yaw;
    for(int i=1;i<N;i++){
        double s = s_profile[i];
        double k = 0;
        if(s < sf_2){
            // k = calculate_cubic_function(s, omega.coefficients[0]);
            // k = calculate_quadratic_function(s, omega.coefficients_abc[0]);
            k = calculate_linear_function(s, omega.coefficients_ab[0]);
        }else{
            // k = calculate_cubic_function(s, omega.coefficients[1]);
            // k = calculate_quadratic_function(s, omega.coefficients_abc[1]);
            k = calculate_linear_function(s, omega.coefficients_ab[1]);
        }
        update(state, v_profile[i], k, dt, state);
    }
    output << state.x, state.y, state.yaw; // 终点 x y yaw 状态
    // std::cout << "generate_last_state end" << trajectory_length << std::endl;
}

/**
 * @brief 求解曲线参数  (a, b, c, d) <- ax^3+bx^2+cx+d
 *       std::vector<Eigen::Vector4d> coefficients;
 * @param ratio 角速度插值的位置，x 轴是曲线长度，y 轴是角速度
 */
void MotionModelDiffDrive::AngularVelocityParams::calculate_spline(double ratio)
{
    // std::cout << "spline" << std::endl;
    // 3d spline interpolation
    // 轨迹点累计的距离，0 起点累计 0 距离， sf * ratio 插值位置的距离，sf 轨迹总距离
    Eigen::Vector3d x(0, sf * ratio, sf);
    // 轨迹点的角速度，k0 起点角速度，km 插值角速度，kf 终点角速度
    Eigen::Vector3d y(k0, km, kf); 
    Eigen::Matrix<double, 8, 8> s;
    // a0*x^3+b0*x^2+c0*x+d0 + a1*x^3+b1*x^2+c1*x+d1
    s << x(0) * x(0) * x(0), x(0) * x(0), x(0), 1, 0, 0, 0, 0, // [s0~sm] ; a0*x^3+b0*x^2+c0*x+d0 + a1*x^3+b1*x^2+c1*x+d1
         x(1) * x(1) * x(1), x(1) * x(1), x(1), 1, 0, 0, 0, 0, // [s0~sm] ; a0*x^3+b0*x^2+c0*x+d0 + a1*x^3+b1*x^2+c1*x+d1
         0, 0, 0, 0, x(1) * x(1) * x(1), x(1) * x(1) , x(1), 1, // [sm~sf] ; a0*x^3+b0*x^2+c0*x+d0 + a1*x^3+b1*x^2+c1*x+d1
         0, 0, 0, 0, x(2) * x(2) * x(2), x(2) * x(2) , x(2), 1, // [sm~sf] ; a0*x^3+b0*x^2+c0*x+d0 + a1*x^3+b1*x^2+c1*x+d1
         3 * x(1) * x(1), 2 * x(1), 1, 0, -3 * x(1) * x(1), -2 * x(1), -1, 0, // [sm] 3a0*x^2+2b0*x+c0+0 - (3a1*x^2+2b1*x+c1+0) 两段函数在 [sm] 位置一阶导相等
         6 * x(1), 2, 0, 0, -6 * x(1), -2, 0, 0, // [sm] 6a0*x+2b0+0+0 - (6a1*x+2b1+0+0) 两段函数在 [sm] 位置二阶导相等，表示两段函数在 [sm] 连续
         6 * x(0), 2, 0, 0, 0, 0, 0, 0, // [s0] 6a0*x+2b0+0+0 + (6a1*x+2b1+0+0) 第一段函数在起始点二阶导为 0，表示起始位置一阶导为常熟，稳定启动
         0, 0, 0, 0, 6 * x(2), 2, 0, 0; // [sf] 6a0*x+2b0+0+0 + (6a1*x+2b1+0+0) 第二段函数在结束点二阶导为 0，表示结束位置一阶导为常熟，稳定结束
    Eigen::VectorXd c = Eigen::VectorXd::Zero(8);
    c << y(0), y(1), y(1), y(2), 0, 0, 0, 0;
    Eigen::VectorXd a = s.inverse() * c;
    coefficients[0] = a.segment(0, 4); // k0~km 曲线的 a,b,c,d 系数
    coefficients[1] = a.segment(4, 4); // km~kf 曲线的 a,b,c,d 系数
}

/**
 * @brief 求解曲线参数  (a, b, c) <- ax^2+bx+c
 *       std::vector<Eigen::Vector3d> coefficients_abc;
 * @param ratio 角速度插值的位置，x 轴是曲线长度，y 轴是角速度
 */
void MotionModelDiffDrive::AngularVelocityParams::calculate_spline_abc(double ratio)
{
    // std::cout << "spline" << std::endl;
    // 3d spline interpolation
    // 轨迹点累计的距离，0 起点累计 0 距离， sf * ratio 插值位置的距离，sf 轨迹总距离
    Eigen::Vector3d x(0, sf * ratio, sf);
    // 轨迹点的角速度，k0 起点角速度，km 插值角速度，kf 终点角速度
    Eigen::Vector3d y(k0, km, kf); 
    Eigen::Matrix<double, 6, 6> s;
    // a0*x^2+b0*x+c0 + a1*x^2+b1*x+c1
    s << x(0) * x(0), x(0), 1, 0, 0, 0, // [s0~sm] ; a0*x^2+b0*x+c0 + a1*x^2+b1*x+c1
         x(1) * x(1), x(1), 1, 0, 0, 0, // [s0~sm] ; a0*x^2+b0*x+c0 + a1*x^2+b1*x+c1
         0, 0, 0, x(1) * x(1) , x(1), 1, // [sm~sf] ; a0*x^2+b0*x+c0 + a1*x^2+b1*x+c1
         0, 0, 0, x(2) * x(2) , x(2), 1, // [sm~sf] ; a0*x^2+b0*x+c0 + a1*x^2+b1*x+c1
         2 * x(1), 1, 0, -2 * x(1), -1, 0, // [sm] 2a0*x+b0+0 - (2a1*x+b1+0) 两段函数在 [sm] 位置一阶导相等
         0, 0, 0, 2 * x(2), 1, 0; // [sf] 2a0*x+b0+0 + 2a1*x+b1+0 第二段函数在 [sf] 位置一阶导为 0
    Eigen::VectorXd c = Eigen::VectorXd::Zero(6);
    c << y(0), y(1), y(1), y(2), 0, 0;
    Eigen::VectorXd a = s.inverse() * c;
    coefficients_abc[0] = a.segment(0, 3); // k0~km 曲线的 a,b,c 系数
    coefficients_abc[1] = a.segment(3, 3); // km~kf 曲线的 a,b,c 系数
}

/**
 * @brief 求解曲线参数  (a, b) <- ax+b
 *       std::vector<Eigen::Vector2d> coefficients_ab;
 * @param ratio 角速度插值的位置，x 轴是曲线长度，y 轴是角速度
 */
void MotionModelDiffDrive::AngularVelocityParams::calculate_spline_ab(double ratio)
{
    // std::cout << "spline" << std::endl;
    // 3d spline interpolation
    // 轨迹点累计的距离，0 起点累计 0 距离， sf * ratio 插值位置的距离，sf 轨迹总距离
    Eigen::Vector3d x(0, sf * ratio, sf);
    // 轨迹点的角速度，k0 起点角速度，km 插值角速度，kf 终点角速度
    Eigen::Vector3d y(k0, km, kf); 
    Eigen::Matrix<double, 4, 4> s;
    // a0*x^2+b0*x+c0 + a1*x^2+b1*x+c1
    s << x(0), 1, 0, 0, // [s0~sm] ; a0*x+b0 + a1*x+b1
         x(1), 1, 0, 0, // [s0~sm] ; a0*x+b0 + a1*x+b1
         0, 0, x(2), 1, // [sm~sf] ; a0*x+b0 + a1*x+b1
         x(1), 1, -x(1), -1; // [sm] 相等
    Eigen::VectorXd c = Eigen::VectorXd::Zero(4);
    c << y(0), y(1), y(2), 0;
    Eigen::VectorXd a = s.inverse() * c;
    coefficients_ab[0] = a.segment(0, 2); // k0~km 曲线的 a,b,c 系数
    coefficients_ab[1] = a.segment(2, 2); // km~kf 曲线的 a,b,c 系数
}

void MotionModelDiffDrive::make_velocity_profile(const double dt, const VelocityParams& v_param)
{
    /*
     *  trapezoid control
     */

    /***************************************
         vt  ________________
           /|                |\
          / |                | \
     v0  /  |                |  \ vf
          a0      a=0         af

    ***************************************/
    int size = v_param.time / dt + 1; // 采样数量
    // std::cout << "size: " << size << std::endl;
    v_profile.resize(size); // 采样时间 v_param.time 内每个时间采样点的速度 v
    s_profile.resize(size); // 采样时间 v_param.time 内每个时间采样点的累积的距离 s

    double s = 0;
    double t = 0;
    for(int i=0;i<size;++i){
        // acceleration time
        double ta = fabs((v_param.vt - v_param.v0) / v_param.a0);
        // std::cout << "ta: " << ta << std::endl;
        // deceleration time
        double td = v_param.time - fabs((v_param.vf - v_param.vt) / v_param.af);
        // std::cout << "td: " << ta << std::endl;

        double v = 0;
        if(t < 0){
            v = v_param.v0;
        }else if(t < ta){
            if(fabs(v_param.v0 + v_param.a0 * t) < fabs(v_param.vt)){
                v = v_param.v0 + v_param.a0 * t;
            }else{
                v = v_param.vt;
            }
        }else if(ta <= t && t < td){
            v = v_param.vt;
        }else if(td <= t && t < v_param.time){
            if(fabs(v_param.vt - v_param.af * (t - td)) > fabs(v_param.vf)){
                v = v_param.vt - v_param.af * (t - td);
            }else{
                v = v_param.vt;
            }
        }else{
            v = v_param.vf;
        }
        v_profile[i] = v;
        // std::cout << "v: " << v << std::endl;
        s_profile[i] = s;
        // std::cout << "s: " << s << std::endl;
        s += v * dt;
        t += dt;
    }
}

double MotionModelDiffDrive::estimate_driving_time(const ControlParams& control)
{
    // acceleration time
    // std::cout << "a0:" << control.vel.a0 << ", af:" << control.vel.af << std::endl;
    double t0 = fabs((control.vel.vt - control.vel.v0) / control.vel.a0);
    // deceleration time
    double td = fabs((control.vel.vf - control.vel.vt) / control.vel.af);
    // std::cout << t0 << ", " << td << std::endl;
    
    // 加速距离
    // std::cout << "v0:" << control.vel.v0 << ", vt:" << control.vel.vt << ", vf:" << control.vel.vf << std::endl;
    double s0 = 0.5 * fabs(control.vel.vt + control.vel.v0) * t0;
    // 减速距离
    double sd = 0.5 * fabs(control.vel.vt + control.vel.vf) * td;
    // std::cout << s0 << ", " << sd << std::endl;
    // 匀速距离
    double st = fabs(control.omega.sf) - s0 - sd;
    // 匀速时间
    double tt = st / fabs(control.vel.vt);
    // std::cout << st << ", " << tt << std::endl;
    double driving_time = t0 + tt + td;
    return driving_time;
}

void MotionModelDiffDrive::update_ratio(const Eigen::Vector3d& goal, const double boundary, const double rang)
{
  int move_dir = goal.segment(0,2).norm() < boundary ? 1 : -1;
  double yaw_max = 0.17*4;
  double move = fabs(goal(2))/yaw_max * rang * move_dir;
  move = std::min(rang, std::max(-rang, move));
  ratio = 0.5 + move;
  // std::cout << "[SLP] new ratio is " << ratio << std::endl;
}
