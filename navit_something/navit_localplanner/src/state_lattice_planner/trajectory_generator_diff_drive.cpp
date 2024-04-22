#include <state_lattice_planner/trajectory_generator/trajectory_generator_diff_drive.h>
#include "Eigen/src/Core/Matrix.h"
#include "angles/angles.h"
#include <chrono>
#include "ros/ros.h"

TrajectoryGeneratorDiffDrive::TrajectoryGeneratorDiffDrive(void)
{
    // default
    h << 0.05, 0.05, 0.1;
    MAX_YAWRATE = 1.0;
    init_hv();
}

// 没用
void TrajectoryGeneratorDiffDrive::set_optimization_param(const double dkm, const double dkf, const double dsf)
{
    h << dkm, dkf, dsf;
}

void TrajectoryGeneratorDiffDrive::set_motion_param(const double max_yawrate, const double max_d_yawrate, const double max_acceleration, const double max_wheel_angular_velocity, const double wheel_radius, const double tread)
{
    MAX_YAWRATE = max_yawrate;
    model.set_param(max_yawrate, max_d_yawrate, max_acceleration, max_wheel_angular_velocity, wheel_radius, tread);
}

void TrajectoryGeneratorDiffDrive::set_verbose(bool verbose_)
{
    verbose = verbose_;
}

/**
 * @brief  根据状态生成目标点的轨迹
 * @param  goal                 目标点
 * @param  init_control_paramMy 根据状态表初始化的当前状态
 * @param  dt                   根据控制频率 HZ 得到控制点的时间间隔
 * @param  tolerance            抵达目标的允许误差，也是 cost
 * @param  max_iteration        最大迭代次数
 * @param  output               控制
 * @param  trajectory           轨迹
 * @return double               cost
 */
double TrajectoryGeneratorDiffDrive::generate_optimized_trajectory(
  const Eigen::Vector3d& goal, const MotionModelDiffDrive::ControlParams& init_control_param,
  const double dt, const double tolerance, const int max_iteration,
  MotionModelDiffDrive::ControlParams& output, MotionModelDiffDrive::Trajectory& trajectory)
{
    Eigen::Vector3d cost(1e2, 1e2, 1e2); // 终点与采样点 x y yaw 差的模
    double last_cost = cost.norm();
    double distance_to_goal = goal.segment(0, 2).norm();

    output = init_control_param;
    int count = 0;
    Eigen::Matrix3d jacobian;
    // update_h(goal);
    std::queue<Eigen::Vector3d> hq = unorder_hv();
    if (!update_h(hq)) {
      ROS_ERROR("[SLP] init hv !");
      return -1;
    }
    auto start = std::chrono::system_clock::now();
    auto tmp_trajectory = trajectory;
    auto tmp_output = output;
    auto tmp_cost = cost;
    double tmp_s = output.omega.sf, s = -1, last_s = -1;
    bool generate = false;
    auto is_dp_illegal = [&](const Eigen::Vector3d& dp, const double min_dist) {
      if(std::isnan(dp(0)) || std::isnan(dp(1)) || std::isnan(dp(2)) ||
         std::isinf(dp(0)) || std::isinf(dp(1)) || std::isinf(dp(2)) ||
         fabs(dp(2)) > min_dist) {
         return true;
      }
      return false;
    };

    // 在各种线速度，角速度，曲率状态中找到可用的轨迹曲线
    while(1) {
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - start);
      double time_lapses = double(duration.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den;
      if (time_lapses > timeout_) break;
      if(count >= max_iteration) break;
      trajectory.trajectory.clear();
      trajectory.velocities.clear();
      trajectory.angular_velocities.clear();
      // 由初始化状态 output 和控制间隔 dt ,状态表中的 sf 预估曲线长度来实时生成轨迹
      model.generate_trajectory(dt, output, trajectory);
      cost = goal - trajectory.trajectory.back(); // x y yaw 的模
      double yaw_diff = fabs(angles::normalize_angle(cost(2)));
      if (cost.segment(0, 2).norm() < min_dist_err_ && yaw_diff < min_yaw_err_) {
        // if (!generate || ((tmp_s < last_s) && (generate && tmp_cost.norm() > cost.norm()))) {
        if (!generate || (tmp_s < last_s)) {
          tmp_trajectory = trajectory;
          tmp_output = output;
          tmp_cost = cost;
          last_s = tmp_s;
          generate = true;
        }
      }
      if (cost.segment(0, 2).norm() < 0.1 && yaw_diff < 0.17) break;

      // Eigen::Vector3d dp = jacobian.inverse() * cost;
      // Ax = b； <==> jacobian · dp = cost 求解 dp
      get_jacobian(dt, output, h, jacobian);
      Eigen::Vector3d dp = jacobian.lu().solve(cost);
      if(is_dp_illegal(dp, distance_to_goal)) {
        ROS_DEBUG("[SLP] diverge to infinity !");
        if (!update_h(hq)) break;
        else { output=init_control_param; last_cost = 1000000; continue; }
      }
      // 调整 output 状态，用新状态再计算一次轨迹
      calculate_scale_factor(dt, tolerance, goal, cost, output, trajectory, dp, s);
      if((cost.norm() > last_cost) || is_dp_illegal(dp, distance_to_goal) || cost.norm() > 100 ) {
        ROS_DEBUG("[SLP] wrong direction !");
        if (!update_h(hq)) break;
        else { output=init_control_param; last_cost = 1000000; continue; }
      }
      last_cost = cost.norm();
      tmp_s = s;
      output.omega.km += dp(0);
      output.omega.kf += dp(1);
      output.omega.sf += dp(2);
      count++;
    }

    if (generate) {
      trajectory = tmp_trajectory;
      output = tmp_output;
      cost = tmp_cost;
      return cost.norm();
    }
    return -1;
}

/**
 * @brief
 * @param  dt               步进
 * @param  control          机器当前位置的状态？还是迭代的状态？
 * @param  h                dkm,dkf,dsf 插值点角速度，终点角速度，曲线长度梯度
 * @param  j                雅克比
 */
void TrajectoryGeneratorDiffDrive::get_jacobian(
  const double dt, const MotionModelDiffDrive::ControlParams& control,
  const Eigen::Vector3d& h, Eigen::Matrix3d& j)
{
    /*
     * h: (dkm, dkf, dsf)
     */
    //std::cout << "j start" << std::endl;
    MotionModelDiffDrive::AngularVelocityParams omega = control.omega;
    Eigen::Vector3d x0;
    model.generate_last_state(dt, omega.sf, control.vel, omega.k0, omega.km - h(0), omega.kf, x0);
    Eigen::Vector3d x1;
    model.generate_last_state(dt, omega.sf, control.vel, omega.k0, omega.km + h(0), omega.kf, x1);

    Eigen::Vector3d dx_dkm;
    dx_dkm << (x1 - x0) / (2.0 * h(0));

    model.generate_last_state(dt, omega.sf, control.vel, omega.k0, omega.km, omega.kf - h(1), x0);
    model.generate_last_state(dt, omega.sf, control.vel, omega.k0, omega.km, omega.kf + h(1), x1);

    Eigen::Vector3d dx_dkf;
    dx_dkf << (x1 - x0) / (2.0 * h(1));

    model.generate_last_state(dt, omega.sf - h(2), control.vel, omega.k0, omega.km, omega.kf, x0);
    model.generate_last_state(dt, omega.sf + h(2), control.vel, omega.k0, omega.km, omega.kf, x1);

    Eigen::Vector3d dx_dsf;
    dx_dsf << (x1 - x0) / (2.0 * h(2));

    j << dx_dkm(0), dx_dkf(0), dx_dsf(0),
         dx_dkm(1), dx_dkf(1), dx_dsf(1),
         dx_dkm(2), dx_dkf(2), dx_dsf(2);
}

void TrajectoryGeneratorDiffDrive::calculate_scale_factor(
  double dt, double tolerance, const Eigen::Vector3d& goal,
  Eigen::Vector3d& cost, MotionModelDiffDrive::ControlParams& output,
  MotionModelDiffDrive::Trajectory& trajectory, Eigen::Vector3d& dp, double& s)
{
  double limit_x = 2.0f;
  double factor = (1.0-goal(0)/limit_x) + 1.0;
  // factor = std::min(std::max(0.9, factor),1.5); // 1x_6x6_p19_42k
  factor = std::min(std::max(0.9, factor),1.2);
  double max_trajectory_length = (fabs(goal(0))+fabs(goal(1))) * factor;
  double min_trajectory_length = std::hypot(goal(0),goal(1));
  for (double beta=0.25;beta<=1.5;beta+=0.25) {
    if(beta == 1.0f) continue;

    Eigen::Vector3d dp_ = beta * dp;
    MotionModelDiffDrive::Trajectory trajectory_;
    MotionModelDiffDrive::ControlParams output_ = output;
    output_.omega.km += dp_(0);
    output_.omega.kf += dp_(1);
    output_.omega.sf += dp_(2);
    s = model.generate_trajectory(dt, output_, trajectory_);
    if(trajectory_.trajectory.size() <= 1) continue;
    Eigen::Vector3d cost_ = goal - trajectory_.trajectory.back();
    if (cost_.norm() < cost.norm()) {
      if ((s >= max_trajectory_length) || (s <= min_trajectory_length)) {
        // ROS_INFO("goal [%f, %f, %f], trajectory (%f) is too long, max length is %f ",
        //   goal(0), goal(1), goal(2), s, max_trajectory_length);
        continue; // 全部试一遍
      }
      cost = cost_;
      output = output_;
      trajectory = trajectory_;
      dp = dp_;
    }
  }
}

void TrajectoryGeneratorDiffDrive::update_h(const Eigen::Vector3d& goal)
{
  double dk = 0.03, ds = 0.03;

  if (goal(1)>0) h(0) = dk;
  else if (goal(1)<0) h(0) = -dk;

  if (goal(0)>0 && goal(1)>0) h(1) = -dk;
  else if (goal(0)>0 && goal(1)<0) h(1) = dk;
  else if (goal(0)<0 && goal(0)>0) h(1) = dk;
  else if (goal(0)<0 && goal(1)<0) h(1) = -dk;

  h(2) = ds;
  // ROS_INFO("update h k [%.3f, %.3f, %.3f]", h(0), h(1), h(2));
}

void TrajectoryGeneratorDiffDrive::init_hv()
{
  double dk = 0.05, ds = 0.08;
  Eigen::Vector3d h;
  for (int i = -1; i <=1; i +=2) {
    for (int j = -1; j <=1; j +=2) {
      for (int k = -1; k <=1; k +=2) {
        h << dk * i, dk * j, ds * k;
        hv_.push_back(h);
      }
    }
  }
}

std::queue<Eigen::Vector3d> TrajectoryGeneratorDiffDrive::unorder_hv()
{
  auto rand = [](int i) { return std::rand()%i; };
  std::srand(unsigned (time(0)));
  random_shuffle(hv_.begin(),hv_.end(),rand);

  std::queue<Eigen::Vector3d> hq;
  for (auto h : hv_) hq.push(h);
  return hq;
}

bool TrajectoryGeneratorDiffDrive::update_h(std::queue<Eigen::Vector3d>& hq)
{
  if (hq.empty()) return false;
  h = hq.front();
  hq.pop();
  return true;
}
