#include "trajectory_generator/trajectory_generator_diff_drive.h"
#include "ros/ros.h"

TrajectoryGeneratorDiffDrive::TrajectoryGeneratorDiffDrive(void)
{
    // default
    h << 0.05, 0.05, 0.1;
    MAX_YAWRATE = 1.0;
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
    // std::cout << "d_goal: " << distance_to_goal << "[m]" << std::endl;

    output = init_control_param;

    int count = 0;

    Eigen::Matrix3d jacobian;
    update_h(goal);
    // model.update_ratio(goal, 4, 0.15);
    set_optimization_param(0.03, 0.03, 0.03);
    // 在各种线速度，角速度，曲率状态中找到可用的轨迹曲线
    while(1) {
      if(cost.norm() < tolerance){ // x y yaw 的模
        // ROS_DEBUG("[SLP] successfully optimized in %d iteration.", count);
        break;
      }else if(count >= max_iteration){
        ROS_DEBUG("[SLP] can't optimize trajectory.");
        return -1;
      }
      trajectory.trajectory.clear();
      trajectory.velocities.clear();
      trajectory.angular_velocities.clear();
      // 由初始化状态 output 和控制间隔 dt ,状态表中的 sf 预估曲线长度来实时生成轨迹
      model.generate_trajectory(dt, output, trajectory);
      // std::cout << "goal[" << goal(0) << "," << goal(1) << "], end[" <<
      //   trajectory.trajectory.back()(0) << "," << trajectory.trajectory.back()(1) << "]" << std::endl;
      /*
      std::cout << "size: " << trajectory.trajectory.size() << std::endl;
      if(trajectory.trajectory.size() <= 1){
          std::cout << "failed to generate trajecotry!!!" << std::endl;
          return -1;
      }
      */
      get_jacobian(dt, output, h, jacobian);
      // std::cout << "j: \n" << jacobian << std::endl;
      // std::cout << "j^-1: \n" << jacobian.inverse() << std::endl;
      cost = goal - trajectory.trajectory.back(); // x y yaw 的模
      // Eigen::Vector3d dp = jacobian.inverse() * cost;
      // Ax = b； <==> jacobian · dp = cost 求解 dp
      Eigen::Vector3d dp = jacobian.lu().solve(cost);
      if(std::isnan(dp(0)) || std::isnan(dp(1)) || std::isnan(dp(2)) ||
        std::isinf(dp(0)) || std::isinf(dp(1)) || std::isinf(dp(2)) ||
        fabs(dp(2)) > distance_to_goal) {
        ROS_DEBUG("[SLP] diverge to infinity !");
        return -1;
      }
      // std::cout << "cost vector: " << cost.transpose() << std::endl;
      // std::cout << "cost: " << cost.norm() << std::endl;
      // std::cout << "dp" << dp.transpose() << std::endl;
      // std::cout << "calculate scale factor" << std::endl;
      // 调整 output 状态，用新状态再计算一次轨迹
      calculate_scale_factor(dt, tolerance, goal, cost, output, trajectory, dp);

      // std::cout << "last state: \n" << trajectory.trajectory.back() << std::endl;
      // std::cout << "cost vector: \n" << cost << std::endl;
      // std::cout << "cost: \n" << cost.norm() << std::endl;
      if(cost.norm() > 100){
        ROS_DEBUG("[SLP] cost is too large, cost:%.3f, limit is 100", cost.norm());
        return -1;
      }
      // std::cout << "dp: \n" << dp << std::endl;
      if((cost.norm() > last_cost) ||
        std::isnan(dp(0)) || std::isnan(dp(1)) || std::isnan(dp(2)) ||
        std::isinf(dp(0)) || std::isinf(dp(1)) || std::isinf(dp(2)) ||
        fabs(dp(2)) > distance_to_goal){
        ROS_DEBUG("[SLP] after calculate factor, diverge to infinity !");
        return -1;
      }
      last_cost = cost.norm();

      output.omega.km += dp(0);
      output.omega.kf += dp(1);
      output.omega.sf += dp(2);
      // output.omega.km = std::min(std::max(output.omega.km, -MAX_YAWRATE), MAX_YAWRATE);
      // output.omega.kf = std::min(std::max(output.omega.kf, -MAX_YAWRATE), MAX_YAWRATE);
      // std::cout << "output: " << output.omega.km << ", " << output.omega.kf << ", " << output.omega.sf << std::endl;

      // if(fabsf(output.omega.sf - distance_to_goal) > distance_to_goal * 0.5){
      //     if(verbose){
      //         std::cout << "optimization error!!!" << std::endl;
      //     }
      //     return -1;
      // }
      //std::cout << output.omega.km << ", " << output.omega.kf << ", " << output.omega.sf << std::endl;

      // std::cout << "count: " << count << std::endl;
      count++;
    }
    ROS_DEBUG("[SLP] final cost [%.4f, %.4f, %.4f]", cost[0], cost[1], cost[2]);
    return cost.norm();
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
  MotionModelDiffDrive::Trajectory& trajectory, Eigen::Vector3d& dp)
{
    double factor = fabs(goal(2))*2/M_PI;
    factor = std::min(2.0, std::max(factor, 1.0));
    double max_trajectory_length = (fabs(goal(0))+fabs(goal(1))) * factor;

    for(double beta=0.25;beta<=1.5;beta+=0.25){
        // std::cout << "beta: " << beta << std::endl;
        if(beta == 1.0f){
            continue;
        }
        Eigen::Vector3d dp_ = beta * dp;
        MotionModelDiffDrive::Trajectory trajectory_;
        MotionModelDiffDrive::ControlParams output_ = output;
        output_.omega.km += dp_(0);
        output_.omega.kf += dp_(1);
        output_.omega.sf += dp_(2);
        // std::cout << "dp_: " << dp_.transpose() << std::endl;
        // output_.omega.km = std::min(std::max(output_.omega.km, -MAX_YAWRATE), MAX_YAWRATE);
        // output_.omega.kf = std::min(std::max(output_.omega.kf, -MAX_YAWRATE), MAX_YAWRATE);
        // std::cout << "output: " << output_.omega.km << ", " << output_.omega.kf << ", " << output_.omega.sf << std::endl;
        double s = model.generate_trajectory(dt, output_, trajectory_); 
        // std::cout << "size: " << trajectory_.trajectory.size() << std::endl;
        if(trajectory_.trajectory.size() <= 1){
          // std::cout << "failed to generate trajecotry!!!" << std::endl;
          continue;
        }
        Eigen::Vector3d cost_ = goal - trajectory_.trajectory.back();
        // std::cout << cost_.norm() << " vs " << cost.norm() << std::endl;
        if(cost_.norm() < cost.norm()){
            if (s >= max_trajectory_length) {
              // ROS_INFO("goal [%f, %f, %f], trajectory (%f) is too long, max length is %f ",
              //   goal(0), goal(1), goal(2), s, max_trajectory_length);
              continue;
            }
            // TODO: output_.omega.sf;
            cost = cost_;
            output = output_;
            trajectory = trajectory_;
            dp = dp_;
            // std::cout << "updated!!!" << std::endl;
        }else{
            // std::cout << "not updated!!!" << std::endl;
        }
        if(cost.norm() < tolerance){
            return;
        }
    }
}

void TrajectoryGeneratorDiffDrive::update_h(const Eigen::Vector3d& goal)
{
  double dk = 0.05, ds = 0.03;

  if (goal(1)>0) h(0) = dk;
  else if (goal(1)<0) h(0) = -dk;

  if (goal(2)>0 && goal(0)>0) h(1) = dk;
  else if (goal(2)>0 && goal(0)<0) h(1) = -dk;
  else if (goal(2)<0 && goal(0)>0) h(1) = -dk;
  else if (goal(2)<0 && goal(0)<0) h(1) = dk;
  
  h(2) = ds;
  // ROS_INFO("update h k [%.3f, %.3f, %.3f]", h(0), h(1), h(2));
}
