/**
* @file state_lattice_planner.cpp
* @author AMSL
*/
#include "state_lattice_planner/state_lattice_planner.h"

StateLatticePlanner::StateLatticePlanner(void)
{
    HZ = 10.0;
    MAX_ITERATION = 100;
    OPTIMIZATION_TOLERANCE = 0.1;
    MAX_ACCELERATION = 1.0;
    TARGET_VELOCITY = 0.8;
    MAX_YAWRATE = 1.0;
    MAX_D_YAWRATE = 3.14;
    MAX_WHEEL_ANGULAR_VELOCITY = 11.6;
    WHEEL_RADIUS = 0.125;
    TREAD = 0.5;
    VERBOSE = false;
    ENABLE_CONTROL_SPACE_SAMPLING = false;
}

StateLatticePlanner::SamplingParams::SamplingParams(void)
{
    n_p = 0.0;
    n_h = 0.0;
    length = 0.0;
    max_alpha = 0.0;
    min_alpha = 0.0;
    span_alpha = max_alpha - min_alpha;
    max_psi = 0.0;
    min_psi = 0.0;
    span_psi = 0.0;
}

StateLatticePlanner::SamplingParams::SamplingParams(
  const int _n_p, const int _n_h, const double _length,
  const double _max_alpha, const double _max_psi)
{
    n_p = _n_p;
    n_h = _n_h;
    length = _length;
    max_alpha = _max_alpha;
    min_alpha = -_max_alpha;
    span_alpha = max_alpha - min_alpha;
    max_psi = _max_psi;
    min_psi = -_max_psi;
    span_psi = max_psi - min_psi;
}

StateLatticePlanner::SamplingParams::SamplingParams(
  const int _n_p, const int _n_h, const double _max_alpha, const double _max_psi)
{
    n_p = _n_p;
    n_h = _n_h;
    length = 0.0;
    max_alpha = _max_alpha;
    min_alpha = -_max_alpha;
    span_alpha = max_alpha - min_alpha;
    max_psi = _max_psi;
    min_psi = -_max_psi;
    span_psi = max_psi - min_psi;
}

// -------------------- Critic --------------------

bool StateLatticePlanner::Critic::pickup_trajectory(
  const std::vector<MotionModelDiffDrive::Trajectory>& trajectories,
  const Eigen::Vector3d& goal, MotionModelDiffDrive::Trajectory& best_trajectory)
{
  double cost_min = std::numeric_limits<double>::max();
  int cost_min_index = -1;
  for (size_t i = 0; i < trajectories.size(); i ++) {
    double cost = 0;
    cost += evaluate_dist_err(trajectories.at(i), goal);
    cost += evaluate_yaw_err(trajectories.at(i), goal);
    // cost += evaluate_angular_err(trajectories.at(i));
    if (cost < cost_min) {
      cost_min = cost;
      cost_min_index = i;
    }
  }
  if (cost_min_index < 0) {
    std::cout << "can't find best trajectory." << std::endl;
    return false;
  }
  best_trajectory = trajectories.at(cost_min_index);
  return true;
}

double StateLatticePlanner::Critic::evaluate_dist_err(
  const MotionModelDiffDrive::Trajectory& trajectory, const Eigen::Vector3d& goal)
{
  auto dist = (trajectory.trajectory.back()-goal).segment(0,2).norm();
  return dist / dist_err_* dist_scale_;
}

double StateLatticePlanner::Critic::evaluate_yaw_err(
  const MotionModelDiffDrive::Trajectory& trajectory, const Eigen::Vector3d& goal)
{
  // auto yaw = fabs((trajectory.trajectory.back()-goal)(2));
  // return yaw / yaw_err_* yaw_scale_;
  return fabs(trajectory.trajectory.front()(2)) / yaw_err_* yaw_scale_;
}

double StateLatticePlanner::Critic::evaluate_angular_err(
  const MotionModelDiffDrive::Trajectory& trajectory)
{
  int size = trajectory.angular_velocities.size();
  int limit = size >= 100 ? 100 : size;
  int step = std::ceil(trajectory.angular_velocities.size() / limit);
  int num = std::floor(trajectory.angular_velocities.size() / step);
  double angular_max = angular_err_ * num;
  double angular_sum = 0;
  for (int i = 0; i < num; i += step) {
    angular_sum += fabs(trajectory.angular_velocities.at(i));
  }
  double angular_cost = angular_sum / angular_max * angular_scale_;
  // std::cout << "angular cost:" << angular_cost << std::endl;
  return angular_cost;
}

// -------------------- StateLatticePlanner --------------------

void StateLatticePlanner::set_optimization_params(int max_iteration_, double tolerance_)
{
    MAX_ITERATION = std::max(max_iteration_, 0);
    OPTIMIZATION_TOLERANCE = std::max(tolerance_, 0.0);
}

void StateLatticePlanner::set_target_velocity(double v)
{
    TARGET_VELOCITY = v;
}

void StateLatticePlanner::set_motion_params(double max_acceleration_, double max_yawrate_, double max_d_yawrate_)
{
    MAX_ACCELERATION = max_acceleration_;
    MAX_YAWRATE = max_yawrate_;
    MAX_D_YAWRATE = max_d_yawrate_;
}

void StateLatticePlanner::set_vehicle_params(double wheel_radius_, double tread_)
{
    WHEEL_RADIUS = wheel_radius_;
    TREAD = tread_;
}

void StateLatticePlanner::set_sampling_params(const SamplingParams& sampling_params_)
{
    sampling_params = sampling_params_;
}

void StateLatticePlanner::load_lookup_table(const std::string& LOOKUP_TABLE_FILE_NAME, std::vector<Eigen::Vector3d>& xyyaw_table)
{
    LookupTableUtils::load_lookup_table(LOOKUP_TABLE_FILE_NAME, lookup_table, xyyaw_table);
}
// 采样终点角度和航向角
void StateLatticePlanner::sample_states(const std::vector<double>& sample_angles,
  const SamplingParams& params, std::vector<Eigen::Vector3d>& states)
{
    /*
     * sample_angles: [0, 1]
     */
    std::vector<Eigen::Vector3d> _states;
    for(auto angle_ratio : sample_angles){
        double angle = params.min_alpha + params.span_alpha * angle_ratio;
        double x = params.length * cos(angle);
        double y = params.length * sin(angle);
        double base_angle = angle;
        if(params.min_alpha > params.max_alpha){
            base_angle -= M_PI;
            base_angle = atan2(sin(base_angle), cos(base_angle));
        }
        for(int i=0;i<params.n_h;i++){
            if(params.n_h > 0){
                double yaw = 0;
                if(params.n_h != 1){
                    double ratio = double(i) / (params.n_h - 1);
                    yaw = params.min_psi + (params.span_psi) * ratio + base_angle;
                }else{
                    yaw = (params.span_psi) * 0.5 + base_angle;
                }
                yaw = atan2(sin(yaw), cos(yaw));
                Eigen::Vector3d state(x, y, yaw);
                _states.push_back(state);
            }else{
                std::cout << "sampling param error" << std::endl;
                exit(-1);
            }
        }
    }
    states = _states;
}

void StateLatticePlanner::generate_biased_polar_states(
  const int n_s, const Eigen::Vector3d& goal,
  double target_velocity, std::vector<Eigen::Vector3d>& states)
{
    /*
     * n_s: param for biased polar sampling
     */
    // params.length is ignored in this function (distance for goal is used)
    SamplingParams _params = sampling_params;
    double alpha_coeff = _params.span_alpha / double(n_s - 1);
    // 当 goal 在后方，采样范围改变，没意义
    if(target_velocity < 0.0){ 
        _params.min_alpha = M_PI - sampling_params.max_alpha;
        _params.max_alpha = -sampling_params.min_alpha;
    }
    std::cout << "biased polar sampling" << std::endl;
    std::vector<double> cnav; // 保存采样后的坐标与 goal 的欧氏距离
    double goal_distance = goal.segment(0, 2).norm();
    for(int i=0;i<n_s;i++){
        double angle = _params.min_alpha + double(i) * alpha_coeff;
        angle = atan2(sin(angle), cos(angle));
        _params.length = goal_distance;
        Eigen::Vector2d terminal;
        terminal <<  _params.length * cos(angle),
                     _params.length * sin(angle);
        double diff = (goal.segment(0, 2) - terminal).norm();
        cnav.push_back(diff);
    }
    double cnav_sum = 0;
    double cnav_max = 0;
    for(const auto& alpha_s : cnav){
        cnav_sum += alpha_s;
        if(cnav_max < alpha_s){
            cnav_max = alpha_s;
        }
    }
    // normalize
    //std::cout << "normalize" << std::endl;
    // TODO:LMR 归一化，采样点跟 goal 之间的欧氏误差归一化，离 goal 越近，值越高
    for(auto& alpha_s : cnav){
        alpha_s = (cnav_max - alpha_s) / (cnav_max * n_s - cnav_sum);
        //std::cout << alpha_s << std::endl;
    }
    // cumsum
    //std::cout << "cumsum" << std::endl;
    std::vector<double> cnav2;
    double cumsum = 0;
    std::vector<double> cumsum_list;
    for(auto cnav_it=cnav.begin();cnav_it!=cnav.end()-1;++cnav_it){
        cumsum += *cnav_it;
        cnav2.push_back(cumsum);
        //std::cout << cumsum << std::endl;
    }
    // cnav2 曲线是 S 型，越接近 goal 的位置越陡

    // sampling 根据 cnav2 曲线，离 goal 越近采样点多，越远采样点越少
    std::vector<double> biased_angles;
    for(int i=0;i<_params.n_p;i++){
        double sample_angle = double(i) / (_params.n_p - 1);
        //std::cout << "sample angle: " << sample_angle << std::endl;
        int count = 0;
        for(;count<n_s-1;count++){
            // if this loop finish without break, count is n_s - 1
            if(cnav2[count] >= sample_angle){
                break;
            }
        }
        //std::cout << "count: " << count << std::endl;
        biased_angles.push_back(count / double(n_s - 1));
    }
    //std::cout << "biased angles" << std::endl;
    /*
    for(auto angle : biased_angles){
        std::cout << angle << std::endl;
    }
    */
    sample_states(biased_angles, _params, states);
}

/**
 * @brief 
 * @param  boundary_states  终点各个采样状态
 * @param  velocity         当前线速度
 * @param  angular_velocity 当前角速度
 * @param  target_velocity  目标速度
 * @param  trajectories     返回轨迹
 * @return true 
 * @return false 
 */
bool StateLatticePlanner::generate_trajectories(
  const std::vector<Eigen::Vector3d>& boundary_states, const double velocity,
  const double angular_velocity, const double target_velocity,
  std::vector<MotionModelDiffDrive::Trajectory>& trajectories)
{
    ROS_DEBUG("[SLP] sample size:%lu, v:%.3f, a:%.3f, tv:%.3f",
      boundary_states.size(), velocity, angular_velocity, target_velocity);
    int count = 0;
    int trajectory_num = boundary_states.size();
    std::vector<MotionModelDiffDrive::Trajectory> trajectories_(trajectory_num);
    // #pragma omp parallel for
    // for循环里的内容必须满足可以并行执行，即每次循环互不相干，后一次循环不依赖于前面的循环。
    #pragma omp parallel for
    for(int i=0;i<trajectory_num;i++){
      Eigen::Vector3d boundary_state = boundary_states[i];
      TrajectoryGeneratorDiffDrive tg;
      tg.set_verbose(VERBOSE);
      tg.set_motion_param(MAX_YAWRATE, MAX_D_YAWRATE, MAX_ACCELERATION, MAX_WHEEL_ANGULAR_VELOCITY, WHEEL_RADIUS, TREAD);
      MotionModelDiffDrive::ControlParams output;
      double k0 = angular_velocity;

      MotionModelDiffDrive::ControlParams param;
      // 根据采样点在状态表中找到能到达 goal 附近的机器状态信息
      LookupTableUtils::get_optimized_param_from_lookup_table(lookup_table, boundary_state, velocity, k0, param);
      ROS_DEBUG("[SLP] lookup msg, v:%.3f, k0:%.3f, km:%.3f, kf:%.3f, sf:%.3f",
        velocity, k0, param.omega.km, param.omega.kf, param.omega.sf);

      // 初始化机器当前状态
      MotionModelDiffDrive::ControlParams init(
        MotionModelDiffDrive::VelocityParams(velocity, MAX_ACCELERATION, target_velocity, target_velocity, MAX_ACCELERATION),
        MotionModelDiffDrive::AngularVelocityParams(k0, param.omega.km, param.omega.kf, param.omega.sf));

      MotionModelDiffDrive::Trajectory trajectory;
      // 根据上面配置的初始状态计算到目标的轨迹
      double cost = tg.generate_optimized_trajectory(
        boundary_state, init, 1.0 / HZ, OPTIMIZATION_TOLERANCE, MAX_ITERATION, output, trajectory);
      if(cost > 0){
          trajectories_[i] = trajectory;
          count++;
      }
    }
    for(auto it=trajectories_.begin();it!=trajectories_.end();){
        if(it->trajectory.size() == 0){
            it = trajectories_.erase(it);
        }else{
            ++it;
        }
    }
    // 补充一条控制采样的轨迹
    if(ENABLE_CONTROL_SPACE_SAMPLING){
      int min_trajectory_size = trajectories_[0].trajectory.size();
      // min_trajectory_size 最小轨迹点数
      for(auto& traj : trajectories_){
        min_trajectory_size = std::min(min_trajectory_size, (int)traj.trajectory.size());
      }
      for(int i=0;i<3;i++){ // 角速度
        for(int j=0;j<3;j++){ // 速度
          MotionModelDiffDrive::Trajectory traj;
          MotionModelDiffDrive mmdd;
          mmdd.set_param(MAX_YAWRATE, MAX_D_YAWRATE, MAX_ACCELERATION, MAX_WHEEL_ANGULAR_VELOCITY, WHEEL_RADIUS, TREAD);
          MotionModelDiffDrive::State state(0, 0, 0, velocity, angular_velocity);
          traj.trajectory.emplace_back(Eigen::Vector3d(state.x, state.y, state.yaw));
          traj.velocities.emplace_back(state.v);
          traj.angular_velocities.emplace_back(state.omega);
          double velocity_ = velocity + (j - 1) * MAX_ACCELERATION / HZ;
          velocity_ = std::min(TARGET_VELOCITY, std::max(-TARGET_VELOCITY, velocity_));
          double omega = angular_velocity + (i - 1) * MAX_D_YAWRATE / HZ;
          omega = std::min(MAX_YAWRATE, std::max(omega, -MAX_YAWRATE));
          for(int j=0;j<min_trajectory_size;j++){
            mmdd.update(state, velocity_, omega, 1.0 / HZ, state);
            traj.trajectory.emplace_back(Eigen::Vector3d(state.x, state.y, state.yaw));
            traj.velocities.emplace_back(state.v);
            traj.angular_velocities.emplace_back(state.omega);
          }
          trajectories_.emplace_back(traj);
        }
      }
    }
    std::copy(trajectories_.begin(), trajectories_.end(), std::back_inserter(trajectories));
    if(trajectories.size() == 0){
      ROS_WARN("[SLP] no trajectory was generated.");
      return false;
    }
    return true;
}

bool StateLatticePlanner::pickup_trajectory(const std::vector<MotionModelDiffDrive::Trajectory>& candidate_trajectories, const Eigen::Vector3d& goal, MotionModelDiffDrive::Trajectory& output)
{
    /*
     * outputs a trajectory that is nearest to the goal
     */
    std::vector<MotionModelDiffDrive::Trajectory> trajectories;
    for(const auto& candidate_trajectory : candidate_trajectories){
        MotionModelDiffDrive::Trajectory traj;
        traj = candidate_trajectory;
        trajectories.push_back(traj);
    }
    // ascending sort by cost
    auto compare_trajectories = [=](const MotionModelDiffDrive::Trajectory& lhs, const MotionModelDiffDrive::Trajectory& rhs)
    {
        double l_distance = (lhs.trajectory.back().segment(0, 2) - goal.segment(0, 2)).norm();
        double r_distance = (rhs.trajectory.back().segment(0, 2) - goal.segment(0, 2)).norm();
        return l_distance < r_distance;
    };
    std::sort(trajectories.begin(), trajectories.end(), compare_trajectories); // 轨迹按终点与 goal 欧氏距离差排序，从小排到大

    // 找轨迹终点航向角误差最小
    double min_diff_yaw = 100;
    const int N = ((int)trajectories.size() < sampling_params.n_h) ? trajectories.size() : sampling_params.n_h;
    for(int i=0;i<N;i++){
        double diff_yaw = trajectories[i].trajectory.back()(2) - goal(2);
        diff_yaw = fabs(atan2(sin(diff_yaw), cos(diff_yaw)));
        if(min_diff_yaw > diff_yaw){
            min_diff_yaw = diff_yaw;
            output = trajectories[i];
        }
    }
    if(!(output.trajectory.size() > 0)){
        return false;
    }
    return true;
}

void StateLatticePlanner::generate_head_trajectory(
  const std::vector<Eigen::Vector3d>& trajectory,
  const double head,
  std::vector<Eigen::Vector3d>& head_trajectory)
{
  head_trajectory.reserve(trajectory.size());
  for (auto p : trajectory) {
    auto h = p;
    h(0) = p(0) + head * cos(p(2));
    h(1) = p(1) + head * sin(p(2));
    head_trajectory.emplace_back(h);
  }
}

void StateLatticePlanner::generate_bresemhams_line(const std::vector<Eigen::Vector3d>& trajectory, const double& resolution, std::vector<Eigen::Vector3d>& output)
{
    int size = trajectory.size();
    output.clear();
    // maybe too much
    output.reserve(size * 2);
    for(int i=0;i<size-2;i++){
        double x0 = trajectory[i](0);
        double y0 = trajectory[i](1);
        double x1 = trajectory[i+1](0);
        double y1 = trajectory[i+1](1);

        bool steep = (fabs(y1 - y0) > fabs(x1 - x0));

        if(steep){
            std::swap(x0, y0);
            std::swap(x1, y1);
        }
        if(x0 > x1){
            std::swap(x0, x1);
            std::swap(y0, y1);
        }

        double delta_x = x1 - x0;
        double delta_y = fabs(y1 - y0);
        double error = 0;
        double delta_error = delta_y / delta_x;
        double y_step;
        double yt = y0;

        y_step = (y0 < y1) ? resolution : -resolution;

        for(double xt=x0;xt<x1;xt+=resolution){
            if(steep){
                Eigen::Vector3d p(yt, xt, 0);
                output.push_back(p);
            }else{
                Eigen::Vector3d p(xt, yt, 0);
                output.push_back(p);
            }
            error += delta_error;
            if(error >= 0.5){
                yt += y_step;
                error -= 1;
            }
        }
    }
}

/**
 * 当 goal 在机器后面，目标速度取反
 *            x
 *            |
 *            |
 *   y------- . -------
 *          /   \
 *         / 取反 \
 */
double StateLatticePlanner::get_target_velocity(const Eigen::Vector3d& goal)
{
    double direction = atan2(goal(1), goal(0));
    if(fabs(direction) < M_PI * 0.75){
        return TARGET_VELOCITY;
    }else{
        return -TARGET_VELOCITY;
    }
}

bool StateLatticePlanner::check_collision(
  const state_lattice_planner::ObstacleMap<int>& map, const std::vector<Eigen::Vector3d>& trajectory,
  const int collision_cost) 
{
    /*
     * if given trajectory is considered to collide with an obstacle, return true
     */
    double resolution = map.get_resolution();
    std::vector<Eigen::Vector3d> bresenhams_line;
    generate_bresemhams_line(trajectory, resolution, bresenhams_line);
    int size = bresenhams_line.size();
    double map_x, map_y;
    for(int i=0;i<size;i++){
        if (!map.in_map(bresenhams_line[i](0), bresenhams_line[i](1), map_x, map_y)) {
          std::cout << "out of map !" << std::endl;
          return true;
        }
        int index = map.get_index_from_xy(map_x, map_y);
        if(map.data[index] > collision_cost ||
           map.data[index] == -1){
          // std::cout << "cost:" << map.data[index] << std::endl;
          return true;
        }
        // int index = map.get_index_from_xy(bresenhams_line[i](0), bresenhams_line[i](1));
        // if(map.data[index] > collision_cost){
        //     return true;
        // }
    }
    return false;
}

// 重复没用
bool StateLatticePlanner::check_collision(
  const state_lattice_planner::ObstacleMap<int>& map, const std::vector<Eigen::Vector3d>& trajectory, double range)
{
    /*
     * if given trajectory is considered to collide with an obstacle, return true
     */
    double resolution = map.get_resolution();
    std::vector<Eigen::Vector3d> bresenhams_line;
    generate_bresemhams_line(trajectory, resolution, bresenhams_line);
    int size = bresenhams_line.size();
    for(int i=0;i<size;i++){
        int index = map.get_index_from_xy(bresenhams_line[i](0), bresenhams_line[i](1));
        if(map.data[index] != 0){
            return true;
        }
    }
    return false;
}
