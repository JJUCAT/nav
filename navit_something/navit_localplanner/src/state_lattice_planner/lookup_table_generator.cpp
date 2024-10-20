#include "state_lattice_planner/lookup_table_generator.h"
#include <angles/angles.h>

LookupTableGenerator::LookupTableGenerator(void)
:local_nh("~")
{
    local_nh.param("MIN_X", MIN_X, {1.0});
    local_nh.param("MAX_X", MAX_X, {5.0});
    local_nh.param("DELTA_X", DELTA_X, {1.0});
    local_nh.param("MAX_Y", MAX_Y, {2.0});
    local_nh.param("DELTA_Y", DELTA_Y, {1.0});
    local_nh.param("MAX_YAW", MAX_YAW, {M_PI / 3.0});
    local_nh.param("DELTA_YAW", DELTA_YAW, {M_PI / 3.0});
    local_nh.param("LOOKUP_TABLE_FILE_NAME", LOOKUP_TABLE_FILE_NAME, {std::string(std::getenv("HOME")) + "/lookup_table.csv"});
    local_nh.param("TARGET_VELOCITY", TARGET_VELOCITY, {0.8});
    local_nh.param("MIN_V", MIN_V, {0.1});
    local_nh.param("MAX_V", MAX_V, {0.8});
    local_nh.param("DELTA_V", DELTA_V, {0.1});
    local_nh.param("MAX_KAPPA", MAX_KAPPA, {1.0});
    local_nh.param("DELTA_KAPPA", DELTA_KAPPA, {0.2});
    local_nh.param("MAX_ACCELERATION", MAX_ACCELERATION, {1.0});
    local_nh.param("MAX_YAWRATE", MAX_YAWRATE, {0.8});
    local_nh.param("MAX_D_YAWRATE", MAX_D_YAWRATE, {2.0});
    local_nh.param("MAX_WHEEL_ANGULAR_VELOCITY", MAX_WHEEL_ANGULAR_VELOCITY, {11.6});
    local_nh.param("WHEEL_RADIUS", WHEEL_RADIUS, {0.125});
    local_nh.param("TREAD", TREAD, {0.5});

    std::cout << "MIN_X: " << MIN_X << std::endl;
    std::cout << "MAX_X: " << MAX_X << std::endl;
    std::cout << "DELTA_X: " << DELTA_X << std::endl;
    std::cout << "MAX_Y: " << MAX_Y << std::endl;
    std::cout << "DELTA_Y: " << DELTA_Y << std::endl;
    std::cout << "MAX_YAW: " << MAX_YAW << std::endl;
    std::cout << "DELTA_YAW: " << DELTA_YAW << std::endl;
    std::cout << "LOOKUP_TABLE_FILE_NAME: " << LOOKUP_TABLE_FILE_NAME << std::endl;
    std::cout << "TARGET_VELOCITY: " << TARGET_VELOCITY << std::endl;
    std::cout << "MIN_V: " << MIN_V << std::endl;
    std::cout << "MAX_V: " << MAX_V << std::endl;
    std::cout << "DELTA_V: " << DELTA_V << std::endl;
    std::cout << "MAX_KAPPA: " << MAX_KAPPA << std::endl;
    std::cout << "DELTA_KAPPA: " << DELTA_KAPPA << std::endl;
    std::cout << "MAX_ACCELERATION: " << MAX_ACCELERATION << std::endl;
    std::cout << "MAX_YAWRATE: " << MAX_YAWRATE << std::endl;
    std::cout << "MAX_D_YAWRATE: " << MAX_D_YAWRATE << std::endl;
    std::cout << "MAX_WHEEL_ANGULAR_VELOCITY: " << MAX_WHEEL_ANGULAR_VELOCITY << std::endl;
    std::cout << "WHEEL_RADIUS: " << WHEEL_RADIUS << std::endl;
    std::cout << "TREAD: " << TREAD << std::endl;

    std::vector<Eigen::Vector3d> tmp;
    use_existing_lookup_table = LookupTableUtils::load_lookup_table(LOOKUP_TABLE_FILE_NAME, lookup_table, tmp);
}

std::string LookupTableGenerator::process(void)
{
    // 计算终点在 x y yaw 采样状态
    const int N_X = (MAX_X - MIN_X) / DELTA_X + 1;
    if(N_X < 1){
        std::cout << "param error(x)" << std::endl;
        exit(-1);
    }
    const int N_Y = 2 * MAX_Y / DELTA_Y + 1;
    if(N_Y < 1){
        std::cout << "param error(y)" << std::endl;
        exit(-1);
    }
    const int N_YAW = 2 * MAX_YAW / DELTA_YAW + 1;
    if(N_YAW < 1){
        std::cout << "param error(yaw)" << std::endl;
        exit(-1);
    }
    const int N = N_X * N_Y * N_YAW;
    std::cout << "N_X: " << N_X << std::endl;
    std::cout << "N_Y: " << N_Y << std::endl;
    std::cout << "N_YAW: " << N_YAW << std::endl;
    std::vector<Eigen::Vector3d> states;
    // for(int i=0;i<N_YAW;i++){ // 航向角采样
    //     for(int j=0;j<N_Y;j++){ // y 轴采样
    //         for(int k=0;k<N_X;k++){ // x 轴采样
    //             Eigen::Vector3d state;
    //             state << MIN_X + DELTA_X * k,
    //                      -MAX_Y + DELTA_Y * j,
    //                      -MAX_YAW + DELTA_YAW * i;
    //             if(state(0) == 0.0){
    //                 continue;
    //             }
    //             states.push_back(state);
    //         }
    //     }
    // }
    double tx, ty, tyaw;
    double _2_M_PI = 2*M_PI;
    for(int i=0;i<N_YAW;i++){ // 航向角采样
        for(int j=0;j<N_Y;j++){ // y 轴采样
            for(int k=0;k<N_X;k++){ // x 轴采样
                tx = MIN_X + DELTA_X * k;
                ty = -MAX_Y + DELTA_Y * j;
                tyaw = atan2(ty, tx) - MAX_YAW + DELTA_YAW * i;
                if (tyaw < -M_PI) tyaw += _2_M_PI;
                else if (tyaw > M_PI) tyaw -= _2_M_PI;
                Eigen::Vector3d state;
                state << tx, ty, tyaw;
                if(fabs(state(0)) < 0.5 && fabs(state(1)) < 0.5){
                    continue;
                }
                states.push_back(state);
            }
        }
    }
    std::cout << "states num: " << states.size() << " * " <<
                 int((MAX_V - MIN_V) / DELTA_V + 1) << " * " <<
                 int(2*MAX_KAPPA / DELTA_KAPPA + 1) << std::endl;
    
    // 初始化状态表，状态包括：
    // @x：x 轴偏移，@y：y 轴编译，@yaw：航向角偏移
    // @km：，@kf：，@sf：xy 偏移距离
    std::vector<std::vector<double> > lookup_table_data_list;
    lookup_table_data_list.resize(N);
    for(auto& lookup_table_data : lookup_table_data_list){
        // x, y, yaw, km, kf, sf
        lookup_table_data.resize(6);
    }

    int success_count = 0, count = 0;
    std::string output_data = "v0, k0, x, y, yaw, km, kf, sf\n";
    for(double v0=MIN_V;v0<=MAX_V;v0+=DELTA_V){ // 速度采样
        for(double k0=-MAX_KAPPA;k0<=MAX_KAPPA;k0+=DELTA_KAPPA){ // 曲率采样
            for(auto state : states){
                // std::cout << "v0: " << v0 << "[m/s]" << std::endl;
                // std::cout << "k0: " << k0 << "[rad/m]" << std::endl;
                // std::cout << "state:" << std::endl;
                // std::cout << state << std::endl;
                double distance = state.segment(0, 2).norm(); // 从 indix=0 开始取 2 个元素求模
                // std::cout << "distance: " << distance << std::endl;
                // double target_velocity = get_target_velocity(state);
                // double v = update_v(state, TARGET_VELOCITY);
                double v = update_v(v0, 0.1); // 0.2
                MotionModelDiffDrive::ControlParams optimized_param;
                if(use_existing_lookup_table){
                    LookupTableUtils::get_optimized_param_from_lookup_table(lookup_table, state, v0, k0, optimized_param);
                }else{
                    optimized_param.omega.sf = distance;
                }
                // MotionModelDiffDrive::VelocityParams init_v(v0, MAX_ACCELERATION, target_velocity, target_velocity, MAX_ACCELERATION);
                MotionModelDiffDrive::VelocityParams init_v(v0, MAX_ACCELERATION, v, v, MAX_ACCELERATION);
                MotionModelDiffDrive::ControlParams init(init_v, MotionModelDiffDrive::AngularVelocityParams(k0, optimized_param.omega.km, optimized_param.omega.kf, optimized_param.omega.sf));
                MotionModelDiffDrive::ControlParams output;
                MotionModelDiffDrive::Trajectory trajectory;
                TrajectoryGeneratorDiffDrive tg;
                tg.set_motion_param(MAX_YAWRATE, MAX_D_YAWRATE, MAX_ACCELERATION, MAX_WHEEL_ANGULAR_VELOCITY, WHEEL_RADIUS, TREAD);
                double cost = tg.generate_optimized_trajectory(state, init, 1e-1, 1e-1, 1000, output, trajectory);
                if(cost > 0){
                    // std::cout << "\033[032msuccessfully optimized\033[0m" << std::endl;
                    // std::cout << v0 << ", " << k0 << ", " << trajectory.trajectory.back()(0) << ", " << trajectory.trajectory.back()(1) << ", " << trajectory.trajectory.back()(2) << std::endl;
                    // std::cout << output.omega.km << "," << output.omega.kf << "," << output.omega.sf << std::endl;
                    std::stringstream data;
                    data << v0 << "," << k0 << "," << trajectory.trajectory.back()(0) << "," << trajectory.trajectory.back()(1) << "," << trajectory.trajectory.back()(2) << "," <<
                      std::setprecision(6) << output.omega.km << "," << std::setprecision(6)  << output.omega.kf << ","<< std::setprecision(6)  << output.omega.sf << "\n";
                    output_data += data.str();
                    success_count++;
                }else{
                    // std::cout << "\033[031mfailed to optimize trajectory\033[0m" << std::endl;
                }
                count ++;
            }
        }
    }
    std::cout << "success rate: " << success_count << " / " << count << " = " << (double)success_count / count << std::endl;
    return output_data;
}

void LookupTableGenerator::save(std::string& data)
{
    std::ofstream ofs(LOOKUP_TABLE_FILE_NAME);

    if(ofs){
        ofs << data;
        ofs.close();
        std::cout << "lookup table saved as " << LOOKUP_TABLE_FILE_NAME << std::endl;
        exit(0);
    }else{
        std::cout << "cannot open file:" << LOOKUP_TABLE_FILE_NAME << std::endl;
        exit(-1);
    }
}

double LookupTableGenerator::get_target_velocity(const Eigen::Vector3d& goal)
{
    double direction = atan2(goal(1), goal(0));
    if(fabs(direction) < M_PI * 0.75){
        return TARGET_VELOCITY;
    }else{
        return -TARGET_VELOCITY;
    }
}

double LookupTableGenerator::update_v(const Eigen::Vector3d state, const double target_v)
{
  double orient = atan2(state(1), state(0));
  double orient_diff = angles::shortest_angular_distance(state(2), orient);
  double orient_scale = 1.0 - fabs(orient_diff) / M_PI;
  orient_scale = std::max(0.0, orient_scale);

  double dist = state.segment(0,2).norm();
  double dist_limit = 4;
  double dist_scale = dist / dist_limit;

  double orient_weight = 0.5, dist_weight = 0.5;
  double v = v * orient_weight * orient_scale + v * dist_weight * dist_scale;
  v = std::max(0.1, std::min(target_v, v));
  return v;
}

double LookupTableGenerator::update_v(const double v, const double step)
{
  return v + step;
}