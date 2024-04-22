/**
 * @file trajectory_generator_diff_drive.h
 * @author AMSL
 */
#ifndef __TRAJECTORY_GENERATOR_DIFF_DRIVE_H
#define __TRAJECTORY_GENERATOR_DIFF_DRIVE_H

#include <state_lattice_planner/trajectory_generator/motion_model_diff_drive.h>
#include <queue>
#include <algorithm>

/**
 * @brief Class for generating optimized trajectory
 */
class TrajectoryGeneratorDiffDrive
{
public:
    /**
     * @brief Contructor
     */
    TrajectoryGeneratorDiffDrive(void);

    /**
     * @brief Set optimization parameters used in optimization
     * @param[in] dkm
     * @param[in] dkf
     * @param[in] dsf
     */
    void set_optimization_param(const double, const double, const double);
    /**
     * @brief Set motion parameters
     * @param[in] max_yawrate [rad/s]
     * @param[in] max_d_yawrate [rad/ss]
     * @param[in] max_acceleration [m/ss]
     * @param[in] max_wheel_angular_velocity [rad/s]
     * @param[in] wheel_radius [m]
     * @param[in] tread [m]
     */
    void set_motion_param(const double, const double, const double, const double, const double, const double);
    /**
     * @brief Set verbose output
     * @param[in] verbose_ If true, verbose output is enabled
     */
    void set_verbose(bool);
    /**
     * @brief Generate Optimized trajectory
     * @param[in] goal (x, y, yaw)
     * @param[in] init_control_param Initial value of ControlParams
     * @param[in] dt [s]
     * @param[in] tolerance Optimization cost tolerance
     * @param[in] max_iteration Max number of optimization loop
     * @param[out] output Optimized control parameters
     * @param[out] trajectory Optimized trajectory
     */
    double generate_optimized_trajectory(const Eigen::Vector3d&, const MotionModelDiffDrive::ControlParams&, const double, const double, const int, MotionModelDiffDrive::ControlParams&, MotionModelDiffDrive::Trajectory&);
    /**
     * @brief Calculate jacobian
     * @param[in] dt [s]
     * @param[in] control
     * @param[in] h
     * @param[out] j Jacobian
     */
    void get_jacobian(const double, const MotionModelDiffDrive::ControlParams&, const Eigen::Vector3d&, Eigen::Matrix3d&);
    /**
     * @brief Search coefficient to reduce cost
     * @param[in] dt [s]
     * @param[in] tolerance
     * @param[in] goal (x, y, yaw)
     * @param[in,out] cost Optimization cost
     * @param[out] output
     * @param[out] trajectory
     * @param[out] dp
     */
    void calculate_scale_factor(double, double, const Eigen::Vector3d&, Eigen::Vector3d&, MotionModelDiffDrive::ControlParams&, MotionModelDiffDrive::Trajectory&, Eigen::Vector3d&, double& s);

    void init_hv();
    void update_h(const Eigen::Vector3d& goal);
    std::queue<Eigen::Vector3d> unorder_hv();
    bool update_h(std::queue<Eigen::Vector3d>& hq);

    /**
     * @brief 设置优化参数
     * @param  timeout  优化超时时间
     * @param  min_dist_err  最小距离差
     * @param  min_yaw_err  最小角度差
     */
    void set_optimize_config(const double timeout, const double min_dist_err, const double min_yaw_err) {
      timeout_ = timeout;
      min_dist_err_ = min_dist_err;
      min_yaw_err_ = min_yaw_err;
    }

private:
    double MAX_YAWRATE;
    MotionModelDiffDrive model;

    Eigen::Vector3d h;
    std::vector<Eigen::Vector3d> hv_;
    bool verbose;
    double timeout_ = 0.01;
    double min_dist_err_ = 0.1;
    double min_yaw_err_ = 0.17;
};

#endif //__TRAJECTORY_GENERATOR_DIFF_DRIVE_H
