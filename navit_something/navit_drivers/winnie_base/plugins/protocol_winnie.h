#ifndef WINNIE_SDK_PROTOCOL_CONTENT_H
#define WINNIE_SDK_PROTOCOL_CONTENT_H

#include <cstdint>

namespace winnie_base {

#pragma pack(push, 1)

enum DEVICE_ADDRESS {
    JETSON_ADDRESS = 0x00,
    WINNIE_ADDRESS = 0x01,
};

enum CMD_SET {
    // 下发
    CMD_SET_HEART_BEAT = 0x00,  // +
    CMD_SET_PARAM_SET  = 0x01,
    CMD_SET_CMD_VEL    = 0x02,
    CMD_SET_IO_CTRL    = 0x03,  // +
    CMD_SET_MOTOR_CTRL = 0x04,  // +

    // 上传_复合信息
    CMD_SET_WINNIE_INFO_1 = 0x10,
    CMD_SET_WINNIE_INFO_2 = 0x11,
    CMD_SET_WINNIE_INFO_3 = 0x12,

    // 定位相关信息
    CMD_SET_RTK_INFO = 0x20,  // +

    // 规划控制相关
    CMD_SET_IMU_INFO   = 0x30,  // +
    CMD_SET_ODOM_INFO  = 0x31,
    CMD_SET_MOTOR_INFO = 0x32,  // +

    // IO相关
    CMD_SET_IO_INFO = 0x40,  // +

    // 其他传感器
    CMD_SET_BATTERY_INFO = 0x50,  // +
};

// 上传 -------------------------
typedef struct {
} cmd_winnie_info_1;

typedef struct {
    // Ros Custom Msg Header
    // NovatelMessageHeader novatel_msg_header
    // library/GnssParserLib/parsers/header.cpp
    uint8_t port;  // string port; PORT_IDENTIFIERS
    uint16_t sequence_num;
    uint8_t percent_idle_time;
    uint8_t gps_time_status;  // string gps_time_status
    uint16_t gps_week_num;
    double gps_seconds;
    uint32_t receiver_status;
    uint16_t receiver_software_version;

    double gps_seconds1;  // bestpos
    double gps_seconds2;  // bestxyz
    double gps_seconds3;  // heading

    // Position
    double latitude;
    double longitude;
    double altitude;
    float undulation;
    float lat_sigma;
    float lon_sigma;
    float height_sigma;
    double position_covariance[3];  // double[9] position_covariance; 只取0 4 8

    // Velocity
    float x_vel;
    float y_vel;
    float z_vel;
    float v_latency;
    float x_vel_sigma;
    float y_vel_sigma;
    float z_vel_sigma;
    double velocity_covariance[3];  // double[9] velocity_covariance; 只取0 4 8

    // Heading
    float heading;
    float pitch;
    float heading_sigma;
    float pitch_sigma;
    double heading_covariance;
    double pitch_covariance;

    // Rover Status
    // string solution_status   # bestpos
    // string position_type
    // string xyz_sol_status    # bestxyz
    // string xyz_type
    // string heading_type      # heading
    // uint8 num_satellites1
    // uint8 num_satellites2
    // string board_type
    uint16_t solution_status;  // bestpos(bestpos->solution_status;)
                               // SOLUTION_STATUSES
    uint16_t position_type;    // bestpos(bestpos->position_type;)
                               // POSITION_TYPES
    uint16_t xyz_sol_status;   // bestxyz(bestxyz->velocity_solution_status;)
                               // SOLUTION_STATUSES
    uint16_t xyz_type;         // bestxyz(bestxyz->velocity_type;)
                               // POSITION_TYPES
    uint16_t heading_type;     // heading(bestheading->position_type;)
                               // POSITION_TYPES
    uint8_t num_satellites1;
    uint8_t num_satellites2;
    char board_type[4];  // "482" "718D" "NONE"

    // Radio Status
    float diff_age;
    float solution_age;
    float baseline_length;  // Baseline length (m)

    // Ros Custom Msg Tail
    // NovatelExtendedSolutionStatus extended_solution_status
    // NovatelSignalMask signal_mask
    uint8_t extended_solution_status;
    uint8_t signal_mask;
} cmd_rtk_info;  // sizeof(cmd_rtk_info) = 223

typedef struct {
    float imu_orientation[4];          // x y z w
    float imu_angular_velocity[3];     // x y z
    float imu_linear_acceleration[3];  // x y z
} cmd_imu_info;

typedef struct {
    float imu_a[3];  // x y z
    float imu_g[3];  // x y z
    float imu_m[3];  // x y z
    float imu_rpy[3];
    float imu_q[4];
} cmd_imu_raw_info;

typedef struct {
    float odom_pose_position[3];     // x y z
    float odom_pose_orientation[4];  // x y z w
    float odom_twist_linear[3];      // x y z
    float odom_twist_angular[3];     // x y z
} cmd_odom_info;

typedef struct {
    float position;  // 位置
    float velocity;  // 速度
    float torque;    // 力矩
} motor_info;

typedef struct {
    motor_info motor[4];
} cmd_motor_info;

typedef struct {
    uint32_t DI;
    uint32_t DO;
    float AI[8];
    float AO[8];
} cmd_io_info;

typedef struct {
    float voltage;
    float temperature;
    float current;
    float charge;
    float capacity;
    float design_capacity;
    float percentage;
    uint8_t power_supply_status;
    uint8_t power_supply_health;
    uint8_t power_supply_technology;
} cmd_battery_info;

// 下发 -------------------------
typedef struct {
    uint32_t heartbeat;
} cmd_heartbeat;

typedef struct {
    float wheel_radius;     // 轮径
    float wheel_distance;   // 轮距
    float reduction_ratio;  // 减速比
} cmd_param_set;

typedef struct {
    uint8_t ctrl_mode;  //  0：位置  1：速度  2：力矩
    float position;     // 位置
    float velocity;     // 速度
    float torque;       // 力矩
} motor_ctrl;

typedef struct {
    motor_ctrl motor[4];
    uint8_t motor_mask;  // 要控制哪几个电机
} cmd_motor_ctrl;

typedef struct {
    float linear[3];
    float angular[3];
} cmd_cmd_vel;

typedef struct {
    uint32_t set_DO;
    uint32_t set_DO_mask;  // 要控制哪几个IO
    float set_AO[8];
    uint8_t set_AO_mask;   // 要控制哪几个IO
} cmd_io_control_req;

typedef struct {
    bool succeed;
} cmd_io_control_res;

#pragma pack(pop)
}  // namespace winnie_base

#endif  // ROBORTS_SDK_PROTOCOL_CONTENT_H
