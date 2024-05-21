#ifndef WINNIE_SDK_PROTOCOL_CONTENT_H
#define WINNIE_SDK_PROTOCOL_CONTENT_H

#include <stdint.h>

namespace winnie_sdk {

#pragma pack(push, 1)
// DEVICE_ADDRESS
#define JETSON_ADDRESS (0x00u)
#define WINNIE_ADDRESS (0X01u)
#define BROADCAST_ADDRESS (0Xffu)

// CMD_SET
#define UNIVERSAL_CMD_SET (0x00u)
#define WINNIE_CMD_SET (0x02u)

/*----------------------------UNIVERSAL_CMD_SET--- 0x00 ---------------------*/
/*
 *  cmd_set:  UNIVERSAL_CMD_SET
 *  cmd_id:   CMD_HEARTBEAT
 *  sender:   JETSON_ADDRESS
 *  receiver: WINNIE_ADDRESS
 *  msg_type: cmd_heartbeat
 *  session:  no ack
 */
#define CMD_HEARTBEAT (0x01u)
typedef struct {
    uint32_t heartbeat;
} cmd_heartbeat;

/*
 *  cmd_set:  UNIVERSAL_CMD_SET
 *  cmd_id:   CMD_REPORT_VERSION
 *  sender:   JETSON_ADDRESS
 *  receiver: WINNIE_ADDRESS
 *  msg_type: cmd_version_id
 *  session:  need ack
 *  ack_type: cmd_version_id
 */
#define CMD_REPORT_VERSION (0x02u)
typedef struct {
    uint32_t version_id;
} cmd_version_id;

/*-----------------------------WINNIE_CMD_SET---- 0x02 ---------------------*/
/*
 *  cmd_set:  WINNIE_CMD_SET
 *  cmd_id:   CMD_PUSH_WINNIE_INFO
 *  sender:   WINNIE_ADDRESS
 *  receiver: JETSON_ADDRESS
 *  msg_type: cmd_winnie_info
 *  session:  no ack
 */
#define CMD_PUSH_WINNIE_INFO (0x01u)
typedef struct {
    float height;
    float imu_rpy[3];
    float cmd_torque[2];
    float current_hip_pos[4];
    float cmd_hip_pos[4];
    float target_hip_pos[4];
    float hip_current[4];
    float lqr_states[4];
    float lqr_refs[4];
    float lqr_gains[4];
    float lqr_filtered_states[4];
    float current_torque[2];
} cmd_winnie_info;

#define CMD_SET_WINNIE_LQR_GAIN (0x02u)
typedef struct {
    float gain[4];
} cmd_winnie_lqr_gain;

#define CMD_SET_WINNIE_SPEED (0x03u)
typedef struct {
    float vx;
    float wz;
} cmd_winnie_speed;

#pragma pack(pop)
}  // namespace winnie_sdk
#endif  // ROBORTS_SDK_PROTOCOL_CONTENT_H
