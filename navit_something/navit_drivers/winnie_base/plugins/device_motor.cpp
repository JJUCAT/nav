//
// Created by fan on 23-7-3.
//

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <atomic>
#include <pluginlib/class_list_macros.hpp>

#include "protocol_winnie.h"
#include "ros_helper.hpp"
#include "sdk_helper.hpp"
#include "winnie_base/MotorCtrl.h"
#include "winnie_base/MotorInfos.h"
#include "winnie_base/SetMotor.h"
#include "winnie_base/device_basic.hpp"

namespace MessageConverter {

static winnie_base::MotorInfos::Ptr to_ros_msg(const winnie_base::cmd_motor_info& msg_sdk,
                                               const ros::Time& current_time = ros::Time::now()) {
    winnie_base::MotorInfos::Ptr msg_motor_infos(new winnie_base::MotorInfos);

    static const int motor_num = sizeof(msg_sdk.motor) / sizeof(msg_sdk.motor[0]);

    for (int i = 0; i < motor_num; ++i) {
        auto& info    = msg_motor_infos->infos.emplace_back();
        info.position = msg_sdk.motor[i].position;
        info.velocity = msg_sdk.motor[i].velocity;
        info.torque   = msg_sdk.motor[i].torque;
    }

    return msg_motor_infos;
}

static std::shared_ptr<winnie_base::cmd_motor_ctrl> to_sdk_msg(const winnie_base::SetMotorRequest& msg_ros) {
    auto msg_sdk        = std::make_shared<winnie_base::cmd_motor_ctrl>();
    msg_sdk->motor_mask = msg_ros.motor_mask;
    int num             = std::min(sizeof(msg_sdk->motor) / sizeof(msg_sdk->motor[0]), msg_ros.motor.size());
    for (int i = 0; i < num; ++i) {
        msg_sdk->motor[i].ctrl_mode = msg_ros.motor[i].ctrl_mode;
        msg_sdk->motor[i].position  = msg_ros.motor[i].position;
        msg_sdk->motor[i].velocity  = msg_ros.motor[i].velocity;
        msg_sdk->motor[i].torque    = msg_ros.motor[i].torque;
    }
    return msg_sdk;
}

static winnie_base::cmd_cmd_vel to_sdk_msg(const geometry_msgs::Twist& msg_ros) {
    winnie_base::cmd_cmd_vel msg_sdk;
    msg_sdk.linear[0]  = msg_ros.linear.x;
    msg_sdk.linear[1]  = msg_ros.linear.y;
    msg_sdk.linear[2]  = msg_ros.linear.z;
    msg_sdk.angular[0] = msg_ros.angular.x;
    msg_sdk.angular[1] = msg_ros.angular.y;
    msg_sdk.angular[2] = msg_ros.angular.z;
    return msg_sdk;
}

}  // namespace MessageConverter

namespace winnie_base {

class DeviceMotor : public DeviceBasic {
   public:
    typedef struct {
        float wheel_radius;     // 轮径
        float wheel_distance;   // 轮距
        float reduction_ratio;  // 减速比
    } AgvParam;

   private:
    bool OnInit() override {
        std::vector<double> imu_odom_tf_rpy;
        ros_handle_->param("wheel_radius", agv_param_.wheel_radius, 0.09f);
        ros_handle_->param("wheel_distance", agv_param_.wheel_distance, 0.493f);
        ros_handle_->param("reduction_ratio", agv_param_.reduction_ratio, 15.0f);
        ros_handle_->param("odom_use_imu_yaw", odom_use_imu_yaw_, true);
        ros_handle_->param("imu_yaw_by_integral", imu_yaw_by_integral_, true);
        ros_handle_->param("imu_topic_name", imu_topic_name_, std::string("/imu"));
        ros_handle_->param("imu_odom_tf_rpy", imu_odom_tf_rpy, {0, 0, 0});

        tf2::Quaternion quaternion;
        if (imu_odom_tf_rpy.size() >= 3) {
            quaternion.setRPY(imu_odom_tf_rpy[0] * M_PI, imu_odom_tf_rpy[1] * M_PI, imu_odom_tf_rpy[2] * M_PI);
        } else {
            quaternion.setRPY(0, 0, 0);
        }
        tf_imu_to_odom_.setRotation(quaternion);

        if (odom_use_imu_yaw_) {
            ros_sub_imu_.Init(ros_handle_, imu_topic_name_);
            ros_sub_imu_.register_topic_callback([this](const sensor_msgs::Imu::ConstPtr& msg) {
                tf2::Quaternion quat;
                if (imu_yaw_by_integral_) {
                    static ros::Time prev_time = msg->header.stamp;
                    static double prev_yaw     = 0;

                    // 获取当前时间戳
                    ros::Time curr_time = msg->header.stamp;
                    // 计算时间差
                    double dt = (curr_time - prev_time).toSec();
                    // 获取当前角速度
                    double gyro_yaw = msg->angular_velocity.z;
                    // 使用速度积分计算yaw角的变化量
                    double delta_yaw = gyro_yaw * dt;  // 角速度乘以时间差
                    // 更新累积的yaw角
                    double current_yaw = tf2NormalizeAngle(prev_yaw + delta_yaw);

                    // 将当前时间戳、位置和yaw角信息存储为下一次计算的旧值
                    prev_time = curr_time;
                    prev_yaw  = current_yaw;

                    odom_yaw_from_imu_ = current_yaw;
                    return;
                    quat.setRPY(0, 0, current_yaw);
                } else {
                    quat =
                        tf2::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
                }

                // 修正
                tf2::Transform tf_in, tf_out;
                tf_in.setRotation(quat);
                tf_out = tf_in * tf_imu_to_odom_;

                // 计算yaw角
                double roll, pitch, yaw;
                tf2::Matrix3x3(tf_out.getRotation()).getRPY(roll, pitch, yaw);
                odom_yaw_from_imu_ = yaw;
            });
        }

        ros_pub_odom_.Init(ros_handle_, "/odom");
        ros_pub_motor_info_.Init(ros_handle_, "/motor_infos");
        ros_sub_cmd_vel_.Init(ros_handle_, "/cmd_vel", 1);
        ros_server_set_motor_.Init(ros_handle_, "/set_motor");

        sdk_pub_heart_beat_.Init(sdk_handle_, CMD_SET_HEART_BEAT, 1, JETSON_ADDRESS, WINNIE_ADDRESS);
        sdk_pub_motor_ctrl_.Init(sdk_handle_, CMD_SET_MOTOR_CTRL, 1, JETSON_ADDRESS, WINNIE_ADDRESS);
        sdk_sub_motor_info_.Init(sdk_handle_, CMD_SET_MOTOR_INFO, 1, WINNIE_ADDRESS, JETSON_ADDRESS);

        ros_server_set_motor_.register_service_callback(
            [this](winnie_base::SetMotor::RequestType& req, winnie_base::SetMotor::ResponseType& res) {
                // to_sdk_msg
                auto sdk_req = MessageConverter::to_sdk_msg(req);
                sdk_pub_motor_ctrl_.publish(*sdk_req);
                return true;
            });

        ros_sub_cmd_vel_.register_topic_callback([this](const geometry_msgs::Twist::ConstPtr& msg) {
            /// 方案一： mcu 逆解
            // auto sdk_msg = MessageConverter::to_sdk_msg(*msg);
            // sdk_publisher_cmd_vel_.publish(sdk_msg);

            /// 方案二： 工控机 逆解
            double linear_x  = msg->linear.x;
            double angular_z = msg->angular.z;
            auto motor_ctrl  = inverse_kinematics(linear_x, angular_z);

            sdk_pub_motor_ctrl_.publish(motor_ctrl);
        });

        sdk_sub_motor_info_.register_topic_callback([this](const std::shared_ptr<winnie_base::cmd_motor_info>& msg) {
            data_motor_info_       = *msg;
            ros::Time current_time = ros::Time::now();

            auto msg_motor_infos = MessageConverter::to_ros_msg(*msg);
            ros_pub_motor_info_.publish(msg_motor_infos);

            auto msg_odom = calc_odom(*msg, current_time);
            ros_pub_odom_.publish(msg_odom);

            /// tf
            static geometry_msgs::TransformStamped odom_tf;
            odom_tf.header.stamp    = current_time;
            odom_tf.header.frame_id = "odom";
            odom_tf.child_frame_id  = "base_link";

            odom_tf.transform.translation.x = msg_odom->pose.pose.position.x;
            odom_tf.transform.translation.y = msg_odom->pose.pose.position.y;
            odom_tf.transform.translation.z = 0.0;
            odom_tf.transform.rotation      = msg_odom->pose.pose.orientation;
            tf_broadcaster_.sendTransform(odom_tf);

            // PublishStates();
        });

        // AddState("wheel_speed_left_rpm", data_motor_info_.motor[0].velocity);
        // AddState("wheel_speed_right_rpm", data_motor_info_.motor[1].velocity);
        // AddState("mow_speed_rpm", data_motor_info_.motor[2].velocity);
        //
        // AddCtrl("wheel_speed_left_rpm", [this](const std::string& key, const std::string& val) {
        //     cmd_motor_ctrl motor_ctrl     = {};
        //     motor_ctrl.motor[0].ctrl_mode = 1;
        //     motor_ctrl.motor[0].velocity  = stof(val);
        //     motor_ctrl.motor_mask         = 0b0001;
        //     sdk_pub_motor_ctrl_.publish(motor_ctrl);
        //     return true;
        // });
        // AddCtrl("wheel_speed_right_rpm", [this](const std::string& key, const std::string& val) {
        //     cmd_motor_ctrl motor_ctrl     = {};
        //     motor_ctrl.motor[1].ctrl_mode = 1;
        //     motor_ctrl.motor[1].velocity  = stof(val);
        //     motor_ctrl.motor_mask         = 0b0010;
        //     sdk_pub_motor_ctrl_.publish(motor_ctrl);
        //     return true;
        // });
        // AddCtrl("speed", [this](const std::string& key, const std::string& val) {
        //     float speed = stof(val);
        //     if (speed > 1)
        //         speed = 1.0;
        //     if (speed < -1)
        //         speed = -1.0;
        //
        //     cmd_motor_ctrl motor_ctrl     = {};
        //     motor_ctrl.motor[2].ctrl_mode = 1;
        //     motor_ctrl.motor[2].velocity  = speed * 3500;
        //     motor_ctrl.motor_mask         = 0b0100;
        //     sdk_pub_motor_ctrl_.publish(motor_ctrl);
        //     return true;
        // });
        //
        // return DeviceHybrid::OnInit();
        return true;
    }

    void SpinOnce() override {
        ++data_heartbeat_.heartbeat;
        sdk_pub_heart_beat_.publish(data_heartbeat_);
    }

   private:
    // 计算odom
    nav_msgs::Odometry::Ptr calc_odom(const winnie_base::cmd_motor_info& msg,
                                      const ros::Time& current_time = ros::Time::now()) {
        static ros::Time last_time = current_time;

        // 计算时间间隔
        double dt = (current_time - last_time).toSec();
        last_time = current_time;

        // 左右轮电机速度
        // auto left_motor_speed  = msg.motor[0].velocity;
        // auto right_motor_speed = msg.motor[1].velocity;
        auto left_motor_speed  = msg.motor[0].velocity * 2 * M_PI / 60;  // rpm to rad/s
        auto right_motor_speed = msg.motor[1].velocity * 2 * M_PI / 60;  // rpm to rad/s

        // 计算左右轮速度
        double v_left  = left_motor_speed * (agv_param_.wheel_radius / agv_param_.reduction_ratio);
        double v_right = right_motor_speed * (agv_param_.wheel_radius / agv_param_.reduction_ratio);

        // 计算线速度和角速度
        double linear_vel  = (v_left + v_right) / 2;
        double angular_vel = (v_right - v_left) / agv_param_.wheel_distance;

        // 更新位姿信息
        x += linear_vel * cos(theta) * dt;
        y += linear_vel * sin(theta) * dt;
        if (odom_use_imu_yaw_) {
            // 使用imu的角度
            theta = odom_yaw_from_imu_;
        } else {
            theta += angular_vel * dt;
        }

        // 创建并填充里程计消息
        nav_msgs::Odometry::Ptr msg_odom(new nav_msgs::Odometry);
        msg_odom->header.stamp    = current_time;
        msg_odom->header.frame_id = "odom";
        msg_odom->child_frame_id  = "base_link";

        // 设置位置信息
        msg_odom->pose.pose.position.x    = x;
        msg_odom->pose.pose.position.y    = y;
        msg_odom->pose.pose.position.z    = 0.0;
        msg_odom->pose.pose.orientation.w = cos(theta / 2);
        msg_odom->pose.pose.orientation.x = 0.0;
        msg_odom->pose.pose.orientation.y = 0.0;
        msg_odom->pose.pose.orientation.z = sin(theta / 2);

        // 设置速度信息
        msg_odom->twist.twist.linear.x  = linear_vel;
        msg_odom->twist.twist.linear.y  = 0.0;
        msg_odom->twist.twist.angular.z = angular_vel;
        return msg_odom;
    }

    // 双差速底盘运动学逆解
    cmd_motor_ctrl inverse_kinematics(double linear_x, double angular_z) const {
        // 根据给定的速度计算左右轮子线速度
        auto left_speed  = linear_x - angular_z * agv_param_.wheel_distance / 2;
        auto right_speed = linear_x + angular_z * agv_param_.wheel_distance / 2;

        // 计算左右轮子的电机速度 单位是rad/s（弧度每秒）。
        auto left_motor_speed  = left_speed / agv_param_.wheel_radius * agv_param_.reduction_ratio;
        auto right_motor_speed = right_speed / agv_param_.wheel_radius * agv_param_.reduction_ratio;

        // 单位转换成rpm（每分钟转数）
        auto left_motor_speed_rpm  = left_motor_speed * 60 / (2 * M_PI);
        auto right_motor_speed_rpm = right_motor_speed * 60 / (2 * M_PI);

        // 电机消息
        cmd_motor_ctrl motor_ctrl     = {};
        motor_ctrl.motor[0].ctrl_mode = 1;
        motor_ctrl.motor[0].velocity  = left_motor_speed_rpm;
        motor_ctrl.motor[1].ctrl_mode = 1;
        motor_ctrl.motor[1].velocity  = right_motor_speed_rpm;
        motor_ctrl.motor_mask         = 0b0011;
        return motor_ctrl;
    }

   private:
    bool odom_use_imu_yaw_;
    bool imu_yaw_by_integral_;  // true:通过积分计算yaw角，false:直接使用imu的yaw角
    std::string imu_topic_name_;

    tf2::Transform tf_imu_to_odom_;
    std::atomic<double> odom_yaw_from_imu_;

    AgvParam agv_param_{};
    double x{}, y{}, theta{};
    cmd_motor_info data_motor_info_{};
    cmd_heartbeat data_heartbeat_{};

    RosPublisherHelper<nav_msgs::Odometry> ros_pub_odom_;
    RosPublisherHelper<winnie_base::MotorInfos> ros_pub_motor_info_;
    RosSubscriberHelper<geometry_msgs::Twist> ros_sub_cmd_vel_;
    RosSubscriberHelper<sensor_msgs::Imu> ros_sub_imu_;
    RosServiceServerHelper<winnie_base::SetMotor> ros_server_set_motor_;

    SdkPublisherHelper<winnie_base::cmd_heartbeat> sdk_pub_heart_beat_;
    SdkPublisherHelper<winnie_base::cmd_motor_ctrl> sdk_pub_motor_ctrl_;
    SdkSubscriberHelper<winnie_base::cmd_motor_info> sdk_sub_motor_info_;

    tf2_ros::TransformBroadcaster tf_broadcaster_;
};
}  // namespace winnie_base

PLUGINLIB_EXPORT_CLASS(winnie_base::DeviceMotor, winnie_base::DeviceBase)