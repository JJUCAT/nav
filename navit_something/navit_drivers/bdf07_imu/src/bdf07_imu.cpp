//
// Created by fan on 23-11-9.
//

#include "bdf07_imu/bdf07_imu.h"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include "impl/serial_utils.h"
#include <angles/angles.h>

struct IMU_DATA
{
  double Ax;     // Accel x axis
  double Ay;     // Accel y axis
  double Az;     // Accel z axis
  double Gx;     // Gyro x axis
  double Gy;     // Gyro y axis
  double Gz;     // Gyro z axis
  double Roll;   // Roll
  double Pitch;  // Pitch
  double Yaw;    // Yaw
  double Temp;   // Temperature
};

std::string stringToHex(const std::string& input)
{
  static const char* const lut = "0123456789ABCDEF";
  std::size_t len = input.length();

  std::string output;
  output.reserve(2 * len);
  for (std::size_t i = 0; i < len; ++i)
  {
    const unsigned char c = input[i];
    output.push_back(lut[c >> 4]);
    output.push_back(lut[c & 15]);
  }
  return output;
}

std::string char_to_hex(char c)
{
  static const char* const lut = "0123456789ABCDEF";
  return "" + lut[c >> 4] + lut[c & 15];
}

namespace bdf07_imu
{
static const int Sensor_Scale = 65536;
static const int Angle_Scale = 360;
static const int Accel_Scale = 20;
static const int Rate_Scale = 1260;
static const int Temp_Scale = 200;
static const double G = 9.80665;

static const uint8_t sync_head1 = 0x7F;
static const uint8_t sync_head2 = 0x80;

Bdf07Imu::Bdf07Imu(ros::NodeHandle& nh) : nh_(nh)
{
  thread_ = std::thread(&Bdf07Imu::run, this);
}

void Bdf07Imu::run()
{
  std::string port;
  std::string imu_frame_id;
  int cfg_baud_rate = 115200;
  std::string imu_topic;
  double yaw_change_threshold;
  nh_.param<std::string>("port", port, "/dev/ttyUSB0");
  nh_.param<std::string>("frame_id", imu_frame_id, "imu_link");
  nh_.param<int>("baudrate", cfg_baud_rate, 115200);
  nh_.param<std::string>("topic", imu_topic, "/bdf07/imu_data");
  nh_.param<double>("yaw_change_threshold_param", yaw_change_threshold, 0.1);

  serial::SerialConfig config;
  config.baud = cfg_baud_rate;
  serial::SerialPort ser;
  if (ser.Open(port, config))
  {
    ROS_INFO_STREAM("Serial Port initialized ok");
  }
  else
  {
    ROS_INFO_STREAM(port.c_str());
    ROS_INFO_STREAM(ser.ErrorMsg());
    ROS_INFO_STREAM("Serial Port initialized failed");
    ros::shutdown();
  }
  ROS_INFO_STREAM("imu start work...");

  IMU_DATA imuData;
  ros::Publisher imu_pub = nh_.advertise<sensor_msgs::Imu>(imu_topic, 50);

  sensor_msgs::Imu::Ptr imu(new sensor_msgs::Imu);
  imu->header.frame_id = imu_frame_id;
  imu->orientation_covariance[0] = 1e-4;
  imu->orientation_covariance[1] = 0;
  imu->orientation_covariance[2] = 0;
  imu->orientation_covariance[3] = 0;
  imu->orientation_covariance[4] = 1e-4;
  imu->orientation_covariance[5] = 0;
  imu->orientation_covariance[6] = 0;
  imu->orientation_covariance[7] = 0;
  imu->orientation_covariance[8] = 1e-4;

  imu->angular_velocity_covariance[0] = 1e-4;
  imu->angular_velocity_covariance[4] = 1e-4;
  imu->angular_velocity_covariance[8] = 1e-4;

  imu->linear_acceleration_covariance[0] = 1e-4;
  imu->linear_acceleration_covariance[4] = 1e-4;
  imu->linear_acceleration_covariance[8] = 1e-4;

  std::string input_raw;
  std::string buffer;
  uint8_t checksum = 0;
  uint8_t state = 0;
  while (!stop_ && ros::ok())
  {
    input_raw.clear();
    if (ser.ReadBytes(input_raw, 23, 100) != serial::SerialPort::SUCCESS)
    {
      buffer.clear();
      checksum = 0;
      state = 0;
      continue;
    }

    if (!input_raw.size())
    {
      continue;
    }

    uint8_t* datas = reinterpret_cast<uint8_t*>(&input_raw[0]);
    size_t len = input_raw.size();

    for (size_t i = 0; i < len; i++)
    {
      bool valid = false;
      uint8_t ch = datas[i];
      switch (state)
      {
        case 0:
          if (ch == sync_head1)
          {
            state++;
          }
          break;
        case 1:
          if (ch == sync_head2)
          {
            buffer.clear();
            checksum = 0;
            state++;
          }
          else if (ch != sync_head1)
          {
            state = 0;
          }
          break;
        case 2:
          buffer += input_raw.substr(i, 1);
          checksum += ch;
          if (buffer.size() >= 20)
          {
            state++;
          }
          break;
        case 3:
          checksum = ~checksum;
          if (checksum != ch)
          {
            ROS_INFO_STREAM("warning: checksum failed, lost frame.");
          }
          else
          {
            valid = true;
          }
          state = 0;
          break;

        default:
          state = 0;
          break;
      }

      if (valid == true)
      {
        uint8_t* buf = reinterpret_cast<uint8_t*>(&buffer[0]);
        int16_t sensors[10];

        sensors[0] = (int16_t)((uint16_t)buf[0] + ((uint16_t)buf[1] << 8));
        sensors[1] = (int16_t)((uint16_t)buf[2] + ((uint16_t)buf[3] << 8));
        sensors[2] = (int16_t)((uint16_t)buf[4] + ((uint16_t)buf[5] << 8));
        sensors[3] = (int16_t)((uint16_t)buf[6] + ((uint16_t)buf[7] << 8));
        sensors[4] = (int16_t)((uint16_t)buf[8] + ((uint16_t)buf[9] << 8));
        sensors[5] = (int16_t)((uint16_t)buf[10] + ((uint16_t)buf[11] << 8));
        sensors[6] = (int16_t)((uint16_t)buf[12] + ((uint16_t)buf[13] << 8));
        sensors[7] = (int16_t)((uint16_t)buf[14] + ((uint16_t)buf[15] << 8));
        sensors[8] = (int16_t)((uint16_t)buf[16] + ((uint16_t)buf[17] << 8));
        sensors[9] = (int16_t)((uint16_t)buf[18] + ((uint16_t)buf[19] << 8));

        imuData.Ax = (double)sensors[0] * Accel_Scale / Sensor_Scale;
        imuData.Ay = (double)sensors[1] * Accel_Scale / Sensor_Scale;
        imuData.Az = (double)sensors[2] * Accel_Scale / Sensor_Scale;

        imuData.Gx = (double)sensors[3] * Rate_Scale / Sensor_Scale;
        imuData.Gy = (double)sensors[4] * Rate_Scale / Sensor_Scale;
        imuData.Gz = (double)sensors[5] * Rate_Scale / Sensor_Scale;

        imuData.Roll = (double)sensors[6] * Angle_Scale / Sensor_Scale;
        imuData.Pitch = (double)sensors[7] * Angle_Scale / Sensor_Scale;
        imuData.Yaw = (double)sensors[8] * Angle_Scale / Sensor_Scale;

        imuData.Temp = (double)sensors[9] * Temp_Scale / Sensor_Scale;

        double roll_data = imuData.Roll * M_PI / (180);
        double pitch_data = imuData.Pitch * M_PI / (180);
        double yaw_raw = imuData.Yaw * M_PI / (180);

        static double yaw_offset = 0.f;
        static double last_yaw = yaw_raw;
        double yaw_new = yaw_raw + yaw_offset;
        if (fabs(angles::normalize_angle(yaw_new - last_yaw)) > yaw_change_threshold)
        {
          yaw_offset = last_yaw - yaw_raw;
          yaw_new = yaw_raw + yaw_offset;
          ROS_WARN("Detect imu jump, last store yaw = %f, yaw_raw = %f, yaw offset = %f, yaw_new = %f", last_yaw,
                   yaw_raw, yaw_offset, yaw_new);
        }
        last_yaw = yaw_new;

        /// publish
        imu->header.stamp = ros::Time::now();
        imu->header.frame_id = imu_frame_id;
        imu->orientation = tf::createQuaternionMsgFromRollPitchYaw(roll_data, -pitch_data, -yaw_new);
        imu->angular_velocity.x = (double)(imuData.Gx * M_PI / (180));
        imu->angular_velocity.y = (double)(-1 * imuData.Gy * M_PI / (180));
        imu->angular_velocity.z = (double)(-1 * imuData.Gz * M_PI / (180));
        imu->linear_acceleration.x = (double)(((imuData.Ax) * G));
        imu->linear_acceleration.y = (double)(((-1 * imuData.Ay) * G));
        imu->linear_acceleration.z = (double)((-1 * imuData.Az) * G);
        imu_pub.publish(imu);
      }
    }
  }
}

Bdf07Imu::~Bdf07Imu()
{
  stop_ = true;
  if (thread_.joinable())
  {
    thread_.join();
  }
}

}  // namespace bdf07_imu
