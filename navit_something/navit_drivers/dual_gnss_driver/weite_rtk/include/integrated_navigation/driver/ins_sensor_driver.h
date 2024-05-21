#ifndef SRC_INS_DRIVER_H
#define SRC_INS_DRIVER_H

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>

#include "integrated_navigation/driver/gnss/novatel_gps_driver.h"
#include "integrated_navigation/driver/io_common/io_common.h"
#include "integrated_navigation/driver/io_common/serial_common.h"
#include "integrated_navigation/driver/io_common/tcp_common.h"
#include "serial/serial.h"

namespace integrated_navigation {
    class ins_sensor_driver {
    public:
        ins_sensor_driver(ros::NodeHandle &nh);
        ~ins_sensor_driver();

    private:
        void Spin();
        bool Connect();
        void Disconnect();
        bool IsConnected();
        void MultiSensorDataFlow();
        bool read_from_connector();
        bool read_from_connector(const uint16_t number);
        const unsigned int rate_;
        std::string connection_type_;
        std::string port_;
        boost::thread thread_;
        double reconnect_delay_s_;

        std::shared_ptr<io_common> io_common_;
        std::shared_ptr<novatel_gps_driver> gnss_;

        std::vector<uint8_t> data_buffer_;

        uint16_t imu_frame_;
        uint16_t imu_frame_trans_;
        uint16_t novatel_frame_;
        uint16_t novatel_frame_trans_;

        serial::Serial ser;
        unsigned char rbuf[2048];  // 接收缓冲区
        std::string rbuf_str,remain_str;
        int numinbuf;
        int numgetted;    // num get
        std::vector<std::string> r_strs;
         bool firstTime=true;
        int cout_time; 

    };
}
#endif //SRC_INS_DRIVER_H