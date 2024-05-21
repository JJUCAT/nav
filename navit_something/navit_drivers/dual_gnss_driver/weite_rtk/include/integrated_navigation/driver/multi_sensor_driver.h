#ifndef SRC_MULTI_SENSOR_DRIVER_H
#define SRC_MULTI_SENSOR_DRIVER_H

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>

#include "integrated_navigation/driver/io_common/io_common.h"
#include "integrated_navigation/driver/io_common/serial_common.h"
#include "integrated_navigation/driver/io_common/tcp_common.h"
#include "integrated_navigation/driver/imu/ky102n_imu_driver.h"
#include "integrated_navigation/driver/gnss/novatel_gps_driver.h"
#include "integrated_navigation/driver/radio/radio_message_driver.h"

#define DEBUG_SENSOR_DRIVER 1

namespace integrated_navigation {
    class multi_sensor_driver {
    public:
        multi_sensor_driver(ros::NodeHandle &nh);
        ~multi_sensor_driver();

    private:
        void Spin();
        bool Connect();
        void Disconnect();
        bool IsConnected();
        void MultiSensorDataFlow();
        void ky102n_imu_callback();
        void novatel_posa_callback();
        void radio_message_callback();
        bool read_from_connector(const uint16_t number);
        bool read_from_first_connecter();
        bool read_from_imu_connecter();
        bool read_from_novatel_connecter();
        bool read_from_radio_connecter();

        const unsigned int rate_;
        std::string connection_type_;
        std::string port_;
        boost::thread thread_;
        double reconnect_delay_s_;

        std::shared_ptr<io_common> io_common_;
        std::shared_ptr<ky102n_imu_driver> ky102n_imu_;
        std::shared_ptr<novatel_gps_driver> novatel_gps_;
        std::shared_ptr<radio_message_driver> radio_msg_;

        std::vector<uint8_t> data_buffer_;

        uint16_t imu_frame_;
        uint16_t imu_frame_trans_;
        uint16_t novatel_frame_;
        uint16_t novatel_frame_trans_;

    };
}
#endif //SRC_MULTI_SENSOR_DRIVER_H