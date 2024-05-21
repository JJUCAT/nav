#ifndef SRC_SERIAL_COMMON_H
#define SRC_SERIAL_COMMON_H

#include <ros/ros.h>
#include <integrated_navigation/library/Serial/serial_utils.h>

#include "integrated_navigation/log/logging.h"
#include "integrated_navigation/driver/io_common/io_common.h"

namespace integrated_navigation {
    class serial_common : public io_common {
    public:
        serial_common(ros::NodeHandle &nh);
        ~serial_common();

        void Disconnect() override;
        bool Connect() override;
        bool IsConnected() {return is_connected_;}
        bool CreateSerialConnection();
        io_common::ReadResult IORead(std::vector<uint8_t>& output, const size_t max_bytes) override ;
        std::string get_device() {return hw_id_;}

    private:
        std::string port_;
        int baudrate_;

        SerialPort serial_;
        bool is_connected_;
        std::string error_msg_;
        std::string hw_id_;
    };

}
#endif //SRC_SERIAL_COMMON_H