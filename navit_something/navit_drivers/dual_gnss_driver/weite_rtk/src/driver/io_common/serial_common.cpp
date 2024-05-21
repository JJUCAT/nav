#include "integrated_navigation/driver/io_common/serial_common.h"

namespace integrated_navigation {
    serial_common::serial_common(ros::NodeHandle &nh)
        : port_("/dev/ttyS0"),
          baudrate_(460800),
          is_connected_(false) {
        // serial/tcp configuration.
        nh.getParam("/multi_sensor_config/baudrate", baudrate_);
        nh.getParam("/multi_sensor_config/port", port_);
        hw_id_ = "Multi Sensor (" + port_ + ")";
        LOG(INFO) << "serial port and baudrate = " << port_ << ", " << baudrate_<<std::endl;
    }

    serial_common::~serial_common() { }

    void serial_common::Disconnect() {
        serial_.Close();
        is_connected_ = false;
    }

    bool serial_common::Connect() {
        Disconnect();
        return CreateSerialConnection();
    }

    bool serial_common::CreateSerialConnection() {
        SerialConfig config;
        config.baud = baudrate_;
        config.parity = SerialConfig::NO_PARITY;
        config.flow_control = false;
        config.data_bits = 8;
        config.stop_bits = 1;
        config.low_latency_mode = false;

        bool success = serial_.Open(port_, config);

        if (success) {
            is_connected_ = true;
            LOG(INFO) << "connected to device:" << get_device().c_str() << std::endl;
        } else {
            error_msg_ = serial_.ErrorMsg();
        }

        return success;
    }

    io_common::ReadResult serial_common::IORead(std::vector<uint8_t>& output, const size_t max_bytes) {
       // LOG(INFO) << "serial_common::IORead max_bytes:" <<max_bytes<<std::endl;
        SerialPort::Result result = serial_.ReadBytes(output, max_bytes, 1000);
//                std::string data_buffer_str;
// data_buffer_str.insert(data_buffer_str.end(),output.begin(),output.end());
// std::cout<<"serial_common::IORead data_buffer_str:"<<data_buffer_str<<std::endl;
           
        if (result == SerialPort::ERROR) {
            error_msg_ = serial_.ErrorMsg();
            return io_common::READ_ERROR;
        } else if (result == SerialPort::TIMEOUT) {
            error_msg_ = "Timed out waiting for serial device.";
            return io_common::READ_TIMEOUT;
        } else if (result == SerialPort::INTERRUPTED) {
            error_msg_ = "Interrupted during read from serial device.";
            return io_common::READ_INTERRUPTED;
        }

        return io_common::READ_SUCCESS;
    }
}