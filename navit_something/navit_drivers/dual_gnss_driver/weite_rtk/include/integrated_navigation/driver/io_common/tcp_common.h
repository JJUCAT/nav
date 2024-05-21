#ifndef SRC_TCP_COMMON_H
#define SRC_TCP_COMMON_H

#include <ros/ros.h>
#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>

#include "integrated_navigation/driver/io_common/io_common.h"

#include "integrated_navigation/log/logging.h"

namespace integrated_navigation {
    class tcp_common : public io_common {
    public:
        tcp_common(ros::NodeHandle &nh);
        ~tcp_common();

        void Disconnect() override ;
        bool Connect() override ;
        bool IsConnected() { return is_connected_; }
        bool CreateIpConnection();
        io_common::ReadResult IORead(std::vector<uint8_t>& data_buffer, const size_t number) override ;

        std::string get_device() { return hw_id_; }

    private:
        std::string port_;
        bool is_connected_;
        std::string error_msg_;
        std::string hw_id_;

        // TCP / UDP connections
        boost::asio::io_service io_service_;
        boost::asio::ip::tcp::socket tcp_socket_;
        std::string connection_type_;
        static constexpr uint16_t DEFAULT_TCP_PORT = 3001;
    };
}
#endif //SRC_TCP_COMMON_H