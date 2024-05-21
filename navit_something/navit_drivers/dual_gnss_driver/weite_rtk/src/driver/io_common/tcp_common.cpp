#include "integrated_navigation/driver/io_common/tcp_common.h"

namespace integrated_navigation {
    tcp_common::tcp_common(ros::NodeHandle &nh)
            : port_("192.168.0.10:3456"),
              is_connected_(false),
              tcp_socket_(io_service_) {
        // Load yaml file parameters.
        nh.getParam("/multi_sensor_config/port", port_);
        hw_id_ = "Multi Sensor (" + port_ + ")";
        LOG(INFO) << "network port = " << port_<<std::endl;
    }

    tcp_common::~tcp_common() { }

    bool tcp_common::Connect() {
        Disconnect();
        return CreateIpConnection();
    }

    void tcp_common::Disconnect() {
        LOG(INFO) << "Disconnect tcp."<<std::endl;
        tcp_socket_.close();
    }

    bool tcp_common::CreateIpConnection() {
        std::string ip;
        std::string port;
        uint16_t num_port;
        size_t sep_pos = port_.find(':');
        if(sep_pos == std::string::npos || sep_pos == port_.size() - 1){
            LOG(INFO) << "Using default port."<<std::endl;
            std::stringstream ss;
            num_port = DEFAULT_TCP_PORT;
            ss << num_port;
            port = ss.str();
        }else{
            port = port_.substr(sep_pos + 1);
        }

        if(sep_pos != 0){
            ip = port_.substr(0, sep_pos);
        }

        try{
            if(!ip.empty()){
                boost::asio::ip::tcp::resolver resolver(io_service_);
                boost::asio::ip::tcp::resolver::query query(ip, port);
                boost::asio::ip::tcp::resolver::iterator iter = resolver.resolve(query);
                LOG(INFO) << "Connecting via TCP to " << ip << ' ' << port<<std::endl;

                boost::asio::connect(tcp_socket_, iter);
            }else{
                auto port_num = static_cast<uint16_t>(strtoll(port.c_str(), nullptr, 10));
                boost::asio::ip::tcp::acceptor acceptor(io_service_,
                                                        boost::asio::ip::tcp::endpoint(
                                                                boost::asio::ip::tcp::v4(), port_num));
                LOG(INFO) << "Listening on TCP port: " << port<<std::endl;
                acceptor.accept(tcp_socket_);
                LOG(INFO) << "Accepted TCP connection from client: " << 
                         tcp_socket_.remote_endpoint().address().to_string()<<std::endl;
            }
        }
        catch(std::exception& e){
            error_msg_ = e.what();
            LOG(ERROR) << "Unable to connect: " << e.what();
            return false;
        }

        is_connected_ = true;

        LOG(INFO) << "connected to device:" << get_device()<<std::endl; 
        return true;
    }

    io_common::ReadResult tcp_common::IORead(std::vector<uint8_t>& data_buffer, const size_t number){
        try{
            boost::system::error_code error;
            size_t len;

            /// Fixed-size buffer for reading directly from sockets
            char socket_buffer[number];

            // DLOG(WARNING) << "tcp read before";
            len = tcp_socket_.read_some(boost::asio::buffer(socket_buffer, number), error);
            // DLOG(WARNING) << "tcp read after";


            data_buffer.insert(data_buffer.end(), socket_buffer, socket_buffer + len);
            if(error){
                Disconnect();
                return io_common::READ_ERROR;
            }
            return io_common::READ_SUCCESS;
        }
        catch (std::exception& e){
            LOG(WARNING) << "TCP connection error: " << e.what();
            return io_common::READ_ERROR;
        }
    }
}