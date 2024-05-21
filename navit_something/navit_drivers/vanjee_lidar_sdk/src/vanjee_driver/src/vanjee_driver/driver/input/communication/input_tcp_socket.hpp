/*
 * @Author: guo
 * @Date: 2023-01-30 10:12:10
 * @LastEditors: Guo Xuemei 1242143575@qq.com
 * @LastEditTime: 2023-03-02 14:55:31
 * @FilePath: /vanjee_lidar_sdk_test/src/vanjee_lidar_sdk/src/vanjee_driver/src/vanjee_driver/driver/input/unix/input_socket_select.hpp
 */
#pragma once
#ifdef _WIN32

#include <vanjee_driver/driver/input/input.hpp>

#include <winsock2.h>
#include <ws2tcpip.h>

#pragma warning(disable : 4244)

namespace vanjee
{
    namespace lidar
    {
        class InputTcpSocket : public Input
        {
        public:
            InputTcpSocket(const WJInputParam& input_param)
            : Input(input_param), pkt_buf_len_(ETH_LEN),
                sock_offset_(0), sock_tail_(0)
            {
            sock_offset_ += input_param.user_layer_bytes;
            sock_tail_   += input_param.tail_layer_bytes;
            }

            virtual bool init();
            virtual bool start();
            virtual int32 send_(uint8* buf,uint32 size);
            virtual ~InputTcpSocket();

        private:
            inline void recvPacket();
            inline int createTcpSocket(uint16_t lidarPort, const std::string &lidarIp, uint16_t hostPort, const std::string &hostIp);
            inline bool tcpDisConnect(int m_pSocket);
            void reconnect();

            std::thread reconnect_thread_;

        protected:
            size_t pkt_buf_len_;
            int fds_;
            size_t sock_offset_;
            size_t sock_tail_;
            bool m_bConnected_ = false;
                
        };

        inline bool InputTcpSocket::init()
        {
            if (init_flag_)
                return true;

            int msop_fd = -1;

            WORD version = MAKEWORD(2, 2);
            WSADATA wsaData;
            int ret = WSAStartup(version, &wsaData);
            if(ret < 0)
                goto failWsa;

            msop_fd = createTcpSocket(input_param_.lidar_msop_port, input_param_.lidar_address, input_param_.host_msop_port, input_param_.host_address);
            if (msop_fd < 0)
                goto failMsop;

            fds_ = msop_fd;
            init_flag_ = true;
            return true;

            failMsop:
            failWsa:
                return false;
        }

        inline bool InputTcpSocket::start()
        {
            if (start_flag_)
            {
                return true;
            }

            if (!init_flag_)
            {
                cb_excep_(Error(ERRCODE_STARTBEFOREINIT));
                return false;
            }

            to_exit_recv_ = false;
            recv_thread_ = std::thread(std::bind(&InputTcpSocket::recvPacket, this));
            reconnect_thread_ = std::thread(std::bind(&InputTcpSocket::reconnect, this));
            start_flag_ = true;
            return true;
        }

        inline InputTcpSocket::~InputTcpSocket()
        {
            stop();
            closesocket(fds_);
        }

        /// @brief reateSocket()用于创建TCP Socket。
        inline int InputTcpSocket::createTcpSocket(uint16_t lidarPort, const std::string &lidarIp, uint16_t hostPort, const std::string &hostIp)
        {
            int fd;
            int ret;
            int reuse = 1;
            if (hostIp == "0.0.0.0" || lidarIp == "0.0.0.0")
            {
                perror("ip err: ");
                goto failSocket;
            }

            fd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
            if (fd < 0)
            {
                perror("socket: ");
                goto failSocket;
            }

            ret = setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse));
            if (ret < 0)
            {
                perror("setsockopt(SO_REUSEADDR): ");
                goto failOption;
            }

            {
                sleep(1);
                int set_rcv_size = 1024*1024;//设置为32K
                int set_optlen=sizeof(set_rcv_size);
                ret = setsockopt(fd,SOL_SOCKET,SO_RCVBUF,(const int*)&set_rcv_size,set_optlen);
            }

            struct sockaddr_in host_addr;
            memset(&host_addr, 0, sizeof(host_addr));
            host_addr.sin_family = AF_INET;         
            host_addr.sin_port = htons(hostPort);       
            host_addr.sin_addr.s_addr = INADDR_ANY; 
            inet_pton(AF_INET, hostIp.c_str(), &(host_addr.sin_addr));
            
            /// @brief 给 socket 绑定一个地址结构 (IP +port)
            ret = ::bind(fd, (struct sockaddr *)&host_addr, sizeof(host_addr));
            
            if (ret < 0)
            {
                perror("bind: ");
                goto failBind;
            }
            
    #ifdef ENABLE_DOUBLE_RCVBUF
            {
                uint32_t opt_val;  
                socklen_t opt_len; 
                getsockopt(fd, SOL_SOCKET, SO_RCVBUF, (char *)&opt_val, &opt_len);
                opt_val *= 4;
                setsockopt(fd, SOL_SOCKET, SO_RCVBUF, (char *)&opt_val, opt_len);
            }
    #endif
            struct sockaddr_in lidar_addr;
            memset(&lidar_addr, 0, sizeof(lidar_addr));
            lidar_addr.sin_family = AF_INET;         
            lidar_addr.sin_port = htons(lidarPort);       
            lidar_addr.sin_addr.s_addr = INADDR_ANY; 
            inet_pton(AF_INET, lidarIp.c_str(), &(lidar_addr.sin_addr));

            ret = connect(fd, (struct sockaddr *)&lidar_addr, sizeof(lidar_addr));
            if (ret < 0)
            {
                perror("connect fail: ");
                goto failConnect;
            }

            m_bConnected_ = true;
            return fd;

            failBind:
            failOption:
            failConnect:
                closesocket(fd);
            failSocket:
                return -1;
        }

        /// @brief reateSocket()用于TCP Disconnect。
        inline bool InputTcpSocket::tcpDisConnect(int m_pSocket)
        {
            closesocket(m_pSocket);
            m_bConnected_ = false;
            return true;
        }

        /// @brief reateSocket()用于TCP 重连。
        void InputTcpSocket::reconnect()
        {
            while (!to_exit_recv_)    //init_flag_
            {
                Sleep(10000);
                if(m_bConnected_)
                    continue;

                tcpDisConnect(fds_);
                Sleep(2000);
                fds_ = createTcpSocket(input_param_.lidar_msop_port, input_param_.lidar_address, 
                                                input_param_.host_msop_port, input_param_.host_address);
                if (fds_ == -1)
                {
                    WJ_WARNING << "connect fail!" << WJ_REND;
                }
                else 
                {
                    WJ_INFO << "connect succ!" << WJ_REND;
                }

            }
        }

        int32 InputTcpSocket::send_(uint8* buf,uint32 size)
        {
            int32 ret;
            struct sockaddr_in addr;
            socklen_t addrLen = sizeof(struct sockaddr_in);
            memset(&addr,0,addrLen);
            addr.sin_family = AF_INET;
            addr.sin_port = htons(input_param_.lidar_msop_port);

            if(inet_pton(AF_INET,input_param_.lidar_address.c_str(),&addr.sin_addr) <= 0)
            {
                WJ_WARNING << "Invalid LiDAR Ip address." << WJ_REND;
                return -1;
            }

            if(m_bConnected_ && fds_ > 0)
            {
                ret = send(fds_, (const char *)buf, size, 0);
            }

            if(ret == -1)
            {
                WJ_WARNING << "wait connect!" << WJ_REND;
                m_bConnected_ = false;
            }

            return ret;
        }

        inline void InputTcpSocket::recvPacket()
        {
            int wait_outtime_num = 0;
            int max_fd =fds_;

            while (!to_exit_recv_)
            {
                fd_set rfds;
                FD_ZERO(&rfds);
                FD_SET(fds_, &rfds);

                struct timeval tv;
                tv.tv_sec = 1;
                tv.tv_usec = 0;
                int retval = select(max_fd + 1, &rfds, NULL, NULL, &tv);
                if (retval == 0)
                {
                    cb_excep_(Error(ERRCODE_MSOPTIMEOUT));
                    if(wait_outtime_num++ > 10)
                    {
                        m_bConnected_ = false;
                        wait_outtime_num = 0;
                    }
                    continue;
                }
                else if (retval < 0)
                {
                    if (errno == EINTR || !m_bConnected_)
                    {
                        Sleep(1000);
                        continue;
                    }

                    perror("select: ");
                    break;
                }

                if ((fds_ >= 0) && FD_ISSET(fds_, &rfds))
                {
                    std::shared_ptr<Buffer> pkt = cb_get_pkt_(pkt_buf_len_);
                    int ret = recv(fds_, (char*)pkt->buf(), pkt->bufSize(), 0);

                    if (ret < 0)
                    {
                        perror("recv: ");
                    }
                    else if (ret > 0)
                    {
                        pkt->setData(sock_offset_, ret - sock_offset_ - sock_tail_);
                        pushPacket(pkt);
                    }
                }
            }
        }
    }  
}  

#else
#ifdef ENABLE_EPOLL_RECEIVE
#include <vanjee_driver/driver/input/input.hpp>
#include <vanjee_driver/common/super_header.hpp>

#include <unistd.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/epoll.h>

namespace vanjee
{
    namespace lidar
    {
        class InputTcpSocket : public Input
        {
        protected:
            size_t pkt_buf_len_;
            size_t socket_offset_;
            size_t socket_tail_;
            int fds_;
            int epfd_;
            bool m_bConnected_ = false;
            
        private:
            inline void recvPacket();
            /// @brief 用于创建TCP Socket
            inline int createTcpSocket(uint16_t lidarPort, const std::string &lidarIp, uint16_t hostPort, const std::string &hostIp);
            inline bool tcpDisConnect(int m_pSocket);
            void reconnect();

            inline std::string getIpAddr(const struct sockaddr_in& addr);
            std::thread reconnect_thread_;

        public:
            InputTcpSocket(const WJInputParam &input_param);
            /// @brief 调用createSocket()，创建两个Socket,分别接收MSOP Packet和DIFOP Packet
            virtual bool init();
            /// @brief 启动接收线程，线程函数为InputSock::recvPacket()00
            virtual bool start();
            virtual int32 send_(uint8* buf,uint32 size);
            virtual ~InputTcpSocket();
        };

        InputTcpSocket::InputTcpSocket(const WJInputParam &input_param)
            : Input(input_param), pkt_buf_len_(ETH_LEN),
              socket_offset_(0), socket_tail_(0)
        {
            socket_offset_ += input_param.user_layer_bytes;
            socket_tail_ += input_param.tail_layer_bytes;
        }

        inline bool InputTcpSocket::init()
        {
            if (init_flag_)
                return true;
                
            int msop_fd = -1;
            int epfd = epoll_create(1);
            if(epfd < 0)
                goto faileEpfd;

            msop_fd = createTcpSocket(input_param_.lidar_msop_port, input_param_.lidar_address, input_param_.host_msop_port, input_param_.host_address);
            if (msop_fd < 0)
                goto failMsop;

            struct epoll_event ev;
            ev.data.fd = msop_fd;
            ev.events = EPOLLIN;
            epoll_ctl(epfd, EPOLL_CTL_ADD, msop_fd, &ev);
            epfd_ = epfd;
            fds_ = msop_fd;
            init_flag_ = true;
            return true;

            failMsop:
            faileEpfd:
                return false;
        }

        inline bool InputTcpSocket::start()
        {
            if (start_flag_)
                return true;

            if (!init_flag_)
            {
                cb_excep_(Error(ERRCODE_STARTBEFOREINIT));
                return false;
            }
            to_exit_recv_ = false;
            recv_thread_ = std::thread(std::bind(&InputTcpSocket::recvPacket, this));
            reconnect_thread_ = std::thread(std::bind(&InputTcpSocket::reconnect, this));
            start_flag_ = true;
            return true;
        }
        
        InputTcpSocket::~InputTcpSocket()
        {
            stop();
            close(fds_);
        }

        /// @brief reateSocket()用于创建TCP Socket。
        inline int InputTcpSocket::createTcpSocket(uint16_t lidarPort, const std::string &lidarIp, uint16_t hostPort, const std::string &hostIp)
        {
            int fd;
            int ret;
            int reuse = 1;

            if (hostIp == "0.0.0.0" || lidarIp == "0.0.0.0")
            { 
                perror("ip err: ");
                goto failSocket;
            }

            fd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
            if (fd < 0)
            {
                perror("socket: ");
                goto failSocket;
            }

            ret = setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
            if (ret < 0)
            {
                perror("setsockopt(SO_REUSEADDR): ");
                goto failOption;
            }

            {
                sleep(1);
                int set_rcv_size = 1024*1024;//设置为32K
                int set_optlen=sizeof(set_rcv_size);
                ret = setsockopt(fd,SOL_SOCKET,SO_RCVBUF,(const int*)&set_rcv_size,set_optlen);
            }

            struct sockaddr_in host_addr;
            memset(&host_addr, 0, sizeof(host_addr));
            host_addr.sin_family = AF_INET;         
            host_addr.sin_port = htons(hostPort);       
            host_addr.sin_addr.s_addr = INADDR_ANY; 
            inet_pton(AF_INET, hostIp.c_str(), &(host_addr.sin_addr));

            /// @brief 给 socket 绑定一个地址结构 (IP +port)
            ret = bind(fd, (struct sockaddr *)&host_addr, sizeof(host_addr));
            if (ret < 0)
            {
                perror("bind: ");
                goto failBind;
            }
 
#ifdef ENABLE_DOUBLE_RCVBUF
            {
                uint32_t opt_val;  
                socklen_t opt_len; 
                getsockopt(fd, SOL_SOCKET, SO_RCVBUF, (char *)&opt_val, &opt_len);
                opt_val *= 4;
                setsockopt(fd, SOL_SOCKET, SO_RCVBUF, (char *)&opt_val, opt_len);
            }
#endif
            struct sockaddr_in lidar_addr;
            memset(&lidar_addr, 0, sizeof(lidar_addr));
            lidar_addr.sin_family = AF_INET;         
            lidar_addr.sin_port = htons(lidarPort);       
            lidar_addr.sin_addr.s_addr = INADDR_ANY; 
            inet_pton(AF_INET, lidarIp.c_str(), &(lidar_addr.sin_addr));

            ret = connect(fd, (struct sockaddr *)&lidar_addr, sizeof(lidar_addr));
            if (ret < 0)
            {
                perror("connect fail: ");
                goto failConnect;
            }
            m_bConnected_ = true;
            return fd;

            failOption:
            failBind:
            failConnect:
                close(fd);
            failSocket:
                return -1;
        }

        /// @brief reateSocket()用于TCP Disconnect。
        inline bool InputTcpSocket::tcpDisConnect(int m_pSocket)
        {
            close(m_pSocket);
            m_bConnected_ = false;
            return true;
        }

        /// @brief reateSocket()用于TCP 重连。
        void InputTcpSocket::reconnect()
        {
            while (!to_exit_recv_)    //init_flag_
            {
                sleep(10);
                if(m_bConnected_)
                    continue;

                tcpDisConnect(fds_);
                sleep(2);
                fds_ = createTcpSocket(input_param_.lidar_msop_port, input_param_.lidar_address, 
                                                    input_param_.host_msop_port, input_param_.host_address);
                if (fds_ == -1)
                    WJ_WARNING << "connect fail!" << WJ_REND;
                else 
                {
                    int epfd = epoll_create(1);
                    if(epfd < 0)
                    {
                        WJ_WARNING << "connect fail!" << WJ_REND;
                        continue;
                    }
                    
                    struct epoll_event ev;
                    ev.data.fd = fds_;
                    ev.events = EPOLLIN; 
                    epoll_ctl(epfd, EPOLL_CTL_ADD, fds_, &ev);
                    epfd_ = epfd;
                    WJ_INFO << "connect succ!" << WJ_REND;
                }
                
            }
        }

        inline std::string InputTcpSocket::getIpAddr(const struct sockaddr_in& addr)
        {
            char ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &(addr.sin_addr), ip, INET_ADDRSTRLEN);
            return std::string(ip);
        }

        int32 InputTcpSocket::send_(uint8* buf,uint32 size)
        {
            int32 ret;
            struct sockaddr_in addr;
            socklen_t addrLen = sizeof(struct sockaddr_in);
            memset(&addr,0,addrLen);
            addr.sin_family = AF_INET;
            addr.sin_port = htons(input_param_.lidar_msop_port);


            if(inet_pton(AF_INET,input_param_.lidar_address.c_str(),&addr.sin_addr) <= 0)
            {
                WJ_WARNING << "Invalid LiDAR Ip address." << WJ_REND;
                return -1;
            }

            if(m_bConnected_ && fds_ > 0)
            {
                ret = send(fds_, buf, size, 0);
            }
            
            if(ret == -1)
            {
                WJ_WARNING << "wait connect!" << WJ_REND;
                m_bConnected_ = false;
            }

            return ret;
        }

        inline void InputTcpSocket::recvPacket()
        {
            int wait_outtime_num = 0;
            int max_fd = fds_;
            while (!to_exit_recv_)
            {
                struct epoll_event events[8]; 
                int retval = epoll_wait(epfd_, events, 8, 1000);
                if (retval == 0)
                {
                    cb_excep_(Error(ERRCODE_MSOPTIMEOUT));
                    if(wait_outtime_num++ > 10)
                    {
                        m_bConnected_ = false;
                        wait_outtime_num = 0;
                    }
                }
                else if (retval < 0)
                {
                    if (errno == EINTR || !m_bConnected_)
                    {
                        sleep(1);
                        continue;
                    }
                    perror("epoll_wait: ");
                    break;
                }

                for(int i = 0; i < retval; i++)
                {
                    /// @brief 用于从(已连接)套接口上接收数据，并捕获数据发送源的地址
                    std::shared_ptr<Buffer> pkt = cb_get_pkt_(pkt_buf_len_);

                    struct sockaddr_in addr;
                    socklen_t addrLen = sizeof(struct sockaddr_in);
                    ssize_t ret = recv(events[i].data.fd, pkt->buf(), pkt->bufSize(), 0);
                    if (ret < 0)
                    {
                        perror("recv: ");
                    }
                    else if (ret > 0)
                    {
                        pkt->setData(socket_offset_, ret - socket_offset_ - socket_tail_,getIpAddr(addr));
                        pushPacket(pkt);
                    }
                }
            }
        }
    } 
} 
#else
#include <vanjee_driver/driver/input/input.hpp>
#include <vanjee_driver/common/super_header.hpp>

#include <unistd.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>

namespace vanjee
{
    namespace lidar
    {
        class InputTcpSocket : public Input
        {
        protected:
            size_t pkt_buf_len_;
            size_t socket_offset_;
            size_t socket_tail_;
            int fds_;
            bool m_bConnected_ = false;
            

        private:
            inline void recvPacket();
            inline int createTcpSocket(uint16_t lidarPort, const std::string &lidarIp, uint16_t hostPort, const std::string &hostIp);
            inline bool tcpDisConnect(int m_pSocket);
            void reconnect();
            inline std::string getIpAddr(const struct sockaddr_in& addr);
            std::thread reconnect_thread_;

        public:
            InputTcpSocket(const WJInputParam &input_param);
            /// @brief 调用createSocket()，创建两个Socket,分别接收MSOP Packet和DIFOP Packet
            virtual bool init();
            /// @brief 启动接收线程，线程函数为InputSock::recvPacket()00
            virtual bool start();
            virtual int32 send_(uint8* buf,uint32 size);
            virtual ~InputTcpSocket();
        };

        InputTcpSocket::InputTcpSocket(const WJInputParam &input_param)
            : Input(input_param), pkt_buf_len_(ETH_LEN),
              socket_offset_(0), socket_tail_(0)
        {
            socket_offset_ +=input_param.user_layer_bytes;
            socket_tail_ += input_param.tail_layer_bytes;
        }

        inline bool InputTcpSocket::init()
        {
            if (init_flag_)
                return true;

            int msop_fd = -1;
            msop_fd = createTcpSocket(input_param_.lidar_msop_port, input_param_.lidar_address, input_param_.host_msop_port, input_param_.host_address);
            if (msop_fd < 0)
                goto failMsop;

            fds_ = msop_fd;
            init_flag_ = true;
            return true;

            failMsop:
            return false;
        }

        inline bool InputTcpSocket::start()
        {
            if (start_flag_)
                return true;
                
            if (!init_flag_)
            {
                cb_excep_(Error(ERRCODE_STARTBEFOREINIT));
                return false;
            }
            to_exit_recv_ = false;
            recv_thread_ = std::thread(std::bind(&InputTcpSocket::recvPacket, this));
            reconnect_thread_ = std::thread(std::bind(&InputTcpSocket::reconnect, this));
            start_flag_ = true;
            return true;
        }
        
        InputTcpSocket::~InputTcpSocket()
        {
            stop();
            close(fds_);
        }

        /// @brief reateSocket()用于创建TCP Socket。
        inline int InputTcpSocket::createTcpSocket(uint16_t lidarPort, const std::string &lidarIp, uint16_t hostPort, const std::string &hostIp)
        {
            int fd;
            int ret;
            int reuse = 1;
            if (hostIp == "0.0.0.0" || lidarIp == "0.0.0.0")
            { 
                perror("ip err: ");
                goto failSocket;
            }

            fd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
            if (fd < 0)
            {
                perror("socket: ");
                goto failSocket;
            }

            ret = setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
            if (ret < 0)
            {
                perror("setsockopt(SO_REUSEADDR): ");
                goto failOption;
            }

            {
                sleep(1);
                int set_rcv_size = 1024*1024;//设置为32K
                int set_optlen=sizeof(set_rcv_size);
                ret = setsockopt(fd,SOL_SOCKET,SO_RCVBUF,(const int*)&set_rcv_size,set_optlen);
            }

            struct sockaddr_in host_addr;
            memset(&host_addr, 0, sizeof(host_addr));
            host_addr.sin_family = AF_INET;         
            host_addr.sin_port = htons(hostPort);       
            host_addr.sin_addr.s_addr = INADDR_ANY; 
            inet_pton(AF_INET, hostIp.c_str(), &(host_addr.sin_addr));

            /// @brief 给 socket 绑定一个地址结构 (IP +port)
            ret = bind(fd, (struct sockaddr *)&host_addr, sizeof(host_addr));
            if (ret < 0)
            {
                perror("bind: ");
                goto failBind;
            }
 
#ifdef ENABLE_DOUBLE_RCVBUF
            {
                uint32_t opt_val;  
                socklen_t opt_len; 
                getsockopt(fd, SOL_SOCKET, SO_RCVBUF, (char *)&opt_val, &opt_len);
                opt_val *= 4;
                setsockopt(fd, SOL_SOCKET, SO_RCVBUF, (char *)&opt_val, opt_len);
            }
#endif
            struct sockaddr_in lidar_addr;
            memset(&lidar_addr, 0, sizeof(lidar_addr));
            lidar_addr.sin_family = AF_INET;         
            lidar_addr.sin_port = htons(lidarPort);       
            lidar_addr.sin_addr.s_addr = INADDR_ANY; 
            inet_pton(AF_INET, lidarIp.c_str(), &(lidar_addr.sin_addr));

            ret = connect(fd, (struct sockaddr *)&lidar_addr, sizeof(lidar_addr));
            if (ret < 0)
            {
                perror("connect fail: ");
                goto failConnect;
            }
            m_bConnected_ = true;
            return fd;

            failOption:
            failBind:
            failConnect:
                close(fd);
            failSocket:
                return -1;
        }

        /// @brief reateSocket()用于TCP Disconnect。
        inline bool InputTcpSocket::tcpDisConnect(int m_pSocket)
        {
            int ret = close(m_pSocket);
            m_bConnected_ = false;
            return true;
        }

        /// @brief reateSocket()用于TCP 重连。
        void InputTcpSocket::reconnect()
        {
            while (!to_exit_recv_)    //init_flag_
            {
                sleep(10);
                if(m_bConnected_)
                    continue;

                tcpDisConnect(fds_);
                sleep(2);
                fds_ = createTcpSocket(input_param_.lidar_msop_port, input_param_.lidar_address, 
                                                    input_param_.host_msop_port, input_param_.host_address);
                if (fds_ == -1)
                {
                    WJ_WARNING << "connect fail!" << WJ_REND;
                }
                else 
                {
                    WJ_INFO << "connect succ!" << WJ_REND;
                }
            }
        }

        inline std::string InputTcpSocket::getIpAddr(const struct sockaddr_in& addr)
        {
            char ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &(addr.sin_addr), ip, INET_ADDRSTRLEN);
            return std::string(ip);
        }

        int32 InputTcpSocket::send_(uint8* buf,uint32 size)
        {
            int32 ret;
            struct sockaddr_in addr;
            socklen_t addrLen = sizeof(struct sockaddr_in);
            memset(&addr,0,addrLen);
            addr.sin_family = AF_INET;
            addr.sin_port = htons(input_param_.lidar_msop_port);

            if(inet_pton(AF_INET,input_param_.lidar_address.c_str(),&addr.sin_addr) <= 0)
            {
                WJ_WARNING << "Invalid LiDAR Ip address." << WJ_REND;
                return -1;
            }

            if(m_bConnected_ && fds_ > 0)
            {
                ret = send(fds_, buf, size, 0);
            }
            
            if(ret == -1)
            {
                WJ_WARNING << "wait connect!" << WJ_REND;
                m_bConnected_ = false;
            }

            return ret;
        }

        inline void InputTcpSocket::recvPacket()
        {
            int wait_outtime_num = 0;
            int max_fd = fds_;
            while (!to_exit_recv_)
            {
                fd_set rfds;
                FD_ZERO(&rfds);
                FD_SET(fds_, &rfds);
                struct timeval tv;
                tv.tv_sec = 1;
                tv.tv_usec = 0;
                int retval = select(max_fd + 1, &rfds, NULL, NULL, &tv);
                if (retval == 0)
                {
                    cb_excep_(Error(ERRCODE_MSOPTIMEOUT));
                    if(wait_outtime_num++ > 10)
                    {
                        m_bConnected_ = false;
                        wait_outtime_num = 0;
                    }
                }
                else if (retval < 0)
                {
                    if (errno == EINTR || !m_bConnected_)
                    {
                        sleep(1);
                        continue;
                    }
                        
                    perror("select: ");
                    break;
                    
                }

                if ((fds_ >= 0) && FD_ISSET(fds_, &rfds))
                {
                    std::shared_ptr<Buffer> pkt = cb_get_pkt_(pkt_buf_len_);

                    /// @brief 用于从(已连接)套接口上接收数据，并捕获数据发送源的地址
                    struct sockaddr_in addr;
                    socklen_t addrLen = sizeof(struct sockaddr_in);
                    ssize_t ret = recv(fds_, pkt->buf(), pkt->bufSize(), 0);

                    if (ret < 0)
                    {
                        perror("recv: \r\n");
                        //break;
                    }
                    else if (ret > 0)
                    {
                        pkt->setData(socket_offset_, ret - socket_offset_ - socket_tail_,getIpAddr(addr));
                        pushPacket(pkt);
                    }
                }
            }
        }
    } 
} 

#endif
#endif
