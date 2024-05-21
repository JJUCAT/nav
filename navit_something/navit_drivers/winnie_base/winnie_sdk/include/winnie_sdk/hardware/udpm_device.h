#ifndef WINNIE_SDK_UDPM_DEVICE_H
#define WINNIE_SDK_UDPM_DEVICE_H

#include <arpa/inet.h>
#include <sys/socket.h>

#include <cstring>
#include <iostream>

#include "hardware_interface.h"

namespace winnie_sdk {

class UdpmDevice : public HardwareInterface {
   public:
    UdpmDevice(const char *ip, uint16_t port) : ip_(ip), port_(port) {}
    ~UdpmDevice() override { Close(); }

    bool Init() override {
        printf("Init UdpDevice ip:%s port:%d\n", ip_, port_);

        // Create a UDP socket
        send_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (send_fd_ == -1) {
            perror("socket send_fd_");
            Close();
            return false;
        }

        unsigned int send_lo_opt = 1;
        if (setsockopt(send_fd_, IPPROTO_IP, IP_MULTICAST_LOOP, (char *)&send_lo_opt, sizeof(send_lo_opt)) < 0) {
            perror("setsockopt (send_fd_, IPPROTO_IP, IP_MULTICAST_LOOP)");
            Close();
            return false;
        }

        // don't start the receive thread yet.  Only allocate resources for
        // receiving messages when a subscription is made.
        // However, we still need to setup sendfd in multi-cast group
        struct ip_mreq mreq {};
        mreq.imr_multiaddr.s_addr = inet_addr(ip_);
        mreq.imr_interface.s_addr = INADDR_ANY;
        if (setsockopt(send_fd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&mreq, sizeof(mreq)) < 0) {
            perror("setsockopt (send_fd_, IPPROTO_IP, IP_ADD_MEMBERSHIP)");
            Close();
            return false;
        }

        // ------------------------------------------------------------ //

        // allocate multicast socket
        recv_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (recv_fd_ < 0) {
            perror("socket recv_fd_");
            Close();
            return false;
        }

        // Set the reuse option
        int optval = 1;
        if (setsockopt(recv_fd_, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) == -1) {
            perror("setsockopt (recv_fd_, SOL_SOCKET, SO_REUSEADDR)");
            Close();
            return false;
        }

        // Bind to the multicast address and port
        memset(&addr_, 0, sizeof(addr_));
        addr_.sin_family      = AF_INET;
        addr_.sin_addr.s_addr = inet_addr(ip_);
        addr_.sin_port        = htons(port_);
        if (bind(recv_fd_, (struct sockaddr *)&addr_, sizeof(addr_)) == -1) {
            perror("bind (recv_fd_)");
            Close();
            return false;
        }

        // Join the multicast group on the specified interface
        if (setsockopt(recv_fd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) == -1) {
            perror("setsockopt (recv_fd_, IPPROTO_IP, IP_ADD_MEMBERSHIP)");
            Close();
            return false;
        }

        printf("Init UdpDevice ip:%s port:%d succeed.\n", ip_, port_);
        return true;
    }

    size_t Read(uint8_t *buf, size_t len) override {
        struct sockaddr_in src_addr {};
        socklen_t addrlen = sizeof(src_addr);
        size_t nbytes     = recvfrom(recv_fd_, buf, len, 0, (struct sockaddr *)&src_addr, &addrlen);
        if (nbytes == -1) {
            perror("recvfrom");
            return -errno;
        }
        return nbytes;
    }

    size_t Write(const uint8_t *buf, size_t len) override {
        size_t nbytes = sendto(send_fd_, buf, len, 0, (struct sockaddr *)&addr_, sizeof(addr_));
        if (nbytes == -1) {
            perror("sendto");
            return -errno;
        }
        return nbytes;
    }

   private:
    void Close() {
        if (send_fd_ != -1) {
            close(send_fd_);
            send_fd_ = -1;
        }

        if (recv_fd_ != -1) {
            close(recv_fd_);
            recv_fd_ = -1;
        }
    }

    const char *ip_;
    uint16_t port_;
    int send_fd_ = -1;
    int recv_fd_ = -1;
    struct sockaddr_in addr_ {};
};

}  // namespace winnie_sdk

#endif