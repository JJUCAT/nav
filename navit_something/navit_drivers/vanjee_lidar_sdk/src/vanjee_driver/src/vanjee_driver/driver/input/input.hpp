﻿/*
 * @Author: guo
 * @Date: 2023-01-29 13:14:50
 * @LastEditors: Guo Xuemei 1242143575@qq.com
 * @LastEditTime: 2023-02-27 21:08:15
 * @FilePath: /vanjee_lidar_sdk_test/src/vanjee_lidar_sdk/src/vanjee_driver/src/vanjee_driver/driver/input/input.hpp
 */
#pragma once

#include <vanjee_driver/utility/buffer.hpp>
#include <vanjee_driver/driver/driver_param.hpp>
#include <vanjee_driver/common/error_code.hpp>
#include <vanjee_driver/common/super_header.hpp>

#include <functional>
#include <thread>
#include <cstring>

#define VLAN_HDR_LEN 4
#define ETH_HDR_LEN 42
#define ETH_LEN (ETH_HDR_LEN + VLAN_HDR_LEN + 1500)
#define IP_LEN 65536
#define UDP_HDR_LEN 8

namespace vanjee
{
    namespace lidar
    {
        class Input
        {
        public:
            /// @brief 基类
            Input(const WJInputParam &input_param);
            /// @brief 提供两个回调函数
            inline void regCallback(const std::function<void(const Error &)> &cb_excep,
                                    const std::function<std::shared_ptr<Buffer>(size_t)> &cb_get_pkt,
                                    const std::function<void(std::shared_ptr<Buffer>, bool)> &cb_put_pkt);
            virtual bool init() = 0;
            virtual bool start() = 0;
            virtual bool stop();
            virtual int32 send_(uint8* buf,uint32 size);

            virtual ~Input()
            {
            }
            /// @brief 派生类可以访问
        protected:
            /// @brief 填充数据
            inline void pushPacket(std::shared_ptr<Buffer> &pkt, bool stuffed = true);

            WJInputParam input_param_;
            std::function<std::shared_ptr<Buffer>(size_t size)> cb_get_pkt_;
            std::function<void(std::shared_ptr<Buffer>, bool)> cb_put_pkt_;
            std::function<void(const Error &)> cb_excep_;
            std::thread recv_thread_; 
            bool to_exit_recv_;       
            bool init_flag_;          
            bool start_flag_;         
        };

        Input::Input(const WJInputParam &input_param)
            : input_param_(input_param), to_exit_recv_(false), init_flag_(false), start_flag_(false)
        {
            
        }
        inline void Input::regCallback(const std::function<void(const Error &)> &cb_excep,
                                       const std::function<std::shared_ptr<Buffer>(size_t)> &cb_get_pkt,
                                       const std::function<void(std::shared_ptr<Buffer>, bool)> &cb_put_pkt)
        {
            cb_excep_ = cb_excep;
            cb_get_pkt_ = cb_get_pkt;
            cb_put_pkt_ = cb_put_pkt;
        }
        inline bool Input::stop()
        {
            if (start_flag_)
            {
                to_exit_recv_ = true;
                recv_thread_.join();

                start_flag_ = false;
            }
            return true;
        }
        inline void Input::pushPacket(std::shared_ptr<Buffer> &pkt, bool stuffed)
        {
            cb_put_pkt_(pkt, stuffed);
        }

        int32 Input::send_(uint8* buf,uint32 size)
        {
            WJ_INFO << "Default send_,do nothing..." <<WJ_REND;
            return -1;
        }

    } 
} 
