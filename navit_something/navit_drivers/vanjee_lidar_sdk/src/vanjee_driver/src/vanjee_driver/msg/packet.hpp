﻿/*
 * @Author: guo
 * @Date: 2023-01-28 13:40:12
 * @LastEditors: Guo Xuemei 1242143575@qq.com
 * @LastEditTime: 2023-02-08 14:42:51
 * @FilePath: /vanjee_lidar_sdk_test/src/vanjee_lidar_sdk/src/vanjee_driver/src/vanjee_driver/msg/packet.hpp
 */
#pragma once
#include <cstdint>
#include <vector>
#include <string>

namespace vanjee
{
namespace lidar
{
        struct Packet
        {
            double timestamp = 0.0f;    
            uint32_t seq = 0;           
            uint8_t is_difop = 0;       
            uint8_t is_frame_begin = 0; 
            std::vector<uint8_t> buf_;

            Packet(const Packet &msg)
            {
                buf_.assign(msg.buf_.begin(), msg.buf_.end());
            }
            Packet(size_t size = 0)
            {
                buf_.resize(size);
            }
        };

}  
}  
