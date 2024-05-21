/*
 * @Author: guo
 * @Date: 2023-01-31 13:28:24
 * @LastEditors: Guo Xuemei 1242143575@qq.com
 * @LastEditTime: 2023-02-09 13:04:05
 * @FilePath: /vanjee_lidar_sdk_test/src/vanjee_lidar_sdk/src/vanjee_driver/src/vanjee_driver/driver/input/input_factory.hpp
 */
#pragma once
#include <vanjee_driver/driver/input/input.hpp>
#include <vanjee_driver/driver/input/input_socket.hpp>

#ifndef DISABLE_PCAP_PARSE
#include <vanjee_driver/driver/input/input_pcap.hpp>
#endif

namespace vanjee
{
namespace lidar
{
        class InputFactory
        {

        public:
            static std::shared_ptr<Input> createInput(InputType type, const WJInputParam &param,
                                                      double sec_to_delay);
        };

        inline std::shared_ptr<Input> InputFactory::createInput(InputType type, const WJInputParam &param,
                                                                double sec_to_delay)
        {
            std::shared_ptr<Input> input;
            switch (type)
            {
            case InputType::ONLINE_LIDAR:
            {
                if(param.connect_type == 2)
                    input = std::make_shared<InputTcpSocket>(param);
                else
                    input = std::make_shared<InputUdpSocket>(param);
            }
            break;
            
#ifndef DISABLE_PCAP_PARSE
            case InputType::PCAP_FILE:
            {
                input = std::make_shared<InputPcap>(param, sec_to_delay);
            }
            break;

#endif
            default:
                WJ_ERROR << "Wrong Input Type " << type << "." << WJ_REND;
                if (type == InputType::PCAP_FILE)
                {
                    WJ_ERROR << "To use InputType::PCAP_FILE, please do not specify the make option DISABLE_PCAP_PARSE." << WJ_REND;
                }
                exit(-1);
            }
            return input;       
        }
} 
} 
