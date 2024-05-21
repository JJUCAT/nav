/*
 * @Author: guo
 * @Date: 2023-01-28 15:51:57
 * @LastEditors: Guo Xuemei 1242143575@qq.com
 * @LastEditTime: 2023-02-08 14:43:58
 * @FilePath: /vanjee_lidar_sdk_test/src/vanjee_lidar_sdk/src/vanjee_driver/src/vanjee_driver/utility/dbg.hpp
 */
#pragma once

#include <stdio.h>

namespace vanjee
{
namespace lidar
{
        inline void hexdump(const uint8_t *data, size_t size, const char *desc = NULL)
        {
            printf("\n---------------%s(size:%d)------------------", (desc ? desc : ""), (int)size);

            for (size_t i = 0; i < size; i++)
            {
                if (i % 16 == 0)
                    printf("\n");
                printf("%02x ", data[i]);
            }
            printf("\n---------------------------------\n");
        }
} 
} 
