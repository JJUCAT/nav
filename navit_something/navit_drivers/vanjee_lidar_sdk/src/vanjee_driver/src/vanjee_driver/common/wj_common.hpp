/*
 * @Author: guo
 * @Date: 2023-01-19 11:20:59
 * @LastEditors: Guo Xuemei 1242143575@qq.com
 * @LastEditTime: 2023-02-08 13:36:43
 * @FilePath: /vanjee_lidar_sdk_test/src/vanjee_lidar_sdk/src/vanjee_driver/src/vanjee_driver/common/wj_common.hpp
 */

#pragma once

#ifdef _WIN32
#include <ws2tcpip.h>
#else 
#include <arpa/inet.h>
#endif

inline int16_t WJ_SWAP_INT16(int16_t value)
{
  uint8_t *v = (uint8_t *)&value;

  uint8_t temp;
  temp = v[0];
  v[0] = v[1];
  v[1] = temp;

  return value;
}

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES 
#endif

#include <math.h>

#define DEGREE_TO_RADIAN(deg) ((deg)*M_PI / 180)
#define RADIAN_TO_DEGREE(deg) ((deg)*180 / M_PI)
