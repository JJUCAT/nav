/*
 * @Author: guo
 * @Date: 2023-01-18 18:43:55
 * @LastEditors: Guo Xuemei 1242143575@qq.com
 * @LastEditTime: 2023-02-08 14:36:35
 * @FilePath: /vanjee_lidar_sdk/src/vanjee_lidar_sdk/src/vanjee_driver/src/vanjee_driver/common/wj_log.hpp
 */
#pragma once
#include <iostream>
#define WJ_ERROR std::cout << "\033[1m\033[31m"   
#define WJ_WARNING std::cout << "\033[1m\033[33m" 
#define WJ_INFO std::cout << "\033[1m\033[32m"    
#define WJ_INFOL std::cout << "\033[32m"          
#define WJ_DEBUG std::cout << "\033[1m\033[36m"   

#define WJ_TITLE std::cout << "\033[1m\033[35m" 
#define WJ_MSG std::cout << "\033[1m\033[37m"   
#define WJ_REND "\033[0m" << std::endl 