/*
 * @Author: guo
 * @Date: 2023-01-18 16:24:49
 * @LastEditors: Guo Xuemei 1242143575@qq.com
 * @LastEditTime: 2023-02-08 14:35:43
 * @FilePath: /vanjee_lidar_sdk_test/src/vanjee_lidar_sdk/src/utility/ymal_reader.hpp
 */
#pragma once
#include <yaml-cpp/yaml.h>
#include "utility/common.hpp"
namespace vanjee
{
namespace lidar
{
    template <typename T>
    inline void yamlReadAbort(const YAML::Node &yaml, const std::string &key, T &out_val)
    {
        if (!yaml[key] || yaml[key].Type() == YAML::NodeType::Null)
        {
            exit(-1);
        }
        else
        {
            out_val = yaml[key].as<T>();
        }
    }
    template <typename T>
    inline bool yamlRead(const YAML::Node &yaml, const std::string &key, T &out_val, const T &default_val)
    {
        if (!yaml[key] || yaml[key].Type() == YAML::NodeType::Null)
        {
            out_val = default_val;
            return false;
        }
        else
        {
            out_val = yaml[key].as<T>();
            return true;
        }
    }
    inline YAML::Node yamlSubNodeAbort(const YAML::Node &yaml, const std::string &node)
    {
        YAML::Node ret = yaml[node.c_str()];
        if (!ret)
        {
            exit(-1);
        }
        return ret;
    }

} 
} 
