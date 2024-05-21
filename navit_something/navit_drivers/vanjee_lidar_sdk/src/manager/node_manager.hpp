/*
 * @Author: guo
 * @Date: 2023-02-07 11:29:25
 * @LastEditors: Guo Xuemei 1242143575@qq.com
 * @LastEditTime: 2023-02-08 14:33:42
 * @FilePath: /vanjee_lidar_sdk_test/src/vanjee_lidar_sdk/src/manager/node_manager.hpp
 */
#pragma once
#include "utility/yaml_reader.hpp"
#include "source/source.hpp"

namespace vanjee
{
namespace lidar
{
    class NodeManager
    {
    public:
        void init(const YAML::Node &config);
        void start();
        void stop();

        ~NodeManager();
        NodeManager() = default;

    private:
        std::vector<Source::Ptr> sources_;
    };
}     

} 
