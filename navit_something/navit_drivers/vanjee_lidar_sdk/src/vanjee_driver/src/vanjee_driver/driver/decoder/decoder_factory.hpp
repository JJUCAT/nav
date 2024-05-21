/*
 * @Author: guo
 * @Date: 2023-02-01 17:14:58
 * @LastEditors: Guo Xuemei 1242143575@qq.com
 * @LastEditTime: 2023-02-27 17:05:50
 * @FilePath: /vanjee_lidar_sdk_test/src/vanjee_lidar_sdk/src/vanjee_driver/src/vanjee_driver/driver/decoder/decoder_factory.hpp
 */
#pragma once

#include <vanjee_driver/driver/decoder/decoder.hpp>
#include <vanjee_driver/driver/decoder/wlr718h/decoder/decoder_vanjee_718h.hpp>
#include <vanjee_driver/driver/decoder/wlr719/decoder/decoder_vanjee_719.hpp>
#include <vanjee_driver/driver/decoder/wlr719c/decoder/decoder_vanjee_719c.hpp>
#include <vanjee_driver/driver/decoder/wlr720/decoder/decoder_vanjee_720.hpp>
#include <vanjee_driver/driver/decoder/wlr721/decoder/decoder_vanjee_721.hpp>
#include <vanjee_driver/driver/decoder/wlr733/decoder/decoder_vanjee_733.hpp>
#include <vanjee_driver/driver/decoder/wlr740/decoder/decoder_vanjee_740.hpp>
#include <vanjee_driver/driver/decoder/wlr750/decoder/decoder_vanjee_750.hpp>

namespace vanjee
{
namespace lidar
{ 
    template <typename T_PointCloud>
    class DecoderFactory
    {
    public:
        static std::shared_ptr<Decoder<T_PointCloud>> createDecoder(LidarType type, const WJDecoderParam &param);
    };
    template <typename T_PointCloud>
    inline std::shared_ptr<Decoder<T_PointCloud>> DecoderFactory<T_PointCloud>::createDecoder(LidarType type, const WJDecoderParam &param)
    {
        std::shared_ptr<Decoder<T_PointCloud>> ret_ptr;
        switch (type)
        {
            case LidarType::vanjee_718h:
                ret_ptr = std::make_shared<DecoderVanjee718H<T_PointCloud>>(param);
                break;
            case LidarType::vanjee_719:
                ret_ptr = std::make_shared<DecoderVanjee719<T_PointCloud>>(param);
                break;
            case LidarType::vanjee_719c:
                ret_ptr = std::make_shared<DecoderVanjee719C<T_PointCloud>>(param);
                break;
            case LidarType::vanjee_720:
                ret_ptr = std::make_shared<DecoderVanjee720<T_PointCloud>>(param);
                break;
            case LidarType::vanjee_721:
                ret_ptr = std::make_shared<DecoderVanjee721<T_PointCloud>>(param);
                break;
            case LidarType::vanjee_733:
                ret_ptr = std::make_shared<DecoderVanjee733<T_PointCloud>>(param);
                break;
            case LidarType::vanjee_740:
                ret_ptr = std::make_shared<DecoderVanjee740<T_PointCloud>>(param);
                break;
            case LidarType::vanjee_750:
                ret_ptr = std::make_shared<DecoderVanjee750<T_PointCloud>>(param);
                break;
            
            default:
                exit(-1);
        }
        return ret_ptr;
    }

} 
} 
