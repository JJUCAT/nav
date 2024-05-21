//
// Created by fan on 23-7-3.
//

#include <pluginlib/class_list_macros.hpp>

#include "protocol_winnie.h"
#include "ros_helper.hpp"
#include "sdk_helper.hpp"
#include "winnie_base/IOStates.h"
#include "winnie_base/SetIO.h"
#include "winnie_base/device_basic.hpp"

namespace MessageConverter {

static winnie_base::IOStates to_ros_msg(const winnie_base::cmd_io_info& msg_sdk,
                                        const ros::Time& current_time = ros::Time::now()) {
    winnie_base::IOStates msg_ros_io_status;
    msg_ros_io_status.DI = msg_sdk.DI;
    msg_ros_io_status.DO = msg_sdk.DO;
    msg_ros_io_status.AI.assign(msg_sdk.AI, msg_sdk.AI + sizeof(msg_sdk.AI) / sizeof(msg_sdk.AI[0]));
    msg_ros_io_status.AO.assign(msg_sdk.AO, msg_sdk.AO + sizeof(msg_sdk.AO) / sizeof(msg_sdk.AO[0]));
    return msg_ros_io_status;
}

static std::shared_ptr<winnie_base::cmd_io_control_req> to_sdk_msg(const winnie_base::SetIORequest& msg_ros) {
    auto msg_sdk         = std::make_shared<winnie_base::cmd_io_control_req>();
    msg_sdk->set_DO      = msg_ros.set_DO;
    msg_sdk->set_DO_mask = msg_ros.set_DO_mask;
    int num              = std::min(sizeof(msg_sdk->set_AO) / sizeof(msg_sdk->set_AO[0]), msg_ros.set_AO.size());
    for (int i = 0; i < num; ++i) {
        msg_sdk->set_AO[i] = msg_ros.set_AO[i];
    }
    msg_sdk->set_AO_mask = msg_ros.set_AO_mask;

    return msg_sdk;
}

}  // namespace MessageConverter

namespace winnie_base {

class DeviceIo : public DeviceBasic {
    typedef struct {
        int num_DI;  // DI 数量
        int num_DO;  // DO 数量
        int num_AI;  // AI 数量
        int num_AO;  // AO 数量
    } IoParam;

   public:
    bool OnInit() override {
        ros_handle_->param("num_DI", io_param_.num_DI, 20);
        ros_handle_->param("num_DO", io_param_.num_DO, 20);
        ros_handle_->param("num_AI", io_param_.num_AI, 8);
        ros_handle_->param("num_AO", io_param_.num_AO, 8);

        if (io_param_.num_DI > 20) {
            io_param_.num_DI = 20;
        }
        if (io_param_.num_DO > 20) {
            io_param_.num_DO = 20;
        }
        if (io_param_.num_AI > 8) {
            io_param_.num_AI = 8;
        }
        if (io_param_.num_AO > 8) {
            io_param_.num_AO = 8;
        }

        ros_pub_io_states_.Init(ros_handle_, "/io_states", 1);
        ros_server_set_io_.Init(ros_handle_, "/set_io");
        sdk_sub_io_info_.Init(sdk_handle_, CMD_SET_IO_INFO, 1, WINNIE_ADDRESS, JETSON_ADDRESS);
        sdk_client_io_ctrl_.Init(sdk_handle_, CMD_SET_BATTERY_INFO, 1, JETSON_ADDRESS, WINNIE_ADDRESS);

        sdk_sub_io_info_.register_topic_callback([this](const std::shared_ptr<winnie_base::cmd_io_info>& msg) {
            data_io_info_ = *msg;
            auto msg_imu  = MessageConverter::to_ros_msg(*msg);
            ros_pub_io_states_.publish(msg_imu);
            // PublishStates();
        });

        ros_server_set_io_.register_service_callback(
            [this](winnie_base::SetIO::RequestType& req, winnie_base::SetIO::ResponseType& res) {
                // to_sdk_msg
                auto sdk_req = MessageConverter::to_sdk_msg(req);
                auto sdk_res = std::make_shared<cmd_io_control_res>();

                // call
                res.succeed = sdk_client_io_ctrl_.call(sdk_req, sdk_res, 1000);
                return res.succeed;
            });

        // typedef struct {
        //     uint32_t DI;
        //     uint32_t DO;
        //     float AI[8];
        //     float AO[8];
        // } cmd_io_info;
        // for (int i = 0; i < io_param_.num_DI; ++i) {
        //     std::stringstream ss;
        //     ss << "DI_" << std::setw(2) << std::setfill('0') << i + 1;
        //     AddBitState(ss.str(), data_io_info_.DI, 0x01 << i);
        // }
        // for (int i = 0; i < io_param_.num_DO; ++i) {
        //     std::stringstream ss;
        //     ss << "DO_" << std::setw(2) << std::setfill('0') << i + 1;
        //     AddBitState(ss.str(), data_io_info_.DO, 0x01 << i);
        // }
        //
        // for (int i = 0; i < io_param_.num_AI; ++i) {
        //     std::stringstream ss;
        //     ss << "AI_" << std::setw(2) << std::setfill('0') << i + 1;
        //     AddState(ss.str(), data_io_info_.AI[i]);
        // }
        // for (int i = 0; i < io_param_.num_AO; ++i) {
        //     std::stringstream ss;
        //     ss << "AO_" << std::setw(2) << std::setfill('0') << i + 1;
        //     AddState(ss.str(), data_io_info_.AO[i]);
        // }
        //
        // AddUniversalCtrl([this](const std::string& key, const std::string& val) {
        //     if (key.find("DO_") != std::string::npos) {
        //         int index = std::stoi(key.substr(3));
        //         if (index > 0 && index <= io_param_.num_DO) {
        //             auto sdk_req         = std::make_shared<cmd_io_control_req>();
        //             auto sdk_res         = std::make_shared<cmd_io_control_res>();
        //             sdk_req->set_DO      = std::stoi(val) ? 0xffffffff : 0;
        //             sdk_req->set_DO_mask = 0x01 << (index - 1);
        //             return sdk_client_io_ctrl_.call(sdk_req, sdk_res, 1000);
        //         }
        //     } else if (key.find("AO_") != std::string::npos) {
        //         int index = std::stoi(key.substr(3));
        //         if (index > 0 && index <= io_param_.num_AO) {
        //             auto sdk_req               = std::make_shared<cmd_io_control_req>();
        //             auto sdk_res               = std::make_shared<cmd_io_control_res>();
        //             sdk_req->set_AO[index - 1] = std::stof(val);
        //             sdk_req->set_AO_mask       = 0x01 << (index - 1);
        //             return sdk_client_io_ctrl_.call(sdk_req, sdk_res, 1000);
        //         }
        //     }
        //     return false;
        // });
        //
        // return DeviceHybrid::OnInit();
        return true;
    }

   private:
    IoParam io_param_{};
    cmd_io_info data_io_info_{};

    RosPublisherHelper<winnie_base::IOStates> ros_pub_io_states_;
    RosServiceServerHelper<winnie_base::SetIO> ros_server_set_io_;

    SdkSubscriberHelper<cmd_io_info> sdk_sub_io_info_;
    SdkServiceClientHelper<cmd_io_control_req, cmd_io_control_res> sdk_client_io_ctrl_;
};

}  // namespace winnie_base

PLUGINLIB_EXPORT_CLASS(winnie_base::DeviceIo, winnie_base::DeviceBase)
