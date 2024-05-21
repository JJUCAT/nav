//
// Created by fan on 23-7-3.
//

#ifndef WINNIE_BASE_DEVICE_HYBRID_HPP
#define WINNIE_BASE_DEVICE_HYBRID_HPP

#include <component_msgs/ComponentValue.h>
#include <component_msgs/ItemValue.h>
#include <component_msgs/SetItem.h>

#include <winnie_base/device_base.hpp>

namespace winnie_base {

class DeviceHybrid : public DeviceBase {
   public:
    inline static const std::string UNIVERSAL = "UNIVERSAL";
    using GetStateValFunc                     = std::function<std::string()>;
    using GetFaultValFunc                     = std::function<bool()>;
    using CtrlFunc                            = std::function<bool(const std::string&, const std::string&)>;

    bool OnInit() override {
        ros::NodeHandle nh(device_name_);

        if (!get_state_val_funcs_.empty()) {
            ros_status_publisher_ = nh.advertise<component_msgs::ComponentValue>("state", 10);
        }

        if (!get_fault_val_funcs_.empty()) {
            ros_fault_publisher_ = nh.advertise<component_msgs::ComponentValue>("fault", 10);
        }

        if (!ctrl_funcs_.empty()) {
            ros_ctrl_server_ = nh.advertiseService("ctrl", &DeviceHybrid::OnCtrl, this);
        }

        return true;
    }

    void AddCtrl(const std::string& name, const CtrlFunc& ctrl_func) { ctrl_funcs_[name] = ctrl_func; }
    void AddUniversalCtrl(const CtrlFunc& ctrl_func) { AddCtrl(UNIVERSAL, ctrl_func); }

    template <typename T>
    void AddState(const std::string& name, const T& state) {
        get_state_val_funcs_[name] = [&]() { return std::to_string(state); };
    }
    template <typename T>
    void AddBitState(const std::string& name, const T& state,
                     uint64_t bit_mask = std::numeric_limits<uint64_t>::max()) {
        get_state_val_funcs_[name] = [&]() { return (state & bit_mask) ? "1" : "0"; };
    }

    template <typename T>
    void AddFault(const std::string& name, const T& state, uint64_t bit_mask = std::numeric_limits<uint64_t>::max()) {
        get_fault_val_funcs_[name] = [&]() { return state & bit_mask; };
    }

    void PublishStates() {
        component_msgs::ComponentValue msg;
        msg.component_name = device_name_;
        for (auto& [name, item] : get_state_val_funcs_) {
            component_msgs::ItemValue itemValue;
            itemValue.item_name  = name;
            itemValue.item_value = item();
            msg.item_values.emplace_back(itemValue);
        }
        if (!msg.item_values.empty()) {
            ros_status_publisher_.publish(msg);
        }
    }

    void PublishFaults() {
        component_msgs::ComponentValue msg;
        msg.component_name = device_name_;
        for (auto& [name, item] : get_fault_val_funcs_) {
            bool fault_current_value = item();
            if (fault_current_value == fault_last_value_[name]) {
                continue;
            }

            component_msgs::ItemValue itemValue;
            itemValue.item_name  = name;
            itemValue.item_value = fault_current_value ? "1" : "0";
            msg.item_values.emplace_back(itemValue);
            fault_last_value_[name] = fault_current_value;
        }
        if (!msg.item_values.empty()) {
            ros_fault_publisher_.publish(msg);
        }
    }

   protected:
    bool OnCtrl(component_msgs::SetItemRequest& request, component_msgs::SetItemResponse& response) {
        for (auto& item : request.item_values) {
            ROS_INFO("Device:[%s] set item:%s value:%s", device_name_.c_str(), item.item_name.c_str(),
                     item.item_value.c_str());
            if (ctrl_funcs_.find(item.item_name) != ctrl_funcs_.end()) {
                if (!ctrl_funcs_[item.item_name](item.item_name, item.item_value)) {
                    response.code    = -1;
                    response.message = "set item:" + item.item_name + " value:" + item.item_value + " failed.";
                    return false;
                }
            } else if (ctrl_funcs_.find(UNIVERSAL) != ctrl_funcs_.end()) {
                if (!ctrl_funcs_[UNIVERSAL](item.item_name, item.item_value)) {
                    response.code    = -1;
                    response.message = "set item:" + item.item_name + " value:" + item.item_value + " failed.";
                    return false;
                }
            } else {
                response.code    = -1;
                response.message = "unsupported ctrl for item:" + item.item_name;
                return false;
            }
        }
        response.code = 0;
        return true;
    }

   private:
    ros::Publisher ros_status_publisher_;
    ros::Publisher ros_fault_publisher_;
    ros::ServiceServer ros_ctrl_server_;

    std::map<std::string, GetStateValFunc> get_state_val_funcs_;
    std::map<std::string, CtrlFunc> ctrl_funcs_;

    std::map<std::string, GetFaultValFunc> get_fault_val_funcs_;
    std::map<std::string, bool> fault_last_value_;
};

}  // namespace winnie_base

#endif  // WINNIE_BASE_DEVICE_HYBRID_HPP
