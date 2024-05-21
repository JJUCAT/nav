#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "integrated_navigation/driver/multi_sensor_driver.h"
#include "integrated_navigation/log/logging.h"

namespace integrated_navigation {
    class MultiSensorDriverNodelet : public nodelet::Nodelet {
    public:
        MultiSensorDriverNodelet() {}

        ~MultiSensorDriverNodelet() {}

    private:
        virtual void onInit() {
            NODELET_INFO(" -------------------- Multi Sensor Driver Nodelet On Init ------------------------- ");
            
            // GLogHandle glog_handle("MultiSensorDriverNodelet");
            // google::InitGoogleLogging("MultiSensorDriverNodelet");
            // FLAGS_log_dir          = "/tmp";
            p_.reset(new multi_sensor_driver(getMTNodeHandle()));

        };
        boost::shared_ptr<multi_sensor_driver> p_;
    };
}

// Register nodelet plugin
// parameters: class type, base class type
PLUGINLIB_EXPORT_CLASS(integrated_navigation::MultiSensorDriverNodelet, nodelet::Nodelet)