#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "integrated_navigation/log/logging.h"
#include "integrated_navigation/driver/ins_sensor_driver.h"

namespace integrated_navigation {
    class InsDriverNodelet : public nodelet::Nodelet {
    public:
        InsDriverNodelet() {}

        ~InsDriverNodelet() {}

    private:
        virtual void onInit() {
            NODELET_INFO(" -------------------- INS Driver Nodelet On Init -------------------------");

            p_.reset(new ins_sensor_driver(getMTNodeHandle()));

        };
        boost::shared_ptr<ins_sensor_driver> p_;
    };
}

// Register nodelet plugin
// parameters: class type, base class type
PLUGINLIB_EXPORT_CLASS(integrated_navigation::InsDriverNodelet, nodelet::Nodelet)