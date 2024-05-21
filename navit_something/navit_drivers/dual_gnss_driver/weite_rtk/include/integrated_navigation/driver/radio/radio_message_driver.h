#ifndef SRC_RADIO_MESSAGE_DRIVER_H
#define SRC_RADIO_MESSAGE_DRIVER_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "integrated_navigation/driver/io_common/io_common.h"
#include "navit_msgs/RadioMessage.h"
#include "integrated_navigation/log/logging.h"

#define DEBUG_SENSOR_DRIVER 1

namespace integrated_navigation {
    class radio_message_driver {
    public:
        radio_message_driver(ros::NodeHandle &nh);
        ~radio_message_driver();

        void radio_message_callback(uint8_t *data, const uint16_t crc);

    private:
        uint8_t radio_message_decode(uint8_t *data, const uint16_t crc);
        void radio_message_published();

        ros::Publisher publisher_;

        typedef struct radio {
            uint8_t num_satellites;
            std::string sol_status;
            std::string pos_type;
            std::string battery1;
            std::string battery2;
            std::string base_status;
            std::string radio_status;
            std::string diff_status;
            std::string sat_status;
        } radio_message_;

        radio_message_ radio_;
    };
}
#endif //SRC_RADIO_MESSAGE_DRIVER_H