#ifndef NOVATEL_GPS_DRIVER_BINARY_HEADER_H
#define NOVATEL_GPS_DRIVER_BINARY_HEADER_H

#include <cstdint>

#include "parsers/parsing_utils.h"

namespace integrated_navigation {
/**
 * Represents the header of a binary NovAtel message.
 */
struct BinaryHeader {
    BinaryHeader()
        : sync0_(0xAA),
          sync1_(0x44),
          sync2_(0x12),
          header_length_(0),
          message_id_(0),
          message_type_(0),
          port_address_(0),
          message_length_(0),
          sequence_(0),
          idle_time_(0),
          time_status_(0),
          week_(0),
          gps_ms_(0),
          receiver_status_(0),
          reserved_(0),
          receiver_sw_version_(0) {}

    uint8_t sync0_;
    uint8_t sync1_;
    uint8_t sync2_;
    uint8_t header_length_;
    uint16_t message_id_;
    int8_t message_type_;
    uint8_t port_address_;
    uint16_t message_length_;
    uint16_t sequence_;
    uint8_t idle_time_;
    uint8_t time_status_;
    uint16_t week_;
    uint32_t gps_ms_;
    uint32_t receiver_status_;
    uint16_t reserved_;
    uint16_t receiver_sw_version_;

    void ParseHeader(const uint8_t* data) {
        sync0_               = data[0];
        sync1_               = data[1];
        sync2_               = data[2];
        header_length_       = data[3];
        message_id_          = ParseUInt16(&data[4]);
        message_type_        = data[6];
        port_address_        = data[7];
        message_length_      = ParseUInt16(&data[8]);
        sequence_            = ParseUInt16(&data[10]);
        idle_time_           = data[12];
        time_status_         = data[13];
        week_                = ParseUInt16(&data[14]);
        gps_ms_              = ParseUInt32(&data[16]);
        receiver_status_     = ParseUInt32(&data[20]);
        receiver_sw_version_ = ParseUInt16(&data[26]);
    }
};
}  // namespace integrated_navigation
#endif  // NOVATEL_GPS_DRIVER_BINARY_HEADER_H
