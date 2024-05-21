#ifndef NOVATEL_GPS_DRIVER_BINARY_MESSAGE_H
#define NOVATEL_GPS_DRIVER_BINARY_MESSAGE_H

#include <vector>

#include "binary_header.h"

namespace integrated_navigation {
/**
 * Contains the header, raw data bytes, and CRC of a binary NovAtel message.
 */
struct BinaryMessage {
    BinaryHeader header_;
    std::vector<uint8_t> data_;
    uint32_t crc_;
};
}  // namespace integrated_navigation

#endif  // NOVATEL_GPS_DRIVER_BINARY_MESSAGE_H
