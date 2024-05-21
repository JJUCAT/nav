#ifndef NOVATEL_GPS_DRIVER_GPGSV_H
#define NOVATEL_GPS_DRIVER_GPGSV_H

#include <GnssParserLib/parsers/message_parser.h>
#include <navit_msgs/Gpgsv.h>

namespace integrated_navigation {
class GpgsvParser : MessageParser<navit_msgs::GpgsvPtr> {
   public:
    uint32_t GetMessageId() const override;

    const std::string GetMessageName() const override;

    navit_msgs::GpgsvPtr ParseAscii(const NmeaSentence& sentence) noexcept(false) override;

    static const std::string MESSAGE_NAME;
};
}  // namespace integrated_navigation
#endif  // NOVATEL_GPS_DRIVER_GPGSV_H