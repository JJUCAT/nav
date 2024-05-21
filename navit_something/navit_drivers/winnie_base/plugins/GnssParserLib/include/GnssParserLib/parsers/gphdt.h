#ifndef NOVATEL_GPS_DRIVER_GPHDT_H
#define NOVATEL_GPS_DRIVER_GPHDT_H

#include <GnssParserLib/parsers/message_parser.h>
#include <navit_msgs/Gphdt.h>

namespace integrated_navigation {
class GphdtParser : MessageParser<navit_msgs::GphdtPtr> {
   public:
    uint32_t GetMessageId() const override;

    const std::string GetMessageName() const override;

    navit_msgs::GphdtPtr ParseAscii(const NmeaSentence& sentence) noexcept(false) override;

    static const std::string MESSAGE_NAME;
};
}  // namespace integrated_navigation
#endif  // NOVATEL_GPS_DRIVER_GPHDT_H