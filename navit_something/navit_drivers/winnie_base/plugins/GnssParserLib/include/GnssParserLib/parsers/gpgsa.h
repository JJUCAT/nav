#ifndef NOVATEL_GPS_DRIVER_GPGSA_H
#define NOVATEL_GPS_DRIVER_GPGSA_H

#include <GnssParserLib/parsers/message_parser.h>
#include <navit_msgs/Gpgsa.h>

namespace integrated_navigation {
class GpgsaParser : public MessageParser<navit_msgs::GpgsaPtr> {
   public:
    uint32_t GetMessageId() const override;

    const std::string GetMessageName() const override;

    navit_msgs::GpgsaPtr ParseAscii(const NmeaSentence& sentence) noexcept(false) override;

    static const std::string MESSAGE_NAME;
};
};      // namespace integrated_navigation
#endif  // NOVATEL_GPS_DRIVER_GPGSA_H