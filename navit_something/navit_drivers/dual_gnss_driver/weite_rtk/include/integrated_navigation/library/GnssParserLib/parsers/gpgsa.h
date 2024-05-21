#ifndef NOVATEL_GPS_DRIVER_GPGSA_H
#define NOVATEL_GPS_DRIVER_GPGSA_H

#include <integrated_navigation/library/GnssParserLib/parsers/message_parser.h>
#include <navit_msgs/Gpgsa.h>

namespace integrated_navigation
{
    class GpgsaParser : public MessageParser<integrated_navigation::GpgsaPtr>
    {
    public:
        uint32_t GetMessageId() const override;

        const std::string GetMessageName() const override;

        integrated_navigation::GpgsaPtr ParseAscii(const NmeaSentence& sentence) noexcept(false) override;

        static const std::string MESSAGE_NAME;
    };
};
#endif //NOVATEL_GPS_DRIVER_GPGSA_H