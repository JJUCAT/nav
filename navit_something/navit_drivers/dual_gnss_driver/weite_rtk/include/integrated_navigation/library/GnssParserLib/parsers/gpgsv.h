#ifndef NOVATEL_GPS_DRIVER_GPGSV_H
#define NOVATEL_GPS_DRIVER_GPGSV_H

#include <integrated_navigation/library/GnssParserLib/parsers/message_parser.h>
#include <navit_msgs/Gpgsv.h>

namespace integrated_navigation
{
    class GpgsvParser : MessageParser<integrated_navigation::GpgsvPtr>
    {
    public:
        uint32_t GetMessageId() const override;

        const std::string GetMessageName() const override;

        integrated_navigation::GpgsvPtr ParseAscii(const NmeaSentence& sentence) noexcept(false) override;

        static const std::string MESSAGE_NAME;
    };
}
#endif //NOVATEL_GPS_DRIVER_GPGSV_H