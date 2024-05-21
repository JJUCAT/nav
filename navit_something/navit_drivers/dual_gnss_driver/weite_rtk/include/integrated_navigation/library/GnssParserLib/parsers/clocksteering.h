#ifndef NOVATEL_GPS_DRIVER_CLOCKSTEERING_H
#define NOVATEL_GPS_DRIVER_CLOCKSTEERING_H

#include <integrated_navigation/library/GnssParserLib/parsers/message_parser.h>
#include <navit_msgs/ClockSteering.h>

namespace integrated_navigation
{
    class ClockSteeringParser : MessageParser<integrated_navigation::ClockSteeringPtr>
    {
    public:
        uint32_t GetMessageId() const override;

        const std::string GetMessageName() const override;

        integrated_navigation::ClockSteeringPtr ParseAscii(const NovatelSentence& sentence) noexcept(false) override;

        static const std::string MESSAGE_NAME;
    };
}
#endif //NOVATEL_GPS_DRIVER_GPGSV_H