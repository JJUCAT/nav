#ifndef NOVATEL_GPS_DRIVER_CLOCKSTEERING_H
#define NOVATEL_GPS_DRIVER_CLOCKSTEERING_H

#include <GnssParserLib/parsers/message_parser.h>
#include <navit_msgs/ClockSteering.h>

namespace integrated_navigation {
class ClockSteeringParser : MessageParser<navit_msgs::ClockSteeringPtr> {
   public:
    uint32_t GetMessageId() const override;

    const std::string GetMessageName() const override;

    navit_msgs::ClockSteeringPtr ParseAscii(const NovatelSentence& sentence) noexcept(false) override;

    static const std::string MESSAGE_NAME;
};
}  // namespace integrated_navigation
#endif  // NOVATEL_GPS_DRIVER_GPGSV_H