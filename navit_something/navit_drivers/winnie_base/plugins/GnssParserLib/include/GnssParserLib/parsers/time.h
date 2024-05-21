#ifndef NOVATEL_GPS_DRIVER_TIME_H
#define NOVATEL_GPS_DRIVER_TIME_H

#include <GnssParserLib/parsers/message_parser.h>
#include <navit_msgs/Time.h>

namespace integrated_navigation {
class TimeParser : public MessageParser<navit_msgs::TimePtr> {
   public:
    uint32_t GetMessageId() const override;

    const std::string GetMessageName() const override;

    navit_msgs::TimePtr ParseBinary(const BinaryMessage& bin_msg) noexcept(false) override;

    navit_msgs::TimePtr ParseAscii(const NovatelSentence& sentence) noexcept(false) override;

    static constexpr size_t BINARY_LENGTH = 44;
    static constexpr uint16_t MESSAGE_ID  = 101;
    static constexpr size_t ASCII_FIELD   = 11;
    static const std::string MESSAGE_NAME;
};
}  // namespace integrated_navigation
#endif  // NOVATEL_GPS_DRIVER_TIME_H