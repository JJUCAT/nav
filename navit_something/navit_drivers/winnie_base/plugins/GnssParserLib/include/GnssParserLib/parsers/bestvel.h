#ifndef NOVATEL_GPS_DRIVER_BESTVEL_H_H
#define NOVATEL_GPS_DRIVER_BESTVEL_H_H

#include <GnssParserLib/parsers/message_parser.h>
#include <navit_msgs/NovatelVelocity.h>

namespace integrated_navigation {
class BestvelParser : public MessageParser<navit_msgs::NovatelVelocityPtr> {
   public:
    uint32_t GetMessageId() const override;

    const std::string GetMessageName() const override;

    navit_msgs::NovatelVelocityPtr ParseBinary(const BinaryMessage &bin_msg) noexcept(false) override;

    navit_msgs::NovatelVelocityPtr ParseAscii(const NovatelSentence &sentence) noexcept(false) override;

    static constexpr uint16_t MESSAGE_ID  = 99;
    static constexpr size_t ASCII_LENGTH  = 8;
    static constexpr size_t BINARY_LENGTH = 44;
    static const std::string MESSAGE_NAME;
};
}  // namespace integrated_navigation
#endif  // NOVATEL_GPS_DRIVER_BESTVEL_H_H