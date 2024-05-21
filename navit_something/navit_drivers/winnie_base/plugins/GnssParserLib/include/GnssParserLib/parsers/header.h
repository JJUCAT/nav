#ifndef NOVATEL_GPS_DRIVER_HEADER_H
#define NOVATEL_GPS_DRIVER_HEADER_H

#include <GnssParserLib/parsers/message_parser.h>
#include <GnssParserLib/parsers/parsing_utils.h>
#include <navit_msgs/NovatelMessageHeader.h>

namespace integrated_navigation {
class HeaderParser : public MessageParser<navit_msgs::NovatelMessageHeader> {
   public:
    uint32_t GetMessageId() const override;

    const std::string GetMessageName() const override;

    navit_msgs::NovatelMessageHeader ParseBinary(const BinaryMessage &bin_msg) noexcept(false) override;

    navit_msgs::NovatelMessageHeader ParseAscii(const NovatelSentence &sentence) noexcept(false) override;

    static constexpr uint32_t BINARY_HEADER_LENGTH = 28;
};
}  // namespace integrated_navigation

#endif  // NOVATEL_GPS_DRIVER_HEADER_H
