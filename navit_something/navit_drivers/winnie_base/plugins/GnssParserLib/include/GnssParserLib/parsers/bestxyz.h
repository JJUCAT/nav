#ifndef NOVATEL_GPS_DRIVER_BESTXYZ_H
#define NOVATEL_GPS_DRIVER_BESTXYZ_H

#include <GnssParserLib/parsers/message_parser.h>
#include <GnssParserLib/parsers/parsing_utils.h>
#include <navit_msgs/NovatelXYZ.h>

namespace integrated_navigation {
class BestxyzParser : public MessageParser<navit_msgs::NovatelXYZPtr> {
   public:
    uint32_t GetMessageId() const override;

    const std::string GetMessageName() const override;

    navit_msgs::NovatelXYZPtr ParseBinary(const BinaryMessage &bin_msg) noexcept(false) override;

    navit_msgs::NovatelXYZPtr ParseAscii(const NovatelSentence &sentence) noexcept(false) override;

    static constexpr uint16_t MESSAGE_ID  = 241;
    static constexpr size_t BINARY_LENGTH = 112;
    static constexpr size_t ASCII_LENGTH  = 28;
    static const std::string MESSAGE_NAME;
};
}  // namespace integrated_navigation

#endif  // NOVATEL_GPS_DRIVER_BESTXYZ_H
