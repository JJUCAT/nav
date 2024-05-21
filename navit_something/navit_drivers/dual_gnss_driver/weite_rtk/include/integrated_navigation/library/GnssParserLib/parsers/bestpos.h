#ifndef NOVATEL_GPS_DRIVER_BESTPOS_H
#define NOVATEL_GPS_DRIVER_BESTPOS_H

#include <integrated_navigation/library/GnssParserLib/parsers/message_parser.h>
#include <integrated_navigation/library/GnssParserLib/parsers/parsing_utils.h>
#include <navit_msgs/NovatelPosition.h>

namespace integrated_navigation {
    class BestposParser : public MessageParser<navit_msgs::NovatelPositionPtr> {
    public:
        uint32_t GetMessageId() const override;

        const std::string GetMessageName() const override;

        navit_msgs::NovatelPositionPtr
        ParseBinary(const BinaryMessage &bin_msg) noexcept(false) override;

        navit_msgs::NovatelPositionPtr
        ParseAscii(const NovatelSentence &sentence) noexcept(false) override;

        static constexpr uint16_t MESSAGE_ID = 42;
        static constexpr size_t BINARY_LENGTH = 72;
        static constexpr size_t ASCII_LENGTH = 21;
        static const std::string MESSAGE_NAME;
    };
} // namespace integrated_navigation
#endif // NOVATEL_GPS_DRIVER_BESTPOS_H