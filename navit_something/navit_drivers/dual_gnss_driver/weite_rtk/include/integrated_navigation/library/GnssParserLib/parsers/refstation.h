#ifndef SRC_REFSTATION_H
#define SRC_REFSTATION_H

#include <integrated_navigation/library/GnssParserLib/parsers/message_parser.h>
#include <integrated_navigation/library/GnssParserLib/parsers/parsing_utils.h>
#include <navit_msgs/NovatelRefStation.h>

namespace integrated_navigation {
    class RefStationParser : public MessageParser<navit_msgs::NovatelRefStationPtr> {
    public:
        uint32_t GetMessageId() const override;

        const std::string GetMessageName() const override;

        navit_msgs::NovatelRefStationPtr
        ParseAscii(const NovatelSentence &sentence) noexcept(false) override;

        static constexpr uint16_t MESSAGE_ID = 175;
        static constexpr size_t ASCII_LENGTH = 7;
        static const std::string MESSAGE_NAME;
    };
} // namespace integrated_navigation
#endif // SRC_REFSTATION_H