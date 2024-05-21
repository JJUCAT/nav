#ifndef NOVATEL_GPS_DRIVER_DUAL_ANTENNA_HEADING_H
#define NOVATEL_GPS_DRIVER_DUAL_ANTENNA_HEADING_H

#include <integrated_navigation/library/GnssParserLib/parsers/message_parser.h>
#include <integrated_navigation/library/GnssParserLib/parsers/parsing_utils.h>
#include <navit_msgs/NovatelDualAntennaHeading.h>

namespace integrated_navigation {
    class DualAntennaHeadingParser : public MessageParser<navit_msgs::NovatelDualAntennaHeadingPtr> {
    public:
        uint32_t GetMessageId() const override;

        const std::string GetMessageName() const override;

        navit_msgs::NovatelDualAntennaHeadingPtr
        ParseBinary(const BinaryMessage &bin_msg) noexcept(false) override;

        navit_msgs::NovatelDualAntennaHeadingPtr
        ParseAscii(const NovatelSentence &sentence) noexcept(false) override;

        static constexpr uint16_t MESSAGE_ID_482 = 971;
        static constexpr uint16_t MESSAGE_ID_718D = 2042;
        static constexpr size_t BINARY_LENGTH = 44;
        static constexpr size_t ASCII_LENGTH = 17;
        static const std::string MESSAGE_NAME;

    private:
        /*
         * @brief Converts a Solution Source byte mask to the format used in the NovatelDualAntennaHeading ros message
         *
         * ParseException thrown if source_mask cannot be parsed
         * @param[in] source_mask The byte to parse
         * @return NovatelDualAntennaHeading.solution_source enum value
         */
        uint8_t SolutionSourceToMsgEnum(uint8_t source_mask) noexcept(false);
    };
} // namespace integrated_navigation
#endif // NOVATEL_GPS_DRIVER_DUAL_ANTENNA_HEADING_H