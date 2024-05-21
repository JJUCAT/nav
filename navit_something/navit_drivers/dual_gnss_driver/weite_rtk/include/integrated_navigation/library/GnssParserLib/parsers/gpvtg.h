#ifndef NOVATEL_GPS_DRIVER_GPGGA_H
#define NOVATEL_GPS_DRIVER_GPGGA_H

#include <integrated_navigation/library/GnssParserLib/parsers/message_parser.h>
#include <navit_msgs/Gpvtg.h>

namespace integrated_navigation {
    class GpggaParser : public MessageParser<navit_msgs::GpggaPtr> {
    public:
        GpggaParser() : MessageParser<navit_msgs::GpggaPtr>(), was_last_gps_valid_(false) {}
        uint32_t GetMessageId() const override;

        const std::string GetMessageName() const override;

        navit_msgs::GpggaPtr ParseAscii(const NmeaSentence &sentence) noexcept(false) override;

        bool WasLastGpsValid() const;

        static const std::string MESSAGE_NAME;

    private:
        bool was_last_gps_valid_;
    };
} // namespace integrated_navigation
#endif // NOVATEL_GPS_DRIVER_GPGGA_H