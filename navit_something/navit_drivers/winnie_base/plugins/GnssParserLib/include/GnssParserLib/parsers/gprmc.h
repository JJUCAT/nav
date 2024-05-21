#ifndef NOVATEL_GPS_DRIVER_GPRMC_H
#define NOVATEL_GPS_DRIVER_GPRMC_H

#include <GnssParserLib/parsers/message_parser.h>
#include <navit_msgs/Gprmc.h>

namespace integrated_navigation {
class GprmcParser : public MessageParser<navit_msgs::GprmcPtr> {
   public:
    GprmcParser() : MessageParser<navit_msgs::GprmcPtr>(), was_last_gps_valid_(false) {}

    uint32_t GetMessageId() const override;

    const std::string GetMessageName() const override;

    navit_msgs::GprmcPtr ParseAscii(const NmeaSentence &sentence) noexcept(false) override;

    bool WasLastGpsValid() const;

    static const std::string MESSAGE_NAME;
    static constexpr double KNOTS_TO_MPS = 0.5144444;

   private:
    bool was_last_gps_valid_;
};
}  // namespace integrated_navigation
#endif  // NOVATEL_GPS_DRIVER_GPRMC_H