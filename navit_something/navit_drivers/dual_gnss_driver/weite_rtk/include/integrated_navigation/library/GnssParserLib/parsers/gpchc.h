#ifndef NOVATEL_GPS_DRIVER_GPCHC_H
#define NOVATEL_GPS_DRIVER_GPCHC_H

#include <integrated_navigation/library/GnssParserLib/parsers/message_parser.h>
#include <navit_msgs/Gpchc.h>

namespace integrated_navigation
{
  class GpchcParser : public MessageParser<navit_msgs::GpchcPtr>
  {
  public:
    GpchcParser(): MessageParser<navit_msgs::GpchcPtr>(),
                   was_last_gps_valid_(false)
    {}
    uint32_t GetMessageId() const override;

    const std::string GetMessageName() const override;

    navit_msgs::GpchcPtr ParseAscii(const NmeaSentence& sentence) noexcept(false) override;

    bool WasLastGpsValid() const;

    static const std::string MESSAGE_NAME;

  private:
    bool was_last_gps_valid_;
  };
}

#endif //NOVATEL_GPS_DRIVER_GPCHC_H