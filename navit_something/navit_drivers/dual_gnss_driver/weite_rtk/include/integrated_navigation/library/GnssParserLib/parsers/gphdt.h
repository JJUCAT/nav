#ifndef NOVATEL_GPS_DRIVER_GPHDT_H
#define NOVATEL_GPS_DRIVER_GPHDT_H

#include <integrated_navigation/library/GnssParserLib/parsers/message_parser.h>
#include <navit_msgs/Gphdt.h>

namespace integrated_navigation
{
  class GphdtParser : MessageParser<integrated_navigation::GphdtPtr>
  {
  public:
    uint32_t GetMessageId() const override;

    const std::string GetMessageName() const override;

      integrated_navigation::GphdtPtr ParseAscii(const NmeaSentence& sentence) noexcept(false) override;

    static const std::string MESSAGE_NAME;
  };
}
#endif //NOVATEL_GPS_DRIVER_GPHDT_H