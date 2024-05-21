#ifndef NOVATEL_GPS_DRIVER_MESSAGE_PARSER_H
#define NOVATEL_GPS_DRIVER_MESSAGE_PARSER_H

#include <integrated_navigation/library/GnssParserLib/binary_message.h>
#include <integrated_navigation/library/GnssParserLib/nmea_sentence.h>
#include <integrated_navigation/library/GnssParserLib/novatel_sentence.h>

#include <integrated_navigation/library/GnssParserLib/parsers/parsing_utils.h>
#include <integrated_navigation/library/GnssParserLib/parsers/parse_exception.h>

#include <cstdint>

namespace integrated_navigation
{
  /**
   * Base class for converting extracted NMEA and NovAtel sentences into ROS
   * messages
   *
   * Subclasses that parse NMEA messages should implement
   * ParseAscii(const NmeaSentence&); subclasses that parse NovAtel messages
   * should implement both ParseAscii(const NovatelSentence&) and
   * ParseBinary(const BinaryMessage&).
   *
   * For documentation on exact message structures, see:
   * http://docs.novatel.com/OEM7/Content/Logs/Core_Logs.htm
   *
   * @tparam T The ROS message Ptr type that the parser should produce.
   */
  template<typename T>
  class MessageParser
  {
  public:
    virtual ~MessageParser() = default;

    /**
     * @return The binary message ID. Should be 0 for messages that have no
     * binary representation.
     */
    virtual uint32_t GetMessageId() const = 0;
    /**
     * @return The ASCII message name.
     */
    virtual const std::string GetMessageName() const = 0;

    /**
     * @brief Converts bin_msg into a ROS message pointer and returns it.
     *
     * The returned value must not be NULL.  ParseException should be thrown
     * if there are any issues parsing the message.
     * @param[in] bin_msg The message to convert.
     * @return A valid ROS message pointer.
     */
    virtual T ParseBinary(const BinaryMessage& bin_msg) noexcept(false)
    {
      throw ParseException("ParseBinary not implemented.");
    };

    /**
     * @brief Converts sentence into a ROS message pointer and returns it.
     *
     * The returned value must not be NULL.  ParseException should be thrown
     * if there are any issues parsing the message.
     * @param[in] bin_msg The message to convert.
     * @return A valid ROS message pointer.
     */
    virtual T ParseAscii(const NovatelSentence& sentence) noexcept(false)
    {
      throw ParseException("ParseAscii not implemented.");
    };

    /**
     * @brief Converts sentence into a ROS message pointer and returns it.
     *
     * The returned value must not be NULL.  ParseException should be thrown
     * if there are any issues parsing the message.
     * @param[in] bin_msg The message to convert.
     * @return A valid ROS message pointer.
     */
    virtual T ParseAscii(const NmeaSentence& sentence) noexcept(false)
    {
      throw ParseException("ParseAscii not implemented.");
    };
  };
}
#endif //NOVATEL_GPS_DRIVER_MESSAGE_PARSER_H