#include <integrated_navigation/library/GnssParserLib/parsers/gphdt.h>
#include <integrated_navigation/library/Serial/string_utils.h>
#include <boost/make_shared.hpp>

const std::string integrated_navigation::GphdtParser::MESSAGE_NAME = "GPHDT";

uint32_t integrated_navigation::GphdtParser::GetMessageId() const
{
  return 0;
}

const std::string integrated_navigation::GphdtParser::GetMessageName() const
{
  return MESSAGE_NAME;
}

integrated_navigation::GphdtPtr integrated_navigation::GphdtParser::ParseAscii(const integrated_navigation::NmeaSentence& sentence) noexcept(false)
{
  const size_t EXPECTED_LEN = 3;

  if (sentence.body.size() != EXPECTED_LEN)
  {
    std::stringstream error;
    error << "Expected GPHDT length = "
          << EXPECTED_LEN << ", "
          << "actual length = " << sentence.body.size();
    throw ParseException(error.str());
  }

  integrated_navigation::GphdtPtr msg = boost::make_shared<integrated_navigation::Gphdt>();
  msg->message_id = sentence.body[0];

  double heading;
  if (ToDouble(sentence.body[1], heading))
  {
      msg->heading = heading;
  }
  else
  {
    throw ParseException("Error parsing heading as double in GPHDT");
  }

  msg->t = sentence.body[2];
  return msg;
}