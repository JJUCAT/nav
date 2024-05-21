#include <integrated_navigation/library/GnssParserLib/parsers/gpgsa.h>
#include <boost/make_shared.hpp>

const std::string integrated_navigation::GpgsaParser::MESSAGE_NAME = "GPGSA";

uint32_t integrated_navigation::GpgsaParser::GetMessageId() const
{
  return 0;
}

const std::string integrated_navigation::GpgsaParser::GetMessageName() const
{
  return MESSAGE_NAME;
}

integrated_navigation::GpgsaPtr integrated_navigation::GpgsaParser::ParseAscii(const integrated_navigation::NmeaSentence& sentence) noexcept(false)
{
  // Check the length first -- should be 18 elements long
  const size_t LENGTH = 18;
  if (sentence.body.size() != LENGTH)
  {
    std::stringstream error;
    error << "Expected GPGSA length " << LENGTH
          << ", actual length " << sentence.body.size();
    throw ParseException(error.str());
  }

  integrated_navigation::GpgsaPtr msg = boost::make_shared<integrated_navigation::Gpgsa>();
  msg->message_id = sentence.body[0];
  msg->auto_manual_mode = sentence.body[1];
  ParseUInt8(sentence.body[2], msg->fix_mode);
  // Words 3-14 of the sentence are SV IDs. Copy only the non-null strings.
  msg->sv_ids.resize(12, 0);
  size_t n_svs = 0;
  for (std::vector<std::string>::const_iterator id = sentence.body.begin()+3; id < sentence.body.begin()+15; ++id)
  {
    if (! id->empty())
    {
      ParseUInt8(*id, msg->sv_ids[n_svs]);
      ++n_svs;
    }
  }
  msg->sv_ids.resize(n_svs);

  ParseFloat(sentence.body[15], msg->pdop);
  ParseFloat(sentence.body[16], msg->hdop);
  ParseFloat(sentence.body[17], msg->vdop);
  return msg;
}
