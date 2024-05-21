#include <integrated_navigation/library/GnssParserLib/parsers/header.h>
#include <integrated_navigation/library/GnssParserLib/parsers/refstation.h>
#include <ros/ros.h>

#include <boost/make_shared.hpp>

namespace integrated_navigation {
    const std::string RefStationParser::MESSAGE_NAME = "REFSTATIONA";

    uint32_t RefStationParser::GetMessageId() const { return MESSAGE_ID; }

    const std::string RefStationParser::GetMessageName() const { return MESSAGE_NAME; }

    navit_msgs::NovatelRefStationPtr
    RefStationParser::ParseAscii(const NovatelSentence &sentence) noexcept(false) {
        navit_msgs::NovatelRefStationPtr msg =
            boost::make_shared<navit_msgs::NovatelRefStation>();
        HeaderParser h_parser;
        msg->novatel_msg_header = h_parser.ParseAscii(sentence);

        if (sentence.body.size() != ASCII_LENGTH) {
            std::stringstream error;
            error << "Unexpected number of BESTPOS message fields: " << sentence.body.size();
            throw ParseException(error.str());
        }

        bool valid = true;

        valid = valid && ParseUInt32(sentence.body[0], msg->status);
        valid = valid && ParseDouble(sentence.body[1], msg->X);
        valid = valid && ParseDouble(sentence.body[2], msg->Y);
        valid = valid && ParseDouble(sentence.body[3], msg->Z);
        valid = valid && ParseUInt32(sentence.body[4], msg->health);
        msg->stn_type = sentence.body[5];
        msg->stn_id = sentence.body[6];

        if (!valid) {
            throw ParseException("Invalid field in REFSTATION message");
        }

        return msg;
    }
} // namespace integrated_navigation