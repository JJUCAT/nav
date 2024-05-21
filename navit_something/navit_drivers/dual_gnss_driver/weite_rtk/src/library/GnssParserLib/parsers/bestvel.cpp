#include <integrated_navigation/library/GnssParserLib/parsers/bestvel.h>
#include <integrated_navigation/library/GnssParserLib/parsers/header.h>

#include <boost/make_shared.hpp>

const std::string integrated_navigation::BestvelParser::MESSAGE_NAME = "BESTVEL";

uint32_t integrated_navigation::BestvelParser::GetMessageId() const { return MESSAGE_ID; }

const std::string integrated_navigation::BestvelParser::GetMessageName() const { return MESSAGE_NAME; }

navit_msgs::NovatelVelocityPtr
integrated_navigation::BestvelParser::ParseBinary(const BinaryMessage &bin_msg) noexcept(false) {
    if (bin_msg.data_.size() != BINARY_LENGTH) {
        std::stringstream error;
        error << "Unexpected velocity message size: " << bin_msg.data_.size();
        throw ParseException(error.str());
    }
    navit_msgs::NovatelVelocityPtr ros_msg =
        boost::make_shared<navit_msgs::NovatelVelocity>();
    HeaderParser h_parser;
    ros_msg->novatel_msg_header = h_parser.ParseBinary(bin_msg);
    ros_msg->novatel_msg_header.message_name = MESSAGE_NAME;

    uint16_t solution_status = ParseUInt16(&bin_msg.data_[0]);
    if (solution_status > MAX_SOLUTION_STATUS) {
        std::stringstream error;
        error << "Unknown solution status: " << solution_status;
        throw ParseException(error.str());
    }
    ros_msg->solution_status = SOLUTION_STATUSES[solution_status];
    uint16_t pos_type = ParseUInt16(&bin_msg.data_[4]);
    if (pos_type > MAX_POSITION_TYPE) {
        std::stringstream error;
        error << "Unknown position type: " << pos_type;
        throw ParseException(error.str());
    }
    ros_msg->velocity_type = POSITION_TYPES[pos_type];
    ros_msg->latency = ParseFloat(&bin_msg.data_[8]);
    ros_msg->age = ParseFloat(&bin_msg.data_[12]);
    ros_msg->horizontal_speed = ParseDouble(&bin_msg.data_[16]);
    ros_msg->track_ground = ParseDouble(&bin_msg.data_[24]);
    ros_msg->vertical_speed = ParseDouble(&bin_msg.data_[32]);

    return ros_msg;
}

navit_msgs::NovatelVelocityPtr
integrated_navigation::BestvelParser::ParseAscii(const NovatelSentence &sentence) noexcept(false) {
    navit_msgs::NovatelVelocityPtr msg =
        boost::make_shared<navit_msgs::NovatelVelocity>();
    HeaderParser h_parser;
    msg->novatel_msg_header = h_parser.ParseAscii(sentence);

    if (sentence.body.size() != ASCII_LENGTH) {
        std::stringstream error;
        error << "Unexpected number of BESTVEL message fields: " << sentence.body.size();
        throw ParseException(error.str());
    }
    bool valid = true;
    msg->solution_status = sentence.body[0];
    msg->velocity_type = sentence.body[1];
    valid = valid && ParseFloat(sentence.body[2], msg->latency);
    valid = valid && ParseFloat(sentence.body[3], msg->age);
    valid = valid && ParseDouble(sentence.body[4], msg->horizontal_speed);
    valid = valid && ParseDouble(sentence.body[5], msg->track_ground);
    valid = valid && ParseDouble(sentence.body[6], msg->vertical_speed);

    if (!valid) {
        throw ParseException("Invalid field in BESTVEL message");
    }

    return msg;
}
