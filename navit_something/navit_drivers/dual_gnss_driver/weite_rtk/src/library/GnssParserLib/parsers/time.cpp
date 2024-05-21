#include <integrated_navigation/library/GnssParserLib/parsers/time.h>

#include <boost/make_shared.hpp>

const std::string integrated_navigation::TimeParser::MESSAGE_NAME = "TIME";

uint32_t integrated_navigation::TimeParser::GetMessageId() const { return MESSAGE_ID; }

const std::string integrated_navigation::TimeParser::GetMessageName() const { return MESSAGE_NAME; }

navit_msgs::TimePtr
integrated_navigation::TimeParser::ParseBinary(const integrated_navigation::BinaryMessage &msg) noexcept(false) {
    if (msg.data_.size() != BINARY_LENGTH) {
        std::stringstream error;
        error << "Unexpected time message size: " << msg.data_.size();
        throw ParseException(error.str());
    }

    navit_msgs::TimePtr ros_msg = boost::make_shared<navit_msgs::Time>();

    uint32_t clock_status = ParseUInt32(&msg.data_[0]);
    switch (clock_status) {
    case 0:
        ros_msg->clock_status = "VALID";
        break;
    case 1:
        ros_msg->clock_status = "CONVERGING";
        break;
    case 2:
        ros_msg->clock_status = "ITERATING";
        break;
    case 3:
        ros_msg->clock_status = "INVALID";
        break;
    default: {
        std::stringstream error;
        error << "Unexpected clock status: " << clock_status;
        throw ParseException(error.str());
    }
    }
    ros_msg->offset = ParseDouble(&msg.data_[4]);
    ros_msg->offset_std = ParseDouble(&msg.data_[12]);
    ros_msg->utc_offset = ParseDouble(&msg.data_[20]);
    ros_msg->utc_year = ParseUInt32(&msg.data_[28]);
    ros_msg->utc_month = msg.data_[32];
    ros_msg->utc_day = msg.data_[33];
    ros_msg->utc_hour = msg.data_[34];
    ros_msg->utc_minute = msg.data_[35];
    ros_msg->utc_millisecond = ParseUInt32(&msg.data_[36]);
    uint32_t utc_status = ParseUInt32(&msg.data_[40]);
    switch (utc_status) {
    case 0:
        ros_msg->utc_status = "Invalid";
        break;
    case 1:
        ros_msg->utc_status = "Valid";
        break;
    case 2:
        ros_msg->utc_status = "Warning";
        break;
    default: {
        std::stringstream error;
        error << "Unexpected UTC status: " << utc_status;
        throw ParseException(error.str());
    }
    }

    return ros_msg;
}

navit_msgs::TimePtr
integrated_navigation::TimeParser::ParseAscii(const integrated_navigation::NovatelSentence &sentence) noexcept(false) {
    navit_msgs::TimePtr msg = boost::make_shared<navit_msgs::Time>();
    if (sentence.body.size() != ASCII_FIELD) {
        std::stringstream error;
        error << "Unexpected number of fields in TIME log: " << sentence.body.size();
        throw ParseException(error.str());
    }
    bool valid = true;
    msg->clock_status = sentence.body[0];
    valid &= ParseDouble(sentence.body[1], msg->offset);
    valid &= ParseDouble(sentence.body[2], msg->offset_std);
    valid &= ParseDouble(sentence.body[3], msg->utc_offset);
    valid &= ParseUInt32(sentence.body[4], msg->utc_year, 10);
    valid &= ParseUInt8(sentence.body[5], msg->utc_month, 10);
    valid &= ParseUInt8(sentence.body[6], msg->utc_day, 10);
    valid &= ParseUInt8(sentence.body[7], msg->utc_hour, 10);
    valid &= ParseUInt8(sentence.body[8], msg->utc_minute, 10);
    valid &= ParseUInt32(sentence.body[9], msg->utc_millisecond, 10);
    msg->utc_status = sentence.body[10];

    if (!valid) {
        throw ParseException("Error parsing TIME log.");
    }

    return msg;
}
