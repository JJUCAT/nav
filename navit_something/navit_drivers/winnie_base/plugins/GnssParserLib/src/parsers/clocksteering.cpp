#include <GnssParserLib/parsers/clocksteering.h>

#include <boost/make_shared.hpp>

const std::string integrated_navigation::ClockSteeringParser::MESSAGE_NAME = "CLOCKSTEERING";

uint32_t integrated_navigation::ClockSteeringParser::GetMessageId() const { return 0; }

const std::string integrated_navigation::ClockSteeringParser::GetMessageName() const { return MESSAGE_NAME; }

navit_msgs::ClockSteeringPtr integrated_navigation::ClockSteeringParser::ParseAscii(
    const integrated_navigation::NovatelSentence& sentence) noexcept(false) {
    const size_t MIN_LENGTH = 8;
    // Check that the message is at least as long as a a ClockSteering with no satellites
    if (sentence.body.size() != MIN_LENGTH) {
        std::stringstream error;
        error << "Expected ClockSteering length >= " << MIN_LENGTH << ", actual length = " << sentence.body.size();
        throw ParseException(error.str());
    }
    navit_msgs::ClockSteeringPtr msg = boost::make_shared<navit_msgs::ClockSteering>();

    msg->source         = sentence.body[0];
    msg->steering_state = sentence.body[1];

    if (!ParseUInt32(sentence.body[2], msg->period)) {
        throw ParseException("Error parsing period in ClockSteering.");
    }

    if (!ParseDouble(sentence.body[3], msg->pulse_width)) {
        throw ParseException("Error parsing pulse_width in ClockSteering.");
    }

    if (!ParseDouble(sentence.body[4], msg->bandwidth)) {
        throw ParseException("Error parsing bandwidth in ClockSteering.");
    }

    if (!ParseFloat(sentence.body[5], msg->slope)) {
        throw ParseException("Error parsing slope in ClockSteering.");
    }

    if (!ParseDouble(sentence.body[6], msg->offset)) {
        throw ParseException("Error parsing offset in ClockSteering.");
    }

    if (!ParseDouble(sentence.body[7], msg->drift_rate)) {
        throw ParseException("Error parsing drift_rate in ClockSteering.");
    }

    return msg;
}
