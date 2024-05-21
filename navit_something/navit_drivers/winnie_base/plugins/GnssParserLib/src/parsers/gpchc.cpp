#include <GnssParserLib/parsers/gpchc.h>
#include <GnssParserLib/parsers/header.h>
#include <GnssParserLib/parsers/parsing_utils.h>
#include <GnssParserLib/parsers/string_utils.h>

#include <boost/make_shared.hpp>
#include <cstdint>

const std::string integrated_navigation::GpchcParser::MESSAGE_NAME = "GPCHC";

uint32_t integrated_navigation::GpchcParser::GetMessageId() const { return 0; }

const std::string integrated_navigation::GpchcParser::GetMessageName() const { return MESSAGE_NAME; }

navit_msgs::GpchcPtr integrated_navigation::GpchcParser::ParseAscii(
    const integrated_navigation::NmeaSentence& sentence) noexcept(false) {
    // Check the length first -- should be 15 elements long
    // const size_t MAX_LEN = 15;
    // const size_t MIN_LEN = 14;
    // if (sentence.body.size() > MAX_LEN || sentence.body.size() < MIN_LEN)
    // {
    //   std::stringstream error;
    //   error << "Expected GPchc length " << MIN_LEN << "  <= length <= "
    //         << MAX_LEN << ", actual length = " << sentence.body.size();
    //   throw ParseException(error.str());
    // }

    navit_msgs::GpchcPtr msg = boost::make_shared<navit_msgs::Gpchc>();

    msg->message_id = sentence.body[0];

    if (sentence.body[2].empty() || sentence.body[2] == "0") {
        msg->utc_seconds = 0;
    } else {
        double utc_float;
        if (ToDouble(sentence.body[2], utc_float)) {
            // msg->utc_seconds = UtcFloatToSeconds(utc_float);
            msg->utc_seconds = utc_float;
        } else {
            throw ParseException("Error parsing UTC seconds in GPchc");
        }
    }

    bool valid = true;

    uint32_t gps_week = 0;
    valid             = valid && ParseUInt32(sentence.body[1], gps_week);
    msg->gps_week     = gps_week;

    double heading = 0;
    valid          = valid && ParseDouble(sentence.body[3], heading);
    msg->heading   = heading;

    double pitch = 0;
    valid        = valid && ParseDouble(sentence.body[4], pitch);
    msg->pitch   = pitch;

    double roll = 0;
    valid       = valid && ParseDouble(sentence.body[5], roll);
    msg->roll   = roll;

    double gyro_x = 0;
    valid         = valid && ParseDouble(sentence.body[6], gyro_x);
    msg->gyro_x   = gyro_x;

    double gyro_y = 0;
    valid         = valid && ParseDouble(sentence.body[7], gyro_y);
    msg->gyro_y   = gyro_y;

    double gyro_z = 0;
    valid         = valid && ParseDouble(sentence.body[8], gyro_z);
    msg->gyro_z   = gyro_z;

    double acc_x = 0;
    valid        = valid && ParseDouble(sentence.body[9], acc_x);
    msg->acc_x   = acc_x;

    double acc_y = 0;
    valid        = valid && ParseDouble(sentence.body[10], acc_y);
    msg->acc_y   = acc_y;

    double acc_z = 0;
    valid        = valid && ParseDouble(sentence.body[11], acc_z);
    msg->acc_z   = acc_z;

    double latitude = 0.0;
    valid           = valid && ParseDouble(sentence.body[12], latitude);
    msg->lattitude  = latitude;

    double longitude = 0.0;
    valid            = valid && ParseDouble(sentence.body[13], longitude);
    msg->longitude   = longitude;

    double altitude = 0.0;
    valid           = valid && ParseDouble(sentence.body[14], altitude);
    msg->altitude   = altitude;

    double vel_e = 0;
    valid        = valid && ParseDouble(sentence.body[15], vel_e);
    msg->vel_e   = vel_e;

    double vel_n = 0;
    valid        = valid && ParseDouble(sentence.body[16], vel_n);
    msg->vel_n   = vel_n;

    double vel_u = 0;
    valid        = valid && ParseDouble(sentence.body[17], vel_u);
    msg->vel_u   = vel_u;

    valid = valid && ParseDouble(sentence.body[18], msg->vel_vehicle);
    valid = valid && ParseUInt32(sentence.body[19], msg->navsatelites_1);
    valid = valid && ParseUInt32(sentence.body[20], msg->navsatelites_2);

    valid = valid && ParseUInt8(sentence.body[21], msg->status);
    valid = valid && ParseUInt32(sentence.body[22], msg->diff_age);

    valid = valid && ParseUInt8(sentence.body[23], msg->warning);

    if (!valid) {
        was_last_gps_valid_ = false;
        throw ParseException("GPchc log was invalid.");
    }

    // If we got this far, we successfully parsed the message and will consider
    // it valid
    was_last_gps_valid_ = true;

    return msg;
}

bool integrated_navigation::GpchcParser::WasLastGpsValid() const { return was_last_gps_valid_; }