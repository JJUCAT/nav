#include <integrated_navigation/library/GnssParserLib/parsers/gprmc.h>
#include <integrated_navigation/library/Serial/string_utils.h>

#include <boost/make_shared.hpp>

const std::string integrated_navigation::GprmcParser::MESSAGE_NAME = "GNRMC";

uint32_t integrated_navigation::GprmcParser::GetMessageId() const { return 0; }

const std::string integrated_navigation::GprmcParser::GetMessageName() const { return MESSAGE_NAME; }

navit_msgs::GprmcPtr
integrated_navigation::GprmcParser::ParseAscii(const integrated_navigation::NmeaSentence &sentence) noexcept(false) {
    // Check the length first; should be 13 elements long for OEM6 & 7,
    // but only 12 elements for OEM4.
    const size_t EXPECTED_LEN_OEM6 = 14;//1026 xiugai
    const size_t EXPECTED_LEN_OEM4 = 13;

    if (sentence.body.size() != EXPECTED_LEN_OEM4 && sentence.body.size() != EXPECTED_LEN_OEM6) {
        std::stringstream error;
        error << "Expected GPRMC lengths = " << EXPECTED_LEN_OEM4 << " (for OEM4), " << EXPECTED_LEN_OEM6
              << " (for OEM6), "
              << "actual length = " << sentence.body.size();
        throw ParseException(error.str());
    }

    bool success = true;
    navit_msgs::GprmcPtr msg = boost::make_shared<navit_msgs::Gprmc>();
    msg->message_id = sentence.body[0];

    if (sentence.body[1].empty() || sentence.body[1] == "0") {
        msg->utc_seconds = 0;
    } else {
        double utc_float;
        if (ToDouble(sentence.body[1], utc_float)) {
            msg->utc_seconds = UtcFloatToSeconds(utc_float);
        } else {
            throw ParseException("Error parsing UTC seconds in GNRMC log.");
        }
    }

    msg->position_status = sentence.body[2];
    // Check to see whether this message is listed as valid
    success &= (sentence.body[2].compare("A") == 0);
    success &= !(sentence.body[3].empty() || sentence.body[5].empty());

    bool valid = true;

    double latitude = 0.0;
    valid = valid && ParseDouble(sentence.body[3], latitude);
    msg->lat = ConvertDmsToDegrees(latitude);

    double longitude = 0.0;
    valid = valid && ParseDouble(sentence.body[5], longitude);
    msg->lon = ConvertDmsToDegrees(longitude);

    msg->lat_dir = sentence.body[4];
    msg->lon_dir = sentence.body[6];

    valid = valid && ParseFloat(sentence.body[7], msg->speed);
    msg->speed *= KNOTS_TO_MPS;

    valid = valid && ParseFloat(sentence.body[8], msg->track);

    std::string date_str = sentence.body[9];
    if (!date_str.empty()) {
        msg->date = std::string("20") + date_str.substr(4, 2) + std::string("-") + date_str.substr(2, 2) +
                    std::string("-") + date_str.substr(0, 2);
    }
    valid = valid && ParseFloat(sentence.body[10], msg->mag_var);
    msg->mag_var_direction = sentence.body[11];
    if (sentence.body.size() == EXPECTED_LEN_OEM6) {
        msg->mode_indicator = sentence.body[12];
    }

    if (!valid) {
        was_last_gps_valid_ = false;
        throw ParseException("Error parsing GPRMC message.");
    }

    was_last_gps_valid_ = success;

    return msg;
}

bool integrated_navigation::GprmcParser::WasLastGpsValid() const { return was_last_gps_valid_; }