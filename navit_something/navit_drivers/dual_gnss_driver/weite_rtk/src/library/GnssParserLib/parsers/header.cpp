#include <integrated_navigation/library/GnssParserLib/parsers/header.h>
#include <ros/ros.h>

uint32_t integrated_navigation::HeaderParser::GetMessageId() const { return 0; }

const std::string integrated_navigation::HeaderParser::GetMessageName() const { return "HEADER"; }

navit_msgs::NovatelMessageHeader
integrated_navigation::HeaderParser::ParseBinary(const integrated_navigation::BinaryMessage &bin_msg) noexcept(false) {
    // No point in checking whether the port identifier is valid here, because
    // the variable's range is 0-255 and this array has 256 values in it.
    navit_msgs::NovatelMessageHeader msg;
    msg.port = PORT_IDENTIFIERS[bin_msg.header_.port_address_];
    msg.sequence_num = bin_msg.header_.sequence_;
    msg.percent_idle_time = bin_msg.header_.idle_time_;
    switch (bin_msg.header_.time_status_) {
    case 20:
        msg.gps_time_status = "UNKNOWN";
        break;
    case 60:
        msg.gps_time_status = "APPROXIMATE";
        break;
    case 80:
        msg.gps_time_status = "COARSEADJUSTING";
        break;
    case 100:
        msg.gps_time_status = "COARSE";
        break;
    case 120:
        msg.gps_time_status = "COARSESTEERING";
        break;
    case 130:
        msg.gps_time_status = "FREEWHEELING";
        break;
    case 140:
        msg.gps_time_status = "FINEADJUSTING";
        break;
    case 160:
        msg.gps_time_status = "FINE";
        break;
    case 170:
        msg.gps_time_status = "FINEBACKUPSTEERING";
        break;
    case 180:
        msg.gps_time_status = "FINESTEERING";
        break;
    case 200:
        msg.gps_time_status = "SATTIME";
        break;
    default: {
        std::stringstream error;
        error << "Unknown GPS time status: " << bin_msg.header_.time_status_;
        throw ParseException(error.str());
    }
    }
    msg.gps_week_num = bin_msg.header_.week_;
    msg.gps_seconds = static_cast<double>(bin_msg.header_.gps_ms_) / 1000.0;
    GetNovatelReceiverStatusMessage(bin_msg.header_.receiver_status_, msg.receiver_status);
    msg.receiver_software_version = bin_msg.header_.receiver_sw_version_;

    return msg;
}

navit_msgs::NovatelMessageHeader integrated_navigation::HeaderParser::ParseAscii(
    const integrated_navigation::NovatelSentence &sentence) noexcept(false) {
    if (sentence.header.size() != NOVATEL_MESSAGE_HEADER_LENGTH) {
        std::stringstream error;
        error << "Novatel message header size wrong: expected " << NOVATEL_MESSAGE_HEADER_LENGTH << ", got %zu"
              << sentence.header.size();
        throw ParseException(error.str());
    }

    bool valid = true;

    navit_msgs::NovatelMessageHeader msg;
    msg.message_name = sentence.header[0];
    msg.port = sentence.header[1];
    valid = valid && ParseUInt32(sentence.header[2], msg.sequence_num);
    if(!valid)
       std::cout<<"2 "<<sentence.header[2]<<std::endl;
    valid = valid && ParseFloat(sentence.header[3], msg.percent_idle_time);
     if(!valid)
       std::cout<<"3 "<<sentence.header[3]<<std::endl;
    msg.gps_time_status = sentence.header[4];
    valid = valid && ParseUInt32(sentence.header[5], msg.gps_week_num);
     if(!valid)
       std::cout<<"5 "<<sentence.header[5]<<std::endl;
    valid = valid && ParseDouble(sentence.header[6], msg.gps_seconds);
     if(!valid)
       std::cout<<"6 "<<sentence.header[6]<<std::endl;

    uint32_t receiver_status_code = 0;
    valid = valid && ParseUInt32(sentence.header[7], receiver_status_code, 16);
     if(!valid)
       std::cout<<"7 "<<sentence.header[7]<<std::endl;
    GetNovatelReceiverStatusMessage(receiver_status_code, msg.receiver_status);

    valid = valid && ParseUInt32(sentence.header[9], msg.receiver_software_version);
     if(!valid)
       std::cout<<"7 "<<sentence.header[7]<<std::endl;

    if (!valid) {
        throw ParseException("Header was invalid.");
    }
    return msg;
}
