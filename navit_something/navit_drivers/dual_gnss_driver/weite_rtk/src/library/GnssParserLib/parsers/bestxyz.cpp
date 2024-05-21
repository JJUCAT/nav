#include <integrated_navigation/library/GnssParserLib/parsers/bestxyz.h>
#include <integrated_navigation/library/GnssParserLib/parsers/header.h>

#include <boost/make_shared.hpp>

namespace integrated_navigation {
    const std::string BestxyzParser::MESSAGE_NAME = "BESTNAVXYZA";

    uint32_t BestxyzParser::GetMessageId() const { return MESSAGE_ID; }

    const std::string BestxyzParser::GetMessageName() const { return MESSAGE_NAME; }

    navit_msgs::NovatelXYZPtr BestxyzParser::ParseBinary(const BinaryMessage &bin_msg) noexcept(false) {
        if (bin_msg.data_.size() != BINARY_LENGTH) {
            std::stringstream error;
            error << "Unexpected BESTXYZ message length: " << bin_msg.data_.size();
            throw ParseException(error.str());
        }
        navit_msgs::NovatelXYZPtr ros_msg = boost::make_shared<navit_msgs::NovatelXYZ>();
        HeaderParser header_parser;
        ros_msg->novatel_msg_header = header_parser.ParseBinary(bin_msg);
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
        ros_msg->position_type = POSITION_TYPES[pos_type];

        ros_msg->x = ParseDouble(&bin_msg.data_[8]);
        ros_msg->y = ParseDouble(&bin_msg.data_[16]);
        ros_msg->z = ParseDouble(&bin_msg.data_[24]);

        ros_msg->x_sigma = ParseFloat(&bin_msg.data_[32]);
        ros_msg->y_sigma = ParseFloat(&bin_msg.data_[36]);
        ros_msg->z_sigma = ParseFloat(&bin_msg.data_[40]);

        uint16_t vel_solution_status = ParseUInt16(&bin_msg.data_[44]);
        if (vel_solution_status > MAX_SOLUTION_STATUS) {
            std::stringstream error;
            error << "Unknown solution status: " << vel_solution_status;
            throw ParseException(error.str());
        }
        ros_msg->velocity_solution_status = SOLUTION_STATUSES[vel_solution_status];

        uint16_t vel_type = ParseUInt16(&bin_msg.data_[9]);//uint16_t vel_type = ParseUInt16(&bin_msg.data_[48]);
        if (vel_type > MAX_POSITION_TYPE) // Position types array includes velocity types
        {
            std::stringstream error;
            error << "Unknown position type: " << vel_type;
            throw ParseException(error.str());
        }
        ros_msg->velocity_type = POSITION_TYPES[vel_type];

        ros_msg->x_vel = ParseDouble(&bin_msg.data_[52]);
        ros_msg->y_vel = ParseDouble(&bin_msg.data_[60]);
        ros_msg->z_vel = ParseDouble(&bin_msg.data_[68]);

        ros_msg->x_vel_sigma = ParseFloat(&bin_msg.data_[76]);
        ros_msg->y_vel_sigma = ParseFloat(&bin_msg.data_[80]);
        ros_msg->z_vel_sigma = ParseFloat(&bin_msg.data_[84]);

        ros_msg->base_station_id.resize(4);
        std::copy(&bin_msg.data_[88], &bin_msg.data_[92], &ros_msg->base_station_id[0]);

        ros_msg->velocity_latency = ParseFloat(&bin_msg.data_[92]);

        ros_msg->diff_age = ParseFloat(&bin_msg.data_[96]);
        ros_msg->solution_age = ParseFloat(&bin_msg.data_[100]);

        ros_msg->num_satellites_tracked = bin_msg.data_[104];
        ros_msg->num_satellites_used_in_solution = bin_msg.data_[105];
        ros_msg->num_gps_and_glonass_l1_used_in_solution = bin_msg.data_[106];
        ros_msg->num_gps_and_glonass_l1_and_l2_used_in_solution = bin_msg.data_[107];
        // Byte 108 is reserved
        GetExtendedSolutionStatusMessage(bin_msg.data_[109], ros_msg->extended_solution_status);
        // Byte 110 is reserved
        GetSignalsUsed(bin_msg.data_[111], ros_msg->signal_mask);

        return ros_msg;
    }

    navit_msgs::NovatelXYZPtr
    BestxyzParser::ParseAscii(const NovatelSentence &sentence) noexcept(false) {
        navit_msgs::NovatelXYZPtr msg = boost::make_shared<navit_msgs::NovatelXYZ>();
        HeaderParser h_parser;
       // msg->novatel_msg_header = h_parser.ParseAscii(sentence);

        // if (sentence.body.size() != ASCII_LENGTH) {
        //     std::stringstream error;
        //     error << "Unexpected number of BESTXYZ message fields: " << sentence.body.size();
        //     throw ParseException(error.str());
        // }

        bool valid = true;

        msg->solution_status = sentence.body[0];
        msg->position_type = sentence.body[1];

        valid = valid && ParseDouble(sentence.body[2], msg->x);
        valid = valid && ParseDouble(sentence.body[3], msg->y);
        valid = valid && ParseDouble(sentence.body[4], msg->z);

        valid = valid && ParseFloat(sentence.body[5], msg->x_sigma);
        valid = valid && ParseFloat(sentence.body[6], msg->y_sigma);
        valid = valid && ParseFloat(sentence.body[7], msg->z_sigma);

        msg->velocity_solution_status = sentence.body[8];
        msg->velocity_type = sentence.body[9];

        valid = valid && ParseDouble(sentence.body[10], msg->x_vel);
        valid = valid && ParseDouble(sentence.body[11], msg->y_vel);
        valid = valid && ParseDouble(sentence.body[12], msg->z_vel);

        valid = valid && ParseFloat(sentence.body[13], msg->x_vel_sigma);
        valid = valid && ParseFloat(sentence.body[14], msg->y_vel_sigma);
        valid = valid && ParseFloat(sentence.body[15], msg->z_vel_sigma);

        msg->base_station_id = sentence.body[16];
        valid = valid && ParseFloat(sentence.body[17], msg->velocity_latency);

        valid = valid && ParseFloat(sentence.body[18], msg->diff_age);
        valid = valid && ParseFloat(sentence.body[19], msg->solution_age);
        valid = valid && ParseUInt8(sentence.body[20], msg->num_satellites_tracked);
        valid = valid && ParseUInt8(sentence.body[21], msg->num_satellites_used_in_solution);
        valid = valid && ParseUInt8(sentence.body[22], msg->num_gps_and_glonass_l1_used_in_solution);
        valid = valid && ParseUInt8(sentence.body[23], msg->num_gps_and_glonass_l1_and_l2_used_in_solution);

        // skip reserved field
        uint32_t extended_solution_status = 0;
        valid = valid && ParseUInt32(sentence.body[25], extended_solution_status, 16);
        GetExtendedSolutionStatusMessage(extended_solution_status, msg->extended_solution_status);

        // skip reserved field
        uint32_t signal_mask = 0;
        valid = valid && ParseUInt32(sentence.body[27], signal_mask, 16);
        GetSignalsUsed(signal_mask, msg->signal_mask);

        if (!valid) {
            throw ParseException("Invalid field in BESTXYZ message");
        }

        return msg;
    }
} // namespace integrated_navigation
