#include <boost/optional/optional.hpp>
#include <net/ethernet.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <netinet/udp.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include <boost/algorithm/string/join.hpp>
#include <boost/make_shared.hpp>
#include <sstream>

#include "integrated_navigation/driver/gnss/NovatelGps.h"

namespace integrated_navigation {
    NovatelGps::NovatelGps()
        : gpsfix_sync_tol_(0.01), wait_for_sync_(true), utc_offset_(0), novatel_positions_(MAX_BUFFER_SIZE), novatel_velocities_(MAX_BUFFER_SIZE),
          novatel_xyz_positions_(MAX_BUFFER_SIZE), dual_antenna_heading_msgs_(MAX_BUFFER_SIZE), gpchc_msgs_(MAX_BUFFER_SIZE), gpgga_msgs_(MAX_BUFFER_SIZE), gprmc_msgs_(MAX_BUFFER_SIZE),
          bestpos_fix_buffer_(SYNC_BUFFER_SIZE), bestpos_sync_buffer_(SYNC_BUFFER_SIZE), bestvel_sync_buffer_(SYNC_BUFFER_SIZE),
          bestxyz_sync_buffer_(SYNC_BUFFER_SIZE), heading_sync_buffer_(SYNC_BUFFER_SIZE), gpgga_sync_buffer_(SYNC_BUFFER_SIZE),
          ref_station_sync_buffer_(SYNC_BUFFER_SIZE), time_msgs_(MAX_BUFFER_SIZE), heading_482_(false) { }

    NovatelGps::~NovatelGps() { }

    serial_common::ReadResult NovatelGps::ProcessData(std::vector<uint8_t> &data_buffer,std::string data_str) {
        ros::Time stamp = ros::Time::now();
        std::vector<NmeaSentence> nmea_sentences;
        std::vector<NovatelSentence> novatel_sentences;
        std::vector<BinaryMessage> binary_messages;

        serial_common::ReadResult read_result = serial_common::ReadResult::READ_ERROR;
        if (!data_buffer.empty()) {
            read_result = serial_common::ReadResult::READ_SUCCESS;
            //nmea_buffer_.insert(nmea_buffer_.end(), data_buffer.begin(), data_buffer.end());
            data_buffer.clear();
            nmea_buffer_=std::string("");
            ROS_DEBUG("nmea_buffer_: %s",nmea_buffer_);
            nmea_buffer_=data_str;
            std::string remaining_buffer;
            if (!extractor_.ExtractCompleteMessages(nmea_buffer_, nmea_sentences, novatel_sentences, binary_messages,
                                                    remaining_buffer)) {
                read_result = serial_common::ReadResult::READ_PARSE_FAILED;
                error_msg_ = "Parse failure extracting sentences.";
                return read_result;
            }
            nmea_buffer_ = remaining_buffer;

            ROS_DEBUG("Parsed: %lu NMEA / %lu NovAtel / %lu Binary messages", nmea_sentences.size(),
                      novatel_sentences.size(), binary_messages.size());
            if (!nmea_buffer_.empty()) {
                ROS_DEBUG("%lu unparsed bytes left over.", nmea_buffer_.size());
            }
        }
        double most_recent_utc_time = extractor_.GetMostRecentUtcTime(nmea_sentences);

        for (const auto &sentence : nmea_sentences) {
            try {
                serial_common::ReadResult result = ParseNmeaSentence(sentence, stamp, most_recent_utc_time);

                if (result != serial_common::ReadResult::READ_SUCCESS) {
                    read_result = result;
                }
            } catch (const ParseException &p) {
                error_msg_ = p.what();
                ROS_WARN("%s", p.what());
                ROS_WARN("For sentence: [%s]", boost::algorithm::join(sentence.body, ",").c_str());
                read_result = serial_common::ReadResult::READ_PARSE_FAILED;
            }
        }

        for (const auto &sentence : novatel_sentences) {
            try {
                serial_common::ReadResult result = ParseNovatelSentence(sentence, stamp);

                if (result != serial_common::ReadResult::READ_SUCCESS) {
                    read_result = result;
                }
            } catch (const ParseException &p) {
                error_msg_ = p.what();
                ROS_WARN("%s", p.what());
                read_result = serial_common::ReadResult::READ_PARSE_FAILED;
            }
        }

        for (const auto &msg : binary_messages) {
            try {
                serial_common::ReadResult result = ParseBinaryMessage(msg, stamp);
                if (result != serial_common::ReadResult::READ_SUCCESS) {
                    read_result = result;
                }
            } catch (const ParseException &p) {
                error_msg_ = p.what();
                ROS_WARN("%s", p.what());
                read_result = serial_common::ReadResult::READ_PARSE_FAILED;
            }
        }

        return read_result;
    }

    serial_common::ReadResult NovatelGps::ParseBinaryMessage(const BinaryMessage &msg,
                                                            const ros::Time &stamp) noexcept(false) {
        switch (msg.header_.message_id_) {
            case BestposParser::MESSAGE_ID: {
                navit_msgs::NovatelPositionPtr position = bestpos_parser_.ParseBinary(msg);
                position->header.stamp = stamp;
                novatel_positions_.push_back(position);
                bestpos_sync_buffer_.push_back(position);
                bestpos_fix_buffer_.push_back(position);
                break;
            }
            case BestxyzParser::MESSAGE_ID: {
                navit_msgs::NovatelXYZPtr xyz_position = bestxyz_parser_.ParseBinary(msg);
                xyz_position->header.stamp = stamp;
                novatel_xyz_positions_.push_back(xyz_position);
                bestxyz_sync_buffer_.push_back(xyz_position);
                break;
            }
            case BestvelParser::MESSAGE_ID: {
                navit_msgs::NovatelVelocityPtr velocity = bestvel_parser_.ParseBinary(msg);
                velocity->header.stamp = stamp;
                novatel_velocities_.push_back(velocity);
                bestvel_sync_buffer_.push_back(velocity);
                break;
            }
            case DualAntennaHeadingParser::MESSAGE_ID_482: {
                navit_msgs::NovatelDualAntennaHeadingPtr heading =
                        dual_antenna_heading_parser_.ParseBinary(msg);
                heading->header.stamp = stamp;
                dual_antenna_heading_msgs_.push_back(heading);
                heading_sync_buffer_.push_back(heading);
                heading_482_ = true;
                board_type_ = boost::make_optional<std::string>("482");
                break;
            }
            case DualAntennaHeadingParser::MESSAGE_ID_718D: {
                navit_msgs::NovatelDualAntennaHeadingPtr heading =
                        dual_antenna_heading_parser_.ParseBinary(msg);
                heading->header.stamp = stamp;
                dual_antenna_heading_msgs_.push_back(heading);
                heading_sync_buffer_.push_back(heading);
                heading_482_ = false;
                board_type_ = boost::make_optional<std::string>("718D");
                break;
            }
            case TimeParser::MESSAGE_ID: {
                navit_msgs::TimePtr time = time_parser_.ParseBinary(msg);
                utc_offset_ = time->utc_offset;
                ROS_DEBUG("Got a new TIME with offset %f. UTC offset is %f", time->utc_offset, utc_offset_);
                time->header.stamp = stamp;
                time_msgs_.push_back(time);
                break;
            }
            default:
                ROS_WARN("Unexpected binary message id: %u", msg.header_.message_id_);
                break;
        }

        return serial_common::ReadResult::READ_SUCCESS;
    }

    void NovatelGps::GetNovatelPositions(std::vector<navit_msgs::NovatelPositionPtr> &positions) {
        positions.clear();
        positions.insert(positions.end(), novatel_positions_.begin(), novatel_positions_.end());
        novatel_positions_.clear();
    }

    void NovatelGps::GetNovatelXYZPositions(std::vector<navit_msgs::NovatelXYZPtr> &positions) {
        positions.clear();
        positions.insert(positions.end(), novatel_xyz_positions_.begin(), novatel_xyz_positions_.end());
        novatel_xyz_positions_.clear();
    }

    void NovatelGps::GetNovatelDualAntennaHeadingMessages(
            std::vector<navit_msgs::NovatelDualAntennaHeadingPtr> &headings) {
        headings.clear();
        headings.insert(headings.end(), dual_antenna_heading_msgs_.begin(), dual_antenna_heading_msgs_.end());
        dual_antenna_heading_msgs_.clear();
    }

    void NovatelGps::GetNovatelVelocities(std::vector<navit_msgs::NovatelVelocityPtr> &velocities) {
        velocities.resize(novatel_velocities_.size());
        std::copy(novatel_velocities_.begin(), novatel_velocities_.end(), velocities.begin());
        novatel_velocities_.clear();
    }

    void NovatelGps::GetFixMessages(std::vector<sensor_msgs::NavSatFixPtr> &fix) {
        fix.clear();

        while(!bestpos_fix_buffer_.empty()){
            auto &bestpos = bestpos_fix_buffer_.front();
            auto gps_fix = boost::make_shared<sensor_msgs::NavSatFix>();
            gps_fix->latitude = bestpos->lat;
            if (gps_fix->latitude < 0.) {
                gps_fix->latitude = -gps_fix->latitude;
            }
            gps_fix->longitude = bestpos->lon;
            if (gps_fix->longitude < 0.) {
                gps_fix->longitude = -gps_fix->longitude;
            }
            gps_fix->altitude = bestpos->height + bestpos->undulation;

            if (bestpos->solution_status == "SOL_COMPUTED"){
                gps_fix->status.status = 0;
                if(bestpos->position_type == "NARROW_INT"){
                    gps_fix->status.status = 2;
                }else{
                    gps_fix->status.status = 1;
                }
            }else{
                gps_fix->status.status = -1;
            }
            gps_fix->status.service = 4;

            double sigma_x = bestpos->lon_sigma;
            double sigma_x_squared = sigma_x * sigma_x;
            gps_fix->position_covariance[0] = sigma_x_squared;

            double sigma_y = bestpos->lat_sigma;
            double sigma_y_squared = sigma_y * sigma_y;
            gps_fix->position_covariance[4] = sigma_y_squared;

            double sigma_z = bestpos->height_sigma;
            double sigma_z_squared = sigma_z * sigma_z;
            gps_fix->position_covariance[8] = sigma_z_squared;

            // handle cov.
            gps_fix->position_covariance[1] = handleCov(sigma_x);
            gps_fix->position_covariance[2] = handleCov(sigma_y);
            gps_fix->position_covariance[3] = handleCov(sigma_z);

            gps_fix->position_covariance_type = 2;

            fix.push_back(gps_fix);
            bestpos_fix_buffer_.pop_front();
        }
    }

    void NovatelGps::GetSyncedPosVelHeading(
        std::vector<navit_msgs::NovatelPosVelHeadingPtr> &pos_vel_heading) {
        pos_vel_heading.clear();
        // std::cout<<"bestpos_sync_buffer__size="<<bestpos_sync_buffer_.size()
        // <<" bestxyz_sync_buffer_.zise="<<bestxyz_sync_buffer_.size()
        // <<" heading_sync_buffer_.size="<<heading_sync_buffer_.size()<<std::endl;
        while (!bestpos_sync_buffer_.empty()) {
            auto &bestpos = bestpos_sync_buffer_.front();
            auto NovatelPosVelHeading = boost::make_shared<navit_msgs::NovatelPosVelHeading>();
            bool synced1 = false;
            bool synced2 = false;
            static double heading_time_pre = 0.;

            while (!bestxyz_sync_buffer_.empty()) {
                auto &bestxyz = bestxyz_sync_buffer_.front();
                double time_diff =
                    std::fabs(bestxyz->novatel_msg_header.gps_seconds - bestpos->novatel_msg_header.gps_seconds);
                if (time_diff < gpsfix_sync_tol_) {
                    NovatelPosVelHeading->gps_seconds2 = bestxyz->novatel_msg_header.gps_seconds;
                    NovatelPosVelHeading->xyz_sol_status = bestxyz->velocity_solution_status;
                    NovatelPosVelHeading->xyz_type = bestxyz->velocity_type;
                    NovatelPosVelHeading->v_latency = bestxyz->velocity_latency;
                    NovatelPosVelHeading->x_vel = bestxyz->x_vel;
                    NovatelPosVelHeading->y_vel = bestxyz->y_vel;
                    NovatelPosVelHeading->z_vel = bestxyz->z_vel;
                    NovatelPosVelHeading->x_vel_sigma = bestxyz->x_vel_sigma;
                    NovatelPosVelHeading->y_vel_sigma = bestxyz->y_vel_sigma;
                    NovatelPosVelHeading->z_vel_sigma = bestxyz->z_vel_sigma;
                    NovatelPosVelHeading->velocity_covariance[0] = bestxyz->x_vel_sigma * bestxyz->x_vel_sigma;
                    NovatelPosVelHeading->velocity_covariance[4] = bestxyz->y_vel_sigma * bestxyz->y_vel_sigma;
                    NovatelPosVelHeading->velocity_covariance[8] = bestxyz->z_vel_sigma * bestxyz->z_vel_sigma;
                    synced1 = true;
                    break;
                } else if (bestxyz->novatel_msg_header.gps_seconds < bestpos->novatel_msg_header.gps_seconds) {
                    // Bestxyz timestamp is too old, throw it away and try again
                    bestxyz_sync_buffer_.pop_front();
                } else {
                    // Latest bestxyz message is too new, do nothing for now
                    break;
                }
            }
            if(!heading_sync_buffer_.empty()){
               while (!heading_sync_buffer_.empty()) {
                auto &bestheading = heading_sync_buffer_.front();
                double time_diff =
                    std::fabs(bestheading->novatel_msg_header.gps_seconds - bestpos->novatel_msg_header.gps_seconds);
                if (time_diff < gpsfix_sync_tol_) {
                    NovatelPosVelHeading->gps_seconds3 = bestheading->novatel_msg_header.gps_seconds;
                    NovatelPosVelHeading->heading_type = bestheading->position_type;
                    NovatelPosVelHeading->baseline_length = bestheading->baseline_length;
                    NovatelPosVelHeading->num_satellites2 = bestheading->num_satellites_used_in_solution;
                    NovatelPosVelHeading->heading = bestheading->heading;
                    NovatelPosVelHeading->pitch = bestheading->pitch;
                    NovatelPosVelHeading->heading_sigma = bestheading->heading_sigma;
                    NovatelPosVelHeading->heading_covariance = bestheading->heading_sigma * bestheading->heading_sigma;
                    NovatelPosVelHeading->pitch_sigma = bestheading->pitch_sigma;
                    NovatelPosVelHeading->pitch_covariance = bestheading->pitch_sigma * bestheading->pitch_sigma;
                    heading_time_pre = bestheading->novatel_msg_header.gps_seconds;
                    NovatelPosVelHeading->header.stamp = bestheading->header.stamp;
                    heading_sync_buffer_.pop_front();
                    synced2 = true;
                    break;
                } else if (bestheading->novatel_msg_header.gps_seconds < bestpos->novatel_msg_header.gps_seconds) {
                    // headinga timestamp is too old, throw it away and try again
                    heading_sync_buffer_.pop_front();
                } else {
                    // Latest headinga message is too new, do nothing for now
                    break;
                }
              }
            }
            else{
                 NovatelPosVelHeading->heading_type = "NONE";
                 synced2 = true;
            }       
            // if board type is 718d, it does not output heading sentence when inside.
            if(!heading_482_){
                double heading_time_delta = bestpos->novatel_msg_header.gps_seconds - heading_time_pre;
                if (heading_time_delta > 0.5) {
                    if (heading_time_delta > 100.) {
                        heading_time_pre = bestpos->novatel_msg_header.gps_seconds - 1.0;
                    }
                    NovatelPosVelHeading->heading_type = "NONE";
                    synced2 = true;
                }
            }

            if ((!synced1 || !synced2) && wait_for_sync_) {
                break;
            }

            NovatelPosVelHeading->board_type = GetBoardType(); 
            NovatelPosVelHeading->gps_seconds1 = bestpos->novatel_msg_header.gps_seconds;
            NovatelPosVelHeading->novatel_msg_header = bestpos->novatel_msg_header;
            NovatelPosVelHeading->solution_status = bestpos->solution_status;
            NovatelPosVelHeading->position_type = bestpos->position_type;
            NovatelPosVelHeading->diff_age = bestpos->diff_age;
            NovatelPosVelHeading->solution_age = bestpos->solution_age;
            NovatelPosVelHeading->num_satellites1 = bestpos->num_satellites_used_in_solution;

            NovatelPosVelHeading->latitude = bestpos->lat;
            if (NovatelPosVelHeading->latitude < 0.) {
                NovatelPosVelHeading->latitude = -NovatelPosVelHeading->latitude;
            }
            NovatelPosVelHeading->longitude = bestpos->lon;
            if (NovatelPosVelHeading->longitude < 0.) {
                NovatelPosVelHeading->longitude = -NovatelPosVelHeading->longitude;
            }
            NovatelPosVelHeading->altitude = bestpos->height + bestpos->undulation;
            NovatelPosVelHeading->undulation = bestpos->undulation;

            double sigma_x = bestpos->lon_sigma;
            double sigma_x_squared = sigma_x * sigma_x;
            NovatelPosVelHeading->lon_sigma = sigma_x;
            NovatelPosVelHeading->position_covariance[0] = sigma_x_squared;

            double sigma_y = bestpos->lat_sigma;
            double sigma_y_squared = sigma_y * sigma_y;
            NovatelPosVelHeading->lat_sigma = sigma_y;

            NovatelPosVelHeading->position_covariance[4] = sigma_y_squared;

            double sigma_z = bestpos->height_sigma;
            double sigma_z_squared = sigma_z * sigma_z;
            NovatelPosVelHeading->height_sigma = sigma_z;
            NovatelPosVelHeading->position_covariance[8] = sigma_z_squared;

            NovatelPosVelHeading->extended_solution_status = bestpos->extended_solution_status;
            NovatelPosVelHeading->signal_mask = bestpos->signal_mask;

            pos_vel_heading.push_back(NovatelPosVelHeading);
            bestpos_sync_buffer_.pop_front();
        }
    }

    void NovatelGps::GetGpchcMessages(std::vector<navit_msgs::GpchcPtr>& gpchc_msgs){
        gpchc_msgs.clear();
        gpchc_msgs.insert(gpchc_msgs.end(), gpchc_msgs_.begin(), gpchc_msgs_.end());
        gpchc_msgs_.clear();
    }

    void NovatelGps::GetGpggaMessages(std::vector<navit_msgs::GpggaPtr> &gpgga_messages) {
        gpgga_messages.clear();
        gpgga_messages.insert(gpgga_messages.end(), gpgga_msgs_.begin(), gpgga_msgs_.end());
        gpgga_msgs_.clear();
    }

    void NovatelGps::GetGprmcMessages(std::vector<navit_msgs::GprmcPtr>& gprmc_messages)
    {
        gprmc_messages.clear();
        gprmc_messages.insert(gprmc_messages.end(), gprmc_msgs_.begin(), gprmc_msgs_.end());
        gprmc_msgs_.clear();
    }

    void NovatelGps::GetTimeMessages(std::vector<navit_msgs::TimePtr> &time_messages) {
        time_messages.resize(time_msgs_.size());
        std::copy(time_msgs_.begin(), time_msgs_.end(), time_messages.begin());
        time_msgs_.clear();
    }

    serial_common::ReadResult NovatelGps::ParseNmeaSentence(const NmeaSentence &sentence, const ros::Time &stamp,
                                                           double most_recent_utc_time) noexcept(false) {
        //std::cout<<"ParseNmeaSentence sentence.id="<< sentence.id<<std::endl;                                                   
        if (sentence.id == GpggaParser::MESSAGE_NAME) {
            navit_msgs::GpggaPtr gpgga = gpgga_parser_.ParseAscii(sentence);
            gpgga_msgs_.push_back(gpgga);
            gpgga_sync_buffer_.push_back(gpgga);
        } else if (sentence.id == GprmcParser::MESSAGE_NAME) {
            navit_msgs::GprmcPtr gprmc = gprmc_parser_.ParseAscii(sentence);

            auto gprmc_time = UtcFloatToSeconds(gprmc->utc_seconds);

            if (most_recent_utc_time < gprmc_time) {
                most_recent_utc_time = gprmc_time;
            }

            gprmc->header.stamp = stamp - ros::Duration(most_recent_utc_time - gprmc_time);

            gprmc_msgs_.push_back(std::move(gprmc));
        } else if (sentence.id == GpchcParser::MESSAGE_NAME){
            ROS_DEBUG("receive gpchc data");
            navit_msgs::GpchcPtr gpchc = gpchc_parser_.ParseAscii(sentence);
            ROS_DEBUG("gpchc week = %d", gpchc->gps_week);
            gpchc_msgs_.push_back(gpchc);
        }
        // else if(sentence.id == ){

        // } 
        else {
            ROS_DEBUG_STREAM("Unrecognized NMEA sentence " << sentence.id);
        }

        return serial_common::ReadResult::READ_SUCCESS;
    }

    serial_common::ReadResult NovatelGps::ParseNovatelSentence(const NovatelSentence &sentence,
                                                              const ros::Time &stamp) noexcept(false) {
        if (sentence.id == "BESTNAVA") {//BESTPOSA
            navit_msgs::NovatelPositionPtr position = bestpos_parser_.ParseAscii(sentence);
            position->header.stamp = stamp;
            novatel_positions_.push_back(position);
            bestpos_sync_buffer_.push_back(position);
            bestpos_fix_buffer_.push_back(position);
        } else if (sentence.id == "BESTNAVXYZA") {//BESTXYZA
            navit_msgs::NovatelXYZPtr position = bestxyz_parser_.ParseAscii(sentence);
            position->header.stamp = stamp;
            novatel_xyz_positions_.push_back(position);
            bestxyz_sync_buffer_.push_back(position);
        } else if (sentence.id == "BESTVELA") {
            navit_msgs::NovatelVelocityPtr velocity = bestvel_parser_.ParseAscii(sentence);
            velocity->header.stamp = stamp;
            novatel_velocities_.push_back(velocity);
            bestvel_sync_buffer_.push_back(velocity);
        } else if (sentence.id == "UNIHEADINGA") {//HEADINGA
            navit_msgs::NovatelDualAntennaHeadingPtr heading =
                dual_antenna_heading_parser_.ParseAscii(sentence);
            heading->header.stamp = stamp;
            dual_antenna_heading_msgs_.push_back(heading);
            heading_sync_buffer_.push_back(heading);
        } else if (sentence.id == "TIMEA") {
            navit_msgs::TimePtr time = time_parser_.ParseAscii(sentence);
            utc_offset_ = time->utc_offset;
            ROS_DEBUG("Got a new TIME with offset %f. UTC offset is %f", time->utc_offset, utc_offset_);
            time->header.stamp = stamp;
            time_msgs_.push_back(time);
        }

        return serial_common::ReadResult::READ_SUCCESS;
    }

    std::string  NovatelGps::GetBoardType(){
        if(board_type_){
            return *board_type_;
        }else{
            return "NONE";
        }
    }
} // namespace integrated_navigation