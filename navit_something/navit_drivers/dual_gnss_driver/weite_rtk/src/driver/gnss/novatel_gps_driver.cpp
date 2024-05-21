#include "integrated_navigation/driver/gnss/novatel_gps_driver.h"
#include "std_msgs/Bool.h"

namespace integrated_navigation {
    novatel_gps_driver::novatel_gps_driver(ros::NodeHandle &nh, std::shared_ptr<io_common> &serial)
        : frame_id_("gnss_link"),
          publish_novatel_pose_vel_(true),
          publish_novatel_positions_(false),
          publish_novatel_xyz_positions_(false),
          publish_novatel_velocity_(false),
          publish_novatel_dual_antenna_heading_(false),
          publish_gpchc_(false),
          publish_gpgga_(false),
          publish_navsatfix_(false),
          serial_(serial),
          gpchc_count_(0) {
        // Load yaml file parameters.
        // novatel gps configuration.
        std::string novatel_topic = "Novatel/PosVelHeadingData";
        nh.getParam("/novatel_config/frame", frame_id_);
        nh.getParam("/novatel_config/topic", novatel_topic);
        nh.getParam("/novatel_config/publish_novatel_pose_vel", publish_novatel_pose_vel_);
        nh.getParam("/novatel_config/publish_novatel_positions", publish_novatel_positions_);
        nh.getParam("/novatel_config/publish_novatel_xyz_positions", publish_novatel_xyz_positions_);
        nh.getParam("/novatel_config/publish_novatel_velocity", publish_novatel_velocity_);
        nh.getParam("/novatel_config/publish_novatel_dual_antenna_heading", publish_novatel_dual_antenna_heading_);
        nh.getParam("/novatel_config/publish_gpchc", publish_gpchc_);
        nh.getParam("/novatel_config/publish_gpgga", publish_gpgga_);
        nh.getParam("/novatel_config/publish_gprmc", publish_gprmc_);
        nh.getParam("/novatel_config/publish_navsatfix", publish_navsatfix_);

        LOG(INFO) << "novatel frame and topic = " << frame_id_ << ", " << novatel_topic<<std::endl; 
        LOG(INFO) << "publish_novatel_pose_vel = " << publish_novatel_pose_vel_<<std::endl; 
        LOG(INFO) << "publish_novatel_positions = " << publish_novatel_positions_<<std::endl; 
        LOG(INFO) << "publish_novatel_xyz_positions = " << publish_novatel_xyz_positions_<<std::endl; 
        LOG(INFO) << "publish_novatel_velocity = " << publish_novatel_velocity_<<std::endl; 
        LOG(INFO) << "publish_novatel_dual_antenna_heading = " << publish_novatel_dual_antenna_heading_<<std::endl; 
        LOG(INFO) << "publish_gpchc = " << publish_gpchc_<<std::endl;
        LOG(INFO) << "publish_gpgga = " << publish_gpgga_<<std::endl;
        LOG(INFO) << "publish_navsatfix = " << publish_navsatfix_<<std::endl; 

        if (publish_novatel_pose_vel_) {
            fusion_pub_ = nh.advertise<navit_msgs::NovatelPosVelHeading>(novatel_topic, 100);
        }

        if (publish_novatel_positions_) {
            novatel_position_pub_ = nh.advertise<navit_msgs::NovatelPosition>("Novatel/BestPos", 100);
        }

        if (publish_novatel_xyz_positions_) {
            novatel_xyz_position_pub_ = nh.advertise<navit_msgs::NovatelXYZ>("Novatel/BestXYZ", 100);
        }
        if (publish_novatel_velocity_) {
            novatel_velocity_pub_ = nh.advertise<navit_msgs::NovatelVelocity>("Novatel/BestVel", 100);
        }

        if (publish_novatel_dual_antenna_heading_) {
            novatel_dual_antenna_heading_pub_ =
                    nh.advertise<navit_msgs::NovatelDualAntennaHeading>("Novatel/Heading", 100);
        }

        if (publish_gpchc_) {
            gpchc_pub_ = nh.advertise<navit_msgs::Gpchc>("gpchc", 100);
            gpchc_ = std::make_shared<GpchcPublisher>(nh,"rtk_sat_fix","rtk_data_ready", frame_id_);
        }

        if (publish_gpgga_) {
            gpgga_pub_ = nh.advertise<navit_msgs::Gpgga>("Novatel/Gpgga", 100);
        }

        if (publish_gprmc_) {
            gprmc_pub_ = nh.advertise<navit_msgs::Gprmc>("Novatel/Gprmc", 100);
        }

        if(publish_navsatfix_) {
            navsatfix_pub_ = nh.advertise<sensor_msgs::NavSatFix>("Novatel/Fix", 100);
        }

        diag_updater_.add("RTK STATUS", diagnostic_updater::TaskFunction{});

        gps_ = std::make_shared<NovatelGps>();
    }

    void novatel_gps_driver::novatel_data_callback(std::vector<uint8_t> &data_buffer,std::string data_str) {
        io_common::ReadResult result = gps_->ProcessData(data_buffer,data_str);
        //io_common::ReadResult result =serial_common::ReadResult::READ_PARSE_FAILED;
        if (result == io_common::ReadResult::READ_ERROR) {
            //LOG(ERROR) << "Error reading from novatel device: %s" << serial_->get_device().c_str() << std::endl;
            LOG(ERROR) << "Error reading from novatel device" << std::endl;
            device_errors_++; return;
        } else if (result == io_common::ReadResult::READ_TIMEOUT) {
            device_timeouts_++;return;
        } else if (result == io_common::ReadResult::READ_INTERRUPTED) {
            // If we are interrupted by a signal, ROS is probably
            // quitting, but we'll wait for ROS to tell us to quit.
            device_interrupts_++;return;
        } else if (result == io_common::ReadResult::READ_PARSE_FAILED) {
                LOG(ERROR) << "Error reading from novatel device" << std::endl;
                gps_parse_failures_++;return;  
        } else if (result == io_common::ReadResult::READ_INSUFFICIENT_DATA) {
            gps_insufficient_data_warnings_++;return;
        }

        std::vector<navit_msgs::NovatelPosVelHeadingPtr> synced_ins_msgs;
        std::vector<navit_msgs::NovatelPositionPtr> position_msgs;
        std::vector<navit_msgs::NovatelXYZPtr> xyz_msgs;
        std::vector<navit_msgs::NovatelDualAntennaHeadingPtr> heading_msgs;
        std::vector<navit_msgs::GpchcPtr> gpchc_msgs;
        std::vector<navit_msgs::GpggaPtr> gpgga_msgs;
        std::vector<navit_msgs::GprmcPtr> gprmc_msgs;
        std::vector<sensor_msgs::NavSatFixPtr> fix_msgs;

        // GPSFix messages are always published, and Gpgga and Position messages
        // are used for generating some diagnostics.  Other message types will
        // only be retrieved if we're configured to publish them.
     
        gps_->GetGpchcMessages(gpchc_msgs);
        gps_->GetGpggaMessages(gpgga_msgs);
        gps_->GetNovatelPositions(position_msgs);
        gps_->GetNovatelXYZPositions(xyz_msgs);
        gps_->GetNovatelDualAntennaHeadingMessages(heading_msgs);
        gps_->GetSyncedPosVelHeading(synced_ins_msgs);

        // Increment the measurement count by the number of messages we just read
        measurement_count_ += position_msgs.size();

        // If there are new position messages, store the most recent
        if (!position_msgs.empty()) {
            last_novatel_position_ = position_msgs.back();
        }
        // If timesync messages are available, CalculateTimeSync will
        // update a stat accumulator of the offset of the TimeSync message
        // stamp from the GPS message stamp
        CalculateTimeSync();

        // If TimeSync messages are available, CalculateTimeSync keeps
        // an acculumator of their offset, which is used to
        // calculate a rolling mean of the offset to apply to all messages
        ros::Duration sync_offset(0); // If no TimeSyncs, assume 0 offset
        // LOG(INFO) << "GPS TimeSync offset is " << sync_offset << std::endl;

        static ros::Time last_pub_state_time=ros::Time::now();
        static bool pub_state=false;
        static std::vector<navit_msgs::NovatelPosVelHeadingPtr> synced_ins_msgs_buffer;
        static std::vector<navit_msgs::NovatelDualAntennaHeadingPtr> heading_msgs_buffer;
        static   std::vector<sensor_msgs::NavSatFixPtr> fix_msgs_buffer;
        if(last_pub_state_time+ros::Duration(1.0)<ros::Time::now()){
            pub_state=true;
            last_pub_state_time=ros::Time::now();
        }

        if (publish_novatel_pose_vel_) {
            //std::cout<<"publish_novatel_pose_vel_,synced_ins_msgs.size()="<<synced_ins_msgs.size()<<std::endl;
            for (const auto &msg : synced_ins_msgs) {
                // msg->header.stamp = ros::Time::now();
                msg->header.stamp += sync_offset;
                msg->header.frame_id = frame_id_;
                fusion_pub_.publish(msg);
            }
            synced_ins_msgs_buffer.insert(synced_ins_msgs_buffer.end(),synced_ins_msgs.begin(),synced_ins_msgs.end());
            if(pub_state){
                float latitude, longitude, altitude;
                int count=0;
                std::reverse(synced_ins_msgs_buffer.begin(),synced_ins_msgs_buffer.end());
                latitude=synced_ins_msgs_buffer[0]->latitude;
                longitude=synced_ins_msgs_buffer[0]->longitude;
                altitude=synced_ins_msgs_buffer[0]->altitude;
                for(int i=1;i<synced_ins_msgs_buffer.size();i++){
                    if(synced_ins_msgs_buffer[i]->latitude==latitude &&
                    synced_ins_msgs_buffer[i]->longitude==longitude &&
                    synced_ins_msgs_buffer[i]->altitude==altitude)
                    count++;
                }
                if(count>=3)
                    diag_updater_.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, "RTK ERROR!");
                synced_ins_msgs_buffer.swap(synced_ins_msgs);
            }
            
        }
        if (publish_novatel_positions_) {
            for (const auto &msg : position_msgs) {
                msg->header.stamp += sync_offset;
                msg->header.frame_id = frame_id_;
                novatel_position_pub_.publish(msg);
            }
        }

        if(publish_novatel_xyz_positions_){
            for (const auto &msg : xyz_msgs) {
                msg->header.stamp += sync_offset;
                msg->header.frame_id = frame_id_;
                novatel_xyz_position_pub_.publish(msg);
            }
        }

        if(publish_novatel_dual_antenna_heading_){
            for (const auto &msg : heading_msgs) {
                msg->header.stamp += sync_offset;
                msg->header.frame_id = frame_id_;
                novatel_dual_antenna_heading_pub_.publish(msg);
            }
             heading_msgs_buffer.insert(heading_msgs_buffer.end(),heading_msgs.begin(),heading_msgs.end());
            if(pub_state){
                float heading;
                int count=0;
                std::reverse(heading_msgs_buffer.begin(),heading_msgs_buffer.end());
                heading=heading_msgs_buffer[0]->heading;
                for(int i=1;i<heading_msgs_buffer.size();i++){
                    if(heading_msgs_buffer[i]->heading==heading)
                    count++;
                }
                if(count>=3)
                    diag_updater_.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, "RTK ERROR!");
                heading_msgs_buffer.swap(heading_msgs);
            }
        }

        if (publish_gpgga_) {
            for (const auto &msg : gpgga_msgs) {
                msg->header.stamp = ros::Time::now();
                msg->header.frame_id = frame_id_;
                gpgga_pub_.publish(msg);
            }
        }

        if (publish_gprmc_) {
            gps_->GetGprmcMessages(gprmc_msgs);
            for (const auto &msg : gprmc_msgs) {
                msg->header.stamp = ros::Time::now();
                msg->header.frame_id = frame_id_;
                gprmc_pub_.publish(msg);
            }
        }

        if (publish_navsatfix_) {
            gps_->GetFixMessages(fix_msgs);
            for (const auto &msg : fix_msgs) {
                msg->header.stamp = ros::Time::now();
                msg->header.frame_id = frame_id_;
                navsatfix_pub_.publish(msg);
            }
             fix_msgs_buffer.insert(fix_msgs_buffer.end(),fix_msgs.begin(),fix_msgs.end());
            if(pub_state){
                float latitude, longitude, altitude;
                int count=0;
                std::reverse(fix_msgs_buffer.begin(),fix_msgs_buffer.end());
                latitude=fix_msgs_buffer[0]->latitude;
                longitude=fix_msgs_buffer[0]->longitude;
                altitude=fix_msgs_buffer[0]->altitude;
                for(int i=1;i<fix_msgs_buffer.size();i++){
                    if(fix_msgs_buffer[i]->latitude==latitude &&
                    fix_msgs_buffer[i]->longitude==longitude &&
                    fix_msgs_buffer[i]->altitude==altitude)
                    count++;
                }
                if(count>=3)
                    diag_updater_.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, "RTK ERROR!");
                fix_msgs_buffer.swap(fix_msgs);
            }
        }

        if (publish_gpchc_) {
            for(const auto& msg : gpchc_msgs) {
                msg->header.stamp = ros::Time::now();
                msg->header.frame_id = frame_id_;
                gpchc_pub_.publish(msg);

                gpchc_->Publish(msg);

                gpchc_count_++;
                if(gpchc_count_ == 200) {
                    gpchc_->PublishHealth(msg);
                    gpchc_count_ = 0;
                }
            }
        }
        pub_state=false;
    }

    void novatel_gps_driver::CalculateTimeSync() {
        boost::unique_lock<boost::mutex> lock(mutex_);
        int32_t synced_i = -1; /// Index of last synced timesync msg
        int32_t synced_j = -1; /// Index of last synced gps msg
        // Loop over sync times buffer
        for (size_t i = 0; i < sync_times_.size(); i++) {
            // Loop over message times buffer
            for (size_t j = synced_j + 1; j < msg_times_.size(); j++) {
                // Offset is the difference between the sync time and message time
                double offset = (sync_times_[i] - msg_times_[j]).toSec();
                if (std::fabs(offset) < 0.49) {
                    // If the offset is less than 0.49 sec, the messages match
                    synced_i = static_cast<int32_t>(i);
                    synced_j = static_cast<int32_t>(j);
                    // Add the offset to the stats accumulators
                    offset_stats_(offset);
                    // Update the last sync
                    last_sync_ = sync_times_[i];
                    // Break out of the inner loop and continue looping through sync times
                    break;
                }
            }
        }

        // Remove all the timesync messages that have been matched from the queue
        for (int i = 0; i <= synced_i && !sync_times_.empty(); i++) {
            sync_times_.pop_front();
        }

        // Remove all the gps messages that have been matched from the queue
        for (int j = 0; j <= synced_j && !msg_times_.empty(); j++) {
            msg_times_.pop_front();
        }
    }

    void novatel_gps_driver::FixDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &status) {
        status.clear();
        status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Nominal");

        if (!last_novatel_position_) {
            status.summary(diagnostic_msgs::DiagnosticStatus::WARN, "No Status");
            LOG(ERROR) << "No GPS status data." << std::endl;
            return;
        }

        status.add("Solution Status", last_novatel_position_->solution_status);
        status.add("Position Type", last_novatel_position_->position_type);
        status.add("Solution Age", last_novatel_position_->solution_age);
        status.add("Satellites Tracked", static_cast<int>(last_novatel_position_->num_satellites_tracked));
        status.add("Satellites Used", static_cast<int>(last_novatel_position_->num_satellites_used_in_solution));
        status.add("Software Version", last_novatel_position_->novatel_msg_header.receiver_software_version);

        const navit_msgs::NovatelReceiverStatus &rcvr_status =
                last_novatel_position_->novatel_msg_header.receiver_status;
        status.add("Status Code", rcvr_status.original_status_code);

        if (last_novatel_position_->novatel_msg_header.receiver_status.original_status_code != 0) {
            uint8_t level = diagnostic_msgs::DiagnosticStatus::WARN;
            std::string msg = "Status Warning";
            // If the antenna is disconnected/broken, this is an error
            if (rcvr_status.antenna_is_open || rcvr_status.antenna_is_shorted || !rcvr_status.antenna_powered) {
                msg += " Antenna Problem";
                level = diagnostic_msgs::DiagnosticStatus::ERROR;
            }
            status.add("Error Flag", rcvr_status.error_flag ? "true" : "false");
            status.add("Temperature Flag", rcvr_status.temperature_flag ? "true" : "false");
            status.add("Voltage Flag", rcvr_status.voltage_supply_flag ? "true" : "false");
            status.add("Antenna Not Powered", rcvr_status.antenna_powered ? "false" : "true");
            status.add("Antenna Open", rcvr_status.antenna_is_open ? "true" : "false");
            status.add("Antenna Shorted", rcvr_status.antenna_is_shorted ? "true" : "false");
            status.add("CPU Overloaded", rcvr_status.cpu_overload_flag ? "true" : "false");
            status.add("COM1 Buffer Overrun", rcvr_status.com1_buffer_overrun ? "true" : "false");
            status.add("COM2 Buffer Overrun", rcvr_status.com2_buffer_overrun ? "true" : "false");
            status.add("COM3 Buffer Overrun", rcvr_status.com3_buffer_overrun ? "true" : "false");
            status.add("USB Buffer Overrun", rcvr_status.usb_buffer_overrun ? "true" : "false");
            status.add("RF1 AGC Flag", rcvr_status.rf1_agc_flag ? "true" : "false");
            status.add("RF2 AGC Flag", rcvr_status.rf2_agc_flag ? "true" : "false");
            status.add("Almanac Flag", rcvr_status.almanac_flag ? "true" : "false");
            status.add("Position Solution Flag", rcvr_status.position_solution_flag ? "true" : "false");
            status.add("Position Fixed Flag", rcvr_status.position_fixed_flag ? "true" : "false");
            status.add("Clock Steering Status", rcvr_status.clock_steering_status_enabled ? "true" : "false");
            status.add("Clock Model Flag", rcvr_status.clock_model_flag ? "true" : "false");
            status.add("OEMV External Oscillator Flag", rcvr_status.oemv_external_oscillator_flag ? "true" : "false");
            status.add("Software Resource Flag", rcvr_status.software_resource_flag ? "true" : "false");
            status.add("Auxiliary1 Flag", rcvr_status.aux1_status_event_flag ? "true" : "false");
            status.add("Auxiliary2 Flag", rcvr_status.aux2_status_event_flag ? "true" : "false");
            status.add("Auxiliary3 Flag", rcvr_status.aux3_status_event_flag ? "true" : "false");
            // ROS_INFO("Novatel status code: %d", rcvr_status.original_status_code);
            status.summary(level, msg);
        }
    }

    void novatel_gps_driver::SyncDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &status) {
        status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Nominal");

        if (last_sync_ == ros::TIME_MIN) {
            status.summary(diagnostic_msgs::DiagnosticStatus::WARN, "No Sync");
            return;
        } else if (last_sync_ < ros::Time::now() - ros::Duration(10)) {
            status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Sync Stale");
            LOG(ERROR) << "GPS time synchronization is stale." << std::endl;
        }

        status.add("Last Sync", last_sync_);
        status.add("Mean Offset", stats::mean(offset_stats_));
        // status.add("Mean Offset (rolling)", stats::rolling_mean(0));
        status.add("Offset Variance", stats::variance(offset_stats_));
        status.add("Min Offset", stats::min(offset_stats_));
        status.add("Max Offset", stats::max(offset_stats_));
    }

    void novatel_gps_driver::DeviceDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &status) {
        status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Nominal");

        if (device_errors_ > 0) {
            status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Device Errors");
        } else if (device_interrupts_ > 0) {
            status.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Device Interrupts");
            LOG(WARNING) << "device interrupts detected <%s>: %d" << serial_->get_device().c_str()
                         << device_interrupts_ << std::endl;
        } else if (device_timeouts_) {
            status.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Device Timeouts");
            LOG(WARNING) << "device timeouts detected <%s>: %d" << serial_->get_device().c_str()
                         << device_timeouts_ << std::endl;
        }

        status.add("Errors", device_errors_);
        status.add("Interrupts", device_interrupts_);
        status.add("Timeouts", device_timeouts_);

        device_timeouts_ = 0;
        device_interrupts_ = 0;
        device_errors_ = 0;
    }

    void novatel_gps_driver::GpsDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &status) {
        status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Nominal");

        if (gps_parse_failures_ > 0) {
            status.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Parse Failures");
            LOG(WARNING) << "gps parse failures detected <%s>: %d" << serial_->get_device().c_str() << gps_parse_failures_
                         << std::endl;
        }

        status.add("Parse Failures", gps_parse_failures_);
        status.add("Insufficient Data Warnings", gps_insufficient_data_warnings_);

        gps_parse_failures_ = 0;
        gps_insufficient_data_warnings_ = 0;
    }

    void novatel_gps_driver::DataDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &status) {
        status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Nominal");

        double period = diagnostic_updater_.getPeriod();
        double measured_rate = measurement_count_ / period;

        if (measured_rate < 0.5 * expected_rate_) {
            status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Insufficient Data Rate");
            LOG(ERROR) << "insufficient data rate <%s>: %lf < %lf" << serial_->get_device().c_str() << measured_rate << expected_rate_
                       << std::endl;
        } else if (measured_rate < 0.95 * expected_rate_) {
            status.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Insufficient Data Rate");
            LOG(WARNING) << "insufficient data rate <%s>: %lf < %lf" << serial_->get_device().c_str() << measured_rate
                         << expected_rate_ << std::endl;
        }

        status.add("Measurement Rate (Hz)", measured_rate);

        measurement_count_ = 0;
    }

    void novatel_gps_driver::RateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &status) {
        status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Nominal Publish Rate");

        double elapsed = (ros::Time::now() - last_published_).toSec();
        bool gap_detected = false;
        if (elapsed > 2.0 / expected_rate_) {
            publish_rate_warnings_++;
            gap_detected = true;
        }

        if (publish_rate_warnings_ > 1 || gap_detected) {
            status.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Insufficient Publish Rate");
            LOG(WARNING) << "publish rate failures detected <%s>: %d" << serial_->get_device().c_str() << publish_rate_warnings_
                         << std::endl;
        }

        status.add("Warnings", publish_rate_warnings_);

        publish_rate_warnings_ = 0;
    }
}