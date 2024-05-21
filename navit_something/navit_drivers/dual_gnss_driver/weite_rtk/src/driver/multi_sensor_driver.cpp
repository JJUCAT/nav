#include "integrated_navigation/driver/multi_sensor_driver.h"

namespace integrated_navigation {
    multi_sensor_driver::multi_sensor_driver(ros::NodeHandle &nh)
        : rate_(1000),
          reconnect_delay_s_(0.5),
          imu_frame_(0),
          novatel_frame_(0) {

        // Load yaml file parameters.
        nh.getParam("/multi_sensor_config/connection_type", connection_type_);
        LOG(INFO) << "connection type = " << connection_type_<<std::endl; 

        if(connection_type_ == "serial"){
            io_common_ = std::make_shared<serial_common>(nh);
            LOG(INFO) << "IO type: serial"<<std::endl;
        }else if(connection_type_ == "tcp"){
            io_common_ = std::make_shared<tcp_common>(nh);
            LOG(INFO) << "IO type: tcp"<<std::endl;
        }else{
            LOG(ERROR) << "Invalid connection type ." << std::endl;
            return;
        }

        ky102n_imu_ = std::make_shared<ky102n_imu_driver>(nh);
        novatel_gps_ = std::make_shared<novatel_gps_driver>(nh, io_common_);
        radio_msg_ = std::make_shared<radio_message_driver>(nh);

        thread_ = boost::thread(&multi_sensor_driver::Spin, this);
    }

    multi_sensor_driver::~multi_sensor_driver() {
        io_common_->Disconnect();
    }

    void multi_sensor_driver::Spin() {
        ros::WallRate rate(rate_);
        while (ros::ok()) {
            if (io_common_->Connect()) {
                while(io_common_->IsConnected()){
                    // DLOG(INFO) << "read data process";
                    // Read data from the device and publish any received messages
                    MultiSensorDataFlow();
                }    // While (gps_.IsConnected() && ros::ok()) (inner loop to process data from device)
            }else{ // Could not connect to the device
                LOG(ERROR) << "Error connecting to device: " << io_common_->get_device().c_str() << std::endl;
            }

            /*
            if (ros::ok()) {
                // If ROS is still OK but we got disconnected, we're going to try
                // to reconnect, but wait just a bit so we don't spam the device.
                ros::WallDuration(reconnect_delay_s_).sleep();
            }*/

            // Sleep for a microsecond to prevent CPU hogging
            rate.sleep();
        } // While (ros::ok) (outer loop to reconnect to device)

        io_common_->Disconnect();
    }

    void multi_sensor_driver::MultiSensorDataFlow() {
        uint16_t first_number = 5;
        if(!read_from_connector(first_number)){
            LOG(WARNING) << "Read from connector failed, first number 5." << std::endl;
            return;
        }

        // DLOG(INFO) << "read data done";

//        printf("%x, %x, %x, %x\n", data_buffer_.at(0), data_buffer_.at(1), data_buffer_.at(2), data_buffer_.at(3));
        if (data_buffer_.size() == first_number && data_buffer_.at(0) == 0x52 && data_buffer_.at(1) == 0x54){
            if(data_buffer_.at(2) == 0x24){
                // ky102n imu data.
                ky102n_imu_callback();
            }else if(data_buffer_.at(2) == 0x21){
                // novatel gps data.
                novatel_posa_callback();
            }else if(data_buffer_.at(2) == 0x22){
                // radio data.
                radio_message_callback();
            }else{
                printf("%x,%x,%x\n", data_buffer_.at(2),data_buffer_.at(3),data_buffer_.at(4));
                printf("error msg\n");
            }
            data_buffer_.clear();
        }else{
            data_buffer_.erase(data_buffer_.begin());
        }
    }

    void multi_sensor_driver::ky102n_imu_callback() {
        uint16_t number = 36;
        if(!read_from_connector(number)){
            LOG(WARNING) << "Read from imu connector failed." << std::endl;
            return;
        }

        if(data_buffer_.size() == number && data_buffer_.at(number-1) == 0x0D && data_buffer_.at(number-2) == 0x0A){
            imu_frame_trans_ = data_buffer_.at(0) << 8 | data_buffer_.at(1) << 0;
            imu_frame_ = data_buffer_.at(2) << 8 | data_buffer_.at(3) << 0;
            uint16_t crc = data_buffer_.at(number-3) << 0 | data_buffer_.at(number-4) << 8;
            data_buffer_.erase(data_buffer_.begin(),data_buffer_.begin()+4);
            data_buffer_.erase(data_buffer_.end()-4, data_buffer_.end());
            uint8_t recvBuffer[data_buffer_.size()] = {0};
            for(uint8_t i = 0;i < data_buffer_.size();i++){
                recvBuffer[i] = data_buffer_.at(i);
            }
            ky102n_imu_->ky102n_data_callback(recvBuffer, crc);

            if(DEBUG_SENSOR_DRIVER){
                static uint16_t imu_frame_tmp = 0;
                static uint16_t total_loss_imu = 0;
                static uint32_t total_imu = 0;
                static uint16_t imu_frame_first = 0;
                static uint16_t k1 = 0;
                static uint8_t count_print = 0;
                static bool initilised = false;
                if(imu_frame_ != imu_frame_tmp){
                    if(!initilised){
                        imu_frame_tmp = imu_frame_;
                        imu_frame_first = imu_frame_;
                        initilised = true;
                    }
                    int16_t delta_frame = imu_frame_ - imu_frame_tmp - 1;
                    if(imu_frame_ == 0) {
                        k1++;
                    }
                    if(delta_frame > 0){
                        printf("imu loss\n");
                        total_loss_imu += delta_frame;
                    }
                    total_imu = k1*65536+imu_frame_;

                    count_print ++;
                    if(count_print == 200){
                        count_print = 0;
                        printf("start, loop, total_imu_receive, total_imu_loss, host_loss is:%d, %d, %d, %d, %d\n",imu_frame_first, k1, total_imu, total_loss_imu, imu_frame_trans_-imu_frame_);
                    }
                }
                imu_frame_tmp = imu_frame_;
            }
        }
    }

    void multi_sensor_driver::novatel_posa_callback() {
        uint16_t number = (data_buffer_.at(3) << 8 | data_buffer_.at(4))+2;
       // uint16_t number = 20000;
        if(!read_from_connector(number)){
            LOG(WARNING) << "Read from novatel connector failed." << std::endl;
            return;
        }

        if(data_buffer_.size() == number && data_buffer_.at(number-1) == 0x0D && data_buffer_.at(number-2) == 0x0A){
            novatel_frame_trans_ = data_buffer_.at(0) << 8 | data_buffer_.at(1);
            novatel_frame_ = data_buffer_.at(2) << 8 | data_buffer_.at(3);
            data_buffer_.erase(data_buffer_.begin(),data_buffer_.begin()+4);
            data_buffer_.erase(data_buffer_.end()-2, data_buffer_.end());
            novatel_gps_->novatel_data_callback(data_buffer_);

            if(DEBUG_SENSOR_DRIVER){
                static uint16_t novatel_frame_tmp = 0;
                static uint16_t total_loss_novatel = 0;
                static uint32_t total_novatel = 0;
                static uint16_t novatel_frame_first = 0;
                static uint8_t count_print = 0;
                static uint16_t k2 = 0;
                static bool initilised1 = false;
                if(novatel_frame_ != novatel_frame_tmp){
                    if(!initilised1){
                        novatel_frame_tmp = novatel_frame_;
                        novatel_frame_first = novatel_frame_;
                        initilised1 = true;
                    }
                    int16_t delta_frame = novatel_frame_ - novatel_frame_tmp - 1;
                    if(novatel_frame_ == 0) {
                        k2++;
                    }
                    if(delta_frame > 0){
                        printf("novatel loss\n");
                        total_loss_novatel += delta_frame;
                    }
                    total_novatel = k2*65536+novatel_frame_;
                    count_print ++;
                    if(count_print == 15){
                        count_print = 0;
                        printf("start, loop, total_novatel_receive, total_novatel_loss, host loss is:%d, %d, %d, %d, %d\n",novatel_frame_first, k2, total_novatel, total_loss_novatel, novatel_frame_trans_ - novatel_frame_);
                    }
                }
                novatel_frame_tmp = novatel_frame_;
            }
        }
    }

    void multi_sensor_driver::radio_message_callback() {
        uint16_t number = 46;
        if(!read_from_connector(number)){
            LOG(WARNING) << "Read from radio connector failed." << std::endl;
            return;
        }

        if(data_buffer_.size() == number && data_buffer_.at(number-1) == 0x0D && data_buffer_.at(number-2) == 0x0A
                                         && data_buffer_.at(number-3) == 0x0D && data_buffer_.at(number-4) == 0x0A){
            uint16_t crc = data_buffer_.at(number-5) << 0 | data_buffer_.at(number-6) << 8;
            data_buffer_.erase(data_buffer_.end()-6, data_buffer_.end());
            data_buffer_.erase(data_buffer_.begin(),data_buffer_.begin()+8);
            uint8_t recvBuffer[data_buffer_.size()] = {0};
            for(uint8_t i = 0;i < data_buffer_.size();i++){
                recvBuffer[i] = data_buffer_.at(i);
                //printf("%x, ", data_buffer_.at(i));
            }
            //printf("\n");
            radio_msg_->radio_message_callback(recvBuffer, crc);
        }
    }

    bool multi_sensor_driver::read_from_connector(const uint16_t number) {
        data_buffer_.clear();
        io_common::ReadResult read_result = io_common::ReadResult::READ_ERROR;
        while(data_buffer_.size() < number){
            read_result = io_common_->IORead(data_buffer_, number-data_buffer_.size());
            switch (read_result){
                    case io_common::ReadResult::READ_INSUFFICIENT_DATA :{
                        LOG(WARNING) << "Buffer data is not enough!";
                        break;
                    }
                    case io_common::ReadResult::READ_TIMEOUT :{
                        LOG(WARNING) << "Read timeout!";
                        break;
                    }
                    case io_common::ReadResult::READ_INTERRUPTED :{
                        LOG(WARNING) << "Read interrupted!";
                        break;
                    }
                    case io_common::ReadResult::READ_ERROR :{
                        LOG(WARNING) << "Read error!";
                        break;
                    }
                    case io_common::ReadResult::READ_PARSE_FAILED :{
                        LOG(WARNING) << "Read parse failed!";
                        break;
                    }
                    
                    default:{
                        // DLOG(WARNING) << "Read default!";
                        break;
                    }
                } 
        }

        if(read_result == io_common::ReadResult::READ_SUCCESS){
            return true;
        }

        LOG(ERROR) << "Invalid connection type, Read Error.";
        return false;
    }
}