#include "integrated_navigation/driver/radio/radio_message_driver.h"

namespace integrated_navigation {
    radio_message_driver::radio_message_driver(ros::NodeHandle &nh) {
        // Load yaml file parameters.
        // radio configuration.
        std::string radio_topic = "radio/base_info";
        nh.getParam("/radio_config/topic", radio_topic);
        LOG(INFO) << "radio topic = " << radio_topic<<std::endl;

        publisher_ = nh.advertise<navit_msgs::RadioMessage>(radio_topic, 10);
    }

    radio_message_driver::~radio_message_driver() { }

    void radio_message_driver::radio_message_callback(uint8_t *data, const uint16_t crc) {
        int ret = radio_message_decode(data, crc);
        if(ret == -1){
            return;
        }else if(ret == 1){
            radio_message_published();
        }
    }

    void radio_message_driver::radio_message_published() {
        navit_msgs::RadioMessagePtr radio(new navit_msgs::RadioMessage());

        // set frame_id
        radio->header.frame_id = "radio";
        // set timestamps
        radio->header.stamp = ros::Time::now();

        radio->base_satellites = radio_.num_satellites;
        radio->sol_status = radio_.sol_status;
        radio->pos_type = radio_.pos_type;
        radio->battery1 = radio_.battery1;
        radio->battery2 = radio_.battery2;
        radio->base_status = radio_.base_status;
        radio->radio_status = radio_.radio_status;
        radio->diff_status = radio_.diff_status;
        radio->sat_status = radio_.sat_status;

        publisher_.publish(radio);
    }

    uint8_t radio_message_driver::radio_message_decode(uint8_t *data, const uint16_t crc) {
        uint8_t *p = data;

        if(ChkCrcValueEx(p, 0x20, 0xffff) != crc){
            printf("radio crc error.\r\n");
            return -1;
        }

        // satellites.
        radio_.num_satellites = data[20];
        // sol status.
        if((data[21] & 0xf0) == 0x10){
            radio_.sol_status = "SOL_COMPUTED";
        }else if((data[21] & 0xf0) == 0x20){
            radio_.sol_status = "INSUFFICIENT_OBS";
        }else if((data[21] & 0xf0) == 0x40){
            radio_.sol_status = "NO_CONVERGENCE";
        }else if((data[21] & 0xf0) == 0x80){
            radio_.sol_status = "COV_TRACE";
        }else{
            radio_.sol_status = "NONE";
        }

        // pos type.
        if((data[21] & 0x0f) == 0x01){
            radio_.pos_type = "INS_RTKFLOAT";
        }else if((data[21] & 0x0f) == 0x02){
            radio_.pos_type = "INS_RTKFIXED";
        }else if((data[22] & 0xf0) == 0x10){
            radio_.pos_type = "NARROW_INT";
        }else if((data[22] & 0xf0) == 0x20){
            radio_.pos_type = "INS";
        }else if((data[22] & 0xf0) == 0x40){
            radio_.pos_type = "INS_PSRSP";
        }else if((data[21] & 0xf0) == 0x80){
            radio_.pos_type = "INS_PSRDIFF";
        }else if((data[22] & 0x0f) == 0x01){
            radio_.pos_type = "IONOFREE_FLOAT";
        }else if((data[22] & 0x0f) == 0x02){
            radio_.pos_type = "NARROW_FLOAT";
        }else if((data[22] & 0x0f) == 0x04){
            radio_.pos_type = "L1_INT";
        }else if((data[22] & 0x0f) == 0x08){
            radio_.pos_type = "WIDE_INT";
        }else if((data[23] & 0xf0) == 0x10){
            radio_.pos_type = "SINGLE";
        }else if((data[23] & 0xf0) == 0x20){
            radio_.pos_type = "PSRDIFF";
        }else if((data[23] & 0xf0) == 0x40){
            radio_.pos_type = "WAAS";
        }else if((data[23] & 0xf0) == 0x80){
            radio_.pos_type = "L1_FLOAT";
        }else if((data[23] & 0x0f) == 0x01){
            radio_.pos_type = "NONE";
        }else if((data[23] & 0x0f) == 0x02){
            radio_.pos_type = "FIXEDPOS";
        }else if((data[23] & 0x0f) == 0x04){
            radio_.pos_type = "FIXEDHEIGHT";
        }else if((data[23] & 0x0f) == 0x08){
            radio_.pos_type = "DOPPLER_VELOCITY";
        }else{
            radio_.pos_type = "NONE";
        }

        if((data[24] & 0x10) == 0x00){
            radio_.diff_status = "DIFF_OK";
        }else{
            radio_.diff_status = "DIFF_ERROR";
        }

        if((data[24] & 0x20) == 0x00){
            radio_.radio_status = "RADIO_OK";
        }else{
            radio_.radio_status = "RADIO_ERROR";
        }

        if((data[24] & 0x40) == 0x00){
            radio_.sat_status = "GNSS_OK";
        }else{
            radio_.sat_status = "NO_GNSS";
        }

        if((data[24] & 0x0f) == 0x01){
            radio_.battery2 = "SUPER_LOW";
        }else if((data[24] & 0x0f) == 0x02){
            radio_.battery2 = "LOW";
        }else if((data[24] & 0x0f) == 0x04){
            radio_.battery2 = "MID";
        }else if((data[24] & 0x0f) == 0x08){
            radio_.battery2 = "HIGH";
        }else{
            radio_.battery2 = "0";
        }

        if((data[25] & 0xf0) == 0x10){
            radio_.battery1 = "SUPER_LOW";
        }else if((data[25] & 0xf0) == 0x20){
            radio_.battery1 = "LOW";
        }else if((data[25] & 0xf0) == 0x40){
            radio_.battery1 = "MID";
        }else if((data[25] & 0xf0) == 0x80){
            radio_.battery1 = "HIGH";
        }else{
            radio_.battery1 = "0";
        }

        if((data[25] & 0x0f) == 0x01){
            radio_.base_status = "OPEN";
        }else if((data[25] & 0x0f) == 0x02){
            radio_.base_status = "CLOSE";
        }else{
            radio_.base_status = "NONE";
        }

        if(DEBUG_SENSOR_DRIVER){
            uint16_t diff_age_rec = data[26] << 8 | data[27] << 0;
            uint16_t diff_age_tra = data[28] << 8 | data[29] << 0;
            uint16_t gnss_tra = data[30] << 8 | data[31] << 0;

            static uint16_t radio_frame_tmp = 0;
            static uint16_t total_loss_radio = 0;
            static uint32_t total_radio = 0;
            static uint16_t radio_frame_first = 0;
            static uint16_t k = 0;
            static bool initialised = false;
            if(diff_age_tra != radio_frame_tmp){
                if(!initialised){
                    radio_frame_tmp = diff_age_tra;
                    radio_frame_first = diff_age_tra;
                    initialised = true;
                }
                int16_t delta_frame = diff_age_tra - radio_frame_tmp - 1;
                if(delta_frame == 0) {
                    k++;
                }
                if(delta_frame > 0){
                    printf("diff age loss\n");
                    total_loss_radio += delta_frame;
                }
                total_radio = k*65536+diff_age_tra;
                printf("start, loop, total_diff_receive, total_diff_loss, base loss is:%d, %d, %d, %d, %d\n",radio_frame_first, k, total_radio, total_loss_radio, diff_age_rec - diff_age_tra);
            }
            radio_frame_tmp = diff_age_tra;

            // satellites loss check.
            static uint16_t gnss_frame_tmp = 0;
            static uint16_t total_loss_gnss = 0;
            static uint32_t total_gnss = 0;
            static uint16_t gnss_frame_first = 0;
            static uint16_t k1 = 0;
            static bool initialised1 = false;
            if(gnss_tra != gnss_frame_tmp){
                if(!initialised1){
                    gnss_frame_tmp = gnss_tra;
                    gnss_frame_first = gnss_tra;
                    initialised1 = true;
                }
                int16_t delta_frame = gnss_tra - gnss_frame_tmp - 1;
                if(delta_frame == 0) {
                    k1++;
                }
                if(delta_frame > 0){
                    printf("base gnss satellites loss\n");
                    total_loss_gnss += delta_frame;
                }
                total_gnss = k*65536+gnss_tra;
                printf("start, loop, total_satellite_receive, total_satellite_loss is:%d, %d, %d, %d\n",gnss_frame_first, k1, total_gnss, total_loss_gnss);
            }
            gnss_frame_tmp = gnss_tra;
        }

        return 1;
    }
}
