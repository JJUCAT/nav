#ifndef GNSS_STATE_HPP
#define GNSS_STATE_HPP

#include "integrated_navigation/filtering/filter_defination.h"
#include "integrated_navigation/sensor_data/gnss_data.h"
#include "integrated_navigation/log/logging.h"
#include <map>

namespace integrated_navigation {

    const std::map<std::string, uint8_t> GnssPoseStatus = {
        {"NONE",             0},
        {"SINGLE",           1},         
        {"PSRDIFF",          2},          
        {"L1_INT",           3},          
        {"NARROW_INT",       4},       
        {"NARROW_FLOAT",     5},     
        {"FIXEDPOS",         6},         
        {"FIXEDHEIGHT",      7},      
        {"DOPPLER_VELOCITY", 8}, 
        {"WAAS",             9},          
        {"L1_FLOAT",         10},         
        {"IONOFREE_FLOAT",   11},
        {"WIDE_INT",         12}       
    };

    const std::map<std::string, uint8_t> GnssHeadingStatus{
        {"NONE",       0},
        {"NARROW_INT", 1}
    };

    inline void GnssStateInfo(State * s, const GnssPositionDataPtr gnss_data_ptr){
        //2表示组合导航模式, 3表示惯导模式
        s->filter_status = 2;
        s->gnss_quality = 13;

        uint8_t pose_status = 0;
        uint8_t heading_status = 13;

        if(gnss_data_ptr->heading_type != "NARROW_INT"){
            LOG(WARNING) << "novatel heading is not NARROW_INT, heading type is: " << gnss_data_ptr->heading_type;
        }
        if(gnss_data_ptr->pos_type != "NARROW_INT"){
            s->filter_status = 3;
            LOG(WARNING) << "novatel pose is not NARROW_INT, pose type is: " << gnss_data_ptr->pos_type;
        }

        auto pos_it = GnssPoseStatus.find(gnss_data_ptr->pos_type);
        if(pos_it != GnssPoseStatus.end()){
            pose_status = pos_it->second;
        }

        if(gnss_data_ptr->heading_type == "NARROW_INT"){
            heading_status = 0;
        }else{
            heading_status = 13;
        }

        s->gnss_quality = pose_status + heading_status;
    }

}  // integrated_navigation
#endif //SRC_FILTER_DEFINATION_HPP