#ifndef NAVIT_COMMON_LOG_H
#define NAVIT_COMMON_LOG_H
#include <ros/ros.h>
#include <string>
#include <sstream>

inline std::string get_filename(const char* path) {
    static std::string filename;
    if (filename.empty()) {
        std::string full_path = path;
        size_t last_slash_pos = full_path.find_last_of("/\\");
        if (std::string::npos != last_slash_pos) {
            filename = full_path.substr(last_slash_pos + 1);
        } else {
            filename = full_path;
        }
    }
    return filename;
}

#define NAVIT_ROS_INFO_STREAM(msg) { \
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) { \
        std::ostringstream oss; \
        oss << "[" << get_filename(__FILE__) << ":" << __LINE__ << "] " << msg; \
        ROS_INFO_STREAM(oss.str()); \
    } \
}
#define NAVIT_ROS_WARN_STREAM(msg) { \
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn)) { \
        std::ostringstream oss; \
        oss << "[" << get_filename(__FILE__) << ":" << __LINE__ << "] " << msg; \
        ROS_WARN_STREAM(oss.str()); \
    } \
}
#define NAVIT_ROS_ERROR_STREAM(msg) { \
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Error)) { \
        std::ostringstream oss; \
        oss << "[" << get_filename(__FILE__) << ":" << __LINE__ << "] " << msg; \
        ROS_ERROR_STREAM(oss.str()); \
    } \
}
#endif // NAVIT_COMMON_LOG_H