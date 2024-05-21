#ifndef INTEGRATED_NAV_LOGGING_H
#define INTEGRATED_NAV_LOGGING_H

//#include <glog/logging.h>
 //#include"glog/logging.h.in"
 //#include"logging.h.in"
namespace integrated_navigation {

const int GLOG_INFO = 0, GLOG_WARNING = 1, GLOG_ERROR = 2, GLOG_FATAL = 3,
  NUM_SEVERITIES = 4;
const int INFO = GLOG_INFO, WARNING = GLOG_WARNING,
  ERROR = GLOG_ERROR, FATAL = GLOG_FATAL;
#define LOG(level)    std::cerr
#define VLOG(level)   if (true) {} else std::cerr
#define DVLOG(level)  if (true) {} else std::cerr



// class GLogHandle{
// public:
//     GLogHandle(const char* program){
//         google::InitGoogleLogging(program);
//         FLAGS_minloglevel = 0;
//         FLAGS_stderrthreshold = 1;
//         FLAGS_log_dir          = "/tmp";
//         FLAGS_alsologtostderr  = true;
//         FLAGS_colorlogtostderr = true;
//         FLAGS_max_log_size = 1000;
//         // FLAGS_stop_logging_if_full_disk = true;

//         google::InstallFailureSignalHandler();
//     }

//     ~GLogHandle() { google::ShutdownGoogleLogging(); }

// };

}// namespace integrated_navigation

#endif