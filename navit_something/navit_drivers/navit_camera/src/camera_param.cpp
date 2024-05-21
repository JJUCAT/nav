#include <navit_camera/camera_param.h>

namespace navit_camera {

    CameraParam::CameraParam(ros::NodeHandle& nh):
        nh_(nh)
    {
        loadCameraParam();
    }

    void CameraParam::loadCameraParam()
    {
        XmlRpc::XmlRpcValue camera_info;
        if (!nh_.getParam("camera_info", camera_info))
        {
            ROS_ERROR("No cameras yaml config is found!");
            return;
        }

        int camera_num = camera_info.size();
        cameras_param_.resize(camera_num);

        for (unsigned int i = 0; i < camera_num; i++)
        {
            XmlRpc::XmlRpcValue elem = camera_info[i];
            cameras_param_[i].camera_name = static_cast<std::string>(elem["camera_name"]); 
            cameras_param_[i].camera_type = static_cast<std::string>(elem["camera_type"]);
            cameras_param_[i].camera_path = static_cast<std::string>(elem["camera_path"]);
            cameras_param_[i].resolution_width =  static_cast<int>(elem["resolution_width"]);
            cameras_param_[i].resolution_height = static_cast<int>(elem["resolution_height"]);
            cameras_param_[i].width_offset=  static_cast<int>(elem["width_offset"]);
            cameras_param_[i].height_offset= static_cast<int>(elem["height_offset"]);
            cameras_param_[i].fps = static_cast<int>(elem["fps"]);
            cameras_param_[i].auto_exposure = static_cast<bool>(elem["auto_exposure"]);
            cameras_param_[i].exposure_value = static_cast<int>(elem["exposure_value"]);
            cameras_param_[i].exposure_time = static_cast<int>(elem["exposure_time"]);
            cameras_param_[i].auto_white_balance = static_cast<bool>(elem["auto_white_balance"]);
            cameras_param_[i].auto_gain = static_cast<bool>(elem["auto_gain"]);
            cameras_param_[i].contrast = static_cast<int>(elem["contrast"]);


            //TODO: fix it
            int camera_m_size = elem["camera_matrix"].size();
            //std::vector<double> camera_matrix_vector = elem["camera_matrix"];
            std::vector<double> camera_matrix_vector = {0.0,0.0,0.0,
                                                        0.0,0.0,0.0,
                                                        0.0,0.0,0.0};
                
            double camera_m[camera_m_size];
            std::copy(camera_matrix_vector.begin(),
                      camera_matrix_vector.end(),
                      camera_m);
            cameras_param_[i].camera_matrix = cv::Mat(3,3, CV_64F, camera_m).clone();

            int rows = elem["camera_distortion"].size();
            //std::vector<double> camera_distortion_vector = elem["camera_distortion_vector"];
            std::vector<double> camera_distortion_vector = {0.0,
                                                            0.0,
                                                            0.0,
                                                            0.0,
                                                            0.0}; 
            double camera_dis[rows];
            std::copy(camera_distortion_vector.begin(),
                      camera_distortion_vector.end(),
                      camera_dis);
            cameras_param_[i].camera_distortion = cv::Mat(rows,1,CV_64F, camera_dis).clone();
            
            cameras_param_[i].ros_camera_info = boost::make_shared<sensor_msgs::CameraInfo>();
            cameras_param_[i].ros_camera_info->header.frame_id = cameras_param_[i].camera_name;
            cameras_param_[i].ros_camera_info->width = cameras_param_[i].resolution_width;
            cameras_param_[i].ros_camera_info->height = cameras_param_[i].resolution_height;
            cameras_param_[i].ros_camera_info->D = cameras_param_[i].camera_distortion;
            cameras_param_[i].ros_camera_info->roi.x_offset = cameras_param_[i].width_offset;
            cameras_param_[i].ros_camera_info->roi.y_offset = cameras_param_[i].height_offset;
            std::copy(camera_m, camera_m + camera_m_size, cameras_param_[i].ros_camera_info->K.begin());
        }
    }

    std::vector<CameraInfo>& CameraParam::getCameraParam()
    {
        return cameras_param_;
    }
}
