/*********************************************************************
 * navigation support func
 * Author: Cai Zhihong
 *********************************************************************/
#ifndef NAVI_SUPPORT_H_
#define NAVI_SUPPORT_H_

#include <dirent.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <jsoncpp/json/json.h>
#include <navit_move_base/LoadNemData.h>
#include <navit_msgs/MoveBaseActionGoal.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <tf/tf.h>
#include <tf2_msgs/TFMessage.h>
#include <visualization_msgs/Marker.h>
#include <yaml-cpp/yaml.h>

#include <boost/filesystem.hpp>
#include <boost/thread.hpp>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <map>

namespace navi_support {

struct TopicItem {
    std::string topic_type;
    int number{};
    std::vector<std::string> topic_list;
    TopicItem() = default;
};

enum TopicTpye {
    nav_msgs_OccupancyGrid = 1,
    sensor_msgs_LaserScan,
    sensor_msgs_PointCloud2,
    sensor_msgs_Imu,
    nav_msgs_Path,
    nav_msgs_Odometry,
    tf2_msgs_TFMessage,
    geometry_msgs_PoseStamped,
    geometry_msgs_PoseWithCovarianceStamped,
    geometry_msgs_Twist,
    geometry_msgs_PolygonStamped,
    visualization_msgs_Marker,
    std_msgs_Float64,
    move_base_msgs_MoveBaseActionGoal
};

class NaviSupport {
   public:
    NaviSupport();

    ~NaviSupport();

    void NavigationBagRecv(const std_msgs::BoolConstPtr& data);

    template <typename MessageType>
    void MsgsCallBack(const std::string& topic, const typename MessageType::ConstPtr& msg);

    void TfCallBack(const tf2_msgs::TFMessageConstPtr& msg);

    void TimeCount(const ros::TimerEvent&);

    int HandleLogFileNum(std::string bag_prefix, const char* path, int maxLogNum);

    std::string GetCurrentTime();

    void CreatNaviDir(const std::string& prefix, std::string& file_name);

    void DeleteDir(const char* path);

    inline bool FileExist(const std::string& name) {
        std::ifstream f(name.c_str());
        return f.good();
    };

    bool CopyFile(const std::string& src, const std::string& dest, const bool& overwrite_if_exists = true);

    bool CopyNaviFileToPath(const std::string& node_file, const std::string& edge_file, const std::string& target_path);

    bool ReadFromJson(const std::istream& stream, Json::Value& val);

    bool LoadNodeStream(const std::istream& in);

    bool LoadEdgeStream(const std::istream& in);

    bool ReLoadJson(const std::string& node_file, const std::string& edge_file);

    bool LoadConfigFile(const std::string& config_file, std::vector<TopicItem>& topic_list);

    void LaunchSubscriber(const std::vector<TopicItem>& topic_item, ros::NodeHandle& private_nh);

    bool GetNemVisualData(const std::string& node_file, const std::string& edge_file);

    void NodesVisualFull(const int node_num, const long node_id, const float x, const float y, const float yaw);

    void NodesIdVisualFull(const int node_num, const long node_id, const float x, const float y, const float yaw);

    void EdgesVisualFull(const int edge_num, const long node_A, const long node_B, const int available,
                         const int direction);

    void VisualizationPub();

    void VisualThread();

   private:
    ros::NodeHandle nh_;
    ros::Publisher node_markers_pub_;
    ros::Publisher node_id_markers_pub_;
    ros::Publisher edge_markers_pub_;
    ros::Subscriber navi_bag_sub_;
    ros::Subscriber tf_static_sub_;
    std::vector<visualization_msgs::Marker> node_markers_;
    std::vector<visualization_msgs::Marker> node_id_markers_;
    std::vector<visualization_msgs::Marker> edge_markers_;
    std::vector<ros::Subscriber> subscribers_;
    std::map<long, geometry_msgs::Point> nodes_index_;

    ros::Timer navi_bag_record_timer_;
    std::vector<tf2_msgs::TFMessage> tf_static_;
    rosbag::Bag bag_;
    bool navi_bag_opend_;
    std::atomic_bool bag_opend_;
    std::string navi_bag_path_;
    std::string nem_node_path_;
    std::string nem_edge_path_;
    std::string record_bag_cfg_path_;
    int navi_bag_max_file_;
    double navi_bag_record_time_;

    boost::thread* visual_thread_;
    boost::recursive_mutex visual_mutex_;
    boost::recursive_mutex mutex_{};
    boost::mutex bag_mutex_{};
};
};  // namespace navi_support
#endif
