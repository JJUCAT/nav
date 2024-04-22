/*********************************************************************
 * navigation support func
 * Author: Cai Zhihong
 *********************************************************************/
#include "navit_bt_nodes/navi_support.h"

namespace navi_support {

template <typename MessageType>
ros::Subscriber SubscribeWithHandler(void (NaviSupport::*handler)(const std::string&,
                                                                  const typename MessageType::ConstPtr&),
                                     const std::string& topic, ros::NodeHandle& node_handle,
                                     NaviSupport* const navi_support) {
    return node_handle.subscribe<MessageType>(
        topic, 1,
        boost::function<void(const typename MessageType::ConstPtr&)>(
            [navi_support, handler, topic](const typename MessageType::ConstPtr& msg) {
                (navi_support->*handler)(topic, msg);
            }));
}

NaviSupport::NaviSupport() : navi_bag_opend_(false), bag_opend_(false) {
    ros::NodeHandle private_nh("~");
    node_markers_pub_    = nh_.advertise<visualization_msgs::Marker>("nem_nodes", false, 1);
    edge_markers_pub_    = nh_.advertise<visualization_msgs::Marker>("nem_edges", false, 1);
    node_id_markers_pub_ = nh_.advertise<visualization_msgs::Marker>("nem_nodes_id", false, 1);
    navi_bag_sub_        = nh_.subscribe<std_msgs::Bool>("navi_bag_record", 10, &NaviSupport::NavigationBagRecv, this);
    tf_static_sub_       = nh_.subscribe<tf2_msgs::TFMessage>("tf_static", 5, &NaviSupport::TfCallBack, this);
    visual_thread_       = new boost::thread(boost::bind(&NaviSupport::VisualThread, this));
    std::string navi_bag_path, nem_node_path, nem_edge_path, record_bag_cfg_path;
    navi_bag_path       = "/home/robot/.ros/";
    nem_node_path       = "/home/robot/.ros/sync_data/actual_nodes.json";
    nem_edge_path       = "/home/robot/.ros/sync_data/actual_edges.json";
    record_bag_cfg_path = "/home/robot/install/share/move_base/cfg/navi_bag_record_topics.yaml";
    private_nh.param("navi_bag_max_file", navi_bag_max_file_, 2);
    private_nh.param("navi_bag_record_time", navi_bag_record_time_, double(300.0));
    private_nh.param("navi_bag_path", navi_bag_path_, navi_bag_path);
    private_nh.param("nem_node_data_default_path", nem_node_path_, nem_node_path);
    private_nh.param("nem_edge_data_default_path", nem_edge_path_, nem_edge_path);
    private_nh.param("record_bag_cfg_path", record_bag_cfg_path_, record_bag_cfg_path);

    std::vector<TopicItem> record_topics;
    LoadConfigFile(record_bag_cfg_path_, record_topics);
    LaunchSubscriber(record_topics, nh_);
    ROS_INFO("[navi_support]: init finished");
}

NaviSupport::~NaviSupport() {
    visual_thread_->interrupt();
    visual_thread_->detach();
    delete visual_thread_;
}

void NaviSupport::VisualThread() { VisualizationPub(); }

template <typename MessageType>
void NaviSupport::MsgsCallBack(const std::string& topic, const typename MessageType::ConstPtr& msg) {
    if (bag_opend_) {
        boost::lock_guard<boost::recursive_mutex> guard(mutex_);
        try {
            bag_.write(topic, ros::Time::now(), msg);
        } catch (rosbag::BagException& e){
            ROS_ERROR("rosbag exception %s", e.what());
        }
    }
}

void NaviSupport::TfCallBack(const tf2_msgs::TFMessageConstPtr& msg) {
    bag_mutex_.lock();
    tf_static_.emplace_back(*msg);
    bag_mutex_.unlock();
}

void NaviSupport::NavigationBagRecv(const std_msgs::BoolConstPtr& data) {
    ROS_INFO("[navi_support]: Navigation callback data: %d", data->data);
    if (data->data) {
        ROS_INFO("[navi_support]: Received navigation bag record topic");
        if (bag_opend_) {
            boost::lock_guard<boost::recursive_mutex> guard(mutex_);
            bag_.close();
            bag_opend_ = false;
        }
        if (!bag_opend_) {
            // 0.清理多余的navi_bag_path_路径下navi_开头的文件夹，维持最大数量为navi_bag_max_file_
            HandleLogFileNum("navi_", navi_bag_path_.c_str(), navi_bag_max_file_);
            // 1.创建以navi_开头加上当前时间的文件夹
            std::string file_local_path;
            CreatNaviDir(navi_bag_path_ + "navi_", file_local_path);
            // 2.开始录包，存放于刚创建的文件夹中
            char szName[256] = {0};
            struct tm* ptm;
            long ts;
            ts  = time(nullptr);
            ptm = localtime(&ts);
            sprintf(szName, "navi_%04d%02d%02d%02d%02d%02d.bag", ptm->tm_year + 1900, ptm->tm_mon + 1, ptm->tm_mday,
                    ptm->tm_hour, ptm->tm_min, ptm->tm_sec);

            {
                boost::lock_guard<boost::recursive_mutex> guard(mutex_);
                bag_.open(file_local_path + '/' + szName, rosbag::bagmode::Write);
                bag_.setCompression(rosbag::compression::LZ4);
                for (const auto& tf : tf_static_) {
                    bag_.write("tf_static", ros::Time::now(), tf);
                }
                bag_opend_ = true;
            }

            // 3.将当前的点边文件拷贝于刚创建的文件夹中
            if (!CopyNaviFileToPath(nem_node_path_, nem_edge_path_, file_local_path))
                ROS_ERROR("[navi_support]: Copy nem data failed, please check the nem data!");

            navi_bag_record_timer_ =
                nh_.createTimer(ros::Duration(navi_bag_record_time_), &NaviSupport::TimeCount, this, true);
            ROS_INFO("[navi_support]: Start record navigation bag");
        }
    } else {
        ROS_INFO("[navi_support]: Received navigation bag end record topic");
        if (bag_opend_) {
            boost::lock_guard<boost::recursive_mutex> guard(mutex_);
            bag_.close();
            bag_opend_ = false;
            ROS_INFO("[navi_support]: End record navigation bag");
        }
    }
}

int NaviSupport::HandleLogFileNum(std::string bag_prefix, const char* path, int max_bag_num) {
    DIR* open_D;
    struct dirent* file;
    std::vector<std::string> bag_name;

    if (!(open_D = opendir(path))) {
        return -1;
    }

    while ((file = readdir(open_D)) != nullptr) {
        if (strncmp(bag_prefix.c_str(), file->d_name, strlen(bag_prefix.c_str())) == 0) {
            bag_name.emplace_back(file->d_name);
        } else {
            continue;
        }
    }

    int info_bag_num = bag_name.size();
    ROS_INFO("[navi_support]: start with the character \"%s\" bag num is %d", bag_prefix.c_str(), info_bag_num);
    while (info_bag_num >= max_bag_num) {
        std::sort(bag_name.begin(), bag_name.end());
        std::string originalFileName = bag_name[0];
        std::string P                = path;
        std::string wholePath        = P + originalFileName;
        DeleteDir(wholePath.data());
        bag_name.erase(bag_name.begin());
        info_bag_num--;
    }
    closedir(open_D);
    return 0;
}

void NaviSupport::DeleteDir(const char* path) {
    std::string dir_path = path;
    ROS_INFO("[navi_support]: delete dir path = %s", dir_path.c_str());
    DIR* d = opendir(path);  //打开这个目录
    if (d == nullptr) {
        ROS_ERROR("[navi_support]: Cannot open the dir, %s", path);
        return;
    }
    struct dirent* dt = nullptr;
    dt                = readdir(d);
    while (dt)  //读目录项
    {
        std::string file_path = dir_path + "/" + dt->d_name;
        unlink(file_path.c_str());
        dt = readdir(d);
    }
    closedir(d);
    rmdir(path);
}

void NaviSupport::CreatNaviDir(const std::string& prefix, std::string& file_name) {
    std::string dir_name;
    dir_name  = prefix + GetCurrentTime();
    file_name = dir_name;
    mkdir(dir_name.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
}

std::string NaviSupport::GetCurrentTime() {
    time_t cur_time;
    struct tm* cur_time_info;
    time(&cur_time);
    cur_time_info = localtime(&cur_time);
    std::string cur_time_str;
    int n_zero = 2;
    std::string month = std::to_string(cur_time_info->tm_mon + 1);
    std::string day   = std::to_string(cur_time_info->tm_mday);
    std::string hour  = std::to_string(cur_time_info->tm_hour);
    std::string min   = std::to_string(cur_time_info->tm_min);
    std::string sec   = std::to_string(cur_time_info->tm_sec);
    cur_time_str = std::to_string(cur_time_info->tm_year + 1900) + 
                   std::string(n_zero - month.length(), '0') + month +
                   std::string(n_zero - day.length(), '0')   + day   +
                   std::string(n_zero - hour.length(), '0')  + hour  +
                   std::string(n_zero - min.length(), '0')   + min   +
                   std::string(n_zero - sec.length(), '0')   + sec;

    return cur_time_str;
}

void NaviSupport::TimeCount(const ros::TimerEvent&) {
    if (bag_opend_) {
        boost::lock_guard<boost::recursive_mutex> guard(mutex_);
        bag_.close();
        bag_opend_ = false;
        ROS_INFO("[navi_support]: navi bag record time out, and kill the node");
    }
}

bool NaviSupport::CopyFile(const std::string& src, const std::string& dest, const bool& overwrite_if_exists) {
    using namespace boost::filesystem;
    boost::filesystem::path src_path(src);
    boost::filesystem::path dest_path(dest);
    try {
        boost::filesystem::copy_file(src_path, dest_path,
                                     overwrite_if_exists ? boost::filesystem::copy_option::overwrite_if_exists
                                                         : boost::filesystem::copy_option::none);
        return true;
    } catch (std::exception& e) {
        ROS_ERROR("copyFile error: %s", e.what());
        return false;
    }
}

bool NaviSupport::CopyNaviFileToPath(const std::string& node_file, const std::string& edge_file,
                                     const std::string& target_path) {
    boost::filesystem::path dir_path = boost::filesystem::path(target_path);
    std::string node_file_path       = (dir_path / boost::filesystem::path(node_file).filename()).string();
    std::string edge_file_path       = (dir_path / boost::filesystem::path(edge_file).filename()).string();

    bool copy_node_file = CopyFile(node_file, node_file_path);
    bool copy_edge_file = CopyFile(edge_file, edge_file_path);
    return copy_node_file && copy_edge_file;
}

bool NaviSupport::ReadFromJson(const std::istream& stream, Json::Value& val) {
    std::ostringstream sin;
    sin << stream.rdbuf();
    std::string str = sin.str();
    Json::Reader reader;
    return reader.parse(str, val);
}

bool NaviSupport::LoadNodeStream(const std::istream& in) {
    if (!in) {
        return false;
    }
    node_markers_.clear();
    node_id_markers_.clear();
    nodes_index_.clear();
    Json::Value node_array;
    if (!ReadFromJson(in, node_array))
        return false;
    int node_num = 0;
    for (const auto& node : node_array) {
        long node_id;
        float x, y, r;
        node_id = node["id"].asInt64();
        x       = node["x"].asFloat();
        y       = node["y"].asFloat();
        r       = node["r"].asFloat();
        NodesVisualFull(node_num, node_id, x, y, r);
        NodesIdVisualFull(node_num, node_id, x, y, r);
        geometry_msgs::Point this_node;
        this_node.x = x;
        this_node.y = y;
        this_node.z = 0.0;
        nodes_index_.insert(std::pair<long, geometry_msgs::Point>(node_id, this_node));
        node_num++;
    }
    return true;
}

bool NaviSupport::LoadEdgeStream(const std::istream& in) {
    if (!in) {
        return false;
    }
    edge_markers_.clear();
    Json::Value edge_array;
    if (!ReadFromJson(in, edge_array))
        return false;

    int edge_num = 0;
    for (const auto& edge : edge_array) {
        int available, direction;
        long nodeA, nodeB;
        nodeA     = edge["p1"].asInt64();
        nodeB     = edge["p2"].asInt64();
        available = edge["available"].asInt();
        direction = edge["direction"].asInt();
        EdgesVisualFull(edge_num, nodeA, nodeB, available, direction);
        edge_num++;
    }
    return true;
}

bool NaviSupport::ReLoadJson(const std::string& node_file, const std::string& edge_file) {
    std::ifstream node_stream(node_file.c_str(), std::ios::in);
    std::ifstream edge_stream(edge_file.c_str(), std::ios::in);
    if (!LoadNodeStream(node_stream) || !LoadEdgeStream(edge_stream))
        return false;
    return true;
}

bool NaviSupport::LoadConfigFile(const std::string& config_file, std::vector<TopicItem>& topic_list) {
    try {
        YAML::Node yaml_file = YAML::LoadFile(config_file);
        YAML::Node nodes     = yaml_file["all_topics"];
        TopicItem topic_item;
        for (YAML::iterator it = nodes.begin(); it != nodes.end(); ++it) {
            const YAML::Node& node = *it;
            topic_item.topic_type  = node["topic_type"].as<std::string>();
            topic_item.number      = node["number"].as<int>();

            for (YAML::const_iterator topic = node["topics"].begin(); topic != node["topics"].end(); ++topic) {
                topic_item.topic_list.emplace_back(topic->second.as<std::string>());
            }
            topic_list.emplace_back(topic_item);
            topic_item.topic_list.clear();
        }
    } catch (std::exception& e) {
        ROS_ERROR("Error load yaml: %s", e.what());
        return false;
    }
    return true;
}

void NaviSupport::LaunchSubscriber(const std::vector<TopicItem>& topic_item, ros::NodeHandle& private_nh) {
    if (!topic_item.empty()) {
        for (const auto& it : topic_item) {
            int topic_num    = it.number;
            auto topic_lists = it.topic_list;
            auto mode        = static_cast<TopicTpye>(topic_num);
            ROS_INFO("Record topic type %s", it.topic_type.c_str());
            switch (mode) {
                case nav_msgs_OccupancyGrid:
                    for (auto topic = topic_lists.begin(); topic != topic_lists.end(); ++topic) {
                        subscribers_.push_back(SubscribeWithHandler<nav_msgs::OccupancyGrid>(
                            &NaviSupport::MsgsCallBack<nav_msgs::OccupancyGrid>, *topic, private_nh, this));
                    }
                    break;
                case sensor_msgs_LaserScan:
                    for (auto topic = topic_lists.begin(); topic != topic_lists.end(); ++topic) {
                        subscribers_.push_back(SubscribeWithHandler<sensor_msgs::LaserScan>(
                            &NaviSupport::MsgsCallBack<sensor_msgs::LaserScan>, *topic, private_nh, this));
                    }
                    break;
                case sensor_msgs_PointCloud2:
                    for (auto topic = topic_lists.begin(); topic != topic_lists.end(); ++topic) {
                        subscribers_.push_back(SubscribeWithHandler<sensor_msgs::PointCloud2>(
                            &NaviSupport::MsgsCallBack<sensor_msgs::PointCloud2>, *topic, private_nh, this));
                    }
                    break;
                case sensor_msgs_Imu:
                    for (auto topic = topic_lists.begin(); topic != topic_lists.end(); ++topic) {
                        subscribers_.push_back(SubscribeWithHandler<sensor_msgs::Imu>(
                            &NaviSupport::MsgsCallBack<sensor_msgs::Imu>, *topic, private_nh, this));
                    }
                    break;
                case nav_msgs_Path:
                    for (auto topic = topic_lists.begin(); topic != topic_lists.end(); ++topic) {
                        subscribers_.push_back(SubscribeWithHandler<nav_msgs::Path>(
                            &NaviSupport::MsgsCallBack<nav_msgs::Path>, *topic, private_nh, this));
                    }
                    break;
                case nav_msgs_Odometry:
                    for (auto topic = topic_lists.begin(); topic != topic_lists.end(); ++topic) {
                        subscribers_.push_back(SubscribeWithHandler<nav_msgs::Odometry>(
                            &NaviSupport::MsgsCallBack<nav_msgs::Odometry>, *topic, private_nh, this));
                    }
                    break;
                case tf2_msgs_TFMessage:
                    for (auto topic = topic_lists.begin(); topic != topic_lists.end(); ++topic) {
                        subscribers_.push_back(SubscribeWithHandler<tf2_msgs::TFMessage>(
                            &NaviSupport::MsgsCallBack<tf2_msgs::TFMessage>, *topic, private_nh, this));
                    }
                    break;
                case geometry_msgs_PoseStamped:
                    for (auto topic = topic_lists.begin(); topic != topic_lists.end(); ++topic) {
                        subscribers_.push_back(SubscribeWithHandler<geometry_msgs::PoseStamped>(
                            &NaviSupport::MsgsCallBack<geometry_msgs::PoseStamped>, *topic, private_nh, this));
                    }
                    break;
                case geometry_msgs_PoseWithCovarianceStamped:
                    for (auto topic = topic_lists.begin(); topic != topic_lists.end(); ++topic) {
                        subscribers_.push_back(SubscribeWithHandler<geometry_msgs::PoseWithCovarianceStamped>(
                            &NaviSupport::MsgsCallBack<geometry_msgs::PoseWithCovarianceStamped>, *topic, private_nh,
                            this));
                    }
                    break;
                case geometry_msgs_Twist:
                    for (auto topic = topic_lists.begin(); topic != topic_lists.end(); ++topic) {
                        subscribers_.push_back(SubscribeWithHandler<geometry_msgs::Twist>(
                            &NaviSupport::MsgsCallBack<geometry_msgs::Twist>, *topic, private_nh, this));
                    }
                    break;
                case geometry_msgs_PolygonStamped:
                    for (auto topic = topic_lists.begin(); topic != topic_lists.end(); ++topic) {
                        subscribers_.push_back(SubscribeWithHandler<geometry_msgs::PolygonStamped>(
                            &NaviSupport::MsgsCallBack<geometry_msgs::PolygonStamped>, *topic, private_nh, this));
                    }
                    break;
                case visualization_msgs_Marker:
                    for (auto topic = topic_lists.begin(); topic != topic_lists.end(); ++topic) {
                        subscribers_.push_back(SubscribeWithHandler<visualization_msgs::Marker>(
                            &NaviSupport::MsgsCallBack<visualization_msgs::Marker>, *topic, private_nh, this));
                    }
                    break;
                case std_msgs_Float64:
                    for (auto topic = topic_lists.begin(); topic != topic_lists.end(); ++topic) {
                        subscribers_.push_back(SubscribeWithHandler<std_msgs::Float64>(
                            &NaviSupport::MsgsCallBack<std_msgs::Float64>, *topic, private_nh, this));
                    }
                    break;
                case move_base_msgs_MoveBaseActionGoal:
                    for (auto topic = topic_lists.begin(); topic != topic_lists.end(); ++topic) {
                        subscribers_.push_back(SubscribeWithHandler<navit_msgs::MoveBaseActionGoal>(
                            &NaviSupport::MsgsCallBack<navit_msgs::MoveBaseActionGoal>, *topic, private_nh, this));
                    }
                    break;
                default:
                    break;
            }
        }
    } else {
        ROS_INFO("Record topics is empty");
    }
}

bool NaviSupport::GetNemVisualData(const std::string& node_file, const std::string& edge_file) {
    ROS_INFO("[navi_support]: Get nem data cmd,\n node file path = %s,\n edge file path = %s", node_file.c_str(),
             edge_file.c_str());
    if (FileExist(node_file) && FileExist(edge_file)) {
        ROS_INFO("[navi_support]: Nem data is in target path");
        if (!ReLoadJson(node_file, edge_file)) {
            ROS_ERROR("Load nem data failed! 点边文件加载失败");
            return false;
        }
        return true;
    } else {
        ROS_ERROR("[navi_support]: Node path = %s, edge path = %s. But nem data maybe is empty! "
                  "点边文件中存在一个或两个为空现象",
                  node_file.c_str(), edge_file.c_str());
        return false;
    }
}

void NaviSupport::NodesVisualFull(const int node_num, const long node_id, const float x, const float y,
                                  const float yaw) {
    visualization_msgs::Marker this_node;
    this_node.header.frame_id  = "map";
    this_node.header.stamp     = ros::Time::now();
    this_node.ns               = "nem_node";
    this_node.id               = node_num;
    this_node.type             = visualization_msgs::Marker::ARROW;
    this_node.action           = visualization_msgs::Marker::ADD;
    this_node.pose.position.x  = x;
    this_node.pose.position.y  = y;
    this_node.pose.position.z  = 0.4;
    this_node.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    geometry_msgs::Point node_begin, node_end;
    node_begin.x = node_begin.y = node_begin.z = node_end.y = node_end.z = 0.0;
    node_end.x                                                           = 0.3;
    this_node.points.push_back(node_begin);
    this_node.points.push_back(node_end);
    this_node.scale.x  = 0.15;
    this_node.scale.y  = 0.3;
    this_node.scale.z  = 0.15;
    this_node.color.a  = 1.0;  // Don't forget to set the alpha!
    this_node.color.r  = 1.0;
    this_node.color.g  = 0.0;
    this_node.color.b  = 0.0;
    this_node.lifetime = ros::Duration();
    boost::unique_lock<boost::recursive_mutex> lock(visual_mutex_);
    node_markers_.push_back(this_node);
    lock.unlock();
}

void NaviSupport::NodesIdVisualFull(const int node_num, const long node_id, const float x, const float y,
                                    const float yaw) {
    visualization_msgs::Marker this_node;
    this_node.header.frame_id  = "map";
    this_node.header.stamp     = ros::Time::now();
    this_node.ns               = "nem_node";
    this_node.id               = node_num;
    this_node.type             = visualization_msgs::Marker::TEXT_VIEW_FACING;
    this_node.action           = visualization_msgs::Marker::ADD;
    this_node.pose.position.x  = x;
    this_node.pose.position.y  = y;
    this_node.pose.position.z  = 0.6;
    this_node.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    this_node.text             = std::to_string(node_id);
    this_node.scale.x          = 0.5;
    this_node.scale.y          = 0.5;
    this_node.scale.z          = 0.5;
    this_node.color.a          = 1.0;  // Don't forget to set the alpha!
    this_node.color.r          = 0.0;
    this_node.color.g          = 0.0;
    this_node.color.b          = 0.0;
    this_node.lifetime         = ros::Duration();
    boost::unique_lock<boost::recursive_mutex> lock(visual_mutex_);
    node_id_markers_.push_back(this_node);
    lock.unlock();
}

void NaviSupport::EdgesVisualFull(const int edge_num, const long node_A, const long node_B, const int available,
                                  const int direction) {
    visualization_msgs::Marker this_node;
    this_node.header.frame_id = "map";
    this_node.header.stamp    = ros::Time::now();
    this_node.ns              = "nem_node";
    this_node.id              = edge_num;
    this_node.action          = visualization_msgs::Marker::ADD;
    if (direction == 1) {
        this_node.type                                              = visualization_msgs::Marker::ARROW;
        std::map<long, geometry_msgs::Point>::iterator node_A_index = nodes_index_.find(node_A);
        std::map<long, geometry_msgs::Point>::iterator node_B_index = nodes_index_.find(node_B);
        this_node.points.push_back(node_A_index->second);
        this_node.points.push_back(node_B_index->second);
        this_node.color.r         = 0.0;
        this_node.color.g         = 1.0;
        this_node.color.b         = 0.0;
        this_node.pose.position.z = 0.4;
    } else if (direction == -1) {
        this_node.type                                              = visualization_msgs::Marker::ARROW;
        this_node.type                                              = visualization_msgs::Marker::ARROW;
        std::map<long, geometry_msgs::Point>::iterator node_A_index = nodes_index_.find(node_A);
        std::map<long, geometry_msgs::Point>::iterator node_B_index = nodes_index_.find(node_B);
        this_node.points.push_back(node_B_index->second);
        this_node.points.push_back(node_A_index->second);
        this_node.color.r         = 0.0;
        this_node.color.g         = 1.0;
        this_node.color.b         = 0.0;
        this_node.pose.position.z = 0.0;
    } else if (direction == 2) {
        this_node.type                                              = visualization_msgs::Marker::LINE_STRIP;
        std::map<long, geometry_msgs::Point>::iterator node_A_index = nodes_index_.find(node_A);
        std::map<long, geometry_msgs::Point>::iterator node_B_index = nodes_index_.find(node_B);
        this_node.points.push_back(node_A_index->second);
        this_node.points.push_back(node_B_index->second);
        this_node.color.r         = 139.0;
        this_node.color.g         = 69.0;
        this_node.color.b         = 19.0;
        this_node.pose.position.z = 0.3;
    }else {
        this_node.type                                              = visualization_msgs::Marker::LINE_STRIP;
        std::map<long, geometry_msgs::Point>::iterator node_A_index = nodes_index_.find(node_A);
        std::map<long, geometry_msgs::Point>::iterator node_B_index = nodes_index_.find(node_B);
        this_node.points.push_back(node_A_index->second);
        this_node.points.push_back(node_B_index->second);
        this_node.color.r         = 0.0;
        this_node.color.g         = 0.0;
        this_node.color.b         = 1.0;
        this_node.pose.position.z = 0.2;
    }
    this_node.scale.x = 0.1;
    this_node.scale.y = 0.2;
    this_node.scale.z = 0.0;

    this_node.color.a = 1.0;  // Don't forget to set the alpha!

    if (available != 1) {
        this_node.color.r = 0.0;
        this_node.color.g = 0.0;
        this_node.color.b = 0.0;
    }
    this_node.lifetime = ros::Duration();
    boost::unique_lock<boost::recursive_mutex> lock(visual_mutex_);
    edge_markers_.push_back(this_node);
    lock.unlock();
}

void NaviSupport::VisualizationPub() {
    ros::Rate r(2);
    while (ros::ok()) {
        boost::unique_lock<boost::recursive_mutex> lock(visual_mutex_);
        for (int i = 0; i < node_markers_.size(); i++) {
            node_markers_pub_.publish(node_markers_[i]);
        }
        for (int k = 0; k < node_id_markers_.size(); k++) {
            node_id_markers_pub_.publish(node_id_markers_[k]);
        }
        for (int j = 0; j < edge_markers_.size(); j++) {
            edge_markers_pub_.publish(edge_markers_[j]);
        }
        lock.unlock();
        ros::spinOnce();
        r.sleep();
    }
}

}  // namespace navi_support
