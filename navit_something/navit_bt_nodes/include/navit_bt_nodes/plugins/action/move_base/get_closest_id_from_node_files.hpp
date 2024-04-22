#include <string>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

#include "navit_bt_nodes/navi_support.h"
#include "behaviortree_cpp_v3/action_node.h"
/**
 * @brief A BT node 
 */
namespace navit_bt_nodes {
class GetClosestId : public BT::ActionNodeBase
{
public:
    GetClosestId(const std::string& xml_tag_name, const BT::NodeConfiguration& conf) : BT::ActionNodeBase(xml_tag_name, conf)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("nodes_path", "nodes directory path"),
            BT::InputPort<std::string>("edges_path", "edges directory path"),
            BT::InputPort<geometry_msgs::Pose>("current_pos"),
            BT::OutputPort<uint64_t>("closest_id", "distance"),
        };
    }

private:
    void halt() override
    {
    }
    inline float distance(geometry_msgs::Pose& pose, geometry_msgs::Point& point) {
        return sqrt(pow(pose.position.x - point.x, 2) + pow(pose.position.y - point.y, 2));
    }
    
    BT::NodeStatus tick() override {
        ROS_INFO("GetClosestId tick");
        geometry_msgs::Pose current_pose;

        getInput("current_pos", current_pose);

        float final_dis = std::numeric_limits<float>::max();
        float dis = 0.0;
        uint64_t closest_id = 0;
        for (auto node : nodes_index_) {
            dis = distance(current_pose, node.second);
            if (dis < final_dis) {
                final_dis = dis;
                closest_id = node.first;
            }
        }
        setOutput("closest_id", closest_id);
        return BT::NodeStatus::SUCCESS;
    };

    bool ReadFromJson(const std::istream& stream, Json::Value& val) {
        std::ostringstream sin;
        sin << stream.rdbuf();
        std::string str = sin.str();
        Json::Reader reader;
        return reader.parse(str, val);
    }

    bool LoadNodeStream(const std::istream& in) {
        if (!in) {
            return false;
        }
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
            geometry_msgs::Point this_node;
            this_node.x = x;
            this_node.y = y;
            this_node.z = 0.0;
            nodes_index_.insert(std::pair<long, geometry_msgs::Point>(node_id, this_node));
            node_num++;
        }
        return true;
    }
    std::map<long, geometry_msgs::Point> nodes_index_;
};
} // namespace navit_bt_nodes