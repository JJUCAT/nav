#ifndef JSON_UTIL_H
#define JSON_UTIL_H

#include <jsoncpp/json/json.h>
#include "nem_global_planner/cedge.h"
#include "nem_global_planner/cnode.h"
#include "nem_global_planner/common.h"
#include "nem_global_planner/pose.h"
#include "nem_global_planner/util.h"

namespace json_util {

/** An iterator implementing Bresenham Ray-Tracing by resolution of distance.*/
class JsonUtil {
   public:
    bool ReadFromJson(const std::istream &node_stream, Json::Value &val) {
        std::ostringstream sin;
        sin << node_stream.rdbuf();
        std::string str = sin.str();
        Json::Reader reader;
        return reader.parse(str, val);
    }

    bool LoadNodeStream(std::istream &in, vector<CNode> &nodes, CNode::IdType &node_max_id,
                        map<CNode::IdType, CNode::IdType> &node_id_to_index) {
        if (!in) {
            return false;
        }
        Json::Value node_array;
        if (!ReadFromJson(in, node_array))
            return false;
        for (const auto &node : node_array) {
            CNode::IdType type, map_id, node_id;
            float x, y, r;
            node_id = node["id"].asInt64();
            map_id  = node["map_id"].asInt64();
            type    = node["type"].asInt64();
            x       = node["x"].asFloat();
            y       = node["y"].asFloat();
            r       = node["r"].asFloat();
            AddNode(x, y, r, map_id, type, node_id, nodes, node_max_id, node_id_to_index);
        }
        return true;
    }

    bool LoadEdgeStream(std::istream &in, vector<CNode> &nodes, vector<CEdge> &edges, CNode::IdType &edge_max_id,
                        map<CNode::IdType, CNode::IdType> &node_id_to_index,
                        map<CNode::IdType, CNode::IdType> &edge_id_to_index) {
        if (!in) {
            return false;
        }
        Json::Value edge_array;
        if (!ReadFromJson(in, edge_array))
            return false;

        for (const auto &edge : edge_array) {
            int available, direction, avoidance, anti_drop;
            CNode::IdType nodeA, nodeB;
            float max_speed;
            nodeA     = edge["p1"].asInt64();
            nodeB     = edge["p2"].asInt64();
            available = edge["available"].asBool();
            max_speed = edge["max_speed"].asFloat();
            direction = edge["direction"].asInt();
            avoidance = edge["avoidance"].asBool();
            anti_drop = edge["anti_drop"].asBool();
            //以下参数暂不使用
            float leftDist      = 1.0;
            float rightDist     = 1.0;
            float minConfidence = 0.3;
            int isUltraOn       = 1;
            AddEdge(nodeA, nodeB, max_speed, leftDist, rightDist, available, minConfidence,
                    (CEdge::DIRECTION_FIXED)direction, isUltraOn == 1, anti_drop == 1, avoidance == 1, nodes, edges,
                    edge_max_id, node_id_to_index, edge_id_to_index);
        }
        return true;
    }

    void AddNode(float x, float y, float r, CNode::IdType map_id, int type, CNode::IdType id, vector<CNode> &nodes, CNode::IdType &node_max_id,
                 map<CNode::IdType, CNode::IdType> &node_id_to_index) {
        CNode temp(x, y, r, map_id, type, id, "unset");
        nodes.push_back(temp);
        if (id > node_max_id)
            node_max_id = id;
        node_id_to_index.insert(pair<CNode::IdType, CNode::IdType>(id, nodes.size() - 1));
    }

    void AddEdge(CNode::IdType A, CNode::IdType B, float speedRate, float leftDist, float rightDist,
                 unsigned int nAvaiable, float _minConfidence, CEdge::DIRECTION_FIXED _fixDirection, bool _isUltraOn,
                 bool _isFallOn, bool _isLaserOn, vector<CNode> &nodes, vector<CEdge> &edges, CNode::IdType &edge_max_id,
                 map<CNode::IdType, CNode::IdType> &node_id_to_index, map<CNode::IdType, CNode::IdType> &edge_id_to_index) {
        if (!node_id_to_index.count(A) || !node_id_to_index.count(B)) {
            return;
        }

        stringstream ss;
        ss << edge_max_id;
        string edgeName = ss.str();

        nodes[node_id_to_index[A]].SetAdjacentNodes(B);
        nodes[node_id_to_index[B]].SetAdjacentNodes(A);

        CEdge temp(nodes[node_id_to_index[A]], nodes[node_id_to_index[B]], edge_max_id, edgeName, nAvaiable,
                   _minConfidence, _fixDirection, _isUltraOn, _isFallOn, _isLaserOn);
        temp.max_speed_    = speedRate;
        temp.leftSafeDist  = leftDist;
        temp.rightSafeDist = rightDist;

        edges.push_back(temp);
        edge_id_to_index.insert(pair<CNode::IdType, CNode::IdType>(edge_max_id++, edges.size() - 1));
    }
};

}  // end namespace json_util

#endif  // JSON_UTIL_H
