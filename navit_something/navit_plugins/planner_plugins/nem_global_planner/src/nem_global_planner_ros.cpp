#include "nem_global_planner/nem_global_planner_ros.h"
#include <ext/stdio_filebuf.h>
#include <fcntl.h>
#include <pluginlib/class_list_macros.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <algorithm>
#include <cstdio>
#include <map>
#include "nem_global_planner/util.h"
#include "nem_global_planner/xform.h"

#define NAVI_FOR_INFRARED_AUTO_POLL_FLAG (3)

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(nem_global_planner::NemGlobalPlannerROS, navit_core::GlobalPlanner)
namespace {
void VersionInfo()
{
    ROS_INFO("\n");
    ROS_INFO("---M100导航规划运行信息---");
	ROS_INFO("Navigation node start.");
	ROS_INFO("Version:1.0.0");
	ROS_INFO("Coding date:2022-09-08");
    ROS_INFO("无");
	ROS_INFO("Build date:%s  %s\n", __DATE__, __TIME__);
}
}

namespace nem_global_planner {

void CSegmentPath::OutputLOG() {
    stringstream strPath;
    for (int i = 0; i < vecPath.size(); i++) {
        strPath << vecPath[i] << " ";
    }
}

bool CSegmentPath::InitSegmentPath(std::string strData) {
    bool bRetVal = true;
    vecPath.clear();

    std::stringstream ss;
    ss << strData;
    unsigned int uNodeID = 0;
    std::vector<unsigned int> t_vecPath;
    while (ss >> uNodeID) {
        t_vecPath.push_back(uNodeID);
    }
    do {
        if (t_vecPath.size() < 3) {
            bRetVal = false;
            break;
        }

        uDoorStartID = t_vecPath[0];
        uDoorEndID   = t_vecPath[t_vecPath.size() - 1];

        for (int i = 1; i < t_vecPath.size() - 1; i++) {
            vecPath.push_back(t_vecPath[i]);
        }

    } while (0);
    return bRetVal;
}
CNode::IdType NemGlobalPlannerROS::map_id_ = 0;
NemGlobalPlannerROS::NemGlobalPlannerROS() : costmap_(NULL), initialized_(false) {
    node_max_id_ = edge_max_id_ = 0;
    map_changed_                = false;  //不能刪除
    dist_                       = NULL;
    prenode_                    = NULL;
    on_node_dist_               = 1.0;
    on_edge_dist_               = 1.0;
    VersionInfo();
}

// navit_core interface
void NemGlobalPlannerROS::initialize(const std::string& name,
                    const std::shared_ptr<navit_costmap::Costmap2DROS>& costmap_ros)
{
    initialize(name, costmap_ros.get());
}

void NemGlobalPlannerROS::initialize(std::string name, navit_costmap::Costmap2DROS *costmap_ros) {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~/" + name);
    plan_pub_    = private_nh.advertise<nav_msgs::Path>("plan", 1);
    node_max_id_ = edge_max_id_ = 0;
    map_changed_                = false;  //不能刪除
    dist_                       = NULL;
    prenode_                    = NULL;
    on_node_dist_               = 1.0;
    on_edge_dist_               = 1.0;
    costmap_                    = NULL;
    bool sim_mode;
    nh.param("sim_mode", sim_mode, false);
    private_nh.param("onNodeDist", on_node_dist_, 0.05);
    private_nh.param("onEdgeDist", on_edge_dist_, 0.05);
    ROS_INFO("[nem_global_planner]: sim_mode = %d, onNodeDist = %f, onEdgeDist = %f", sim_mode, on_node_dist_,
             on_edge_dist_);
    if (sim_mode)
        robot_sim_pose_sub_ = nh.subscribe("amcl_pose", 1, &NemGlobalPlannerROS::PoseReceived_amcl, this);
    else
        robot_pose_sub_ = nh.subscribe("pose", 1, &NemGlobalPlannerROS::PoseReceived, this);
    map_id_sub_     = nh.subscribe("car_loc_mapid_cmd", 1, &NemGlobalPlannerROS::MapIdReceived, this);
    marker_pub      = private_nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    navi_error_pub_ = nh.advertise<std_msgs::Int32>("navi_error", false, 1);

    costmap_ = costmap_ros->getCostmap();
}

bool NemGlobalPlannerROS::initNemData(const std::string &node_file_path, const std::string &edge_file_path) {
    if (!Init(node_file_path, edge_file_path))  //報false，拋出異常
    {
        initialized_ = false;
        ROS_ERROR("[nem_global_planner]: Initialize nem data failed");
        std_msgs::Int32 error_code;
        error_code.data = 111;
        navi_error_pub_.publish(error_code);
        return false;
    }
    initialized_ = true;
    return true;
}

void NemGlobalPlannerROS::PoseReceived(const geometry_msgs::PoseStampedConstPtr pose) {
    robot_pose_.x          = pose->pose.position.x;
    robot_pose_.y          = pose->pose.position.y;
    robot_pose_.r          = tf::getYaw(pose->pose.orientation);
    robot_pose_.confidence = pose->pose.position.z;
}

void NemGlobalPlannerROS::PoseReceived_amcl(const geometry_msgs::PoseWithCovarianceStampedConstPtr pose) {
    robot_pose_.x = pose->pose.pose.position.x;
    robot_pose_.y = pose->pose.pose.position.y;
    robot_pose_.r = tf::getYaw(pose->pose.pose.orientation);
}
void NemGlobalPlannerROS::MapIdReceived(const std_msgs::StringConstPtr &mapId) {
    CNode::IdType tempMapId;
    istringstream is(mapId->data);
    is >> tempMapId;
    if (map_id_ != tempMapId) {
        map_id_ = tempMapId;
        ROS_WARN("Current mapID = %ld", map_id_);
    }
}

                
nav_msgs::Path NemGlobalPlannerROS::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal) 
{
    nav_msgs::Path planned_path;
    if ( makePlan(start, goal, planned_path.poses) )
        return planned_path;
    else
    {
        ROS_WARN("Failed to plan! Return empty path");
        return planned_path;
    }
}

bool NemGlobalPlannerROS::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                                   std::vector<geometry_msgs::PoseStamped> &plan) {
    robot_pose_.x    = start.pose.position.x;
    robot_pose_.y    = start.pose.position.y;
    global_start_id_ = goal.header.seq;

    if (!initialized_) {
        ROS_ERROR("[nem_global_planner]: Nem data not init yet");
        std_msgs::Int32 error_code;
        error_code.data = 111;
        navi_error_pub_.publish(error_code);
        return false;
    }
    CNode::IdType temp_goal;
    CNode::IdType edgeId = -1, nodeId, startNodeID;
    CNode::IdType startMapId, goalMapId;
    // 判断目标点信息
    ROS_WARN_THROTTLE(1, "[nem_global_planner]: Check goal node info!");
    if (goal.pose.position.z == 0) {
        ROS_WARN_THROTTLE(1, "[nem_global_planner]: Not receive goal node id");
        // 根据目标点坐标获取目标点ID
        if (!IsOnNode(goal.pose.position.x, goal.pose.position.y, nodeId, map_id_)) {
            ROS_ERROR_THROTTLE(1,
                               "[nem_global_planner]: Can not get correct goal,goal.x = %f, goal.y = %f, goal.r = %f",
                               goal.pose.position.x, goal.pose.position.x, goal.pose.position.z);
            std_msgs::Int32 error_code;
            error_code.data = 112;
            navi_error_pub_.publish(error_code);
            return false;
        }
    } else {
        ROS_INFO_THROTTLE(1, "[nem_global_planner]: Goal id = %lf", goal.pose.position.z);
        if (!IsOnNode(goal, nodeId, map_id_)) {
            ROS_ERROR_THROTTLE(1,
                               "[nem_global_planner]: Can not get correct goal,goal.x = %f, goal.y = %f, goal.r = %f",
                               goal.pose.position.x, goal.pose.position.x, goal.pose.position.z);
            std_msgs::Int32 error_code;
            error_code.data = 113;
            navi_error_pub_.publish(error_code);
            return false;
        }
    }
    if(!GetMapId(nodeId, goalMapId)){
        ROS_ERROR_THROTTLE(1, "[nem_global_planner]: Can not get Goal Node %ld's MapID", nodeId);
        return false;
    }

    temp_goal = nodeId;
    vector<CNode::IdType> aimPathIndex;
    aimPathIndex.push_back(temp_goal);
    std::vector<CSegmentPath> vecSegmentPath;
    aim_path_nodes_.clear();

    std::vector<CNode::IdType>::iterator it         = aimPathIndex.begin();
    map<CNode::IdType, CNode::IdType>::iterator itF = GetNodeIDToIndex().find(*it);
    
    if (itF == GetNodeIDToIndex().end()) {
        it = aimPathIndex.erase(it);
    } else {
        if (!IsNodeAvaiable(*it)) {
            it = aimPathIndex.erase(it);
            ROS_WARN_THROTTLE(1, "[nem_global_planner]: Then the aimPathIndex is empty");
        }
    }

    if (aimPathIndex.empty()) {
        ROS_ERROR_THROTTLE(1, "[nem_global_planner]: Aim path is empty, maybe there is not any available edges");
        std_msgs::Int32 error_code;
        error_code.data = 114;
        navi_error_pub_.publish(error_code);
        return false;
    }

    bool isOnEdge = false, forceFromEdge = false;
    // 判断起始点信息
    // 获取当前机器人附近的mapID正确的nodeID
    ROS_WARN_THROTTLE(1, "[nem_global_planner]: Check start node info!");
    if(global_start_id_ != 0) {
        ROS_INFO_THROTTLE(1, "[nem_global_planner]: Find start id by software.");
        if (CalDistBetweenNodeAndPose(global_start_id_, robot_pose_.x, robot_pose_.y) < 0.10) {
            ROS_INFO_THROTTLE(1, "[nem_global_planner]: Software tell me robot is on node %ld", global_start_id_);
            start_node_state_ = ON_NODE;
        } else if (!forceFromEdge && IsOnNode(robot_pose_.x, robot_pose_.y, nodeId, map_id_)) {
            ROS_ERROR_THROTTLE(1, "[nem_global_planner]: Software start id error. Received id: %ld, Real id: %ld", global_start_id_, nodeId);
            global_start_id_  = nodeId;
            start_node_state_ = ON_NODE;
        } else if (GetShortestPathEdge(robot_pose_.x, robot_pose_.y, edgeId, map_id_, aimPathIndex) || isOnEdge) {
            ROS_ERROR_THROTTLE(1, "[nem_global_planner]: Software start id error. Received id: %ld, on edge.", global_start_id_);
            start_node_state_ = ON_EDGE;
        }
        else
            start_node_state_ = NOT_ON_ANY_PATH;
    } else {
        ROS_INFO_THROTTLE(1, "[nem_global_planner]: Find start id by myself.");
        if (!forceFromEdge && IsOnNode(robot_pose_.x, robot_pose_.y, nodeId, map_id_)) {
            global_start_id_  = nodeId;
            start_node_state_ = ON_NODE;
        } else if (GetShortestPathEdge(robot_pose_.x, robot_pose_.y, edgeId, map_id_, aimPathIndex) || isOnEdge)
            start_node_state_ = ON_EDGE;
        else
            start_node_state_ = NOT_ON_ANY_PATH;
    }

    switch (start_node_state_) {
        case ON_NODE: {
            nodeId = global_start_id_;
            ROS_INFO("[nem_global_planner]: Robot is on node %ld", nodeId);

            vector<CNode::IdType> path = aimPathIndex;
            path.push_back(nodeId);
            vector<CNode::IdType> aimDetailPathIndex = GetDetailPath(path);
            if (aimDetailPathIndex.empty()) {
                ROS_ERROR("[nem_global_planner]: Get Detail Path Failed");
                std_msgs::Int32 error_code;
                error_code.data = 116;
                navi_error_pub_.publish(error_code);
                return false;
            }
            CNode temp;
            temp.x  = robot_pose_.x;
            temp.y  = robot_pose_.y;
            temp.r  = robot_pose_.r;
            temp.id = -1;
            aim_path_nodes_.push_back(temp);
            for_each(aimDetailPathIndex.begin(), aimDetailPathIndex.end(),
                     [&](CNode::IdType index) { aim_path_nodes_.push_back(GetNode(index)); });
            break;
        }
        case ON_EDGE: {
            ROS_INFO("[nem_global_planner]: Robot is on edge %ld --> %ld", GetEdge(edgeId).nodeA_, GetEdge(edgeId).nodeB_);
            vector<CNode::IdType> path;
            if(!GetNearestNodeOnEdge(aim_path_nodes_, aimPathIndex, path, edgeId)) {
                ROS_ERROR("[nem_global_planner]: The robot is on the edge and the only one targetNode can not reached!");
                std_msgs::Int32 error_code;
                error_code.data = 117;
                navi_error_pub_.publish(error_code);
                return false;
            }

            if (path.empty()) {
                ROS_ERROR("[nem_global_planner]: Plan path failed!This should not be happened");
                std_msgs::Int32 error_code;
                error_code.data = 118;
                navi_error_pub_.publish(error_code);
                return false;
            }

            vector<CNode::IdType> aimDetailPathIndex = GetDetailPath(path);
            if (aimDetailPathIndex.empty()) {
                ROS_ERROR("[nem_global_planner]: Get Detail Path Failed");
                std_msgs::Int32 error_code;
                error_code.data = 116;
                navi_error_pub_.publish(error_code);
                return false;
            }

            for_each(aimDetailPathIndex.begin(), aimDetailPathIndex.end(),
                     [&](CNode::IdType index) { aim_path_nodes_.push_back(GetNode(index)); });

            if (aim_path_nodes_.size() <= 1) {
                ROS_ERROR("[nem_global_planner]: The plan path node is less than two,so stop the task!");
                std_msgs::Int32 error_code;
                error_code.data = 119;
                navi_error_pub_.publish(error_code);
                return false;
            }
            break;
        }
        case NOT_ON_ANY_PATH: {
            ROS_ERROR_THROTTLE(1, "[nem_global_planner]: Error,The robot now is not on any path!");
            std_msgs::Int32 error_code;
            error_code.data = 120;
            navi_error_pub_.publish(error_code);
            return false;
        }
    }

    //对路径进行插值，插值之间的欧式距离为res
    plan.clear();
    double res = 0.1;
    pathInterpolate(aim_path_nodes_, plan, res);

    nav_msgs::Path gui_path;
    gui_path.poses.clear();
    gui_path.header.frame_id = "map";
    gui_path.header.stamp    = ros::Time::now();
    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < plan.size(); i++) {
        geometry_msgs::PoseStamped thisPoint;
        thisPoint                 = plan[i];
        thisPoint.pose.position.z = 0.0;
        gui_path.poses.push_back(thisPoint);
    }

    plan_pub_.publish(gui_path);

    return true;
}

void NemGlobalPlannerROS::pathInterpolate(vector<CNode> &aim_nodes, std::vector<geometry_msgs::PoseStamped> &plan,
                                          double res) {
    int num = 0;
    plan.clear();
    for (int i = 0; i < aim_nodes.size(); i++) {
        geometry_msgs::PoseStamped thisPoint;
        thisPoint.header.stamp     = ros::Time::now();
        thisPoint.header.frame_id  = "map";
        thisPoint.header.seq       = num;
        thisPoint.pose.position.x  = aim_nodes[i].x;
        thisPoint.pose.position.y  = aim_nodes[i].y;
        thisPoint.pose.position.z  = aim_nodes[i].id;
        tf::Quaternion orientation = tf::createQuaternionFromYaw(aim_nodes[i].r);
        geometry_msgs::Quaternion msg;
        tf::quaternionTFToMsg(orientation, msg);
        thisPoint.pose.orientation = msg;
        plan.push_back(thisPoint);
        num++;

        if (i == (aim_nodes.size() - 1))
            continue;

        double x0 = aim_nodes[i].x;
        double y0 = aim_nodes[i].y;
        double x1 = aim_nodes[i + 1].x;
        double y1 = aim_nodes[i + 1].y;
        if (sqrt(pow(x1 - x0, 2) + pow(y1 - y0, 2)) <= res)
            continue;
        for (nem_global_planner::LineIterator line(x0, y0, x1, y1, res); line.isValid(); line.advance()) {
            geometry_msgs::PoseStamped iterPoint;
            iterPoint.header.stamp     = ros::Time::now();
            iterPoint.header.frame_id  = "map";
            iterPoint.header.seq       = num;
            iterPoint.pose.position.x  = line.getX();
            iterPoint.pose.position.y  = line.getY();
            iterPoint.pose.position.z  = -2.0;
            iterPoint.pose.orientation = msg;
            plan.push_back(iterPoint);
            num++;
        }
    }
}

void NemGlobalPlannerROS::DeleteNode(unsigned int id) {
    if (!node_id_to_index_.count(id)) {
        ROS_ERROR("Error Node ID");
        return;
    }
    unsigned int index              = node_id_to_index_[id];
    std::vector<CNode>::iterator it = nodes_.begin() + index;
    nodes_.erase(it);
    node_id_to_index_.erase(id);

    for (int i = id + 1; i <= node_max_id_; i++) {
        if (node_id_to_index_.count(i)) {
            node_id_to_index_[i] = node_id_to_index_[i] - 1;
        }
    }

    for (int i = 0; i < edge_max_id_; i++) {
        if (edge_id_to_index_.count(i)) {
            CEdge &temp = edges_[edge_id_to_index_[i]];
            if (temp.nodeA_ == id || temp.nodeB_ == id) {
                DeleteEdge(i);
            }
        }
    }
    map_changed_ = true;
}

void NemGlobalPlannerROS::DeleteEdge(CNode::IdType id) {
    if (!edge_id_to_index_.count(id)) {
        ROS_ERROR("[nem_global_planner]: Error Node ID");
        return;
    }
    CNode::IdType index             = edge_id_to_index_[id];
    std::vector<CEdge>::iterator it = edges_.begin() + index;
    edges_.erase(it);
    edge_id_to_index_.erase(id);

    for (int i = id + 1; i < edge_max_id_; i++) {
        if (edge_id_to_index_.count(i)) {
            edge_id_to_index_[i] = edge_id_to_index_[i] - 1;
        }
    }
    map_changed_ = true;
}

void NemGlobalPlannerROS::InitMatrix() {
    if (dist_) {
        delete[] dist_[0];
        delete[] dist_;
    }
    if (prenode_) {
        delete[] prenode_[0];
        delete[] prenode_;
    }

    int nodeNum = nodes_.size();

    dist_    = new float *[nodeNum];
    dist_[0] = new float[nodeNum * nodeNum];
    for (int i = 1; i < nodeNum; i++) {
        dist_[i] = dist_[i - 1] + nodeNum;
    }

    for (int i = 0; i < nodeNum; i++) {
        for (int j = 0; j < nodeNum; j++) {
            if (i != j) {
                dist_[i][j] = DIST_MAX;
            } else {
                dist_[i][j] = 0;
            }
        }
    }

    prenode_    = new CNode::IdType *[nodeNum];
    prenode_[0] = new CNode::IdType[nodeNum * nodeNum];
    for (int i = 1; i < nodeNum; i++) {
        prenode_[i] = prenode_[i - 1] + nodeNum;
    }

    for (int i = 0; i < nodeNum; i++) {
        for (int j = 0; j < nodeNum; j++) {
            prenode_[i][j] = MAX_LONG_NUM_;
        }
    }

    for (int i = 0; i < edges_.size(); i++) {
        CEdge temp = edges_[i];
        if (temp.available_ == 0) {
            continue;
        }

        int indexA            = node_id_to_index_[temp.nodeA_];
        int indexB            = node_id_to_index_[temp.nodeB_];
        dist_[indexA][indexB] = dist_[indexB][indexA] = temp.length;
        prenode_[indexA][indexB]                      = indexA;
        prenode_[indexB][indexA]                      = indexB;
    }
}

void NemGlobalPlannerROS::FloydWarshall() {
    if (!map_changed_) {
        ROS_WARN("[nem_global_planner]: Map isn't changed,don't need plan again!");
        return;
    }
    InitMatrix();
    int nodeNum = nodes_.size();

    for (int n = 0; n < nodeNum; n++) {
        for (int i = 0; i < nodeNum; i++) {
            for (int j = 0; j < nodeNum; j++) {
                float a = dist_[i][j];
                float b = dist_[i][n] + dist_[n][j];
                if (a > b) {
                    dist_[i][j]    = b;
                    prenode_[i][j] = prenode_[n][j];
                }
            }
        }
    }

#ifdef _DEBUG
    PrintMatrix(dist, nodeNum, nodeNum);
    PrintMatrix(prenode, nodeNum, nodeNum);
#endif
    map_changed_ = false;
}

bool NemGlobalPlannerROS::CheckFullyConnection() {
    if (!dist_) {
        ROS_ERROR("[nem_global_planner]: Dist matrix is null!");
        return false;
    }
    for (int i = 0; i < nodes_.size(); i++)
        for (int j = 0; j < nodes_.size(); j++) {
            if ((dist_[i][j]) >= DIST_MAX - 20) {
                ROS_ERROR("[nem_global_planner]: This map is not fully connected!");
                nodes_[i].PrintNodeInfo();
                nodes_[j].PrintNodeInfo();
                return false;
            }
        }
    return true;
}

void NemGlobalPlannerROS::SetPathPlanType(int iPathType) {
    path_type_ = iPathType;
    map_node_.clear();
}

void NemGlobalPlannerROS::SetNodeRForQuickCheck(int iNodeID, double dbNodeR) { map_node_[iNodeID] = dbNodeR; }

bool NemGlobalPlannerROS::GetShortestPath(CNode::IdType A, CNode::IdType B, float &minLen,
                                          vector<CNode::IdType> &path) {
    if (A == B) {
        path.push_back(A);
        minLen = 0.0;
        return true;
    }

    int indexA = node_id_to_index_[A];
    int indexB = node_id_to_index_[B];
    minLen     = dist_[indexA][indexB];
    if ((minLen) >= DIST_MAX - 20) {
        ROS_ERROR("[nem_global_planner]: No path between the node %lld and %lld!", A, B);
        return false;
    }
    int last = indexB;
    while (last != indexA) {
        path.push_back(nodes_[last].id);
        last = prenode_[indexA][last];
    }
    path.push_back(nodes_[indexA].id);
    reverse(path.begin(), path.end());
    return true;
}

vector<CNode::IdType> NemGlobalPlannerROS::GetAimPathForDistanceV10(CNode::IdType startID, float &pathLen,
                                                                    vector<CNode::IdType> &aimNodes, bool &bFlag,
                                                                    int mode /*= 0*/) {
    std::lock_guard<std::recursive_mutex> lck(m_mtx);
    //    clock_t startTime = clock();
    bFlag = false;

    vector<CNode::IdType> vb_helper(aimNodes.begin(), aimNodes.end());
    vector<CNode::IdType> va_real;
    vector<CNode::IdType> vecCurDetailPath;
    va_real.push_back(startID);

    // remove startID from target nodes vector
    for (std::vector<CNode::IdType>::iterator it = vb_helper.begin(); it != vb_helper.end();) {
        if (*it == startID) {
            vb_helper.erase(it);
        } else
            ++it;
    }

    float min = 0.0;
    while (vb_helper.size() > 0) {
        min    = MAX_FLOAT_NUM_;
        int bb = vb_helper.size() - 1;  // get the last element of vb_helper to try

        int insertPos  = -1;
        double tempLen = 0.0;

        for (int i = 1; i <= va_real.size(); i++) {
            // first time calculate path length
            if (i == 1) {
                vector<CNode::IdType> tempVa(va_real.begin(), va_real.end());
                tempVa.insert(tempVa.begin() + i, vb_helper[bb]);
                tempLen = GetPathLength(tempVa, mode);

            }
            // calculate path length without direct method to accelerate
            else {
                float add, sub, addTime, subTime;

                add = sub = addTime = subTime = 0.0;
                if (i == va_real.size())  // insert into the end of the vector
                {
                    if (mode) {
                        add = dist_[node_id_to_index_[va_real[i - 1]]][node_id_to_index_[vb_helper[bb]]] +
                              dist_[node_id_to_index_[vb_helper[bb]]][node_id_to_index_[startID]] +
                              dist_[node_id_to_index_[va_real[i - 2]]][node_id_to_index_[va_real[i - 1]]];
                    } else {
                        add = dist_[node_id_to_index_[va_real[i - 1]]][node_id_to_index_[vb_helper[bb]]] +
                              dist_[node_id_to_index_[va_real[i - 2]]][node_id_to_index_[va_real[i - 1]]];
                    }

                    if (mode) {
                        sub = dist_[node_id_to_index_[va_real[i - 2]]][node_id_to_index_[vb_helper[bb]]] +
                              dist_[node_id_to_index_[vb_helper[bb]]][node_id_to_index_[va_real[i - 1]]] -
                              dist_[node_id_to_index_[va_real[i - 1]]][node_id_to_index_[startID]];
                    } else {
                        sub = dist_[node_id_to_index_[va_real[i - 2]]][node_id_to_index_[vb_helper[bb]]] +
                              dist_[node_id_to_index_[vb_helper[bb]]][node_id_to_index_[va_real[i - 1]]];
                    }
                } else {
                    add = dist_[node_id_to_index_[va_real[i - 1]]][node_id_to_index_[vb_helper[bb]]] +
                          dist_[node_id_to_index_[vb_helper[bb]]][node_id_to_index_[va_real[i]]] +
                          dist_[node_id_to_index_[va_real[i - 2]]][node_id_to_index_[va_real[i - 1]]];

                    sub = dist_[node_id_to_index_[va_real[i - 2]]][node_id_to_index_[vb_helper[bb]]] +
                          dist_[node_id_to_index_[vb_helper[bb]]][node_id_to_index_[va_real[i - 1]]] +
                          dist_[node_id_to_index_[va_real[i - 1]]][node_id_to_index_[va_real[i]]];
                }

                tempLen = tempLen + (add - sub);
            }

            if (tempLen < min) {
                insertPos = i;
                min       = tempLen;
            }
        }

        if (tempLen >= DIST_MAX - 20) {
            ROS_ERROR("[nem_global_planner]: can not reach node %lld", vb_helper[bb]);
            bFlag = true;
        } else {
            va_real.insert(va_real.begin() + insertPos, vb_helper[bb]);  // insert into va_real actually
        }

        {
            stringstream strVaQueue;
            for (int k = 0; k < va_real.size(); k++) {
                strVaQueue << va_real[k] << " ";
            }
        }
        std::vector<CNode::IdType>::iterator it = vb_helper.begin() + bb;  // remove from vb_helper
        vb_helper.erase(it);
    }

    if (mode && va_real[va_real.size() - 1] != startID)
        va_real.push_back(startID);

    pathLen = GetPathLength(va_real, mode);

    return va_real;
}

bool NemGlobalPlannerROS::IsInAimPath(CNode::IdType iNodeID, vector<CNode::IdType> &vecSortPath,
                                      CNode::IdType &iInsertPos) {
    bool bRetVal        = false;
    float dbMinDistance = 0.0;
    do {
        if (vecSortPath.size() <= 1) {
            break;
        }
        vector<CNode::IdType> t_vecPartPath;
        for (int i = 1; i < vecSortPath.size(); i++) {
            if (!GetShortestPath(vecSortPath[i - 1], vecSortPath[i], dbMinDistance, t_vecPartPath)) {
                continue;
            }
            vector<CNode::IdType>::iterator iter_find = find(t_vecPartPath.begin(), t_vecPartPath.end(), iNodeID);
            if (iter_find == t_vecPartPath.end()) {
                continue;
            }
            iInsertPos = i;
            bRetVal    = true;
            break;
        }

    } while (0);
    return bRetVal;
}

void NemGlobalPlannerROS::AdjustPathResult(vector<CNode::IdType> &path) {
    vector<CNode::IdType> t_vecUnSortPath = path;
    vector<CNode::IdType> t_vecSortPath;
    CNode::IdType iInsertPos = 0;
    bool bIsInAimPath        = false;
    for (int i = 0; i < t_vecUnSortPath.size(); i++) {
        stringstream strVaQueue;

        for (int k = 0; k < t_vecSortPath.size(); k++) {
            strVaQueue << t_vecSortPath[k] << " ";
        }

        if (bIsInAimPath = IsInAimPath(t_vecUnSortPath[i], t_vecSortPath, iInsertPos)) {
            t_vecSortPath.insert(t_vecSortPath.begin() + iInsertPos, t_vecUnSortPath[i]);

        } else {
            t_vecSortPath.push_back(t_vecUnSortPath[i]);
        }
    }

    path.clear();
    path = t_vecSortPath;
}

bool NemGlobalPlannerROS::IsInLine(CSegmentPath &oSegmentPath, vector<CNode::IdType> &aimPath) {
    bool bRetVal = false;
    do {
        if (aimPath.size() <= 0) {
            break;
        }

        for (int i = 1; i < aimPath.size() - 1; i++) {
            if (IsKeyNode(aimPath[i])) {
                return bRetVal;
            }
        }

        CNode oNodeA;  //= GetNode(aimPath[0]);
        CNode oNodeB;  //= GetNode(aimPath[1]);
        CNode oNodeC = GetNode(aimPath[aimPath.size() - 1]);
        if (oSegmentPath.uStartID == oSegmentPath.uEndID) {
            oNodeA = GetNode(aimPath[0]);
            oNodeB = GetNode(aimPath[1]);
        } else {
            oNodeA = GetNode(oSegmentPath.uStartID);
            oNodeB = GetNode(oSegmentPath.uEndID);
        }

        Point a(oNodeA.x, oNodeA.y), b(oNodeB.x, oNodeB.y), c(oNodeC.x, oNodeC.y);
        bRetVal = IsPointsInLineForInfrared(a, b, c);
        //判断最短路径上是否经过交叉点

        if (bRetVal) {
            //引入二次判断，用于检修区域
            for (int i = 0; i < aimPath.size(); i++) {
                c.x = GetNode(aimPath[i]).x;
                c.y = GetNode(aimPath[i]).y;
                if (!IsNodeAvaiable(aimPath[i])) {
                    bRetVal = false;
                    break;
                }
                if (!IsPointsInLine(a, b, c)) {
                    bRetVal = false;
                    break;
                }
            }
        }
        float dbA      = SlopeLine(b, a);
        float dbB      = SlopeLine(c, b);
        float dbNormal = xform::normalize(dbA - dbB);
    } while (0);

    return bRetVal;
}

bool NemGlobalPlannerROS::IsInNotValiableEdge(CNode::IdType iNodeID) {
    bool bRetVal = false;
    for (int i = 0; i < edges_.size(); i++) {
        if (edges_[i].available_)
            continue;
        if (edges_[i].nodeA_ == iNodeID || edges_[i].nodeB_ == iNodeID) {
            bRetVal = true;
            break;
        }
    }
    return bRetVal;
}

void NemGlobalPlannerROS::GetRelateNodeID(CNode::IdType iNodeID, vector<CNode::IdType> &vecRelateNodeID) {
    for (int i = 0; i < edges_.size(); i++) {
        if (edges_[i].nodeA_ != iNodeID && edges_[i].nodeB_ != iNodeID)
            continue;
        if (0 == edges_[i].available_)
            continue;

        if (edges_[i].nodeA_ == iNodeID) {
            if (!IsInNotValiableEdge(edges_[i].nodeB_)) {
                vecRelateNodeID.push_back(edges_[i].nodeB_);
            }

        } else {
            if (!IsInNotValiableEdge(edges_[i].nodeA_)) {
                vecRelateNodeID.push_back(edges_[i].nodeA_);
            }
        }
    }
}

bool NemGlobalPlannerROS::IsInShortPath(CNode::IdType iStartID, CNode::IdType iEndID, CNode::IdType iMustID) {
    bool bRetVal = false;
    vector<CNode::IdType> vecShortPath;
    float dbMinLen = 0.0;
    do {
        if (!GetShortestPath(iStartID, iEndID, dbMinLen, vecShortPath)) {
            break;
        }

        for (int i = 0; i < vecShortPath.size(); i++) {
            if (vecShortPath[i] == iMustID) {
                bRetVal = true;
                break;
            }
        }
    } while (0);

    {
        stringstream strVal;
        for (int i = 0; i < vecShortPath.size(); i++) {
            strVal << vecShortPath[i] << " ";
        }
    }
    return bRetVal;
}

int NemGlobalPlannerROS::GetMaxNearNodeID(CSegmentPath &oSegmentPath, CNode::IdType iStartID, CNode::IdType iMustID,
                                          CNode::IdType iDistNodeID) {
    int iMaxNearNodeID = -1;
    vector<CNode::IdType> vecRelateNodeID;
    Point a, b, c;
    a.x = GetNode(oSegmentPath.uStartID).x;
    a.y = GetNode(oSegmentPath.uStartID).y;
    if (iStartID == iMustID) {
        b.x = GetNode(oSegmentPath.uEndID).x;
        b.y = GetNode(oSegmentPath.uEndID).y;
    } else {
        b.x = GetNode(iStartID).x;
        b.y = GetNode(iStartID).y;
    }

    float dbCurDist = fabs(GetDist(GetNode(iStartID), GetNode(iDistNodeID)));
    GetRelateNodeID(iStartID, vecRelateNodeID);
    std::string strFailDesc = "success";
    stringstream strVal;
    {
        for (int i = 0; i < vecRelateNodeID.size(); i++) {
            strVal << vecRelateNodeID[i] << " ";
        }
    }

    do {
        if (IsKeyNode(iStartID)) {
            strFailDesc = "Is Key Node";
            break;
        }

        if (vecRelateNodeID.size() <= 0) {
            strFailDesc = "Get Relation Node Failed";
            break;
        }

        for (int i = 0; i < vecRelateNodeID.size(); i++) {
            c.x = GetNode(vecRelateNodeID[i]).x;
            c.y = GetNode(vecRelateNodeID[i]).y;

            if (!IsPointsInLineForInfrared(a, b, c)) {
                stringstream sDesc;
                float dbA           = SlopeLine(b, a);
                float dbB           = SlopeLine(c, a);
                double t_dbDistance = sqrt((c.x - a.x) * (c.x - a.x) + (c.y - a.y) * (c.y - a.y));
                double dbOffset     = asin((t_dbDistance > 20.0 ? 0.2 : 0.2) / t_dbDistance);

                sDesc << "IsPointsInLine Failed"
                      << " TanA:" << dbA << " TanB:" << dbB << " normal:" << xform::normalize(dbA - dbB)
                      << " condition:" << dbOffset;

                strFailDesc = sDesc.str().c_str();
                continue;
            }
            if (!IsInShortPath(iDistNodeID, vecRelateNodeID[i], iMustID)) {
                stringstream sDesc;
                sDesc << "IsInShortPath Failed"
                      << " StartNode: " << iDistNodeID << " EndNode: " << vecRelateNodeID[i]
                      << " MustNode: " << iMustID;
                strFailDesc = sDesc.str().c_str();
                continue;
            }

            float t_dbCurDist = fabs(GetDist(GetNode(vecRelateNodeID[i]), GetNode(iDistNodeID)));
            if (t_dbCurDist <= dbCurDist) {
                strFailDesc = "Distance Failed";
                continue;
            }
            iMaxNearNodeID = vecRelateNodeID[i];
            strFailDesc    = "success";
            break;
        }
    } while (0);
    return iMaxNearNodeID;
}

bool NemGlobalPlannerROS::IsKeyNode(CNode::IdType id) {
    bool bRetVal = true;
    vector<CNode::IdType> vecRelateNodeID;
    GetRelateNodeID(id, vecRelateNodeID);

    do {
        if (vecRelateNodeID.size() > 2) {
            break;
        }

        if (vecRelateNodeID.size() < 2) {
            bRetVal = false;
            break;
        }
        Point a, b, c;
        a.x = GetNode(vecRelateNodeID[0]).x;
        a.y = GetNode(vecRelateNodeID[0]).y;
        b.x = GetNode(vecRelateNodeID[1]).x;
        b.y = GetNode(vecRelateNodeID[1]).y;
        c.x = GetNode(id).x;
        c.y = GetNode(id).y;
        if (!IsPointsInLineForInfrared(a, b, c)) {
            bRetVal = true;
            break;
        }
        bRetVal = false;
    } while (0);
    return bRetVal;
}

int NemGlobalPlannerROS::GetMaxFarDoorID(CSegmentPath &oSegmentPath, CNode::IdType iStartID) {
    int iCalcDistNodeID = -1;
    int iMustID         = oSegmentPath.uStartID;

    if (oSegmentPath.uStartID == iStartID) {
        iMustID = oSegmentPath.uEndID;
    } else {
        iMustID = oSegmentPath.uStartID;
    }
    int iCurNearNodeID = iMustID;
    int iMaxFarDoorID  = iMustID;
    while (1) {
        iCurNearNodeID = GetMaxNearNodeID(oSegmentPath, iCurNearNodeID, iMustID, iStartID);
        if (-1 == iCurNearNodeID) {
            break;
        }
        if (iCurNearNodeID == iMaxFarDoorID) {
            break;
        }
        //如果此点不是有效点，或者是个关键点，则退出循环
        if (!ExistNode(iCurNearNodeID)) {
            break;
        }

        if (!IsNodeAvaiable(iCurNearNodeID)) {
            break;
        }

        iMaxFarDoorID = iCurNearNodeID;
        //判断此点是转折点，则直接跳出，不需要在循环
        if (IsKeyNode(iMaxFarDoorID)) {
            break;
        }
    }
    return iMaxFarDoorID;
}

void NemGlobalPlannerROS::MakeSegmentPathDoor(CSegmentPath &oSegmentPath) {
    do {
        oSegmentPath.uDoorStartID = oSegmentPath.uStartID;
        oSegmentPath.uDoorEndID   = oSegmentPath.uEndID;
        if (oSegmentPath.uStartID == oSegmentPath.uEndID) {
            CEdge oEdge;
            if (GetEdgeByNode(oSegmentPath.uStartID, oEdge)) {
                oSegmentPath.uEndID       = (oEdge.nodeA_ == oSegmentPath.uStartID ? oEdge.nodeB_ : oEdge.nodeA_);
                oSegmentPath.uDoorStartID = GetMaxFarDoorID(oSegmentPath, oSegmentPath.uEndID);
                oSegmentPath.uDoorEndID   = GetMaxFarDoorID(oSegmentPath, oSegmentPath.uStartID);
                oSegmentPath.uEndID       = oSegmentPath.uStartID;
            } else {
                break;
            }
        } else {
            oSegmentPath.uDoorStartID = GetMaxFarDoorID(oSegmentPath, oSegmentPath.uEndID);
            oSegmentPath.uDoorEndID   = GetMaxFarDoorID(oSegmentPath, oSegmentPath.uStartID);
        }

    } while (0);

    if (oSegmentPath.uDoorStartID == oSegmentPath.uDoorEndID) {
        oSegmentPath.dbLineDistance = 0.0;
    } else {
        vector<CNode::IdType> vecShortPath;
        GetShortestPath(oSegmentPath.uDoorStartID, oSegmentPath.uDoorEndID, oSegmentPath.dbLineDistance, vecShortPath);
        oSegmentPath.dbLineDistance = fabs(oSegmentPath.dbLineDistance);
    }
}

void NemGlobalPlannerROS::GetShortPathNodeID(CNode::IdType iOrgID, vector<CNode::IdType> &path,
                                             CNode::IdType &iPathIndex) {
    float dbMinDistance  = MAX_FLOAT_NUM_;
    float dbNodeDistance = MAX_FLOAT_NUM_;
    vector<CNode::IdType> vecShortPath;
    iPathIndex = 0;
    for (int i = 0; i < path.size(); i++) {
        GetShortestPath(iOrgID, path[i], dbNodeDistance, vecShortPath);
        if (dbNodeDistance <= dbMinDistance) {
            dbMinDistance = dbNodeDistance;
            iPathIndex    = i;
        }
    }
}

void NemGlobalPlannerROS::AdjustSegmentPath(CSegmentPath &oSegmentPath) {
    vector<CNode::IdType> vecUnSortPath = oSegmentPath.vecPath;
    vector<CNode::IdType> vecSortPath;
    CNode::IdType uOrgStartID = oSegmentPath.uDoorStartID;
    CNode::IdType iPathIndex  = 0;
    while (vecUnSortPath.size() > 0) {
        GetShortPathNodeID(uOrgStartID, vecUnSortPath, iPathIndex);
        uOrgStartID = vecUnSortPath[iPathIndex];
        vecSortPath.push_back(uOrgStartID);
        vecUnSortPath.erase(vecUnSortPath.begin() + iPathIndex);
    }
    oSegmentPath.vecPath.clear();
    oSegmentPath.vecPath = vecSortPath;
}

void NemGlobalPlannerROS::AdjustSegmentWay(CSegmentPath &oSegmentPath) {
    int iNodeAngleStat = 0;

    for (int i = 0; i < oSegmentPath.vecPath.size(); i++) {
        if (GetNodeTimeForQuickCheck(oSegmentPath.uDoorStartID, oSegmentPath.vecPath[i]) > xform::pi_2) {
            iNodeAngleStat++;
        }
    }
    if (iNodeAngleStat > oSegmentPath.vecPath.size() / 2) {
        int tempID                = oSegmentPath.uStartID;
        oSegmentPath.uStartID     = oSegmentPath.uEndID;
        oSegmentPath.uEndID       = tempID;
        tempID                    = oSegmentPath.uDoorStartID;
        oSegmentPath.uDoorStartID = oSegmentPath.uDoorEndID;
        oSegmentPath.uDoorEndID   = tempID;

        vector<CNode::IdType> t_vecUnSortPath = oSegmentPath.vecPath;
        oSegmentPath.vecPath.clear();
        for (int i = t_vecUnSortPath.size() - 1; i >= 0; i--) {
            oSegmentPath.vecPath.push_back(t_vecUnSortPath[i]);
        }
    }
}

bool NemGlobalPlannerROS::MergeSegmentPath(CSegmentPath &oSegmentPath, vector<CSegmentPath> &vecSegmentPath) {
    bool bRetVal   = false;
    int iFindIndex = 0;
    do {
        for (int i = 0; i < vecSegmentPath.size(); i++) {
            if ((oSegmentPath.uDoorStartID == vecSegmentPath[i].uDoorStartID &&
                 oSegmentPath.uDoorEndID == vecSegmentPath[i].uDoorEndID) ||
                (oSegmentPath.uDoorStartID == vecSegmentPath[i].uDoorEndID &&
                 oSegmentPath.uDoorEndID == vecSegmentPath[i].uDoorStartID)) {
                iFindIndex = i;
                bRetVal    = true;
                break;
            }
        }
        if (!bRetVal) {
            break;
        }
        float dbDistEE, dbDistES;
        vector<CNode::IdType> vecShortPath;
        GetShortestPath(vecSegmentPath[iFindIndex].uEndID, oSegmentPath.uStartID, dbDistES, vecShortPath);
        vecShortPath.clear();
        GetShortestPath(vecSegmentPath[iFindIndex].uEndID, oSegmentPath.uEndID, dbDistEE, vecShortPath);
        if (dbDistES <= dbDistEE) {
            for (int i = 0; i < oSegmentPath.vecPath.size(); i++) {
                vecSegmentPath[iFindIndex].vecPath.push_back(oSegmentPath.vecPath[i]);
            }
            vecSegmentPath[iFindIndex].uEndID = oSegmentPath.uEndID;
        } else {
            for (int i = oSegmentPath.vecPath.size() - 1; i >= 0; i--) {
                vecSegmentPath[iFindIndex].vecPath.push_back(oSegmentPath.vecPath[i]);
            }
            vecSegmentPath[iFindIndex].uEndID = oSegmentPath.uStartID;
        }

    } while (0);

    return bRetVal;
}

void NemGlobalPlannerROS::MakeSegmentPath(vector<unsigned int> &path, vector<CSegmentPath> &vecSegmentPath) {
    vector<CNode::IdType> vecShortPath;
    CSegmentPath oSegmentPath;
    oSegmentPath.uStartID = GetNode(path[0]).id;
    oSegmentPath.vecPath.push_back(path[0]);
    oSegmentPath.uEndID = oSegmentPath.uStartID;

    for (int i = 1; i < path.size(); i++) {
        vecShortPath = GetDetailPath(oSegmentPath.uEndID, path[i]);

        if (IsInLine(oSegmentPath, vecShortPath)) {
            oSegmentPath.uEndID = path[i];
            oSegmentPath.vecPath.push_back(path[i]);

        } else {
            oSegmentPath.OutputLOG();
            MakeSegmentPathDoor(oSegmentPath);
            if (!MergeSegmentPath(oSegmentPath, vecSegmentPath)) {
                vecSegmentPath.push_back(oSegmentPath);
            }

            oSegmentPath.Init();
            oSegmentPath.uStartID = GetNode(path[i]).id;
            oSegmentPath.uEndID   = oSegmentPath.uStartID;
            oSegmentPath.vecPath.push_back(path[i]);
        }
        vecShortPath.clear();
    }
    //适用与尾边加入
    MakeSegmentPathDoor(oSegmentPath);
    if (!MergeSegmentPath(oSegmentPath, vecSegmentPath)) {
        vecSegmentPath.push_back(oSegmentPath);
    }
}

bool NemGlobalPlannerROS::CalcSegmentDistance(vector<CSegmentPath> &vecSegmentPath, CSegmentPath &oSignalSegmentPath,
                                              int iPosition, float &dbDistance) {
    bool bRetVal                          = true;
    vector<CSegmentPath> t_vecSegmentPath = vecSegmentPath;

    iPosition = iPosition + 1;
    if (iPosition >= t_vecSegmentPath.size()) {
        t_vecSegmentPath.push_back(oSignalSegmentPath);
    } else {
        t_vecSegmentPath.insert(t_vecSegmentPath.begin() + iPosition, oSignalSegmentPath);
    }

    vector<CNode::IdType> vecShortPath;
    CNode::IdType uLasID = MAX_LONG_NUM_;
    for (int i = 0; i < t_vecSegmentPath.size(); i++) {
        if (uLasID != t_vecSegmentPath[i].uDoorStartID) {
            vecShortPath.push_back(t_vecSegmentPath[i].uDoorStartID);
            uLasID = t_vecSegmentPath[i].uDoorStartID;
        }

        for (int j = 0; j < t_vecSegmentPath[i].vecPath.size(); j++) {
            if (uLasID != t_vecSegmentPath[i].vecPath[j]) {
                vecShortPath.push_back(t_vecSegmentPath[i].vecPath[j]);
                uLasID = t_vecSegmentPath[i].vecPath[j];
            }
        }

        if (uLasID != t_vecSegmentPath[i].uDoorEndID) {
            vecShortPath.push_back(t_vecSegmentPath[i].uDoorEndID);
            uLasID = t_vecSegmentPath[i].uDoorEndID;
        }
    }
    dbDistance = GetPathLength(vecShortPath, 0);
    return bRetVal;
}

void NemGlobalPlannerROS::SegmentSort(vector<CSegmentPath> &vecSegmentPath) {
    vector<CSegmentPath> vecUnSortSegmentPath = vecSegmentPath;
    vector<CSegmentPath> vecSortSegmentPath;

    if (vecUnSortSegmentPath.size() < 2)
        return;

    vecSortSegmentPath.push_back(vecUnSortSegmentPath[0]);
    vecUnSortSegmentPath.erase(vecUnSortSegmentPath.begin());

    CSegmentPath oCurSegmentPath;

    float dbMinDistance    = 0.0;
    float dbCurMinDistance = 0.0;
    int iAdjustPosition    = 0;
    while (!vecUnSortSegmentPath.empty()) {
        oCurSegmentPath  = vecUnSortSegmentPath[0];
        dbMinDistance    = MAX_FLOAT_NUM_;
        dbCurMinDistance = MAX_FLOAT_NUM_;
        for (int i = 0; i < vecSortSegmentPath.size(); i++) {
            CalcSegmentDistance(vecSortSegmentPath, oCurSegmentPath, i, dbCurMinDistance);
            if (dbCurMinDistance < dbMinDistance) {
                dbMinDistance = dbCurMinDistance;
                //记录对应的位置信息
                iAdjustPosition = i;
            }
        }

        if (dbMinDistance > DIST_MAX - 20) {
            break;
        }
        //将数据加入 sort 队列中的对应位置
        {
            stringstream ss;
            for (int i = 0; i < vecSortSegmentPath.size(); i++) {
                ss << vecSortSegmentPath[i].uDoorStartID << " <-> " << vecSortSegmentPath[i].uDoorEndID << " ";
            }
        }

        if (iAdjustPosition >= (vecSortSegmentPath.size() - 1)) {
            vecSortSegmentPath.push_back(oCurSegmentPath);
        } else {
            vecSortSegmentPath.insert(vecSortSegmentPath.begin() + iAdjustPosition + 1, oCurSegmentPath);
        }

        vecUnSortSegmentPath.erase(vecUnSortSegmentPath.begin());
    }

    vecSegmentPath.clear();
    vecSegmentPath = vecSortSegmentPath;
}

bool NemGlobalPlannerROS::IsExistWorkNode(CSegmentPath &oSegmentPath, vector<CNode::IdType> &vecWorkNode) {
    bool bRetVal = false;

    for (int i = 0; i < oSegmentPath.vecPath.size(); i++) {
        if (IsInWorkNode(oSegmentPath.vecPath[i], vecWorkNode)) {
            bRetVal = true;
            break;
        }
    }

    return bRetVal;
}

bool NemGlobalPlannerROS::IsInWorkNode(CNode::IdType iNode, vector<CNode::IdType> &vecWorkNode) {
    bool bRetVal = false;
    for (int i = 0; i < vecWorkNode.size(); i++) {
        if (iNode == vecWorkNode[i]) {
            bRetVal = true;
            break;
        }
    }
    return bRetVal;
}

float NemGlobalPlannerROS::GetPathLen(vector<CSegmentPath> &vecSegmentPath, vector<CNode::IdType> &vecPosition) {
    vector<CNode::IdType> vecResult;

    CNode::IdType uLastID = MAX_LONG_NUM_;
    for (int i = 0; i < vecPosition.size(); i++) {
        if (vecSegmentPath[vecPosition[i]].uDoorStartID != uLastID) {
            uLastID = vecSegmentPath[vecPosition[i]].uDoorStartID;
            vecResult.push_back(uLastID);
        }

        for (int j = 0; j < vecSegmentPath[vecPosition[i]].vecPath.size(); j++) {
            if (vecSegmentPath[vecPosition[i]].vecPath[j] == uLastID)
                continue;
            uLastID = vecSegmentPath[vecPosition[i]].vecPath[j];
            vecResult.push_back(uLastID);
        }

        if (vecSegmentPath[vecPosition[i]].uDoorEndID != uLastID) {
            uLastID = vecSegmentPath[vecPosition[i]].uDoorEndID;
            vecResult.push_back(uLastID);
        }
    }

    return GetPathLength(vecResult, 0, false);
}

void NemGlobalPlannerROS::SegmentSortV10(vector<CSegmentPath> &vecSegmentPath) {
    int iOrgTime = time(NULL);
    vector<CNode::IdType> vecPosition;
    for (int i = 0; i < vecSegmentPath.size(); i++) {
        vecPosition.push_back(i);
    }
    float dbDefLen   = GetPathLen(vecSegmentPath, vecPosition);
    float dbFristLen = dbDefLen;
    double e = 1e-16, at = 0.99999999, T = 1.0;
    int L        = 200000;
    int iCount   = vecSegmentPath.size();
    int iSwapVal = 0;
    while (L--) {
        int c1 = rand() % iCount;
        int c2 = rand() % iCount;
        if (c1 == c2) {
            L++;
            continue;
        } else {
            usleep(10);
        }

        iSwapVal        = vecPosition[c1];
        vecPosition[c1] = vecPosition[c2];
        vecPosition[c2] = iSwapVal;

        double df = GetPathLen(vecSegmentPath, vecPosition) - dbDefLen;

        double sj = rand() % 10000;
        sj /= 10000;

        if (df < 0) {
            dbDefLen += df;
        } else if (exp(-df / T) > sj) {
            dbDefLen += df;
        } else {
            iSwapVal        = vecPosition[c1];
            vecPosition[c1] = vecPosition[c2];
            vecPosition[c2] = iSwapVal;
        }

        T *= at;
        if (T < e)
            break;
    }
    vector<CSegmentPath> t_oSegmentPath;
    if (dbDefLen < dbFristLen) {
        for (int i = 0; i < vecPosition.size(); i++) {
            t_oSegmentPath.push_back(vecSegmentPath[vecPosition[i]]);
        }
    }
    vecSegmentPath.clear();
    vecSegmentPath = t_oSegmentPath;
}

// vector<unsigned int> NemGlobalPlannerROS::GetAimPath(unsigned int startID, float &pathLen,
//                                                     vector<unsigned int> &aimNodes, bool &bFlag,
//                                                     vector<CSegmentPath> &vecSegmentPath, int mode) {
//    vector<unsigned int> vecResult;
//    vecResult = GetAimPathForDistanceV10(startID, pathLen, aimNodes, bFlag, mode);
//    return vecResult;
//}

std::string NemGlobalPlannerROS::ConvertSegmentPath2Str(vector<CSegmentPath> &vecSegmentPath) {
    std::stringstream ss;

    for (int i = 0; i < vecSegmentPath.size(); i++) {
        ss << vecSegmentPath[i].uDoorStartID << " ";
        for (int k = 0; k < vecSegmentPath[i].vecPath.size(); k++) {
            ss << vecSegmentPath[i].vecPath[k] << " ";
        }

        ss << vecSegmentPath[i].uDoorEndID << ";";
    }
    return ss.str();
}

void NemGlobalPlannerROS::ConvertStr2SegmentPath(std::string strSort, vector<CSegmentPath> &vecSegmentPath) {
    std::string strData  = "";
    std::size_t iFindPos = std::string::npos;
    CSegmentPath oSegmentPath;
    while ((iFindPos = strSort.find(";")) != std::string::npos) {
        strData = strSort.substr(0, iFindPos);
        if (oSegmentPath.InitSegmentPath(strData)) {
            vecSegmentPath.push_back(oSegmentPath);
        }
        strSort = strSort.substr(iFindPos + 1, std::string::npos);
    }
    if (oSegmentPath.InitSegmentPath(strSort)) {
        vecSegmentPath.push_back(oSegmentPath);
    }
}

void NemGlobalPlannerROS::ConvertSegmentPath2Sort(vector<CSegmentPath> &vecSegmentPath, vector<unsigned int> &vecSort,
                                                  int nStartNodeID) {
    do {
        if (vecSegmentPath.size() <= 0) {
            break;
        }
        if (1 == vecSegmentPath.size() && 1 == vecSegmentPath[0].vecPath.size()) {
            vecSort.push_back(nStartNodeID);
            if (nStartNodeID != vecSegmentPath[0].vecPath[0]) {
                vecSort.push_back(vecSegmentPath[0].vecPath[0]);
            }
        } else {
            unsigned int uLastID = nStartNodeID;  // std::numeric_limits<unsigned int>::max();

            for (int i = 0; i < vecSegmentPath.size(); i++) {
                if (vecSegmentPath[i].uDoorStartID != uLastID) {
                    uLastID = vecSegmentPath[i].uDoorStartID;
                    vecSort.push_back(uLastID);
                }
                for (int j = 0; j < vecSegmentPath[i].vecPath.size(); j++) {
                    if (vecSegmentPath[i].vecPath[j] == uLastID)
                        continue;
                    uLastID = vecSegmentPath[i].vecPath[j];
                    vecSort.push_back(uLastID);
                }
                if (i == vecSegmentPath.size() - 1 && vecSegmentPath[i].uEndID != vecSegmentPath[i].uDoorEndID) {
                    break;
                }
                if (vecSegmentPath[i].uDoorEndID != uLastID) {
                    uLastID = vecSegmentPath[i].uDoorEndID;
                    vecSort.push_back(uLastID);
                }
            }
        }
    } while (0);
}

bool NemGlobalPlannerROS::IsExistCheckAreaPath(vector<unsigned int> &vecPath, vector<unsigned int> &vecNormalPath) {
    bool bRetVal        = false;
    float dbMinDistance = 0.0;
    vector<CNode::IdType> vecShortPath;
    if (vecPath.size() > 0) {
        vecNormalPath.push_back(vecPath[0]);
    }

    for (int i = 1; i < vecPath.size(); i++) {
        vecShortPath.clear();
        if (!GetShortestPath(vecPath[i - 1], vecPath[i], dbMinDistance, vecShortPath)) {
            bRetVal = true;
            break;
        }
        vecNormalPath.push_back(vecPath[i]);
    }

    return bRetVal;
}

vector<CNode::IdType> NemGlobalPlannerROS::GetDetailPath(CNode::IdType startID, CNode::IdType endID) {
    vector<CNode::IdType> aimPath;
    aimPath.push_back(startID);
    aimPath.push_back(endID);
    return GetDetailPath(aimPath);
}

vector<CNode::IdType> NemGlobalPlannerROS::GetDetailPath(const vector<CNode::IdType> &aimPath) {
    std::lock_guard<std::recursive_mutex> lck(m_mtx);
    vector<CNode::IdType> result;
    if (!aimPath.size()) {
        ROS_ERROR("[nem_global_planner]: Error,The plan path vector is empty(In GetDetailPath Function)!");
        return result;
    }
    for (int i = 0; i < aimPath.size() - 1; i++) {
        float minLen;
        vector<CNode::IdType> tempV;
        if (GetShortestPath(aimPath[i], aimPath[i + 1], minLen, tempV)) {
            for (int i = 0; i < tempV.size() - 1; i++) {
                result.push_back(tempV[i]);
            }
        } else {
            ROS_ERROR("[nem_global_planner]: Error,Please make sure the map is right!");
            result.clear();
            return result;
        }
    }
    result.push_back(aimPath[aimPath.size() - 1]);
    reverse(result.begin(), result.end());
    return result;
}

//获取两个点之间的距离
float NemGlobalPlannerROS::GetNodePathLength(CNode::IdType nodeIdA, CNode::IdType nodeIdB) {
    return dist_[node_id_to_index_[nodeIdA]][node_id_to_index_[nodeIdB]];
}

float NemGlobalPlannerROS::GetNodeTimeForQuickCheck(CNode::IdType nodeIdA, CNode::IdType nodeIdB) {
    float dbTimeVal      = 0.0;
    float dbPtzSpeed     = 70;
    float dbMoveSpeed    = 0.6;
    double dbNodeTargetR = 0.0;
    do {
        if (NAVI_FOR_INFRARED_AUTO_POLL_FLAG != path_type_) {
            break;
        }
        if (!GetNodeRForQuickCheck(nodeIdB, dbNodeTargetR)) {
            break;
        }
        if (MAX_LONG_NUM_ ==
            prenode_[node_id_to_index_[nodeIdA]][node_id_to_index_[nodeIdB]]) {
            break;
        }

        int iPathNodeIdA = prenode_[node_id_to_index_[nodeIdA]][node_id_to_index_[nodeIdB]];
        CNode oNodeA     = GetNode(nodes_[iPathNodeIdA].id);
        CNode oNodeB     = GetNode(nodeIdB);
        CEdge oEdge;
        int iEdgeDirect    = 0;
        int iFixEdgeDirect = 1;
        if (0 == (iEdgeDirect = FindEdgeByNodes(oNodeA, oNodeB, oEdge))) {
            break;
        }
        if (iEdgeDirect == 1) {
            if (oEdge.single_lane_ == CEdge::DIRECTION_FIXED::DIRECTION_FIXED_POSITIVE) {
                iFixEdgeDirect = 1;
            } else if (oEdge.single_lane_ == CEdge::DIRECTION_FIXED::DIRECTION_FIXED_NEGATIVE) {
                iFixEdgeDirect = -1;
            } else {
                iFixEdgeDirect = 1;
            }

        } else if (iEdgeDirect == 2) {
            iFixEdgeDirect = 1;
        }

        Point a, b;
        a.x               = oNodeA.x;
        a.y               = oNodeA.y;
        b.x               = oNodeB.x;
        b.y               = oNodeB.y;
        float targetSlope = 0.0;
        if (1 == iFixEdgeDirect) {
            targetSlope = SlopeLine(a, b);

        } else {
            targetSlope = SlopeLine(b, a);
        }

        float dbMergeOrgAngle = (-1 * dbNodeTargetR) + oNodeB.r;

        float dbOffsetAnagle = fabs(xform::normalize(targetSlope - dbMergeOrgAngle));
        dbTimeVal            = dbOffsetAnagle;
    } while (0);
    return dbTimeVal;
}

float NemGlobalPlannerROS::GetPathLength(const vector<CNode::IdType> &path, int mode, bool bLogFlag) {
    float sumLen = 0.0;

    // 当前path中只包含一个目标点时，目标点之间的距离即为0，只需计算机器人当前位置到这一目标点的距离
    if (path.size() <= 1) {
        return sumLen;
    }
    for (int i = 0; i < path.size() - 1; i++) {
        // ROS_INFO("%ld dist to %ld is: %f",path[i], path[i+1], dist_[node_id_to_index_[path[i]]][node_id_to_index_[path[i + 1]]]);
        sumLen = sumLen + dist_[node_id_to_index_[path[i]]][node_id_to_index_[path[i + 1]]];
    }
    if (!mode)
        return sumLen;
    else
        return sumLen + dist_[node_id_to_index_[path[path.size() - 1]]][node_id_to_index_[path[0]]];
}

float NemGlobalPlannerROS::GetPathLength(const vector<CNode> &path) {
    float sumLen = 0.0;
    if (path.size() <= 1) {
        ROS_ERROR("[nem_global_planner]: Path nodes is less than two!");
        return sumLen;
    }
    if (path.size() == 2) {
        sumLen = sqrt(pow((path[0].x - path[1].x), 2) + pow((path[0].y - path[1].y), 2));
        return sumLen;
    }
    if(path.size() > 2) {
        for (int i = 1; i < path.size() - 1; i++) {
            ROS_INFO("%ld dist to %ld is: %f",path[i].id, path[i+1].id, dist_[node_id_to_index_[path[i].id]][node_id_to_index_[path[i + 1].id]]);
            sumLen = sumLen + dist_[node_id_to_index_[path[i].id]][node_id_to_index_[path[i + 1].id]];
        }
    }
    return sumLen;
}

bool NemGlobalPlannerROS::ReadFromJson(const std::istream &node_stream, Json::Value &val) {
    std::ostringstream sin;
    sin << node_stream.rdbuf();
    std::string str = sin.str();
    Json::Reader reader;
    return reader.parse(str, val);
}

// 通过 nodeID 在当前nodes容器中查找其对应的 mapID
bool NemGlobalPlannerROS::GetMapId(CNode::IdType &nodeId, CNode::IdType &mapId) {
    for (int i = 0; i < nodes_.size(); ++i) {
        if (nodes_[i].id == nodeId) {
            mapId = nodes_[i].map_id;
            ROS_INFO_THROTTLE(1, "[nem_global_planner]: Node %ld's MapID is %ld", nodeId, mapId);
            return true;
        }
        if (i == (nodes_.size() - 1))
            return false;
    }
}

// 查询距离坐标最近的Node，同时mapID正确
bool NemGlobalPlannerROS::IsOnNode(float x, float y, CNode::IdType &nodeId, const CNode::IdType &mapId) {
    std::lock_guard<std::recursive_mutex> lck(m_mtx);
    CNode::IdType minId = MAX_LONG_NUM_;
    float minLen        = MAX_FLOAT_NUM_;
    for (auto & node : nodes_) {
        float dist = sqrt((x - node.x) * (x - node.x) + (y - node.y) * (y - node.y));
        if (mapId != node.map_id)
            continue;
        if (dist <= on_node_dist_) {
            //ROS_ERROR("Current onNode dist is %f",dist);
            if (dist < minLen) {
                minId  = node.id;
                minLen = dist;
            }
        }
    }
    nodeId = minId;
    if(nodeId == MAX_LONG_NUM_)
        ROS_WARN("[nem_global_planner]: Robot is not on node!");
    else
        ROS_WARN("Current onNode id is %ld",nodeId);
    return minId != MAX_LONG_NUM_;
}

bool NemGlobalPlannerROS::IsOnNode(const geometry_msgs::PoseStamped &goal, CNode::IdType &nodeId,
                                   const CNode::IdType &mapId) {
    std::lock_guard<std::recursive_mutex> lck(m_mtx);
    double id = fabs(goal.pose.position.z);
    nodeId    = (CNode::IdType)id;
    // 判断目标点的mapID与地图mapID是否相同
    for (auto & node : nodes_) {
        if (node.id == nodeId) {
            if (node.map_id != mapId) {
                ROS_ERROR_THROTTLE(1, "Node %ld's mapID is %ld. Current mapID is %ld", 
                                node.id, node.map_id, mapId);
                return false;
            }
        }
    }
    return nodeId != MAX_LONG_NUM_;
}

// 获取距离机器人当前位置最近的边ID
bool NemGlobalPlannerROS::GetNearestEdge(float x, float y, CNode::IdType &nodeId, CNode::IdType &edgeId,
                                         const CNode::IdType &mapId) {
    std::lock_guard<std::recursive_mutex> lck(m_mtx);
    bool getOne = false;
    float min   = MAX_FLOAT_NUM_;
    for (auto & edge : edges_) {
        if (edge.available_ == 0) {
            continue;
        }

        CNode a = GetNode(edge.nodeA_);
        CNode b = GetNode(edge.nodeB_);
        if (a.map_id != mapId || b.map_id != mapId)
            continue;

        // 向量点乘求得P点在向量 ab 上的左右投影距离 * 向量 b 的模 
        float cross_a = (x - a.x) * (b.x - a.x) + (y - a.y) * (b.y - a.y);
        float cross_b = (x - b.x) * (a.x - b.x) + (y - b.y) * (a.y - b.y);

        // 投影值小于0表示P点不在线段内
        if (!(cross_a >= 0 && cross_b >= 0)) {
            continue;
        }
        // a b两点距离平方
        float dd = ((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));

        // 投影距离与ab距离的乘积 / ab距离平方 = 投影距离/ab
        float rate = cross_a / dd;
        // 点p在ab向量上的投影点的坐标
        float px   = a.x + rate * (b.x - a.x);
        float py   = a.y + rate * (b.y - a.y);

        // get distance to the line
        float dt = sqrt((px - x) * (px - x) + (py - y) * (py - y));
        if(isnan(dt))
            continue;

        if (dt <= on_edge_dist_ && dt < min) {
            min         = dt;
            edgeId      = edge.id;
            // 这里也可以获取距离较近的nodeID
            float distA = sqrt((px - a.x) * (px - a.x) + (py - a.y) * (py - a.y));
            float distB = sqrt((px - b.x) * (px - b.x) + (py - b.y) * (py - b.y));
            if (distA <= distB)
                nodeId = a.id;
            else
                nodeId = b.id;
            getOne = true;
        }
    }
    if (getOne)
        return true;
    else {
        ROS_WARN_THROTTLE(1, "[nem_global_planner]: Can't find nearest edge! On_edge_dist is %f.", on_edge_dist_);
        return false;
    }
}

bool NemGlobalPlannerROS::GetShortestPathEdge(float x, float y, CNode::IdType &edgeId, const CNode::IdType &mapId, const vector<CNode::IdType> &aimPathIndex) {

    std::lock_guard<std::recursive_mutex> lck(m_mtx);
    // 距离机器人当前位置最近的边的距离
    float min_dist = MAX_FLOAT_NUM_;
    // 路径总长最短的路径
    float min_path = MAX_FLOAT_NUM_;
    // 满足距离范围的边ID
    CNode::IdType edgeIdInDist;
    
    for (auto & edge : edges_) {
        if (edge.available_ == 0) {
            continue;
        }

        CNode a = GetNode(edge.nodeA_);
        CNode b = GetNode(edge.nodeB_);
        if (a.map_id != mapId || b.map_id != mapId)
            continue;

        // 向量点乘求得P点在向量 ab 上的左右投影距离 * 向量 b 的模 
        float cross_a = (x - a.x) * (b.x - a.x) + (y - a.y) * (b.y - a.y);
        float cross_b = (x - b.x) * (a.x - b.x) + (y - b.y) * (a.y - b.y);

        // 投影值小于0表示P点不在线段内
        if (!(cross_a >= 0 && cross_b >= 0)) {
            continue;
        }
        // a b两点距离平方
        float dd = ((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));

        // 投影距离与ab距离的乘积 / ab距离平方 = 投影距离/ab
        float rate = cross_a / dd;
        // 点p在ab向量上的投影点的坐标
        float px   = a.x + rate * (b.x - a.x);
        float py   = a.y + rate * (b.y - a.y);

        // get distance to the line
        float dt = sqrt((px - x) * (px - x) + (py - y) * (py - y));
        // 两点坐标一致但连成边时，距离为0，会产生nan值
        if(isnan(dt))
            continue;

        if (dt <= on_edge_dist_) {
            edgeIdInDist = edge.id;

            if(GetPathLenForPrePlan(aimPathIndex, edgeIdInDist) < min_path) {
                min_path = GetPathLenForPrePlan(aimPathIndex, edgeIdInDist);
                edgeId   =  edgeIdInDist;
                // ROS_ERROR("[nem_global_planner]: Nearest path len is: %f", min_path);
            }
        }
    }
    if(min_path != MAX_FLOAT_NUM_)
        return true;
    else {
        ROS_WARN_THROTTLE(1, "[nem_global_planner]: Can't find nearest edge! On_edge_dist is %f.", on_edge_dist_);
        return false;
    }
}

// 机器人位于多条重叠边之上时，选取路径最短的边作为当前机器人所在边(相当于已知当前edgeID后进行一次预路径规划)
float NemGlobalPlannerROS::GetPathLenForPrePlan(const vector<CNode::IdType> &aimPathIndex, const CNode::IdType &edgeId) {

    // 包含获取到的目标点和机器人所在边目标点的临时path
    vector<CNode::IdType> aim_path;
    // 预规划计算的路径长度
    float prePathLen;
    // 预规划最终路径
    vector<CNode> prePlanFinalAimPath;

    // 获取机器人在边上时的下一个目标点
    if(GetNearestNodeOnEdge(prePlanFinalAimPath, aimPathIndex, aim_path, edgeId)) {
        if (aim_path.empty()) {
            ROS_ERROR("[nem_global_planner]: Get next target node failed");
            return -1;
        }

        // 计算得到的路径
        vector<CNode::IdType> aimDetailPathIndex = GetDetailPath(aim_path);

        // 将得到的路径加入最终的预规划路径中
        for_each(aimDetailPathIndex.begin(), aimDetailPathIndex.end(),
                    [&](CNode::IdType index) { prePlanFinalAimPath.push_back(GetNode(index)); });

        // 机器人当前位置到第一个目标点距离
        float poseToNextNodeLen = sqrt(pow((prePlanFinalAimPath[0].x - prePlanFinalAimPath[1].x), 2) + 
                                       pow((prePlanFinalAimPath[0].y - prePlanFinalAimPath[1].y), 2));
        // ROS_ERROR("[nem_global_planner]: 当前位置到下一个目标点距离: -1(%f, %f)-->%ld = %fm", 
        //     prePlanFinalAimPath[0].x, prePlanFinalAimPath[0].y,  prePlanFinalAimPath[1].id, poseToNextNodeLen);

        // 最终路径长度 = 目标点之间的距离 + 机器人当前位置到第一个目标点的距离
        prePathLen = GetPathLength(aimDetailPathIndex, 0) + poseToNextNodeLen;

        prePlanFinalAimPath.clear();

        return prePathLen;
    } else {
        ROS_ERROR("[nem_global_planner]: Calculate path len error!");
        return MAX_FLOAT_NUM_;
    }
}

// 已知机器人所在边，计算机器人到当前边哪端路径更近,并将端点加入当前路径
bool NemGlobalPlannerROS::GetNearestNodeOnEdge(vector<CNode> &prePlanFinalAimPath, const vector<CNode::IdType> &aimPathIndex, 
                                                vector<CNode::IdType> &path, const CNode::IdType &edgeId) {

    vector<CNode::IdType> path1, path2;
    float pathLen1, pathLen2;

    CNode::IdType a = GetEdge(edgeId).nodeA_;
    CNode::IdType b = GetEdge(edgeId).nodeB_;

    CNode temp;
    temp.x  = robot_pose_.x;
    temp.y  = robot_pose_.y;
    temp.r  = robot_pose_.r;
    temp.id = -1;

    prePlanFinalAimPath.push_back(temp);

    std::vector<CSegmentPath> vecSegmentPathNode2;
    path1 = aimPathIndex;
    path1.push_back(GetEdge(edgeId).nodeA_);

    path2 = aimPathIndex;
    path2.push_back(GetEdge(edgeId).nodeB_);

    // 路径距离1 = 当前位置到A点的距离 + 目标点到点A的距离
    // 路径距离2 = 当前位置到B点的距离 + 目标点到点B的距离
    pathLen1 = sqrt((robot_pose_.x - GetNode(a).x) * (robot_pose_.x - GetNode(a).x) +
                    (robot_pose_.y - GetNode(a).y) * (robot_pose_.y - GetNode(a).y)) +
                dist_[node_id_to_index_[path1[0]]][node_id_to_index_[path1[1]]];
    pathLen2 = sqrt((robot_pose_.x - GetNode(b).x) * (robot_pose_.x - GetNode(b).x) +
                    (robot_pose_.y - GetNode(b).y) * (robot_pose_.y - GetNode(b).y)) +
                dist_[node_id_to_index_[path2[0]]][node_id_to_index_[path2[1]]];

    if (aimPathIndex.size() == 1 && min(pathLen1, pathLen2) > DIST_MAX - 1000) {
        ROS_ERROR("[nem_global_planner]: Robot is on edge has only one target can't reached! Edge: %ld-->%ld", GetEdge(edgeId).nodeA_, GetEdge(edgeId).nodeB_);
        return false;
    }

    double pathLen;
    if (pathLen1 > pathLen2) {
        path    = path2;
        pathLen = pathLen2;
    } else {
        path    = path1;
        pathLen = pathLen1;
    }
    return true;
}

bool NemGlobalPlannerROS::GetCurrentEdge(float x, float y, const std::vector<unsigned int> &pathNodes, CEdge &edge) {
    std::lock_guard<std::recursive_mutex> lck(m_mtx);
    bool getOne = false;

    if (pathNodes.size() < 2) {
        return false;
    }
    // If path has only one edge, return the edge directly!
    if (pathNodes.size() == 2) {
        int ret = FindEdgeByNodes(GetNode(pathNodes[0]), GetNode(pathNodes[1]), edge);
        if (ret != 0) {
            return true;
        } else {
            return false;
        }
    }

    // Find the edge according to current pose
    for (int i = 0; i < pathNodes.size() - 1; i++) {
        CNode a = GetNode(pathNodes[i]);
        CNode b = GetNode(pathNodes[i + 1]);

        float cross_a = (x - a.x) * (b.x - a.x) + (y - a.y) * (b.y - a.y);
        float cross_b = (x - b.x) * (a.x - b.x) + (y - b.y) * (a.y - b.y);

        // point not at line segment
        if (cross_a < 0 || cross_b < 0) {
            continue;
        } else {
            int ret = FindEdgeByNodes(a, b, edge);
            if (ret != 0) {
                getOne = true;
                // pass through intended. If current pose can match multiple edge, the behind edge will
                // considered as match always.
            } else {
            }
        }
    }
    if (getOne)
        return true;
    return false;
}

template <class T>
void NemGlobalPlannerROS::PrintMatrix(T **m, int l, int h) {
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < l; j++) {
            cout << m[i][j] << " ";
        }
        cout << endl;
    }
}

bool NemGlobalPlannerROS::ExistNode(CNode::IdType id) {
    std::lock_guard<std::recursive_mutex> lck(m_mtx);
    if (node_id_to_index_.count(id) <= 0) {
        return false;
    }
    return true;
}

// todo bool return
CNode NemGlobalPlannerROS::GetNode(CNode::IdType id) {
    std::lock_guard<std::recursive_mutex> lck(m_mtx);
    CNode temp;
    if (!node_id_to_index_.count(id)) {
        ROS_ERROR("[nem_global_planner]: Error,node:%lld is not exist!", id);
        return temp;
    } else
        return nodes_[node_id_to_index_[id]];
}

// todo bool return
CEdge NemGlobalPlannerROS::GetEdge(CNode::IdType id) {
    std::lock_guard<std::recursive_mutex> lck(m_mtx);
    if (!edge_id_to_index_.count(id)) {
        ROS_ERROR("[nem_global_planner]: Error,node is not exist!");
        return edges_[edges_.size()];
    } else
        return edges_[edge_id_to_index_[id]];
}

/**
  0 --- not found
  1 --- A---->B
  2 --- B---->A
*/
int NemGlobalPlannerROS::FindEdgeByNodes(const CNode &nodeA, const CNode &nodeB, CEdge &edge) {
    std::lock_guard<std::recursive_mutex> lck(m_mtx);
    int ret = 0;
    for (int i = 0; i < edges_.size(); i++) {
        if (edges_[i].nodeA_ == nodeA.id && edges_[i].nodeB_ == nodeB.id && edges_[i].available_ != 0) {
            ret  = 1;
            edge = edges_[i];
            break;
        }
        if (edges_[i].nodeA_ == nodeB.id && edges_[i].nodeB_ == nodeA.id && edges_[i].available_ != 0) {
            ret  = 2;
            edge = edges_[i];
            break;
        }
    }
    return ret;
}

bool NemGlobalPlannerROS::GetEdgeByNode(CNode::IdType id, CEdge &oEdge) {
    std::lock_guard<std::recursive_mutex> lck(m_mtx);
    bool bRetVal = false;
    for (int i = 0; i < edges_.size(); i++) {
        if (edges_[i].available_ == 0)
            continue;
        if (edges_[i].nodeA_ == id) {
            if (!IsInNotValiableEdge(edges_[i].nodeB_)) {
                bRetVal = true;
                oEdge   = edges_[i];
                break;
            }
        } else if (edges_[i].nodeB_ == id) {
            if (!IsInNotValiableEdge(edges_[i].nodeA_)) {
                bRetVal = true;
                oEdge   = edges_[i];
                break;
            }
        }
    }
    return bRetVal;
}

float NemGlobalPlannerROS::CalDistBetweenNodeAndPose(const CNode::IdType nodeID, const float poseX, const float poseY) {
    CNode startNode  = GetNode(nodeID);
    float dist = sqrt((startNode.x - poseX) * (startNode.x - poseX) + (startNode.y - poseY) * (startNode.y - poseY));
    return dist;
}

bool NemGlobalPlannerROS::IsNodeAvaiable(CNode::IdType nNodeID) {
    std::lock_guard<std::recursive_mutex> lck(m_mtx);
    bool bAvaiable = false;
    auto it        = std::find_if(edges_.begin(), edges_.end(), [&](CEdge &edge) -> bool {
        if (edge.nodeA_ == nNodeID || edge.nodeB_ == nNodeID) {
            if (edge.available_)
                return true;
        }
        return false;
    });
    if (it != edges_.end())
        bAvaiable = true;
    return bAvaiable;
}

int NemGlobalPlannerROS::FindEdgeByPoints(Point &pointA, Point &pointB, CEdge &edge) {
    std::lock_guard<std::recursive_mutex> lck(m_mtx);
    int ret       = 0;
    double dSpeed = 100;
    for (int i = 0; i < edges_.size(); i++) {
        if (edges_[i].available_ == 0) {
            continue;
        }

        CNode &a = nodes_[node_id_to_index_[edges_[i].nodeA_]];
        CNode &b = nodes_[node_id_to_index_[edges_[i].nodeB_]];

        Point pointA1, pointB1;
        pointA1.x = a.x;
        pointA1.y = a.y;
        pointB1.x = b.x;
        pointB1.y = b.y;
        if (IsPointsInLine(pointA, pointB, pointB1) && IsPointsInLine(pointA, pointB, pointA1)) {
            if (dSpeed > edges_[i].max_speed_) {
                if (((pointA1.x > pointA.x && pointA1.x < pointB.x) || (pointA1.y > pointA.y && pointA1.y < pointB.y) ||
                     (pointA1.x < pointA.x && pointA1.x > pointB.x) ||
                     (pointA1.y < pointA.y && pointA1.y > pointB.y)) ||
                    ((pointB1.x > pointA.x && pointB1.x < pointB.x) || (pointB1.y > pointA.y && pointB1.y < pointB.y) ||
                     (pointB1.x < pointA.x && pointB1.x > pointB.x) ||
                     (pointB1.y < pointA.y && pointB1.y > pointB.y)) ||
                    ((pointA.x > pointA1.x && pointA.x < pointB1.x) || (pointA.y > pointA1.y && pointA.y < pointB1.y) ||
                     (pointA.x < pointA1.x && pointA.x > pointB1.x) ||
                     (pointA.y < pointA1.y && pointA.y > pointB1.y)) ||
                    ((pointB.x > pointA1.x && pointB.x < pointB1.x) || (pointB.y > pointA1.y && pointB.y < pointB1.y) ||
                     (pointB.x < pointA1.x && pointB.x > pointB1.x) ||
                     (pointB.y < pointA1.y && pointB.y > pointB1.y))) {
                    dSpeed = edges_[i].max_speed_;
                    edge   = edges_[i];
                    ret    = 1;
                }
            }
        }
    }
    return ret;
}

#ifndef WIN32
typedef std::basic_ofstream<char>::__filebuf_type buffer_t;
typedef __gnu_cxx::stdio_filebuf<char> io_buffer_t;

FILE *cfile_impl(buffer_t *const fb) { return (static_cast<io_buffer_t *const>(fb))->file(); }

FILE *cfile(std::ofstream const &ofs) { return cfile_impl(ofs.rdbuf()); }

#endif

void NemGlobalPlannerROS::SaveNodes(const char *nodeFilePath) {
    std::lock_guard<std::recursive_mutex> lck(m_mtx);
    ofstream out(nodeFilePath, std::ofstream::out | std::ofstream::trunc);
    if (out.is_open()) {
        for (auto it = nodes_.begin(); it != nodes_.end(); it++) {
            out << it->id << " " << it->type << " " << it->x << " " << it->y << " " << it->r << std::endl;
        }
        out.flush();
#ifndef WIN32
        fflush(cfile(out));
        fsync(cfile(out)->_fileno);
#endif
        out.close();
    }
}

void NemGlobalPlannerROS::SaveEdges(const char *edgeFilePath) {
    std::lock_guard<std::recursive_mutex> lck(m_mtx);
    ofstream out(edgeFilePath, std::ofstream::out | std::ofstream::trunc);
    if (out.is_open()) {
        for (auto it = edges_.begin(); it != edges_.end(); it++) {
            out << it->nodeA_ << " " << it->nodeB_ << " " << it->max_speed_ << " " << it->leftSafeDist << " "
                << it->rightSafeDist << " " << it->available_ << " " << it->minConfidence << " " << it->single_lane_
                << " " << (it->isUltraOn ? 1 : 0) << " " << (it->anti_drop_ ? 1 : 0) << std::endl;
        }
        out.flush();
#ifndef WIN32
        fflush(cfile(out));
        fsync(cfile(out)->_fileno);
#endif
        out.close();
    }
}

bool NemGlobalPlannerROS::RefreshEdgeAndNode() {
    std::lock_guard<std::recursive_mutex> lck(m_mtx);
    std::stringstream ssNodes;
    for (auto it = nodes_.begin(); it != nodes_.end(); it++) {
        ssNodes << it->id << " " << it->type << " " << it->x << " " << it->y << " " << it->r << std::endl;
    }

    std::stringstream ssEdges;
    for (auto it = edges_.begin(); it != edges_.end(); it++) {
        ssEdges << it->nodeA_ << " " << it->nodeB_ << " " << it->max_speed_ << " " << it->leftSafeDist << " "
                << it->rightSafeDist << " " << it->available_ << " " << it->minConfidence << " " << it->single_lane_
                << " " << (it->isUltraOn ? 1 : 0) << " " << (it->anti_drop_ ? 1 : 0) << std::endl;
    }

    return Init(ssNodes, ssEdges);
}

bool NemGlobalPlannerROS::Init(const std::string &nodeFilePath, const std::string &edgeFilePath) {
    std::lock_guard<std::recursive_mutex> lck(m_mtx);
    ifstream finNode(nodeFilePath.c_str(), std::ios::in);
    ifstream finEdge(edgeFilePath.c_str(), std::ios::in);
    bool bRet = Init(finNode, finEdge);
    finNode.close();
    finEdge.close();
    if (!bRet) {
        ROS_ERROR("[nem_global_planner]: Error, file read failed!");
    }
    return bRet;
}

bool NemGlobalPlannerROS::Init(std::istream &inNodes, std::istream &inEdges) {
    ClearPathMap();
    if (!json_util_.LoadNodeStream(inNodes, nodes_, node_max_id_, node_id_to_index_)) {
        ROS_ERROR("[nem_global_planner]: Load node failed");
        return false;
    }

    if (!json_util_.LoadEdgeStream(inEdges, nodes_, edges_, edge_max_id_, node_id_to_index_, edge_id_to_index_)) {
        ROS_ERROR("[nem_global_planner]: Load edge  failed");
        return false;
    }
    map_changed_ = true;
    ROS_INFO("[nem_global_planner]: Print node and edge information");
    for (int i = 0; i < nodes_.size(); i++) nodes_[i].PrintNodeInfo();
    for (int i = 0; i < edges_.size(); i++) edges_[i].PrintEdgeInfo();
    ROS_INFO("[nem_global_planner]: Nodes Number: %lld Edges Number: %lld", nodes_.size(), edges_.size());
    FloydWarshall();
    CheckFullyConnection();
    return true;
}

void NemGlobalPlannerROS::PrintNodes() {
    for (int i = 0; i < nodes_.size(); i++) nodes_[i].PrintNodeInfo();
}

void NemGlobalPlannerROS::PrintEdges() {
    for (int i = 0; i < edges_.size(); i++) edges_[i].PrintEdgeInfo();
}

void NemGlobalPlannerROS::ClearPathMap() {
    if (dist_) {
        delete[] dist_[0];
        delete[] dist_;
    }
    if (prenode_) {
        delete[] prenode_[0];
        delete[] prenode_;
    }

    map_changed_  = false;
    dist_         = NULL;
    prenode_      = NULL;
    node_max_id_ = edge_max_id_ = 0;
    nodes_.clear();
    edges_.clear();
    node_id_to_index_.clear();
    edge_id_to_index_.clear();
    charge_node_ = CNode();
}

void NemGlobalPlannerROS::SetEdgeSpeed(int nNodeA, int nNodeB, double dSpeed) {
    std::lock_guard<std::recursive_mutex> lck(m_mtx);
    for (vector<CEdge>::iterator it = edges_.begin(); it != edges_.end(); it++) {
        if (it->InEdge(nNodeA, nNodeB)) {
            it->SetSpeed(dSpeed);
            break;
        }
    }
}

void NemGlobalPlannerROS::SetEdgeConfidence(int nNodeA, int nNodeB, double dConfidence) {
    std::lock_guard<std::recursive_mutex> lck(m_mtx);
    for (vector<CEdge>::iterator it = edges_.begin(); it != edges_.end(); it++) {
        if (it->InEdge(nNodeA, nNodeB)) {
            it->SetConfidence(dConfidence);
            break;
        }
    }
}

void NemGlobalPlannerROS::SetEdgeFixDirection(int nNodeA, int nNodeB, CEdge::DIRECTION_FIXED direction) {
    std::lock_guard<std::recursive_mutex> lck(m_mtx);
    for (vector<CEdge>::iterator it = edges_.begin(); it != edges_.end(); it++) {
        if (it->InEdge(nNodeA, nNodeB)) {
            it->SetFixDirection(direction);
            break;
        }
    }
}

void NemGlobalPlannerROS::EnableEdge(int nNodeA, int nNodeB, bool bEnable) {
    std::lock_guard<std::recursive_mutex> lck(m_mtx);
    for (vector<CEdge>::iterator it = edges_.begin(); it != edges_.end(); it++) {
        if (it->InEdge(nNodeA, nNodeB)) {
            it->available_ = bEnable ? 1 : 0;
            break;
        }
    }
}

bool NemGlobalPlannerROS::JudgeEdge() {
    std::lock_guard<std::recursive_mutex> lck(m_mtx);
    for (vector<CEdge>::iterator it = edges_.begin(); it != edges_.end(); it++) {
        if (it->available_ == 0)
            return false;
    }
    return true;
}

void NemGlobalPlannerROS::SetEdgeUrtraMode(int nNodeA, int nNodeB, bool bEnable) {
    std::lock_guard<std::recursive_mutex> lck(m_mtx);
    for (vector<CEdge>::iterator it = edges_.begin(); it != edges_.end(); it++) {
        if (it->InEdge(nNodeA, nNodeB)) {
            it->isUltraOn = bEnable;
            break;
        }
    }
}

bool NemGlobalPlannerROS::GetNodeRForQuickCheck(CNode::IdType iNodeID, double &dbR) {
    bool bRetVal                              = false;
    std::map<int, double>::iterator iter_find = map_node_.find(iNodeID);
    if (iter_find != map_node_.end()) {
        dbR     = iter_find->second;
        bRetVal = true;
    }

    return bRetVal;
}

void NemGlobalPlannerROS::LoadNodeDevR(std::string devFilePath) {
    std::ifstream nodeDevRHandle(devFilePath.c_str(), std::ios::in);
    size_t pos;
    do {
        if (!nodeDevRHandle.is_open()) {
            break;
        }
        std::string strDevContext = "";
        std::string strDevID, strDevName, strMethod;
        int iNodeID;
        float fPan;
        while (getline(nodeDevRHandle, strDevContext)) {
            if (0 == strDevContext.compare(""))
                continue;
            while ((pos = strDevContext.find("|")) != string::npos) {
                strDevContext.replace(pos, 1, " ");
            }
            istringstream ssDevContext(strDevContext);
            ssDevContext >> strDevID >> strDevName >> iNodeID >> strMethod >> fPan;
            /*fPan = fPan * xform::pi_180;*/
            map_node_.insert(std::make_pair(iNodeID, fPan));
        }
    } while (0);
    nodeDevRHandle.close();
}

void NemGlobalPlannerROS::SetEdgeFallMode(int nNodeA, int nNodeB, bool bEnable) {
    std::lock_guard<std::recursive_mutex> lck(m_mtx);
    for (vector<CEdge>::iterator it = edges_.begin(); it != edges_.end(); it++) {
        if (it->InEdge(nNodeA, nNodeB)) {
            it->anti_drop_ = bEnable;
            break;
        }
    }
}

float NemGlobalPlannerROS::GetDist(const CNode &nodeA, const CNode &nodeB) {
    std::lock_guard<std::recursive_mutex> lck(m_mtx);
    if (node_id_to_index_.find(nodeA.id) != node_id_to_index_.end() &&
        node_id_to_index_.find(nodeB.id) != node_id_to_index_.end()) {
        int nodeNum = nodes_.size();
        if (node_id_to_index_[nodeA.id] < nodeNum && node_id_to_index_[nodeB.id] < nodeNum) {
            return dist_[node_id_to_index_[nodeA.id]][node_id_to_index_[nodeB.id]];
        }
    }
    return DIST_MAX;
}

CNode NemGlobalPlannerROS::GetChargeNode() {
    std::lock_guard<std::recursive_mutex> lck(m_mtx);
    return charge_node_;
}

vector<CNode> NemGlobalPlannerROS::GetNodes() {
    std::lock_guard<std::recursive_mutex> lck(m_mtx);
    return nodes_;
}

map<CNode::IdType, CNode::IdType> &NemGlobalPlannerROS::GetNodeIDToIndex() {
    std::lock_guard<std::recursive_mutex> lck(m_mtx);
    return node_id_to_index_;
}

std::vector<CEdge> NemGlobalPlannerROS::GetEdges() {
    std::lock_guard<std::recursive_mutex> lck(m_mtx);
    return edges_;
}

map<int, double> NemGlobalPlannerROS::GetNodeRs() {
    std::lock_guard<std::recursive_mutex> lck(m_mtx);
    return map_node_;
}

};  // namespace nem_global_planner
