#ifndef NEM_GLOBAL_PLANNER_H_
#define NEM_GLOBAL_PLANNER_H_

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <jsoncpp/json/json.h>
#include <memory.h>
#include <navit_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <nem_global_planner/line_iterator.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>
#include "nem_global_planner/cedge.h"
#include "nem_global_planner/cnode.h"
#include "nem_global_planner/common.h"
#include "nem_global_planner/json_util.h"
#include "nem_global_planner/pose.h"
#include "nem_global_planner/util.h"

#define DIST_MAX 10e7
using namespace std;

namespace nem_global_planner {

class CSegmentPath {
   public:
    CSegmentPath() {
        uStartID = uEndID = -1;
        uDoorStartID = uDoorEndID = -1;
        vecPath.clear();
        dbLineDistance = 0.0;
    }

    virtual ~CSegmentPath() {}

   public:
    vector<CNode::IdType> vecPath;
    CNode::IdType uStartID, uEndID;
    CNode::IdType uDoorStartID, uDoorEndID;
    float dbLineDistance;

   public:
    void Init() {
        uStartID = uEndID = -1;
        vecPath.clear();
    }

    void OutputLOG();  // {	ROBOT_INFO("Segment Path StartID = %d EndID = %d",uStartID,uEndID);}

    bool InitSegmentPath(std::string strData);
};

class NemGlobalPlannerROS : public CSingleton<NemGlobalPlannerROS>, public navit_core::GlobalPlanner {
    DECLARE_SINGLETON_CLASS(NemGlobalPlannerROS);

   public:
    NemGlobalPlannerROS();

    virtual ~NemGlobalPlannerROS() {
        if (dist_) {
            delete[] dist_[0];
            delete[] dist_;
        }
        if (prenode_) {
            delete[] prenode_[0];
            delete[] prenode_;
        }
    }

   public:
    /***************************************************navit_core::BaseGlobalPlanner******************************************/
    enum GlobalPlanStartNodeState { ON_NODE, ON_EDGE, NOT_ON_ANY_PATH };

    void initialize(const std::string& name,
                    const std::shared_ptr<navit_costmap::Costmap2DROS>& costmap_ros) override;

    nav_msgs::Path makePlan(const geometry_msgs::PoseStamped& start,
                            const geometry_msgs::PoseStamped& goal) override;

    void initialize(std::string name, navit_costmap::Costmap2DROS *costmap_ros);

    void initialize(std::string name, navit_costmap::Costmap2D *costmap, std::string frame_id);

    bool makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                  std::vector<geometry_msgs::PoseStamped> &plan);

    bool initNemData(const std::string &node_file_path, const std::string &edge_file_path) override;

   protected:
    /**
     * @brief Store a copy of the current costmap in \a costmap.  Called by makePlan.
     */
    navit_costmap::Costmap2D *costmap_;
    std::string frame_id_;
    bool initialized_;

   public:
    void LoadNodeDevR(std::string devFilePath);

    bool GetNodeRForQuickCheck(CNode::IdType iNodeID, double &dbR);

    bool Init(const std::string &nodeFilePath, const std::string &edgeFilePath);

    bool ExistNode(CNode::IdType id);

    bool IsKeyNode(CNode::IdType id);

    // todo bool return
    CNode GetNode(CNode::IdType id);

    // todo bool return
    CEdge GetEdge(CNode::IdType id);

    std::vector<CEdge> GetEdges();

    bool GetEdgeByNode(CNode::IdType id, CEdge &oEdge);

    float CalDistBetweenNodeAndPose(const CNode::IdType nodeID, const float x, const float y);

    void SetPathPlanType(int iPathType);

    void SetNodeRForQuickCheck(int iNodeID, double dbNodeR);

    /**
      0 --- not found
      1 --- A---->B
      2 --- B---->A
    */
    int FindEdgeByNodes(const CNode &nodeA, const CNode &nodeB, CEdge &edge);

    int FindEdgeByPoints(Point &pointA, Point &pointB, CEdge &edge);

    bool IsNodeAvaiable(CNode::IdType nNodeID);

    vector<unsigned int> GetAimPath(unsigned int startID, float &pathLen, vector<unsigned int> &aimNodes, bool &bFlag,
                                    vector<CSegmentPath> &vecSegmentPath, int mode = 0);

    vector<unsigned int> GetAimPathForDistanceV20(unsigned int startID, float &pathLen, vector<unsigned int> &aimNodes,
                                                  bool &bFlag, int mode = 0);

    vector<CNode::IdType> GetAimPathForDistanceV10(CNode::IdType startID, float &pathLen, vector<CNode::IdType> &aimNodes,
                                                  bool &bFlag, int mode = 0);

    vector<unsigned int> GetAimPathForTime(unsigned int startID, float &pathLen, vector<unsigned int> &aimNodes,
                                           bool &bFlag, int mode = 0);

    vector<CNode::IdType> GetDetailPath(const vector<CNode::IdType> &aimPath);

    vector<CNode::IdType> GetDetailPath(CNode::IdType startID, CNode::IdType endID);

    bool IsInLine(CSegmentPath &oSegmentPath, vector<CNode::IdType> &aimPath);

    // 获取距离机器人当前位置最近的边ID
    bool GetNearestEdge(float x, float y, CNode::IdType &nodeId, CNode::IdType &edgeId,const CNode::IdType &mapId);

    // 获取机器人从当前位置开始行驶，规划路径最短的边
    bool GetShortestPathEdge(float x, float y, CNode::IdType &edgeId,
                                         const CNode::IdType &mapId, const vector<CNode::IdType> &aimPathIndex);

    // 已知机器人所在边，计算机器人到当前边哪端路径更近,并将端点加入当前路径
    bool GetNearestNodeOnEdge(vector<CNode> &pre_plan_final_aim_path_, const vector<CNode::IdType> &aimPathIndex, 
                                                            vector<CNode::IdType> &path, const CNode::IdType &edgeId);

    bool GetCurrentEdge(float x, float y, const std::vector<unsigned int> &pathNodes, CEdge &edge);

    bool GetMapId(CNode::IdType &nodeId, CNode::IdType &mapId);

    // 机器人位于多条重叠边之上时，选取路径最短的边作为当前机器人所在边(相当于已知当前edgeID后进行一次预路径规划并计算路径长度)
    float GetPathLenForPrePlan(const vector<CNode::IdType> &aimPathIndex, const CNode::IdType &edgeId);

    // 根据机器人位置获取判断当前机器人所处的点位
    bool IsOnNode(float x, float y, CNode::IdType &nodeId, const CNode::IdType &mapId);

    // 判断接收到 move_base/goal.pose.position.z 目标点是否合法
    bool IsOnNode(const geometry_msgs::PoseStamped &goal, CNode::IdType &nodeId, const CNode::IdType &mapId);

    bool RefreshEdgeAndNode();

    void SaveNodes(const char *nodeFilePath);

    void SaveEdges(const char *edgeFilePath);

    void SetEdgeSpeed(int nNodeA, int nNodeB, double dSpeed);

    void SetEdgeConfidence(int nNodeA, int nNodeB, double dConfidence);

    void SetEdgeFixDirection(int nNodeA, int nNodeB, CEdge::DIRECTION_FIXED direction);

    void EnableEdge(int nNodeA, int nNodeB, bool bEnable = true);

    bool JudgeEdge();

    void SetEdgeUrtraMode(int nNodeA, int nNodeB, bool bEnable);

    void SetEdgeFallMode(int nNodeA, int nNodeB, bool bEnable);

    float GetDist(const CNode &nodeA, const CNode &nodeB);

    CNode GetChargeNode();

    map<CNode::IdType, CNode::IdType> &GetNodeIDToIndex();

    vector<CNode> GetNodes();

    //获取两个点之间的距离
    float GetNodePathLength(CNode::IdType nodeIdA, CNode::IdType nodeIdB);

    float GetNodeTimeForQuickCheck(CNode::IdType nodeIdA, CNode::IdType nodeIdB);

   public:
    GlobalPlanStartNodeState start_node_state_;
    string node_file_path_;
    string edge_file_path_;
    vector<CNode> aim_path_nodes_;
    Pose robot_pose_;
    CNode::IdType global_start_id_;
    // 同步点边数据时会重新实例化全局规划器，全局变量会被初始化，这里定义为static变量
    static CNode::IdType map_id_;
    
    ros::Subscriber robot_pose_sub_;
    ros::Subscriber robot_sim_pose_sub_;
    ros::Subscriber map_id_sub_;

    void PoseReceived(const geometry_msgs::PoseStampedConstPtr pose);

    void PoseReceived_amcl(const geometry_msgs::PoseWithCovarianceStampedConstPtr pose);

    void MapIdReceived(const std_msgs::StringConstPtr &mapId);

    ros::Publisher plan_pub_;
    ros::Publisher marker_pub;
    ros::Publisher navi_error_pub_;

    json_util::JsonUtil json_util_;

   public:
    bool Init(std::istream &inNodes, std::istream &inEdges);

    bool ReadFromJson(const std::istream &node_stream, Json::Value &val);

    void DeleteNode(unsigned int id);

    void pathInterpolate(vector<CNode> &aim_nodes, std::vector<geometry_msgs::PoseStamped> &plan, double res);

    void DeleteEdge(CNode::IdType id);

    void PrintNodes();

    void PrintEdges();

    void ClearPathMap();

    void InitMatrix();

    void FloydWarshall();

    bool GetShortestPath(CNode::IdType A, CNode::IdType B, float &minLen, vector<CNode::IdType> &path);

    void GetShortPathNodeID(CNode::IdType iOrgID, vector<CNode::IdType> &path, CNode::IdType &iPathIndex);

    bool CheckFullyConnection();

    float GetPathLength(const vector<CNode::IdType> &path, int mode = 0, bool bLogFlag = true);

    float GetPathLength(const vector<CNode> &path);

    template <class T>
    void PrintMatrix(T **m, int l, int h);

    void AdjustPathResult(vector<CNode::IdType> &path);

    bool IsInAimPath(CNode::IdType iNodeID, vector<CNode::IdType> &vecSortPath, CNode::IdType &iInsertPos);

    void MakeSegmentPath(vector<unsigned int> &path, vector<CSegmentPath> &vecSegmentPath);

    void MakeSegmentPathDoor(CSegmentPath &oSegmentPath);

    void AdjustSegmentPath(CSegmentPath &oSegmentPath);

    void AdjustSegmentWay(CSegmentPath &oSegmentPath);

    int GetMaxFarDoorID(CSegmentPath &oSegmentPath, CNode::IdType iStartID);

    int GetMaxNearNodeID(CSegmentPath &oSegmentPath, CNode::IdType iStartID, CNode::IdType iMustID,
                         CNode::IdType iDistNodeID);

    void GetRelateNodeID(CNode::IdType iNodeID, vector<CNode::IdType> &vecRelateNodeID);

    bool IsInShortPath(CNode::IdType iStartID, CNode::IdType iEndID, CNode::IdType iMustID);

    bool MergeSegmentPath(CSegmentPath &oSegmentPath, vector<CSegmentPath> &vecSegmentPath);

    void SegmentSort(vector<CSegmentPath> &vecSegmentPath);

    void SegmentSortV10(vector<CSegmentPath> &vecSegmentPath);

    float GetPathLen(vector<CSegmentPath> &vecSegmentPath, vector<CNode::IdType> &vecPosition);

    bool IsExistCheckAreaPath(vector<unsigned int> &vecPath, vector<unsigned int> &vecNormalPath);

    bool IsExistWorkNode(CSegmentPath &oSegmentPath, vector<CNode::IdType> &vecWorkNode);

    bool IsInWorkNode(CNode::IdType iNode, vector<CNode::IdType> &vecWorkNode);

    bool CalcSegmentDistance(vector<CSegmentPath> &vecSegmentPath, CSegmentPath &oSignalSegmentPath, int iPosition,
                             float &dbDistance);

    bool IsInNotValiableEdge(CNode::IdType iNodeID);

    void ConvertSegmentPath2Sort(vector<CSegmentPath> &vecSegmentPath, vector<unsigned int> &vecSort, int nStartNodeID);

    void ConvertStr2SegmentPath(std::string strSort, vector<CSegmentPath> &vecSegmentPath);

    std::string ConvertSegmentPath2Str(vector<CSegmentPath> &vecSegmentPath);

    map<int, double> GetNodeRs();

   private:
    const float MAX_FLOAT_NUM_ = std::numeric_limits<float>::max();
    const long  MAX_LONG_NUM_  = std::numeric_limits<CNode::IdType>::max();
    vector<CNode> nodes_;
    vector<CEdge> edges_;
    CNode charge_node_;

    map<CNode::IdType, CNode::IdType> node_id_to_index_;
    map<CNode::IdType, CNode::IdType> edge_id_to_index_;
    bool map_changed_;
    float **dist_;
    CNode::IdType **prenode_;
    double on_node_dist_;
    double on_edge_dist_;
    CNode::IdType node_max_id_;
    CNode::IdType edge_max_id_;
    std::recursive_mutex m_mtx;

    std::map<int, double> map_node_;
    int path_type_;
};
};      // namespace nem_global_planner
#endif  // NEM_GLOBAL_PLANNER_H_
