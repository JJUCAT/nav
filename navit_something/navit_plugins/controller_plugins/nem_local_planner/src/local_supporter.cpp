#include "nem_local_planner/local_supporter.h"
#include <math.h>
#include <algorithm>

#define PI 3.1415926

float getAngelOfTwoVector(const Point &NodeOne, const Point &NodeTwo, const Point &NodeThree) {
    float num1, num2, costheta, theta;
    num1 = (NodeTwo.x - NodeOne.x) * (NodeThree.x - NodeOne.x) + (NodeTwo.y - NodeOne.y) * (NodeThree.y - NodeOne.y);
    num2 = sqrt(pow((NodeTwo.x - NodeOne.x), 2) + pow((NodeTwo.y - NodeOne.y), 2)) *
           sqrt(pow((NodeThree.x - NodeOne.x), 2) + pow((NodeThree.y - NodeOne.y), 2));
    costheta = num1 / num2;
    theta    = acos(costheta) * 180.0 / PI;
    return theta;
}

bool IsPointsInLine(const Point &a, const Point &b, const Point &c, float condition, float conditionB) {
    float area = a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y);

    if (fabs(area) == 0.0) {
        return true;
    } else if ((fabs(area) > 0.0) && (fabs(area) < condition)) {
        float thetaA = getAngelOfTwoVector(a, b, c);
        float thetaB = getAngelOfTwoVector(b, c, a);
        float thetaC = getAngelOfTwoVector(c, a, b);
        float mintheta =
            ((thetaA < thetaB) ? (thetaA < thetaC ? thetaA : thetaC) : (thetaB < thetaC ? thetaB : thetaC));

        if (mintheta < conditionB) {
            return true;
        } else {
            return false;
        }
    } else {
        return false;
    }
}

bool IsPointOnLineStricter(const Point &a, const Point &b, const Point &c, float threshold) {
    float area = (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y))/2;
    if (fabs(area) == 0.0)
        return true;
    else {
        float distAB     = sqrt((b.x - a.x)*(b.x - a.x) + (b.y - a.y)*(b.y - a.y));
        float cDistToAB  = 2 * area / distAB;
        // ROS_WARN("[local_supporter] Distance to the line is %f.", cDistToAB);
        if(fabs(cDistToAB) <= threshold)
            return true;
        else
            return false;
    }
}

float SlopeLine(Point p1, Point p2) { return atan2(p2.y - p1.y, p2.x - p1.x); }

float SlopeLinePose(Pose p1, Pose p2) { return atan2(p2.y - p1.y, p2.x - p1.x); }

float sign(float value) {
    if (value >= 0)
        return 1.0;
    else
        return -1.0;
}

bool TranslationP(geometry_msgs::PoseStamped A, geometry_msgs::PoseStamped B, geometry_msgs::PoseStamped P) {
    //判断P点处于有向线段A->B的位置，在順時針位置則爲true
    tf::Transform transform_A_to_map;
    tf::Vector3 origin_A_to_map;
    origin_A_to_map = {A.pose.position.x, A.pose.position.y, 0.0};

    double yaw_AB_to_map;
    yaw_AB_to_map = atan2(B.pose.position.y - A.pose.position.y, B.pose.position.x - A.pose.position.x);

    transform_A_to_map.setOrigin(origin_A_to_map);
    transform_A_to_map.setRotation(tf::createQuaternionFromYaw(yaw_AB_to_map));

    tf::Vector3 origin_P_to_map;
    origin_P_to_map = {P.pose.position.x, P.pose.position.y, 0.0};

    tf::Vector3 P_to_A = transform_A_to_map.inverse() * origin_P_to_map;

    if (P_to_A.y() > 0.000)
        return true;
    else
        return false;
}

double GetDisPointToLine(const geometry_msgs::PoseStamped &P, const geometry_msgs::PoseStamped &A,
                         const geometry_msgs::PoseStamped &B) {
    double dis;
    double a, b, c;                                                                         //以双精度保存变量
    a = (P.pose.position.x - A.pose.position.x) * (B.pose.position.y - A.pose.position.y);  //分子的左半部分
    b = (P.pose.position.y - A.pose.position.y) * (A.pose.position.x - B.pose.position.x);  //分子的右半部分
    c = a + b;                                                                              //二者相加
    c *= c;                                             //平方(pow(c,2)貌似在这里更加麻烦)
    a = pow(B.pose.position.y - A.pose.position.y, 2);  //分母左半部分
    b = pow(A.pose.position.x - B.pose.position.x, 2);  //分母右半部分
    if (a + b == 0.000000000) {
        dis = 0.0;
        return dis;
    }
    c /= (a + b);               //分子分母相除
    dis = sqrt(c);              //开方
    if (TranslationP(A, B, P))  // TranslationP true为顺时针，false为逆时针
        dis = -dis;
    return dis;
}

double GetDistance(const Pose &p1, const Pose &p2) { return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2)); }

double GetDistance(vector<CNode>::iterator p1, vector<CNode>::iterator p2) {
    return hypot(p2->x - p1->x, p2->y - p1->y);
}

double GetProDist(const geometry_msgs::PoseStamped &start_pose, const geometry_msgs::PoseStamped &goal_pose,
                  const geometry_msgs::PoseStamped &robot_pose) {
    tf::Transform transform_goal_to_map;
    tf::Vector3 origin_goal_to_map;
    origin_goal_to_map       = {goal_pose.pose.position.x, goal_pose.pose.position.y, 0.0};
    double yaw_start_to_goal = atan2(goal_pose.pose.position.y - start_pose.pose.position.y,
                                     goal_pose.pose.position.x - start_pose.pose.position.x);
    transform_goal_to_map.setOrigin(origin_goal_to_map);
    transform_goal_to_map.setRotation(tf::createQuaternionFromYaw(yaw_start_to_goal));

    tf::Vector3 origin_robot_to_map = {robot_pose.pose.position.x, robot_pose.pose.position.y, 0.0};
    tf::Vector3 robot_to_goal       = transform_goal_to_map.inverse() * origin_robot_to_map;
    return fabs(robot_to_goal.x());
}