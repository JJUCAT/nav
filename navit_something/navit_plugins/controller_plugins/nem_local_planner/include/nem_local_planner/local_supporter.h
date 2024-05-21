#ifndef UTIL_H
#define UTIL_H

#include <math.h>
#include <nem_local_planner/line_iterator.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <Eigen/Dense>
#include <valarray>
#include <vector>
#include "nem_global_planner/pose.h"
#include "nem_global_planner/xform.h"
#include "nem_global_planner/cnode.h"

using namespace std;

struct Point {
    float x, y;
    int type;
    double dbOffset;

    Point(float _x, float _y) {
        x        = _x;
        y        = _y;
        type     = 0;
        dbOffset = -3;
    }

    Point(float _x, float _y, int _type) {
        x        = _x;
        y        = _y;
        type     = _type;
        dbOffset = -4;
    }

    Point() {
        x = y    = 0.0;
        type     = 0;
        dbOffset = -5;
    }

    Point(const Point &other) : x(other.x), y(other.y), type(other.type), dbOffset(other.dbOffset) {}

    bool operator==(const Point &other) {
        if (this == &other) {
            return true;
        }

        return (x == other.x && y == other.y && type == other.type);
    }

    bool operator!=(const Point &other) { return !(*this == other); }

    Point &operator=(const Point &other) {
        if (this == &other) {
            return *this;
        }
        x        = other.x;
        y        = other.y;
        type     = other.type;
        dbOffset = other.dbOffset;
        return *this;
    }
};

bool IsPointsInLine(const Point &a, const Point &b, const Point &c, float condition = 0.8, float conditionB = 5.0);

// 判断点c是否在直线ab上(使用点到直线的距离判断)
// @param threshold: 点到直线距离阈值
bool IsPointOnLineStricter(const Point &a, const Point &b, const Point &c, float threshold = 0.05);

float getAngelOfTwoVector(const Point &NodeOne, const Point &NodeTwo, const Point &NodeThree);

float sign(float value);

float SlopeLine(Point p1, Point p2);

float SlopeLinePose(Pose p1, Pose p2);

bool TranslationP(Pose A, Pose B, Pose P);

double GetDisPointToLine(const geometry_msgs::PoseStamped &P, const geometry_msgs::PoseStamped &A,
                         const geometry_msgs::PoseStamped &B);

double GetDistance(const Pose &p1, const Pose &p2);

double GetDistance(vector<CNode>::iterator p1, vector<CNode>::iterator p2);

double GetProDist(const geometry_msgs::PoseStamped &start_pose, const geometry_msgs::PoseStamped &goal_pose,
                  const geometry_msgs::PoseStamped &robot_pose);

#endif
