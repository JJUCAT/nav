#ifndef UTIL_H
#define UTIL_H

#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <valarray>
#include <vector>
#include "xform.h"

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

bool IsPointsInLineForInfrared(const Point &a, const Point &b, const Point &c, float condition = 0.1);

float getAngelOfTwoVector(const Point &NodeOne, const Point &NodeTwo, const Point &NodeThree);

float sign(float value);

float SlopeLine(Point p1, Point p2);

float PointToLineDist(Point pp, Point p1, Point p2);

float PointToLineDist(Point pp, float k, float b);

bool LineFit(float &k, float &b, float &ee, std::valarray<float> &data_x, std::valarray<float> &data_y);

void Smooth(float *const x, float *const y, uint windowSize, uint num, char *const valid);

void DetectCornerPoint(float *const x, float *const y, float *const kDiff, char *const flag, char *const valid,
                       uint num, uint r);

vector<Point> FilterCornerPoint(vector<int> &index, float *const x, float *const y, float *const kDiff,
                                char *const flag, uint num, uint r, float threshold);

vector<Point> DetectJumpPoint(float *const x, float *const y, char *const flag, char *const valid, uint num, uint begin,
                              uint end, double xRange[2], double yRange[2]);

#endif
