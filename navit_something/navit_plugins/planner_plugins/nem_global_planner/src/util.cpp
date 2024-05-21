#include "nem_global_planner/util.h"
#include <math.h>
#include <algorithm>

#define PI 3.1415926

/***************************************/

bool IsPointsInLineForInfrared(const Point &a, const Point &b, const Point &c, float condition /* = 0.8 */) {
    return IsPointsInLine(a, b, c);

    double dbAngleOffset = xform::normalize(SlopeLine(b, a) - SlopeLine(c, a));
    dbAngleOffset        = fabs(dbAngleOffset);
    if (fabs(dbAngleOffset - 0) <= condition || fabs(dbAngleOffset - xform::pi) <= condition) {
        return true;
    }
    return false;
}

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

float SlopeLine(Point p1, Point p2) { return atan2(p2.y - p1.y, p2.x - p1.x); }

float PointToLineDist(Point pp, float k, float b) { return fabs(k * pp.x - pp.y + b) / sqrt(k * k + 1); }

float PointToLineDist(Point pp, Point p1, Point p2) {
    float fDotProduct = (p2.x - p1.x) * (pp.x - p1.x) + (p2.y - p1.y) * (pp.y - p1.y);
    if (fDotProduct <= 0) {
        ROBOT_WARN("Robot pose is after line!");
    }

    float dd = (p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y);
    if (fDotProduct >= dd) {
        ROBOT_WARN("Robot pose is before line!");
    }

    float r      = fDotProduct / dd;
    float px     = p1.x + (p2.x - p1.x) * r;
    float py     = p1.y + (p2.y - p1.y) * r;
    float angle1 = SlopeLine(p1, p2);
    float angle2 = SlopeLine(p1, pp);
    float dx     = xform::normalize(angle2 - angle1);
    /**
      if robot is leftside => dist >0
      if robot is rightsid => dist <0
     */
    if (dx <= 0.0)
        return -sqrt((pp.x - px) * (pp.x - px) + (py - pp.y) * (py - pp.y));
    else
        return sqrt((pp.x - px) * (pp.x - px) + (py - pp.y) * (py - pp.y));
}

bool LineFit(float &kk, float &bb, float &ee, std::valarray<float> &data_x, std::valarray<float> &data_y) {
    if (data_x.size() <= 1) {
        ROBOT_WARN("Line fit point is less than one!");
        return false;
    }
    double A = 0.0;
    double B = 0.0;
    double C = 0.0;
    double D = 0.0;

    A = (data_x * data_x).sum();
    B = data_x.sum();
    C = (data_x * data_y).sum();
    D = data_y.sum();
    float k, b, tmp = 0;
    if (tmp = (A * data_x.size() - B * B)) {
        k = (C * data_x.size() - B * D) / tmp;
        b = (A * D - C * B) / tmp;
    } else {
        k = 0;
        b = B / data_x.size();
        ROBOT_WARN("k is zero!");
    }
    kk = k;
    bb = b;
    std::valarray<float> error(data_x.size());
    error = data_x * k + b - data_y;
    ee    = (error * error).sum() / error.size();
    return true;
}

void DetectCornerPoint(float *const x, float *const y, float *const kDiff, char *const flag, char *const valid,
                       uint num, uint r) {
    for (int i = r + 1; i < num - r; i++) {
        int n = 0;
#ifdef WIN32
        std::vector<float> xx(num);
        std::vector<float> yy(num);
#else
        float xx[num], yy[num];
#endif

        for (int j = i; j >= 0; j--) {
            if (valid[j]) {
                xx[n] = x[j];
                yy[n] = y[j];
                n++;
            }
            if (n >= r)
                break;
        }

        float lk;
        float diff = fabs(xx[n - 1] - xx[0]);
        if (diff < 0.00000000001)
            lk = 10000000.0;
        else
            lk = (yy[n - 1] - yy[0]) / (xx[n - 1] - xx[0]);

        n = 0;
        for (int j = i; j < num; j++) {
            if (valid[j]) {
                xx[n] = x[j];
                yy[n] = y[j];
                n++;
            }
            if (n >= r)
                break;
        }

        float rk;
        diff = fabs(xx[n - 1] - xx[0]);
        if (diff < 0.00000000001)
            rk = 10000000.0;
        else
            rk = (yy[n - 1] - yy[0]) / (xx[n - 1] - xx[0]);

        flag[i]  = 1;
        kDiff[i] = fabs((lk - rk) / (1 + lk * rk));
        // printf("%f ",kDiff[i]);
    }
}

void Smooth(float *const x, float *const y, uint windowSize, uint num, char *const valid) {
    uint r = windowSize / 2;
    for (int i = 0; i < num; i++) {
        float sumX = 0.0;
        float sumY = 0.0;
        int n = 0, count = 0;
        for (int j = i; j >= 0; j--) {
            if (valid[j]) {
                sumX = sumX + x[j];
                sumY = sumY + y[j];
                n++;
                count++;
            }
            if (n >= r)
                break;
        }
        n = 0;
        for (int j = i; j < num; j++) {
            if (valid[j]) {
                sumX = sumX + x[j];
                sumY = sumY + y[j];
                n++;
                count++;
            }
            if (n >= r)
                break;
        }

        x[i] = sumX / count;
        y[i] = sumY / count;
    }
}

vector<Point> FilterCornerPoint(vector<int> &index, float *const x, float *const y, float *const kDiff,
                                char *const flag, uint num, uint r, float threshold) {
    for (int i = 0; i < num; i++) {
        if (flag[i] && kDiff[i] > threshold) {
            int a   = (i - r >= 0 ? i - r : 0);
            int b   = (i + r >= num - 1 ? num - 1 : i + r);
            bool ff = false;
            for (int j = a; j <= b; j++) {
                if (j != i && kDiff[j] > threshold && flag[j]) {
                    if (kDiff[j] > kDiff[i])
                        ff = true;
                }
            }
            if (ff == true)
                continue;
            else {
                for (int j = a; j <= b; j++) {
                    if (j != i && kDiff[j] > threshold && flag[j]) {
                        kDiff[j] = -1.0;
                    }
                }
            }
        }
    }
    vector<Point> corPoint;
    index.clear();
    for (int i = 0; i < num; i++) {
        if (flag[i] && kDiff[i] > threshold) {
            Point temp(x[i], y[i]);
            corPoint.push_back(temp);
            index.push_back(i);
            //  printf("%d:%f",i,kDiff[i]);
        }
    }
    // printf("\n");
    return corPoint;
}

float sign(float value) {
    if (value >= 0)
        return 1.0;
    else
        return -1.0;
}
