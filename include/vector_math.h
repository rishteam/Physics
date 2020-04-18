#pragma once
#include <iostream>
#include <limits>
#include <cmath>

#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)

#define MAX_DOUBLE std::numeric_limits<double>::max()
#define MIN_DOUBLE std::numeric_limits<double>::min()

class Vector{
public:

    Vector() = default;
    Vector(double x, double y);
    ~Vector()=default;
    void set_x(double x);
    void set_y(double y);
    double get_x();
    double get_y();
    friend std::ostream &operator<<(std::ostream &os, const Vector &vec);
    double getLength();
    double dot(Vector &vec2);
    // vec在vec2上的正射影長
    double projectLengthOnto(Vector &vec2);
    // 左法向量
    Vector normalL();
    // 右法向量
    Vector normalR();
    // 旋轉
    // degree
    void rotate(double angle);
    //x′1=[(x1−x0)cosθ−(y1−y0)sinθ]+x0
    //y′1=[(y1−y0)cosθ+(x1−x0)sinθ]+y0
    //degree
    void rotate_ref(double angle, Vector &ref);

private:
    double _x, _y;
};