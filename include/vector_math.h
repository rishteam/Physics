#pragma once
#include <iostream>
#include <limits>
#include <cmath>
#include <fmt/core.h>
#include <SFML/Graphics.hpp>

#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)

#define MAX_float std::numeric_limits<float>::max()
#define MIN_float std::numeric_limits<float>::min()

class Vector : public sf::Vector2<float> {
public:

    Vector(float x, float y);
    ~Vector() = default;
    //print vector
    void print_Vector();
    //拿取向量長度
    float getLength();
    //兩向量之內積
    float dot(Vector &vec2);
    // 向量vec在向量vec2上的正射影長
    float projectLengthOnto(Vector &vec2);
    // 左法向量
    Vector normalL();
    // 右法向量
    Vector normalR();
    // 向量旋轉
    void rotate(float angle);
    //x′1=[(x1−x0)cosθ−(y1−y0)sinθ]+x0
    //y′1=[(y1−y0)cosθ+(x1−x0)sinθ]+y0
    //依照參考點旋轉角度
    void rotate_ref(float angle, Vector &ref);
};