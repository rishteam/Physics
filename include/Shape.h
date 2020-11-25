#pragma once

#include <SFML/Graphics.hpp>
#include <deque>
#include <cmath>

#include "Physics.h"
#include "vector_math.h"

#define MaxVertexCount 64

//include object
class Circle;
class Polygon;
class Box;
class World;
class Arbiter;

class Shape : public sf::Drawable, public Physics
{
public:

    enum class Type{
        Circle,
        Box,
        Polygon
    } type;

    // Helper Function
    virtual void setDebugDraw() = 0;
    /// 找向量的最遠點
    virtual Vec2 GetSupport(const Vec2 dir);
    /// 設定旋轉矩陣
    virtual void SetMatrix(float radians);
    virtual void setPosition(Vec2 mouse);


    // ManiFold Helper Function


    // Detect Collision
//    virtual bool Collide(Arbiter &arb, Shape *s);
//    virtual bool Collide(Arbiter &arb, Box &b) = 0;
//    virtual bool Collide(Arbiter &arb, Polygon &p) = 0;
//    virtual bool Collide(Arbiter &arb, Circle &c) = 0;

    bool selected = false;
    int m_vertexCount = 0;
    // Vertices(只有位移量的點), Normal(法向量), u(旋轉矩陣)
    Mat22 u;
    Vec2 m_vertices[MaxVertexCount];
    Vec2 m_normals[MaxVertexCount];

};