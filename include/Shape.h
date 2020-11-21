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
    /// 找最小穿透軸
    float FindAxisLeastPenetration(int *faceIdx, Shape *A, Shape *B);
    /// 找IncidentFace
    void FindIncidentFace(Vec2 *v, Shape *Ref, Shape *Inc, int refIdx);
    /// 面切除決定點
    int Clip(Vec2 n, float c, Vec2 *face);
    /// 最小穿透軸比較計算公式
    bool BiasGreaterThan( float a, float b );

    // Detect Collision
    virtual bool Collide(Arbiter *m, Shape &s) = 0;
    virtual bool Collide(Arbiter *m, Box &b) = 0;
    virtual bool Collide(Arbiter *m, Polygon &p) = 0;
    virtual bool Collide(Arbiter *m, Circle &c) = 0;


    int m_vertexCount = 0;
    // Vertices(只有位移量的點), Normal(法向量), u(旋轉矩陣)
    bool selected = false;
    Mat22 u;
    Vec2 m_vertices[MaxVertexCount];
    Vec2 m_normals[MaxVertexCount];

};