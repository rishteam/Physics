#pragma once

#include "vector_math.h"
#include <deque>

//include object
class Circle;
class Polygon;
class Box;
class World;
enum class COLLISION;

class Shape : public sf::Transformable, public sf::Drawable
{
public:
    //for SAT
    std::deque<Vec2> getSAT() const;
    std::deque<Vec2> getVertices() const;
    void findSAT();
    virtual void setVertices(){};

    Vec2 SupportFun(Shape &A, Shape &B, Vec2 D);
    virtual Vec2 supportPoint(Vec2 D) = 0;

    virtual bool isCollide(Shape &s) = 0;
    virtual bool isCollide(Box &b) = 0;
    virtual bool isCollide(Polygon &p) = 0;
    virtual bool isCollide(Circle &c) = 0;
    virtual void set_debug_draw() = 0;
    bool selected = false;
protected:
    std::deque<Vec2> Vertices;
    std::deque<Vec2> SAT;
};