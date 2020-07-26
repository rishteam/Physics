#pragma once

#include "vector_math.h"


//include object
class Circle;
class Polygon;
class Box;

class Shape : public sf::Transformable, public sf::Drawable
{
public:
    std::deque<Vec2> getSAT() const;
    std::deque<Vec2> getVertices() const;
    void findSAT();
    //算出SAT實際的點
    virtual void setVertices(){};
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