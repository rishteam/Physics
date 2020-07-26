#ifndef Box_H
#define Box_H

#include <deque>

#include "Shape.h"
#include "Physics.h"

class Polygon;
class Circle;

class Box : public Shape, public Physics
{
public:
    Box(float x, float y, float w, float h, float m);
    ~Box() = default;
    void setVertices();
    void initPhysics(float m);
    virtual bool isCollide(Shape &s) override
    {
        return s.isCollide(*this);
    }
    virtual bool isCollide(Polygon &s) override;
    virtual bool isCollide(Circle &c) override;
    virtual bool isCollide(Box &b) override;
    virtual void set_debug_draw() override;

private:
    sf::ConvexShape polygon;
    virtual void draw(sf::RenderTarget &target, sf::RenderStates states) const
    {
        target.draw(polygon, states);
    };
    std::deque<Vec2> corner;
    float _w;
    float _h;
};

#endif