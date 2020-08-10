#pragma once
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
    float getwidth();
    float getheight();

    virtual Vec2 supportPoint(Vec2 D) override;

    virtual bool isCollide(Shape &s) override;
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