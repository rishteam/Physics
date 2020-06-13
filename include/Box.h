#ifndef Box_H
#define Box_H

#include <deque>

#include "Shape.h"

class Polygon;
class Circle;

class Box : public Shape
{
public:
    Box(float x, float y, float w, float h);
    ~Box() = default;
    void setVertices();
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
    std::deque<Vector> corner;
    float _w;
    float _h;
};

#endif