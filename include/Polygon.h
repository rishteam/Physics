#ifndef Polygon_H
#define Polygon_H

#include "Shape.h"

class Box;
class Polygon;
class Circle;

class Polygon : public Shape
{
public:
    Polygon(std::deque<Vec2> &pt, Vec2 pos);
    ~Polygon() = default;
    void setVertices();

    virtual Vec2 supportPoint(Vec2 D) override;
    virtual bool isCollide(Shape &s) override
    {
        return s.isCollide(*this);
    }
    virtual bool isCollide(Box &b) override;
    virtual bool isCollide(Polygon &p) override;
    virtual bool isCollide(Circle &c) override;
    virtual void set_debug_draw() override;

private:
    sf::ConvexShape polygon;
    std::deque<Vec2> corner;
    virtual void draw(sf::RenderTarget &target, sf::RenderStates states) const
    {
        target.draw(polygon, states);
    };
};

#endif