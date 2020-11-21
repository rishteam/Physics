#pragma once
#include "Shape.h"

class Manifold;
class Polygon;
class Circle;
class Box;

class Box : public Shape
{
public:
    Box(float x, float y, float w, float h);
    ~Box() = default;

    virtual void setDebugDraw() override;

    virtual bool Collide(Manifold *m, Shape &s) override
    {
        return s.Collide(m, *this);
    }
    virtual bool Collide(Manifold *m, Box &b) override;
    virtual bool Collide(Manifold *m, Polygon &p) override;
    virtual bool Collide(Manifold *m, Circle &c) override;


private:
    sf::ConvexShape polygon;
    std::deque<Vec2> cordTranslate;
    virtual void draw(sf::RenderTarget &target, sf::RenderStates states) const
    {
        target.draw(polygon, states);
    };
};