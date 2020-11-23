#pragma once
#include "Shape.h"

class Arbiter;
class Polygon;
class Circle;
class Box;
class Arbiter;

class Box : public Shape
{
public:
    Box(float x, float y, float w, float h, float mass_);
    ~Box() = default;

    virtual void setDebugDraw() override;

//    virtual bool Collide(Arbiter &arb, Shape &s) override
//    {
//        return s.Collide(arb, *this);
//    }
//    virtual bool Collide(Arbiter &arb, Box &b) override;
//    virtual bool Collide(Arbiter &arb, Polygon &p) override;
//    virtual bool Collide(Arbiter &arb, Circle &c) override;


private:
    sf::ConvexShape polygon;
    std::deque<Vec2> cordTranslate;
    virtual void draw(sf::RenderTarget &target, sf::RenderStates states) const
    {
        target.draw(polygon, states);
    };
};