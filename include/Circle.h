#pragma once
#include "Shape.h"

class Box;
class Polygon;
class Arbiter;

class Circle : public Shape
{
public:
    Circle(float x_, float y_, float radius_, float mass);
    ~Circle() = default;

    float getRadius();

    virtual void setDebugDraw() override;

    virtual bool Collide(Arbiter &arb, Shape &s) override
    {
        return s.Collide(arb, *this);
    }
    virtual bool Collide(Arbiter &arb, Box &b) override;
    virtual bool Collide(Arbiter &arb, Polygon &p) override;
    virtual bool Collide(Arbiter &arb, Circle &c) override;

private:
    sf::CircleShape circle;
    float radius;
    virtual void draw(sf::RenderTarget &target, sf::RenderStates states) const
    {
        target.draw(circle, states);
    };
};
