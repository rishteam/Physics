#pragma once
#include "Shape.h"

class Box;
class Polygon;
class Arbiter;

class Circle : public Shape
{
public:
    Circle(float x_, float y_, float radius_);
    ~Circle() = default;

    float getRadius();

    virtual void setDebugDraw() override;

    virtual bool Collide(Arbiter *m, Shape &s) override
    {
        return s.Collide(m, *this);
    }
    virtual bool Collide(Arbiter *m, Box &b) override;
    virtual bool Collide(Arbiter *m, Polygon &p) override;
    virtual bool Collide(Arbiter *m, Circle &c) override;

private:
    sf::CircleShape circle;
    float radius;
    virtual void draw(sf::RenderTarget &target, sf::RenderStates states) const
    {
        target.draw(circle, states);
    };
};
