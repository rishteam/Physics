#pragma once
#include "Shape.h"

class Box;
class Polygon;
class Manifold;

class Circle : public Shape
{
public:
    Circle(float x_, float y_, float radius_);
    ~Circle() = default;

    float getRadius();
    void setPosition(Vec2 mouse);
    virtual void setDebugDraw() override;

    virtual bool Collide(Manifold *m, Shape *s) override
    {
        return s->Collide(m, this);
    }
    virtual bool Collide(Manifold *m, Box *b) override;
    virtual bool Collide(Manifold *m, Polygon *p) override;
    virtual bool Collide(Manifold *m, Circle *c) override;

private:
    sf::CircleShape circle;
    float radius;
    virtual void draw(sf::RenderTarget &target, sf::RenderStates states) const
    {
        target.draw(circle, states);
    };
};
