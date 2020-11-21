#pragma once
#include "Shape.h"

class Box;
class Circle;
class Arbiter;

class Polygon : public Shape
{
public:
    Polygon(std::deque<Vec2> &pt, Vec2 pos, float mass_);
    ~Polygon() = default;

    int getVertexCount();
    // Helper function
    sf::ConvexShape& getPolygonDraw(){ return polygon; };
    virtual void setDebugDraw() override;

    // Physics Collision Detection
    virtual bool Collide(Arbiter *m, Shape &s) override
    {
        return s.Collide(m, *this);
    }
    virtual bool Collide(Arbiter *m, Box &b) override;
    virtual bool Collide(Arbiter *m, Polygon &p) override;
    virtual bool Collide(Arbiter *m, Circle &c) override;

private:
    sf::ConvexShape polygon;
    std::deque<Vec2> cordTranslate;
    virtual void draw(sf::RenderTarget &target, sf::RenderStates states) const
    {
        target.draw(polygon, states);
    };
};
