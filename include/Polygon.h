#pragma once
#include "Shape.h"

class Box;
class Circle;
class Manifold;

class Polygon : public Shape
{
public:
    Polygon(std::deque<Vec2> &pt, Vec2 pos);
    ~Polygon() = default;

    int getVertexCount();
    // Helper function
    sf::ConvexShape& getPolygonDraw(){ return polygon; };
    virtual void setDebugDraw() override;

    // Physics Collision Detection
    virtual bool Collide(Manifold *m, Shape *s) override
    {
        return s->Collide(m, this);
    }
    virtual bool Collide(Manifold *m, Box *b) override;
    virtual bool Collide(Manifold *m, Polygon *p) override;
    virtual bool Collide(Manifold *m, Circle *c) override;

private:
    sf::ConvexShape polygon;
    std::deque<Vec2> cordTranslate;
    virtual void draw(sf::RenderTarget &target, sf::RenderStates states) const
    {
        target.draw(polygon, states);
    };
};
