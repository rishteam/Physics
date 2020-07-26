#ifndef Circle_H
#define Circle_H

#include "Shape.h"

class Box;
class Polygon;

class Circle : public Shape
{
public:
    Circle(float x, float y, float radius_);
    ~Circle() = default;
    float get_radius();
    virtual bool isCollide(Shape &s) override
    {
        return s.isCollide(*this);
    }
    virtual bool isCollide(Box &b) override;
    virtual bool isCollide(Polygon &p) override;
    virtual bool isCollide(Circle &c) override;
    virtual void set_debug_draw() override;

private:
    sf::CircleShape circle;
    float radius;
    virtual void draw(sf::RenderTarget &target, sf::RenderStates states) const
    {
        target.draw(circle, states);
    };
};

#endif