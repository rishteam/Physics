#pragma once

#include "vector_math.h"

class Circle : public Shape
{
public:
    Circle(float x, float y, float radius_);
    float get_radius();
    virtual bool iscollide(const Box &b) const override;
    virtual bool iscollide(const Polygon &p) const override;
    virtual bool iscollide(const Circle &c) const override;
    void set_debug_draw() override;
private:
    sf::CircleShape circle;
    virtual void draw(sf::RenderTarget &target, sf::RenderStates states) const
    {
        target.draw(circle, states);
    };
    float radius;
    bool selected = false;
};