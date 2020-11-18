#include "Circle.h"
#include "Polygon.h"
#include "Box.h"
#include "Manifold.h"

Circle::Circle(float x_, float y_, float radius_)
{
    type = Shape::Type::Box;
    position.x = x_;
    position.y = y_;
    radius = radius_;
}

float Circle::getRadius()
{
    return radius;
}

void Circle::setDebugDraw()
{
    Vec2 calRadiusLeftCoordinate = World::ConvertWorldToScreen(Vec2(position.x - radius, position.y));
    Vec2 center = World::ConvertWorldToScreen(Vec2(position.x , position.y));
    Vec2 screenRadiusVector = calRadiusLeftCoordinate - center;
    float screenRadius = screenRadiusVector.getLength();
    circle.setPosition(center.x, center.y);
    circle.setOrigin(screenRadius, screenRadius);
    circle.setRadius(screenRadius);
    if (this->selected)
    {
        circle.setFillColor(sf::Color::Red);
    }
    else
    {
        circle.setFillColor(sf::Color::White);
    }
}

bool Circle::Collide(Manifold *m, Box *b)
{
    return false;
}

bool Circle::Collide(Manifold *m, Polygon *p)
{
    return false;
}

bool Circle::Collide(Manifold *m, Circle *c)
{
    return false;
}