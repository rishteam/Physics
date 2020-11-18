#include "Box.h"
#include "Circle.h"
#include "Polygon.h"
#include "Manifold.h"

Box::Box(float x, float y, float w, float h, float m)
{
    type = Shape::Type::Box;
    m_vertexCount = 4;
    position.x = x;
    position.y = y;
    wh.x = w;
    wh.y = h;
}

void Box::setDebugDraw()
{
    polygon.setPointCount(m_vertexCount);

    Vec2 center = World::ConvertWorldToScreen(Vec2(position.x, position.y));
    polygon.setPosition(center.x, center.y);

    //sfml needs to give the offset for setting point
    for (int i = 0; i < m_vertexCount; i++)
    {
        polygon.setPoint(i, sf::Vector2f(m_vertices[i].x, m_vertices[i].y));
    }
    if(this->selected)
    {
        polygon.setFillColor(sf::Color::Red);
    }
    else
    {
        polygon.setFillColor(sf::Color::White);
    }
}

bool Box::Collide(Manifold *m, Box *b)
{
    return false;
}
bool Box::Collide(Manifold *m, Polygon *p)
{
    return false;
}
bool Box::Collide(Manifold *m, Circle *c)
{
    return false;
}