#include "Circle.h"
#include "Polygon.h"
#include "Box.h"
#include "Arbiter.h"

Circle::Circle(float x_, float y_, float radius_)
{
    type = Shape::Type::Circle;
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

bool Circle::Collide(Arbiter *m, Box &b)
{
    m->contactCounter = 0;

    // Transform circle center to Polygon model space
    // 找最小穿透軸
    Vec2 center = this->position;
    center = b.u.Transpose( ) * (center - b.position);

    // Find edge with minimum penetration
    // Exact concept as using support points in Polygon vs Polygon
    float separation = -FLT_MAX;
    int faceNormal = 0;
    for(int i = 0; i < b.m_vertexCount; ++i)
    {
        float s = Dot( b.m_normals[i], center - b.m_vertices[i] );
        if(s > this->getRadius())
            return false;
        if(s > separation)
        {
            separation = s;
            faceNormal = i;
        }
    }

    // Grab face's vertices
    // 用點決定Reference Face
    Vec2 v1 = b.m_vertices[faceNormal];
    int i2 = faceNormal + 1 < b.m_vertexCount ? faceNormal + 1 : 0;
    Vec2 v2 = b.m_vertices[i2];

    // Check to see if center is within polygon
    if(separation < EPSILON)
    {
        m->contactCounter = 1;
        m->contacts[0].normal = -(b.u * b.m_normals[faceNormal]);
        m->contacts[0].position = m->contacts[0].normal * this->getRadius() + this->position;
        m->contacts[0].penetration = this->getRadius();
        return true;
    }

    // Determine which voronoi region of the edge center of circle lies within
    float dot1 = Dot( center - v1, v2 - v1 );
    float dot2 = Dot( center - v2, v1 - v2 );
    m->contacts[0].penetration = this->getRadius() - separation;

    // Closest to v1
    // 靠近v1
    if(dot1 <= 0.0f)
    {
        // 檢查是否分離
        if(DistSqr( center, v1 ) > this->getRadius() * this->getRadius())
            return false;

        m->contactCounter = 1;
        Vec2 n = v1 - center;
        n = b.u * n;
        n.Normalize( );
        m->contacts[0].normal = n;
        v1 = b.u * v1 + b.position;
        m->contacts[0].position = v1;
    }

        // Closest to v2
        // 靠近v2
    else if(dot2 <= 0.0f)
    {
        if(DistSqr( center, v2 ) > this->getRadius() * this->getRadius())
            return false;

        m->contactCounter = 1;
        Vec2 n = v2 - center;
        v2 = b.u * v2 + b.position;
        m->contacts[0].position = v2;
        n = b.u * n;
        n.Normalize( );
        m->contacts[0].normal = n;
    }

        // Closest to face
        // 靠近面
    else
    {
        Vec2 n = b.m_normals[faceNormal];
        if(Dot( center - v1, n ) > this->getRadius())
            return false;
        n = b.u * n;
        m->contacts[0].normal = -n;
        m->contacts[0].position =  m->contacts[0].normal * this->getRadius() + this->position;
        m->contactCounter = 1;
    }
    return true;
}

bool Circle::Collide(Arbiter *m, Polygon &p)
{
    m->contactCounter = 0;

    // Transform circle center to Polygon model space
    // 找最小穿透軸
    Vec2 center = this->position;
    center = p.u.Transpose( ) * (center - p.position);

    // Find edge with minimum penetration
    // Exact concept as using support points in Polygon vs Polygon
    float separation = -FLT_MAX;
    int faceNormal = 0;
    for(int i = 0; i < p.m_vertexCount; ++i)
    {
        float s = Dot( p.m_normals[i], center - p.m_vertices[i] );
        if(s > this->getRadius())
            return false;
        if(s > separation)
        {
            separation = s;
            faceNormal = i;
        }
    }

    // Grab face's vertices
    // 用點決定Reference Face
    Vec2 v1 = p.m_vertices[faceNormal];
    int i2 = faceNormal + 1 < p.m_vertexCount ? faceNormal + 1 : 0;
    Vec2 v2 = p.m_vertices[i2];

    // Check to see if center is within polygon
    if(separation < EPSILON)
    {
        m->contactCounter = 1;
        m->contacts[0].normal = -(p.u * p.m_normals[faceNormal]);
        m->contacts[0].position = m->contacts[0].normal * this->getRadius() + this->position;
        m->contacts[0].penetration = this->getRadius();
        return true;
    }

    // Determine which voronoi region of the edge center of circle lies within
    float dot1 = Dot( center - v1, v2 - v1 );
    float dot2 = Dot( center - v2, v1 - v2 );
    m->contacts[0].penetration  = this->getRadius() - separation;

    // Closest to v1
    // 靠近v1
    if(dot1 <= 0.0f)
    {
        // 檢查是否分離
        if(DistSqr( center, v1 ) > this->getRadius() * this->getRadius())
            return false;

        m->contactCounter = 1;
        Vec2 n = v1 - center;
        n = p.u * n;
        n.Normalize( );
        m->contacts[0].normal = n;
        v1 = p.u * v1 + p.position;
        m->contacts[0].position = v1;
    }

        // Closest to v2
        // 靠近v2
    else if(dot2 <= 0.0f)
    {
        if(DistSqr( center, v2 ) > this->getRadius() * this->getRadius())
            return false;

        m->contactCounter = 1;
        Vec2 n = v2 - center;
        v2 = p.u * v2 + p.position;
        m->contacts[0].position = v2;
        n = p.u * n;
        n.Normalize( );
        m->contacts[0].normal = n;
    }

        // Closest to face
        // 靠近面
    else
    {
        Vec2 n = p.m_normals[faceNormal];
        if(Dot( center - v1, n ) > this->getRadius())
            return false;
        n = p.u * n;
        m->contacts[0].normal = -n;
        m->contacts[0].position = m->contacts[0].normal  * this->getRadius() + this->position;
        m->contactCounter = 1;
    }
    return true;
}

bool Circle::Collide(Arbiter *m, Circle &c)
{
    // n
    Vec2 normal = c.position - this->position;

    float dist_sqr = normal.LenSqr( );
    float radius = this->getRadius() + c.getRadius();

    // not Collision
    if(dist_sqr >= radius * radius)
    {
        m->contactCounter = 0;
        return false;
    }

    float distance = std::sqrt( dist_sqr );

    m->contactCounter = 1;

    // Overlap
    if(distance == 0.0f)
    {
        m->contacts[0].penetration = this->radius;
        m->contacts[0].normal = Vec2( 1, 0 );
        m->contacts[0].position = this->position;
    }
    //
    else
    {
        m->contacts[0].penetration = radius - distance;
        // Faster than using Normalized since we already performed sqrt
        m->contacts[0].normal = normal / distance;
        m->contacts[0].position =  m->contacts[0].normal  * this->getRadius() + c.position;
    }

    return true;
}