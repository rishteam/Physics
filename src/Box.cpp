#include "Box.h"
#include "Circle.h"
#include "Polygon.h"
#include "Arbiter.h"

Box::Box(float x, float y, float w, float h, float mass_)
{
    type = Shape::Type::Box;
    m_vertexCount = 4;
    position.x = x;
    position.y = y;
    wh.x = w;
    wh.y = h;

    m_vertices[0] = Vec2(-w / 2.0, -h / 2.0);
    m_vertices[1] = Vec2(w / 2.0, -h / 2.0);
    m_vertices[2] = Vec2(w / 2.0, h / 2.0);
    m_vertices[3] = Vec2(-w / 2.0, h / 2.0);

    // Compute face normals
    for(int i1 = 0; i1 < m_vertexCount; ++i1)
    {
        int i2 = i1 + 1 < m_vertexCount ? i1 + 1 : 0;
        Vec2 face = m_vertices[i2] - m_vertices[i1];

        // Ensure no zero-length edges, because that's bad
        assert( face.LenSqr( ) > EPSILON * EPSILON );

        // Calculate normal with 2D cross product between vector and scalar
        m_normals[i1] = Vec2( face.y, -face.x );
        m_normals[i1].Normalize( );
    }
    mass = mass_;
    if (mass < FLT_MAX)
    {
        invMass = 1.0f / mass;
        I = mass * (wh.x * wh.x + wh.y * wh.y) / 12.0f;
        invI = 1.0f / I;
    }
    else
    {
        invMass = 0.0f;
        I = FLT_MAX;
        invI = 0.0f;
    }

    SetMatrix(0.0f);
    angle = 0.0f;
}

void Box::setDebugDraw()
{
    cordTranslate.clear();
    polygon.setPointCount(m_vertexCount);
    Vec2 center = World::ConvertWorldToScreen(position);
    for(int i = 0; i < m_vertexCount; i++)
    {
        cordTranslate.push_back(World::ConvertWorldToScreen(Vec2(position.x + m_vertices[i].x, position.y + m_vertices[i].y)));
    }
    polygon.setPosition(center.x, center.y);
    polygon.setRotation(-radiansToDegrees(angle));

    //sfml needs to give the offset for setting point
    int i = 0;
    for (auto corner : cordTranslate)
    {
        polygon.setPoint(i, sf::Vector2f(corner.x - center.x, corner.y - center.y));
        i++;
    }
    if (this->selected)
    {
        polygon.setFillColor(sf::Color::Red);
    }
    else
    {
        polygon.setFillColor(sf::Color::White);
    }
}

//bool Box::Collide(Arbiter &arb, Box &b)
//{
//    arb.contactCounter = 0;
//    SetMatrix(angle);
//    b.SetMatrix(b.angle);
//
//    // Check for a separating axis with A's face planes
//    int faceA;
//    float penetrationA = FindAxisLeastPenetration(&faceA, this, &b);
//    if(penetrationA >= 0.0f)
//        return false;
//
//    // Check for a separating axis with B's face planes
//    int faceB;
//    float penetrationB = FindAxisLeastPenetration(&faceB, &b, this);
//    if(penetrationB >= 0.0f)
//        return false;
//
//    int referenceIndex;
//    // Always point from a to b
//    // 設定為 A 指向 B，若沒符合則flip
//    bool flip = false;
//
//    Shape *RefPoly; // Reference
//    Shape *IncPoly; // Incident
//
//    // Determine which shape contains reference face
//    if(BiasGreaterThan(penetrationA, penetrationB))
//    {
//        RefPoly = this;
//        IncPoly = &b;
//        referenceIndex = faceA;
//        flip = false;
//    }
//
//    else
//    {
//        RefPoly = &b;
//        IncPoly = this;
//        referenceIndex = faceB;
//        flip = true;
//    }
//
//    // World space incident face
//    Vec2 incidentFace[2];
//
//    FindIncidentFace( incidentFace, RefPoly, IncPoly, referenceIndex );
//
//    //        y
//    //        ^  ->n       ^
//    //      +---c ------posPlane--
//    //  x < | i |\
//  //      +---+ c-----negPlane--
//    //             \       v
//    //              r
//    //
//    //  r : reference face
//    //  i : incident poly
//    //  c : clipped point
//    //  n : incident normal
//
//    // Setup reference face vertices
//    // 利用點紀錄reference face
//    Vec2 v1 = RefPoly->m_vertices[referenceIndex];
//    referenceIndex = referenceIndex + 1 == RefPoly->m_vertexCount ? 0 : referenceIndex + 1;
//    Vec2 v2 = RefPoly->m_vertices[referenceIndex];
//
//    // Transform vertices to world space
//    v1 = RefPoly->u * v1 + RefPoly->position;
//    v2 = RefPoly->u * v2 + RefPoly->position;
//
//    // Calculate reference face side normal in world space
//    Vec2 sidePlaneNormal = (v2 - v1);
//    // to Normalize vector (轉成單位向量)
//    sidePlaneNormal.Normalize( );
//
//    // Orthogonalize
//    // n 單位向量
//    Vec2 refFaceNormal( sidePlaneNormal.y, -sidePlaneNormal.x );
//
//    // ax + by = c
//    // c is distance from origin
//    float refC = Dot( refFaceNormal, v1 );
//    float negSide = Dot( -sidePlaneNormal, v1 );
//    float posSide = Dot( sidePlaneNormal, v2 );
//
//    // Clip incident face to reference face side planes
//    if(Clip( -sidePlaneNormal, negSide, incidentFace ) < 2)
//        return false; // Due to floating point error, possible to not have required points
//
//    if(Clip(  sidePlaneNormal, posSide, incidentFace ) < 2)
//        return false; // Due to floating point error, possible to not have required points
//
//    // Flip
//    arb.normal = flip ? -refFaceNormal : refFaceNormal;
//    arb.normal = flip ? -refFaceNormal : refFaceNormal;
//
//    // 透過clip截斷點，incidentFace
//    // Keep points behind reference face
//    int cp = 0; // clipped points behind reference face
//    float separation = Dot( refFaceNormal, incidentFace[0] ) - refC;
//    if(separation <= 0.0f)
//    {
//        arb.contacts[cp].position = incidentFace[0];
//        arb.penetration = -separation;
//        ++cp;
//    }
//    else
//    {
//        arb.penetration = 0;
//    }
//
//    separation = Dot( refFaceNormal, incidentFace[1] ) - refC;
//    if(separation <= 0.0f)
//    {
//        arb.contacts[cp].position = incidentFace[1];
//        arb.penetration = -separation;
//        ++cp;
//
//        // Average penetration
//        // arb.contacts[cp].penetration /= (float)cp;
//    }
//
//    arb.contactCounter = cp;
//    return true;
//}
//bool Box::Collide(Arbiter &arb, Polygon &p)
//{
//    arb.contactCounter = 0;
//    SetMatrix(angle);
//    p.SetMatrix(p.angle);
//
//    // Check for a separating axis with A's face planes
//    int faceA;
//    float penetrationA = FindAxisLeastPenetration(&faceA, this, &p);
//    if(penetrationA >= 0.0f)
//        return false;
//
//    // Check for a separating axis with B's face planes
//    int faceB;
//    float penetrationB = FindAxisLeastPenetration(&faceB, &p, this);
//    if(penetrationB >= 0.0f)
//        return false;
//
//    int referenceIndex;
//    // Always point from a to b
//    // 設定為 A 指向 B，若沒符合則flip
//    bool flip = false;
//
//    Shape *RefPoly; // Reference
//    Shape *IncPoly; // Incident
//
//    // Determine which shape contains reference face
//    if(BiasGreaterThan(penetrationA, penetrationB))
//    {
//        RefPoly = this;
//        IncPoly = &p;
//        referenceIndex = faceA;
//        flip = false;
//    }
//
//    else
//    {
//        RefPoly = &p;
//        IncPoly = this;
//        referenceIndex = faceB;
//        flip = true;
//    }
//
//    // World space incident face
//    Vec2 incidentFace[2];
//
//    FindIncidentFace( incidentFace, RefPoly, IncPoly, referenceIndex );
//
//    //        y
//    //        ^  ->n       ^
//    //      +---c ------posPlane--
//    //  x < | i |\
//  //      +---+ c-----negPlane--
//    //             \       v
//    //              r
//    //
//    //  r : reference face
//    //  i : incident poly
//    //  c : clipped point
//    //  n : incident normal
//
//    // Setup reference face vertices
//    // 利用點紀錄reference face
//    Vec2 v1 = RefPoly->m_vertices[referenceIndex];
//    referenceIndex = referenceIndex + 1 == RefPoly->m_vertexCount ? 0 : referenceIndex + 1;
//    Vec2 v2 = RefPoly->m_vertices[referenceIndex];
//
//    // Transform vertices to world space
//    v1 = RefPoly->u * v1 + RefPoly->position;
//    v2 = RefPoly->u * v2 + RefPoly->position;
//
//    // Calculate reference face side normal in world space
//    Vec2 sidePlaneNormal = (v2 - v1);
//    // to Normalize vector (轉成單位向量)
//    sidePlaneNormal.Normalize( );
//
//    // Orthogonalize
//    // n 單位向量
//    Vec2 refFaceNormal( sidePlaneNormal.y, -sidePlaneNormal.x );
//
//    // ax + by = c
//    // c is distance from origin
//    float refC = Dot( refFaceNormal, v1 );
//    float negSide = Dot( -sidePlaneNormal, v1 );
//    float posSide = Dot( sidePlaneNormal, v2 );
//
//    // Clip incident face to reference face side planes
//    if(Clip( -sidePlaneNormal, negSide, incidentFace ) < 2)
//        return false; // Due to floating point error, possible to not have required points
//
//    if(Clip(  sidePlaneNormal, posSide, incidentFace ) < 2)
//        return false; // Due to floating point error, possible to not have required points
//
//    // Flip
//    arb.normal = flip ? -refFaceNormal : refFaceNormal;
//    arb.normal = flip ? -refFaceNormal : refFaceNormal;
//
//    // 透過clip截斷點，incidentFace
//    // Keep points behind reference face
//    int cp = 0; // clipped points behind reference face
//    float separation = Dot( refFaceNormal, incidentFace[0] ) - refC;
//    if(separation <= 0.0f)
//    {
//        arb.contacts[cp].position = incidentFace[0];
//        arb.penetration = -separation;
//        ++cp;
//    }
//    else
//    {
//        arb.penetration = 0;
//    }
//
//    separation = Dot( refFaceNormal, incidentFace[1] ) - refC;
//    if(separation <= 0.0f)
//    {
//        arb.contacts[cp].position = incidentFace[1];
//        arb.penetration = -separation;
//        ++cp;
//
//        // Average penetration
//        arb.penetration /= (float)cp;
//    }
//
//    arb.contactCounter = cp;
//    return true;
//}
//bool Box::Collide(Arbiter &arb, Circle &c)
//{
//    arb.contactCounter = 0;
//    SetMatrix(angle);
//    c.SetMatrix(c.angle);
//
//    // Transform circle center to Polygon model space
//    // 找最小穿透軸
//    Vec2 center = c.position;
//    center = this->u.Transpose( ) * (center - this->position);
//
//    // Find edge with minimum penetration
//    // Exact concept as using support points in Polygon vs Polygon
//    float separation = -FLT_MAX;
//    int faceNormal = 0;
//    for(int i = 0; i < this->m_vertexCount; ++i)
//    {
//        float s = Dot( this->m_normals[i], center - this->m_vertices[i] );
//        if(s > c.getRadius())
//            return false;
//        if(s > separation)
//        {
//            separation = s;
//            faceNormal = i;
//        }
//    }
//
//    // Grab face's vertices
//    // 用點決定Reference Face
//    Vec2 v1 = this->m_vertices[faceNormal];
//    int i2 = faceNormal + 1 < this->m_vertexCount ? faceNormal + 1 : 0;
//    Vec2 v2 = this->m_vertices[i2];
//
//    // Check to see if center is within polygon
//    if(separation < EPSILON)
//    {
//        arb.contactCounter = 1;
//        arb.normal = -(this->u * this->m_normals[faceNormal]);
//        arb.contacts[0].position = arb.normal * c.getRadius() + c.position;
//        arb.penetration = c.getRadius();
//        return true;
//    }
//
//    // Determine which voronoi region of the edge center of circle lies within
//    float dot1 = Dot( center - v1, v2 - v1 );
//    float dot2 = Dot( center - v2, v1 - v2 );
//    arb.penetration = c.getRadius() - separation;
//
//    // Closest to v1
//    // 靠近v1
//    if(dot1 <= 0.0f)
//    {
//        // 檢查是否分離
//        if(DistSqr( center, v1 ) > c.getRadius() * c.getRadius())
//            return false;
//
//        arb.contactCounter = 1;
//        Vec2 n = v1 - center;
//        n = this->u * n;
//        n.Normalize( );
//        arb.normal = n;
//        v1 = this->u * v1 + this->position;
//        arb.contacts[0].position = v1;
//    }
//    // Closest to v2
//    // 靠近v2
//    else if(dot2 <= 0.0f)
//    {
//        if(DistSqr( center, v2 ) > c.getRadius() * c.getRadius())
//            return false;
//
//        arb.contactCounter = 1;
//        Vec2 n = v2 - center;
//        v2 = this->u * v2 + this->position;
//        arb.contacts[0].position = v2;
//        n = this->u * n;
//        n.Normalize( );
//        arb.normal = n;
//    }
//    // Closest to face
//    // 靠近面
//    else
//    {
//        Vec2 n = this->m_normals[faceNormal];
//        if(Dot( center - v1, n ) > c.getRadius())
//            return false;
//        n = this->u * n;
//        arb.normal = -n;
//        arb.contacts[0].position = arb.normal * c.getRadius() + c.position;
//        arb.contactCounter = 1;
//    }
//    return true;
//}