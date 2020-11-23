#include "Shape.h"
#include "Arbiter.h"
#include "Box.h"
#include "Polygon.h"
#include "Circle.h"

Vec2 Shape::GetSupport( const Vec2 dir )
{
    float bestProjection = -FLT_MAX;
    Vec2 bestVertex;

    for(int i = 0; i < m_vertexCount; ++i)
    {
        Vec2 v = m_vertices[i];
        float projection = Dot( v, dir );

        if(projection > bestProjection)
        {
            bestVertex = v;
            bestProjection = projection;
        }
    }
    return bestVertex;
}

void Shape::SetMatrix(float radians)
{
    angle = radians;
    u.SetRotateMatrix(radians);
}

void Shape::setPosition(Vec2 mouse)
{
    Vec2 update = World::ChangeToPhysicsWorld(mouse);
    position = update;
}

bool Shape::Collide(Arbiter &arb, Shape *s)
{
//    int b1Type = 0;
//    Box *box1 = nullptr;
//    Circle *circle1 = nullptr;
//    Polygon *poly1 = nullptr;
//
//    int b2Type = 0;
//    Box *box2 = nullptr;
//    Circle *circle2 = nullptr;
//    Polygon *poly2 = nullptr;
//
//    switch(this->type)
//    {
//        case Shape::Type::Box:
//        {
//            box1 = dynamic_cast<Box *>(this);
//            b1Type = 0;
//            break;
//        }
//        case Shape::Type::Circle:
//        {
//            circle1 = dynamic_cast<Circle *>(this);
//            b1Type = 1;
//            break;
//        }
//        case Shape::Type::Polygon:
//        {
//            poly1 = dynamic_cast<Polygon *>(this);
//            b1Type = 2;
//            break;
//        }
//    }
//
//    switch(s->type)
//    {
//        case Shape::Type::Box:
//        {
//            box2 = dynamic_cast<Box *>(s);
//            if(b1Type == 0)
//            {
//                //    arb.contactCounter = 0;
//                box1->SetMatrix(angle);
//                box2->SetMatrix(box2->angle);
//
//                // Check for a separating axis with A's face planes
//                int faceA;
//                float penetrationA = FindAxisLeastPenetration(&faceA, box1, box2);
//                if(penetrationA >= 0.0f)
//                    return false;
//
//                // Check for a separating axis with B's face planes
//                int faceB;
//                float penetrationB = FindAxisLeastPenetration(&faceB, box2, box1);
//                if(penetrationB >= 0.0f)
//                    return false;
//
//                int referenceIndex;
//                // Always point from a to b
//                // 設定為 A 指向 B，若沒符合則flip
//                bool flip = false;
//
//                Shape *RefPoly; // Reference
//                Shape *IncPoly; // Incident
//
//                // Determine which shape contains reference face
//                if(BiasGreaterThan(penetrationA, penetrationB))
//                {
//                    RefPoly = box1;
//                    IncPoly = box2;
//                    referenceIndex = faceA;
//                    flip = false;
//                }
//
//                else
//                {
//                    RefPoly = box1;
//                    IncPoly = box2;
//                    referenceIndex = faceB;
//                    flip = true;
//                }
//
//                // World space incident face
//                Vec2 incidentFace[2];
//
//                FindIncidentFace( incidentFace, RefPoly, IncPoly, referenceIndex );
//
//                //        y
//                //        ^  ->n       ^
//                //      +---c ------posPlane--
//                //  x < | i |\
//              //      +---+ c-----negPlane--
//                //             \       v
//                //              r
//                //
//                //  r : reference face
//                //  i : incident poly
//                //  c : clipped point
//                //  n : incident normal
//
//                // Setup reference face vertices
//                // 利用點紀錄reference face
//                Vec2 v1 = RefPoly->m_vertices[referenceIndex];
//                referenceIndex = referenceIndex + 1 == RefPoly->m_vertexCount ? 0 : referenceIndex + 1;
//                Vec2 v2 = RefPoly->m_vertices[referenceIndex];
//
//                // Transform vertices to world space
//                v1 = RefPoly->u * v1 + RefPoly->position;
//                v2 = RefPoly->u * v2 + RefPoly->position;
//
//                // Calculate reference face side normal in world space
//                Vec2 sidePlaneNormal = (v2 - v1);
//                // to Normalize vector (轉成單位向量)
//                sidePlaneNormal.Normalize( );
//
//                // Orthogonalize
//                // n 單位向量
//                Vec2 refFaceNormal( sidePlaneNormal.y, -sidePlaneNormal.x );
//
//                // ax + by = c
//                // c is distance from origin
//                float refC = Dot( refFaceNormal, v1 );
//                float negSide = Dot( -sidePlaneNormal, v1 );
//                float posSide = Dot( sidePlaneNormal, v2 );
//
//                // Clip incident face to reference face side planes
//                if(Clip( -sidePlaneNormal, negSide, incidentFace ) < 2)
//                    return false; // Due to floating point error, possible to not have required points
//
//                if(Clip(  sidePlaneNormal, posSide, incidentFace ) < 2)
//                    return false; // Due to floating point error, possible to not have required points
//
//                // Flip
//                arb.normal = flip ? -refFaceNormal : refFaceNormal;
//
//                // 透過clip截斷點，incidentFace
//                // Keep points behind reference face
//                int cp = 0; // clipped points behind reference face
//                float separation = Dot( refFaceNormal, incidentFace[0] ) - refC;
//                if(separation <= 0.0f)
//                {
//                    arb.contacts[cp].position = incidentFace[0];
//                    arb.penetration = -separation;
//                    ++cp;
//                }
//                else
//                {
//                    arb.penetration = 0;
//                }
//
//                separation = Dot( refFaceNormal, incidentFace[1] ) - refC;
//                if(separation <= 0.0f)
//                {
//                    arb.contacts[cp].position = incidentFace[1];
//                    arb.penetration += -separation;
//                    ++cp;
//
//                    // Average penetration
//                    arb.penetration /= (float)cp;
//                }
//
//                arb.contactCounter = cp;
//                return true;
//            }
//            else if(b1Type == 1)
//            {
//                arb.contactCounter = 0;
//
//                // Transform circle center to Polygon model space
//                // 找最小穿透軸
//                Vec2 center = circle1->position;
//                center = box2->u.Transpose( ) * (center - box2->position);
//
//                // Find edge with minimum penetration
//                // Exact concept as using support points in Polygon vs Polygon
//                float separation = -FLT_MAX;
//                int faceNormal = 0;
//                for(int i = 0; i < box1->m_vertexCount; ++i)
//                {
//                    float s = Dot( box1->m_normals[i], center - box1->m_vertices[i] );
//                    if(s > circle1->getRadius())
//                        return false;
//                    if(s > separation)
//                    {
//                        separation = s;
//                        faceNormal = i;
//                    }
//                }
//
//                // Grab face's vertices
//                // 用點決定Reference Face
//                Vec2 v1 = box2->m_vertices[faceNormal];
//                int i2 = faceNormal + 1 < box2->m_vertexCount ? faceNormal + 1 : 0;
//                Vec2 v2 = box2->m_vertices[i2];
//
//                // Check to see if center is within polygon
//                if(separation < EPSILON)
//                {
//                    arb.contactCounter = 1;
//                    arb.normal = -(box2->u * box2->m_normals[faceNormal]);
//                    arb.contacts[0].position = arb.normal * circle1->getRadius() + circle1->position;
//                    arb.penetration = circle1->getRadius();
//                    return true;
//                }
//
//                // Determine which voronoi region of the edge center of circle lies within
//                float dot1 = Dot( center - v1, v2 - v1 );
//                float dot2 = Dot( center - v2, v1 - v2 );
//                arb.penetration = circle1->getRadius() - separation;
//
//                // Closest to v1
//                // 靠近v1
//                if(dot1 <= 0.0f)
//                {
//                    // 檢查是否分離
//                    if(DistSqr( center, v1 ) > circle1->getRadius() * circle1->getRadius())
//                        return false;
//
//                    arb.contactCounter = 1;
//                    Vec2 n = v1 - center;
//                    n = box2->u * n;
//                    n.Normalize( );
//                    arb.normal = n;
//                    v1 = box2->u * v1 + box2->position;
//                    arb.contacts[0].position = v1;
//                }
//
//                    // Closest to v2
//                    // 靠近v2
//                else if(dot2 <= 0.0f)
//                {
//                    if(DistSqr( center, v2 ) > circle1->getRadius() * circle1->getRadius())
//                        return false;
//
//                    arb.contactCounter = 1;
//                    Vec2 n = v2 - center;
//                    v2 = box2->u * v2 + box2->position;
//                    arb.contacts[0].position = v2;
//                    n = box2->u * n;
//                    n.Normalize( );
//                    arb.normal = n;
//                }
//
//                    // Closest to face
//                    // 靠近面
//                else
//                {
//                    Vec2 n = box2->m_normals[faceNormal];
//                    if(Dot( center - v1, n ) > circle1->getRadius())
//                        return false;
//                    n = box2->u * n;
//                    arb.normal = -n;
//                    arb.contacts[0].position = arb.normal * circle1->getRadius() + circle1->position;
//                    arb.contactCounter = 1;
//                }
//                return true;
//            }
//            else if(b1Type == 2)
//            {
//                arb.contactCounter = 0;
//                poly1->SetMatrix(poly1->angle);
//                box2->SetMatrix(box2->angle);
//
//                // Check for a separating axis with A's face planes
//                int faceA;
//                float penetrationA = FindAxisLeastPenetration(&faceA, poly1, box2);
//                if (penetrationA >= 0.0f)
//                    return false;
//
//                // Check for a separating axis with B's face planes
//                int faceB;
//                float penetrationB = FindAxisLeastPenetration(&faceB, box2, poly1);
//                if (penetrationB >= 0.0f)
//                    return false;
//
//                int referenceIndex;
//                // Always point from a to b
//                // 設定為 A 指向 B，若沒符合則flip
//                bool flip = false;
//
//                Shape *RefPoly; // Reference
//                Shape *IncPoly; // Incident
//
//                // Determine which shape contains reference face
//                if (BiasGreaterThan(penetrationA, penetrationB)) {
//                    RefPoly = poly1;
//                    IncPoly = box2;
//                    referenceIndex = faceA;
//                    flip = false;
//                } else {
//                    RefPoly = box2;
//                    IncPoly = poly1;
//                    referenceIndex = faceB;
//                    flip = true;
//                }
//
//                // World space incident face
//                Vec2 incidentFace[2];
//
//                FindIncidentFace(incidentFace, RefPoly, IncPoly, referenceIndex);
//
//                //        y
//                //        ^  ->n       ^
//                //      +---c ------posPlane--
//                //  x < | i |\
//              //      +---+ c-----negPlane--
//                //             \       v
//                //              r
//                //
//                //  r : reference face
//                //  i : incident poly
//                //  c : clipped point
//                //  n : incident normal
//
//                // Setup reference face vertices
//                // 利用點紀錄reference face
//                Vec2 v1 = RefPoly->m_vertices[referenceIndex];
//                referenceIndex = referenceIndex + 1 == RefPoly->m_vertexCount ? 0 : referenceIndex + 1;
//                Vec2 v2 = RefPoly->m_vertices[referenceIndex];
//
//                // Transform vertices to world space
//                v1 = RefPoly->u * v1 + RefPoly->position;
//                v2 = RefPoly->u * v2 + RefPoly->position;
//
//                // Calculate reference face side normal in world space
//                Vec2 sidePlaneNormal = (v2 - v1);
//                // to Normalize vector (轉成單位向量)
//                sidePlaneNormal.Normalize();
//
//                // Orthogonalize
//                // n 單位向量
//                Vec2 refFaceNormal(sidePlaneNormal.y, -sidePlaneNormal.x);
//
//                // ax + by = c
//                // c is distance from origin
//                float refC = Dot(refFaceNormal, v1);
//                float negSide = Dot(-sidePlaneNormal, v1);
//                float posSide = Dot(sidePlaneNormal, v2);
//
//                // Clip incident face to reference face side planes
//                if (Clip(-sidePlaneNormal, negSide, incidentFace) < 2)
//                    return false; // Due to floating point error, possible to not have required points
//
//                if (Clip(sidePlaneNormal, posSide, incidentFace) < 2)
//                    return false; // Due to floating point error, possible to not have required points
//
//                // Flip
//                arb.normal = flip ? -refFaceNormal : refFaceNormal;
//                arb.normal = flip ? -refFaceNormal : refFaceNormal;
//
//                // 透過clip截斷點，incidentFace
//                // Keep points behind reference face
//                int cp = 0; // clipped points behind reference face
//                float separation = Dot(refFaceNormal, incidentFace[0]) - refC;
//                if (separation <= 0.0f) {
//                    arb.contacts[cp].position = incidentFace[0];
//                    arb.penetration = -separation;
//                    ++cp;
//                } else {
//                    arb.penetration = 0;
//                }
//
//                separation = Dot(refFaceNormal, incidentFace[1]) - refC;
//                if (separation <= 0.0f) {
//                    arb.contacts[cp].position = incidentFace[1];
//                    arb.penetration = -separation;
//                    ++cp;
//
//                    // Average penetration
//                    arb.penetration /= (float) cp;
//                }
//
//                arb.contactCounter = cp;
//                return true;
//            }
//            break;
//        }
//        case Shape::Type::Circle:
//        {
//            circle2 = dynamic_cast<Circle *>(s);
//            if(b1Type == 0)
//            {
//
//            }
//            else if(b1Type == 1)
//            {
//
//            }
//            else if(b1Type == 2)
//            {
//
//            }
//            break;
//        }
//        case Shape::Type::Polygon:
//        {
//            poly2 = dynamic_cast<Polygon *>(s);
//            if(b1Type == 0)
//            {
//                arb.contactCounter = 0;
//                box1->SetMatrix(box1->angle);
//                poly2->SetMatrix(poly2->angle);
//
//                // Check for a separating axis with A's face planes
//                int faceA;
//                float penetrationA = FindAxisLeastPenetration(&faceA, box1, poly2);
//                if (penetrationA >= 0.0f)
//                    return false;
//
//                // Check for a separating axis with B's face planes
//                int faceB;
//                float penetrationB = FindAxisLeastPenetration(&faceB, poly2, box1);
//                if (penetrationB >= 0.0f)
//                    return false;
//
//                int referenceIndex;
//                // Always point from a to b
//                // 設定為 A 指向 B，若沒符合則flip
//                bool flip = false;
//
//                Shape *RefPoly; // Reference
//                Shape *IncPoly; // Incident
//
//                // Determine which shape contains reference face
//                if (BiasGreaterThan(penetrationA, penetrationB)) {
//                    RefPoly = box1;
//                    IncPoly = poly2;
//                    referenceIndex = faceA;
//                    flip = false;
//                } else {
//                    RefPoly = poly2;
//                    IncPoly = box1;
//                    referenceIndex = faceB;
//                    flip = true;
//                }
//
//                // World space incident face
//                Vec2 incidentFace[2];
//
//                FindIncidentFace(incidentFace, RefPoly, IncPoly, referenceIndex);
//
//                //        y
//                //        ^  ->n       ^
//                //      +---c ------posPlane--
//                //  x < | i |\
//              //      +---+ c-----negPlane--
//                //             \       v
//                //              r
//                //
//                //  r : reference face
//                //  i : incident poly
//                //  c : clipped point
//                //  n : incident normal
//
//                // Setup reference face vertices
//                // 利用點紀錄reference face
//                Vec2 v1 = RefPoly->m_vertices[referenceIndex];
//                referenceIndex = referenceIndex + 1 == RefPoly->m_vertexCount ? 0 : referenceIndex + 1;
//                Vec2 v2 = RefPoly->m_vertices[referenceIndex];
//
//                // Transform vertices to world space
//                v1 = RefPoly->u * v1 + RefPoly->position;
//                v2 = RefPoly->u * v2 + RefPoly->position;
//
//                // Calculate reference face side normal in world space
//                Vec2 sidePlaneNormal = (v2 - v1);
//                // to Normalize vector (轉成單位向量)
//                sidePlaneNormal.Normalize();
//
//                // Orthogonalize
//                // n 單位向量
//                Vec2 refFaceNormal(sidePlaneNormal.y, -sidePlaneNormal.x);
//
//                // ax + by = c
//                // c is distance from origin
//                float refC = Dot(refFaceNormal, v1);
//                float negSide = Dot(-sidePlaneNormal, v1);
//                float posSide = Dot(sidePlaneNormal, v2);
//
//                // Clip incident face to reference face side planes
//                if (Clip(-sidePlaneNormal, negSide, incidentFace) < 2)
//                    return false; // Due to floating point error, possible to not have required points
//
//                if (Clip(sidePlaneNormal, posSide, incidentFace) < 2)
//                    return false; // Due to floating point error, possible to not have required points
//
//                // Flip
//                arb.normal = flip ? -refFaceNormal : refFaceNormal;
//                arb.normal = flip ? -refFaceNormal : refFaceNormal;
//
//                // 透過clip截斷點，incidentFace
//                // Keep points behind reference face
//                int cp = 0; // clipped points behind reference face
//                float separation = Dot(refFaceNormal, incidentFace[0]) - refC;
//                if (separation <= 0.0f) {
//                    arb.contacts[cp].position = incidentFace[0];
//                    arb.penetration = -separation;
//                    ++cp;
//                } else {
//                    arb.penetration = 0;
//                }
//
//                separation = Dot(refFaceNormal, incidentFace[1]) - refC;
//                if (separation <= 0.0f) {
//                    arb.contacts[cp].position = incidentFace[1];
//                    arb.penetration = -separation;
//                    ++cp;
//
//                    // Average penetration
//                    arb.penetration /= (float) cp;
//                }
//
//                arb.contactCounter = cp;
//                return true;
//            }
//            else if(b1Type == 1)
//            {
//            }
//            else if(b1Type == 2)
//            {
//            }
//            break;
//        }
//    }
}