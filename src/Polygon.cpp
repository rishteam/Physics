#include "Polygon.h"
#include "Circle.h"
#include "Box.h"
#include "Arbiter.h"

#define MaxPolyVertexCount 64

Polygon::Polygon(std::deque<Vec2> &pt, Vec2 pos, float mass_)
{
    type = Shape::Type::Polygon;

    int count = pt.size();

    // 找凸包
    // No hulls with less than 3 vertices (ensure actual polygon)
    assert( count > 2 && count <= MaxPolyVertexCount );
    count = std::min( (int)count, MaxPolyVertexCount );

    // Find the right most point on the hull
    int rightMost = 0;
    float highestXCoord = pt[0].x;
    for(int i = 1; i < count; ++i)
    {
        float x = pt[i].x;
        if(x > highestXCoord)
        {
            highestXCoord = x;
            rightMost = i;
        }

        // If matching x then take farthest negative y
        else if(x == highestXCoord)
            if(pt[i].y < pt[rightMost].y)
                rightMost = i;
    }

    int hull[MaxPolyVertexCount];
    int outCount = 0;
    int indexHull = rightMost;

    for (;;)
    {
        hull[outCount] = indexHull;

        // Search for next index that wraps around the hull
        // by computing cross products to find the most counter-clockwise
        // vertex in the set, given the previous hull index
        int nextHullIndex = 0;
        for(int i = 1; i < (int)count; ++i)
        {
            // Skip if same coordinate as we need three unique
            // points in the set to perform a cross product
            if(nextHullIndex == indexHull)
            {
                nextHullIndex = i;
                continue;
            }

            // Cross every set of three unique vertices
            // Record each counter clockwise third vertex and add
            // to the output hull
            // See : http://www.oocities.org/pcgpe/math2d.html
            Vec2 e1 = pt[nextHullIndex] - pt[hull[outCount]];
            Vec2 e2 = pt[i] - pt[hull[outCount]];
            float c = Cross( e1, e2 );
            if(c < 0.0f)
                nextHullIndex = i;

            // Cross product is zero then e vectors are on same line
            // therefor want to record vertex farthest along that line
            if(c == 0.0f && e2.LenSqr( ) > e1.LenSqr( ))
                nextHullIndex = i;
        }

        ++outCount;
        indexHull = nextHullIndex;

        // Conclude algorithm upon wrap-around
        if(nextHullIndex == rightMost)
        {
            m_vertexCount = outCount;
            break;
        }
    }

    // Copy vertices into shape's vertices
    for(int i = 0; i < m_vertexCount; ++i)
        m_vertices[i] = pt[hull[i]];

    // Compute face normals
    for(int i1 = 0; i1 < m_vertexCount; ++i1)
    {
        int i2 = i1 + 1 < m_vertexCount ? i1 + 1 : 0;
        Vec2 face = m_vertices[i2] - m_vertices[i1];

        // Ensure no zero-length edges, because that's bad
        assert( face.LenSqr( ) > 0.00001 * 0.00001 );

        // Calculate normal with 2D cross product between vector and scalar
        m_normals[i1] = Vec2( face.y, -face.x );
        m_normals[i1].Normalize( );
    }

    position = pos;
    this->SetMatrix(0.0f);
    angle = 0.0f;
    mass = mass_;


    // Calculate centroid and moment of interia
    Vec2 c( 0.0f, 0.0f ); // centroid
    float area = 0.0f;
    float I = 0.0f;
    const float k_inv3 = 1.0f / 3.0f;

    for(int i1 = 0; i1 < m_vertexCount; ++i1)
    {
        // Triangle vertices, third vertex implied as (0, 0)
        Vec2 p1( m_vertices[i1] );
        int i2 = i1 + 1 < m_vertexCount ? i1 + 1 : 0;
        Vec2 p2( m_vertices[i2] );

        float D = Cross( p1, p2 );
        float triangleArea = 0.5f * D;

        area += triangleArea;

        // Use area to weight the centroid average, not just vertex position
        c += triangleArea * k_inv3 * (p1 + p2);

        float intx2 = p1.x * p1.x + p2.x * p1.x + p2.x * p2.x;
        float inty2 = p1.y * p1.y + p2.y * p1.y + p2.y * p2.y;
        I += (0.25f * k_inv3 * D) * (intx2 + inty2);
    }

    c *= 1.0f / area;

    // Translate vertices to centroid (make the centroid (0, 0)
    // for the polygon in model space)
    // Not really necessary, but I like doing this anyway
    for(int i = 0; i < m_vertexCount; ++i)
        m_vertices[i] -= c;

    // TODO: can modify density
    float density = 1.0f;
    if (mass < FLT_MAX)
    {
        I = I * density;
        invI = 1.0f / I;
    }
    else
    {
        invMass = 0.0f;
        I = FLT_MAX;
        invI = 0.0f;
    }

}

int Polygon::getVertexCount()
{
    return m_vertexCount;
}

void Polygon::setDebugDraw()
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

bool Polygon::Collide(Arbiter &arb, Box &b)
{
    arb.contactCounter = 0;
    this->SetMatrix(angle);
    b.SetMatrix(b.angle);

    // Check for a separating axis with A's face planes
    int faceA;
    float penetrationA = FindAxisLeastPenetration(&faceA, this, &b);
    if(penetrationA >= 0.0f)
        return false;

    // Check for a separating axis with B's face planes
    int faceB;
    float penetrationB = FindAxisLeastPenetration(&faceB, &b, this);
    if(penetrationB >= 0.0f)
        return false;

    int referenceIndex;
    // Always point from a to b
    // 設定為 A 指向 B，若沒符合則flip
    bool flip = false;

    Shape *RefPoly; // Reference
    Shape *IncPoly; // Incident

    // Determine which shape contains reference face
    if(BiasGreaterThan(penetrationA, penetrationB))
    {
        RefPoly = this;
        IncPoly = &b;
        referenceIndex = faceA;
        flip = false;
    }

    else
    {
        RefPoly = &b;
        IncPoly = this;
        referenceIndex = faceB;
        flip = true;
    }

    // World space incident face
    Vec2 incidentFace[2];

    FindIncidentFace( incidentFace, RefPoly, IncPoly, referenceIndex );

    //        y
    //        ^  ->n       ^
    //      +---c ------posPlane--
    //  x < | i |\
  //      +---+ c-----negPlane--
    //             \       v
    //              r
    //
    //  r : reference face
    //  i : incident poly
    //  c : clipped point
    //  n : incident normal

    // Setup reference face vertices
    // 利用點紀錄reference face
    Vec2 v1 = RefPoly->m_vertices[referenceIndex];
    referenceIndex = referenceIndex + 1 == RefPoly->m_vertexCount ? 0 : referenceIndex + 1;
    Vec2 v2 = RefPoly->m_vertices[referenceIndex];

    // Transform vertices to world space
    v1 = RefPoly->u * v1 + RefPoly->position;
    v2 = RefPoly->u * v2 + RefPoly->position;

    // Calculate reference face side normal in world space
    Vec2 sidePlaneNormal = (v2 - v1);
    // to Normalize vector (轉成單位向量)
    sidePlaneNormal.Normalize( );

    // Orthogonalize
    // n 單位向量
    Vec2 refFaceNormal( sidePlaneNormal.y, -sidePlaneNormal.x );

    // ax + by = c
    // c is distance from origin
    float refC = Dot( refFaceNormal, v1 );
    float negSide = Dot( -sidePlaneNormal, v1 );
    float posSide = Dot( sidePlaneNormal, v2 );

    // Clip incident face to reference face side planes
    if(Clip( -sidePlaneNormal, negSide, incidentFace ) < 2)
        return false; // Due to floating point error, possible to not have required points

    if(Clip(  sidePlaneNormal, posSide, incidentFace ) < 2)
        return false; // Due to floating point error, possible to not have required points

    // Flip
    arb.contacts[0].normal = flip ? -refFaceNormal : refFaceNormal;

    // 透過clip截斷點，incidentFace
    // Keep points behind reference face
    int cp = 0; // clipped points behind reference face
    float separation = Dot( refFaceNormal, incidentFace[0] ) - refC;
    if(separation <= 0.0f)
    {
        arb.contacts[cp].position = incidentFace[0];
        arb.contacts[cp].penetration = -separation;
        ++cp;
    }
    else
    {
        arb.contacts[cp].penetration = 0;
    }

    separation = Dot( refFaceNormal, incidentFace[1] ) - refC;
    if(separation <= 0.0f)
    {
        arb.contacts[cp].position = incidentFace[1];
        arb.contacts[cp].penetration += -separation;
        ++cp;

        // Average penetration
//        arb.penetration /= (float)cp;
    }

    arb.contactCounter = cp;
    return true;
}

bool Polygon::Collide(Arbiter &arb, Polygon &p)
{
    arb.contactCounter = 0;
    this->SetMatrix(angle);
    p.SetMatrix(p.angle);

    // Check for a separating axis with A's face planes
    int faceA = 0;
    float penetrationA = FindAxisLeastPenetration(&faceA, this, &p);
    if(penetrationA >= 0.0f)
        return false;

    // Check for a separating axis with B's face planes
    int faceB;
    float penetrationB = FindAxisLeastPenetration( &faceB, &p, this);
    if(penetrationB >= 0.0f)
        return false;

    int referenceIndex;
    // Always point from a to b
    // 設定為 A 指向 B，若沒符合則flip
    bool flip = false;

    Polygon *RefPoly; // Reference
    Polygon *IncPoly; // Incident

    // Determine which shape contains reference face
    if(BiasGreaterThan(penetrationA, penetrationB))
    {
        RefPoly = this;
        IncPoly = &p;
        referenceIndex = faceA;
        flip = false;
    }
    else
    {
        RefPoly = &p;
        IncPoly = this;
        referenceIndex = faceB;
        flip = true;
    }

    // World space incident face
    Vec2 incidentFace[2];

    FindIncidentFace( incidentFace, RefPoly, IncPoly, referenceIndex );
    //        y
    //        ^  ->n       ^
    //      +---c ------posPlane--
    //  x < | i |\
  //      +---+ c-----negPlane--
    //             \       v
    //              r
    //
    //  r : reference face
    //  i : incident poly
    //  c : clipped point
    //  n : incident normal

    // Setup reference face vertices
    // 利用點紀錄reference face
    Vec2 v1 = RefPoly->m_vertices[referenceIndex];
    referenceIndex = referenceIndex + 1 == RefPoly->m_vertexCount ? 0 : referenceIndex + 1;
    Vec2 v2 = RefPoly->m_vertices[referenceIndex];

    // Transform vertices to world space
    v1 = RefPoly->u * v1 + RefPoly->position;
    v2 = RefPoly->u * v2 + RefPoly->position;

    // Calculate reference face side normal in world space
    Vec2 sidePlaneNormal = (v2 - v1);
    // to Normalize vector (轉成單位向量)
    sidePlaneNormal.Normalize( );

    // Orthogonalize
    // n 單位向量
    Vec2 refFaceNormal( sidePlaneNormal.y, -sidePlaneNormal.x );

    // ax + by = c
    // c is distance from origin
    float refC = Dot( refFaceNormal, v1 );
    float negSide = Dot( -sidePlaneNormal, v1 );
    float posSide = Dot( sidePlaneNormal, v2 );

    // Clip incident face to reference face side planes
    if(Clip( -sidePlaneNormal, negSide, incidentFace ) < 2)
        return false; // Due to floating point error, possible to not have required points

    if(Clip(  sidePlaneNormal, posSide, incidentFace ) < 2)
        return false; // Due to floating point error, possible to not have required points

    // Flip
    arb.contacts[0].normal = flip ? -refFaceNormal : refFaceNormal;
    arb.contacts[1].normal = flip ? -refFaceNormal : refFaceNormal;

    // 透過clip截斷點，incidentFace
    // Keep points behind reference face
    int cp = 0; // clipped points behind reference face
    float separation = Dot( refFaceNormal, incidentFace[0] ) - refC;
    if(separation <= 0.0f)
    {
        arb.contacts[cp].position = incidentFace[0];
        arb.contacts[cp].penetration = -separation;
        ++cp;
    }
    else
    {
        arb.contacts[cp].penetration = 0;
    }
    separation = Dot( refFaceNormal, incidentFace[1] ) - refC;
    if(separation <= 0.0f)
    {
        arb.contacts[cp].position = incidentFace[1];
        arb.contacts[cp].penetration += -separation;
        ++cp;

        // Average penetration
//        arb.penetration /= (float)cp;
    }

    arb.contactCounter = cp;
    return true;
}

bool Polygon::Collide(Arbiter &arb, Circle &c)
{
    arb.contactCounter = 0;
    this->SetMatrix(angle);
    c.SetMatrix(c.angle);

    // Transform circle center to Polygon model space
    // 找最小穿透軸
    Vec2 center = c.position;
    center = this->u.Transpose( ) * (center - this->position);

    // Find edge with minimum penetration
    // Exact concept as using support points in Polygon vs Polygon
    float separation = -FLT_MAX;
    int faceNormal = 0;
    for(int i = 0; i < this->m_vertexCount; ++i)
    {
        float s = Dot( this->m_normals[i], center - this->m_vertices[i] );
        if(s > c.getRadius())
            return false;
        if(s > separation)
        {
            separation = s;
            faceNormal = i;
        }
    }

    // Grab face's vertices
    // 用點決定Reference Face
    Vec2 v1 = this->m_vertices[faceNormal];
    int i2 = faceNormal + 1 < this->m_vertexCount ? faceNormal + 1 : 0;
    Vec2 v2 = this->m_vertices[i2];

    // Check to see if center is within polygon
    if(separation < EPSILON)
    {
        arb.contactCounter = 1;
        arb.contacts[0].normal = -(this->u * this->m_normals[faceNormal]);
        arb.contacts[0].position = arb.contacts[0].normal * c.getRadius() + c.position;
        arb.contacts[0].penetration = c.getRadius();
        return true;
    }

    // Determine which voronoi region of the edge center of circle lies within
    float dot1 = Dot( center - v1, v2 - v1 );
    float dot2 = Dot( center - v2, v1 - v2 );
    arb.contacts[0].penetration  = c.getRadius() - separation;

    // Closest to v1
    // 靠近v1
    if(dot1 <= 0.0f)
    {
        // 檢查是否分離
        if(DistSqr( center, v1 ) > c.getRadius() * c.getRadius())
            return false;

        arb.contactCounter = 1;
        Vec2 n = v1 - center;
        n = this->u * n;
        n.Normalize( );
        arb.contacts[0].normal = n;
        v1 = this->u * v1 + this->position;
        arb.contacts[0].position = v1;
    }

    // Closest to v2
    // 靠近v2
    else if(dot2 <= 0.0f)
    {
        if(DistSqr( center, v2 ) > c.getRadius() * c.getRadius())
            return false;

        arb.contactCounter = 1;
        Vec2 n = v2 - center;
        v2 = this->u * v2 + this->position;
        arb.contacts[0].position = v2;
        n = this->u * n;
        n.Normalize( );
        arb.contacts[0].normal = n;
    }

    // Closest to face
    // 靠近面
    else
    {
        Vec2 n = this->m_normals[faceNormal];
        if(Dot( center - v1, n ) > c.getRadius())
            return false;
        n = this->u * n;
        arb.contacts[0].normal  = -n;
        arb.contacts[0].position = arb.contacts[0].normal * c.getRadius() + c.position;
        arb.contactCounter = 1;
    }
    return true;
}