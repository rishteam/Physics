#include "Shape.h"
#include "Manifold.h"

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

float Shape::FindAxisLeastPenetration(int *faceIdx, Shape *A, Shape *B)
{
    float bestDistance = -FLT_MAX;
    int bestIndex;

    for(int i = 0; i < A->m_vertexCount; ++i)
    {
        // Retrieve a face normal from A
        // n 為reference face
        Vec2 n = A->m_normals[i];
        // nw = n World Space
        Vec2 nw = A->u * n;

        // Transform face normal into B's model space
        Mat22 buT = B->u.Transpose( );
        // n變成 n 在 b 的local space
        n = buT * nw;

        // Retrieve support point from B along -n
        // s 離-n的最遠點
        Vec2 s = B->GetSupport( -n );

        // Retrieve vertex on face from A, transform into
        // B's model space
        Vec2 v = A->m_vertices[i];

        v = A->u * v + A->position;
        // v為A的目前枚舉到的一點，並轉為world space
        v -= B->position;
        v = buT * v;

        // Compute penetration distance (in B's model space)
        float d = Dot( n, s - v );

        // Store greatest distance
        if(d > bestDistance)
        {
            bestDistance = d;
            bestIndex = i;
        }
    }

    *faceIdx = bestIndex;
    return bestDistance;
}

void Shape::FindIncidentFace(Vec2 *v, Shape *Ref, Shape *Inc, int refIdx)
{
    Vec2 referenceNormal = Ref->m_normals[refIdx];

    // Calculate normal in incident's frame of reference
    referenceNormal = Ref->u * referenceNormal; // To world space
    referenceNormal = Inc->u.Transpose( ) * referenceNormal; // To incident's model space

    // Find most anti-normal face on incident polygon
    int incidentFace = 0;
    float minDot = FLT_MAX;
    for(int i = 0; i < Inc->m_vertexCount; ++i)
    {
        float dot = Dot( referenceNormal, Inc->m_normals[i] );
        if(dot < minDot)
        {
            minDot = dot;
            incidentFace = i;
        }
    }

    // Assign face vertices for incidentFace
    v[0] = Inc->u * Inc->m_vertices[incidentFace] + Inc->position;
    incidentFace = incidentFace + 1 >= (int)Inc->m_vertexCount ? 0 : incidentFace + 1;
    v[1] = Inc->u * Inc->m_vertices[incidentFace] + Inc->position;
}

int Shape::Clip(Vec2 n, float c, Vec2 *face)
{
    int sp = 0;
    Vec2 out[2] = {
            face[0],
            face[1]
    };

    // Retrieve distances from each endpoint to the line
    // d = ax + by - c
    float d1 = Dot( n, face[0] ) - c;
    float d2 = Dot( n, face[1] ) - c;

    // If negative (behind plane) clip
    if(d1 <= 0.0f) out[sp++] = face[0];
    if(d2 <= 0.0f) out[sp++] = face[1];

    // If the points are on different sides of the plane
    // 兩點跨平面，求交點
    if(d1 * d2 < 0.0f) // less than to ignore -0.0f
    {
        // Push interesection point
        float alpha = d1 / (d1 - d2);
        out[sp] = face[0] + alpha * (face[1] - face[0]);
        ++sp;
    }

    // Assign our new converted values
    face[0] = out[0];
    face[1] = out[1];

    return sp;
}

bool Shape::BiasGreaterThan(float a, float b)
{
    const float k_biasRelative = 0.95f;
    const float k_biasAbsolute = 0.01f;
    return a >= b * k_biasRelative + a * k_biasAbsolute;
}