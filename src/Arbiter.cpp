#include "Arbiter.h"

Arbiter::Arbiter(Shape* b1, Shape* b2)
{
    if (b1 < b2)
    {
        body1 = b1;
        body2 = b2;
    }
    else
    {
        body1 = b2;
        body2 = b1;
    }

    calContactPoints(this->contacts, b1, b2);

    friction = sqrtf( dynamic_cast<Box*>(body1)->getfriction() * dynamic_cast<Box*>(body2)->getfriction());
}

void Arbiter::ComputeIncidentEdge(ClipVertex *c, const Vec2& h, const Vec2& pos,
                                       const Mat22& Rot, const Vec2& normal)

{
   // The normal is from the reference box. Convert it
   // to the incident boxe's frame and flip sign.
   Mat22 RotT = Rot.Transpose();
   // n is inverse of reference face vector
   // -normal * RotT
   Vec2 n = -(RotT * normal);
   // change positive
   Vec2 nAbs = Abs(n);

   if (nAbs.x > nAbs.y)
   {
       if (Sign(n.x) > 0.0f)
       {
           //V_4
           c[0].v.Set(h.x, -h.y);
           c[0].fp.e.inEdge2 = EDGE3;
           c[0].fp.e.outEdge2 = EDGE4;
           //V_1
           c[1].v.Set(h.x, h.y);
           c[1].fp.e.inEdge2 = EDGE4;
           c[1].fp.e.outEdge2 = EDGE1;
       }
       else
       {
           //V_2
           c[0].v.Set(-h.x, h.y);
           c[0].fp.e.inEdge2 = EDGE1;
           c[0].fp.e.outEdge2 = EDGE2;
           //V_3
           c[1].v.Set(-h.x, -h.y);
           c[1].fp.e.inEdge2 = EDGE2;
           c[1].fp.e.outEdge2 = EDGE3;
       }
   }
   else
   {
       if (Sign(n.y) > 0.0f)
       {
           c[0].v.Set(h.x, h.y);
           c[0].fp.e.inEdge2 = EDGE4;
           c[0].fp.e.outEdge2 = EDGE1;

           c[1].v.Set(-h.x, h.y);
           c[1].fp.e.inEdge2 = EDGE1;
           c[1].fp.e.outEdge2 = EDGE2;
       }
       else
       {
           c[0].v.Set(-h.x, -h.y);
           c[0].fp.e.inEdge2 = EDGE2;
           c[0].fp.e.outEdge2 = EDGE3;

           c[1].v.Set(h.x, -h.y);
           c[1].fp.e.inEdge2 = EDGE3;
           c[1].fp.e.outEdge2 = EDGE4;
       }
   }
   //旋轉和轉換clip點到original vertex position
   c[0].v = pos + Rot * c[0].v;
   c[1].v = pos + Rot * c[1].v;
}

void Arbiter::update(Contact& contacts, int numContacts)
{

}

void Arbiter::PreStep(float inv_dt)
{

}

void Arbiter::ApplyImpulse()
{

}

void Arbiter::calContactPoints(Contact* contacts, Shape* b1, Shape* b2)
{
    //get the box
    auto bodyA = dynamic_cast<Box*>(b1);
    auto bodyB = dynamic_cast<Box*>(b2);

    //setup
    Vec2 hA = Vec2(bodyA->getwidth()* 0.5, bodyA->getheight() * 0.5);
    Vec2 hB = Vec2(bodyB->getwidth()* 0.5, bodyB->getheight() * 0.5);

    Vec2 posA = Vec2(bodyA->getPosition().x, bodyA->getPosition().y);
    Vec2 posB = Vec2(bodyB->getPosition().x, bodyB->getPosition().y);

    //Rotate Matrix
    Mat22 RotA(bodyA->getRotation()), RotB(bodyB->getRotation());

    //Transpose Matrix
    Mat22 RotAT = RotA.Transpose();
    Mat22 RotBT = RotB.Transpose();

    //delta position
    //red-vector
    Vec2 dp = posB - posA;
    //ch
    Vec2 dA = RotAT * dp;
    Vec2 dB = RotBT * dp;

    Mat22 C = RotAT * RotB;
    Mat22 absC = Abs(C);
    Mat22 absCT = absC.Transpose();

    // magic formula 檢測SAT
    // source: https://stackoverflow.com/questions/40047219/in-box2d-lite-i-cant-figure-out-what-the-box-a-faces-and-box-b-faces-check?answertab=oldest#tab-top

    // Box A faces
    Vec2 faceA = Abs(dA) - hA - absC * hB;

    // Box B faces
    Vec2 faceB = Abs(dB) - absCT * hA - hB;

    // Find best axis
    Axis axis;
    float separation;
    Vec2 normal;


    // 證明: https://www.randygaul.net/2013/03/28/custom-physics-engine-part-2-manifold-generation/
    // 找出最適合的最小穿透軸
    axis = FACE_A_X;
    separation = faceA.x;
    normal = dA.x > 0.0f ? RotA.col1 : -RotA.col1;

    //magic-formula 決定最小穿透軸
    const float relativeTol = 0.95f;
    const float absoluteTol = 0.01f;
    if (faceA.y > relativeTol * separation + absoluteTol * hA.y)
    {
        axis = FACE_A_Y;
        separation = faceA.y;
        normal = dA.y > 0.0f ? RotA.col2 : -RotA.col2;
    }


    if (faceB.x > relativeTol * separation + absoluteTol * hB.x)
    {
        axis = FACE_B_X;
        separation = faceB.x;
        normal = dB.x > 0.0f ? RotB.col1 : -RotB.col1;
    }

    if (faceB.y > relativeTol * separation + absoluteTol * hB.y)
    {
        axis = FACE_B_Y;
        separation = faceB.y;
        normal = dB.y > 0.0f ? RotB.col2 : -RotB.col2;
    }

    // Setup clipping plane data based on the separating axis
    Vec2 frontNormal, sideNormal;
    //與人碰撞的edge
    // Vec2 v; (點點)
    // FeaturePair fp; (點點夾的兩邊)
    ClipVertex incidentEdge[2];
    float front, negSide, posSide;
    char negEdge, posEdge;

    // Compute the clipping lines and the line segment to be clipped.
    // 計算出各種的直線方程式
    switch (axis)
    {
        case FACE_A_X:
        {
            //reference face vector
            frontNormal = normal;
            front = Dot(posA, frontNormal) + hA.x;
            sideNormal = RotA.col2;
            // dot product is to compute c (ax + by)
            float side = Dot(posA, sideNormal);
            // side
            negSide = -side + hA.y;
            posSide =  side + hA.y;
            negEdge = EDGE3;
            posEdge = EDGE1;
            ComputeIncidentEdge(incidentEdge, hB, posB, RotB, frontNormal);
        }
            break;

        case FACE_A_Y:
        {
            frontNormal = normal;
            front = Dot(posA, frontNormal) + hA.y;
            sideNormal = RotA.col1;
            float side = Dot(posA, sideNormal);
            negSide = -side + hA.x;
            posSide =  side + hA.x;
            negEdge = EDGE2;
            posEdge = EDGE4;
            ComputeIncidentEdge(incidentEdge, hB, posB, RotB, frontNormal);
        }
            break;

        case FACE_B_X:
        {
            frontNormal = -normal;
            front = Dot(posB, frontNormal) + hB.x;
            sideNormal = RotB.col2;
            float side = Dot(posB, sideNormal);
            negSide = -side + hB.y;
            posSide =  side + hB.y;
            negEdge = EDGE3;
            posEdge = EDGE1;
            ComputeIncidentEdge(incidentEdge, hA, posA, RotA, frontNormal);
        }
            break;

        case FACE_B_Y:
        {
            frontNormal = -normal;
            front = Dot(posB, frontNormal) + hB.y;
            sideNormal = RotB.col1;
            float side = Dot(posB, sideNormal);
            negSide = -side + hB.x;
            posSide =  side + hB.x;
            negEdge = EDGE2;
            posEdge = EDGE4;
            ComputeIncidentEdge(incidentEdge, hA, posA, RotA, frontNormal);
        }
            break;
    }

    // clip other face with 5 box planes (1 face plane, 4 edge planes)

    ClipVertex clipPoints1[2];
    ClipVertex clipPoints2[2];
    int np;

    // Clip to box side 1
    // 先測試negtive-side，決定候選點
    ClipSegmentToLine(clipPoints1, incidentEdge, -sideNormal, negSide, negEdge);


    // Clip to negative box side 1
    // 再放入positive-side，進行第二次的過濾
    ClipSegmentToLine(clipPoints2, clipPoints1, sideNormal, posSide, posEdge);


    // Now clipPoints2 contains the clipping points.
    // Due to roundoff, it is possible that clipping removes all points.

    int numContacts_ = 0;
    for (int i = 0; i < 2; ++i)
    {
        float separation = Dot(frontNormal, clipPoints2[i].v) - front;

        if (separation <= 0)
        {
            contacts[numContacts_].separation = separation;
            contacts[numContacts_].normal = normal;
            // slide contact point onto reference face (easy to cull)
            contacts[numContacts_].position = clipPoints2[i].v - separation * frontNormal;
            contacts[numContacts_].feature = clipPoints2[i].fp;
            if (axis == FACE_B_X || axis == FACE_B_Y)
                Flip(contacts[numContacts].feature);
            ++numContacts_;
        }
    }
    this->numContacts = numContacts_;
}