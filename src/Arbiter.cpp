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

// update the arbiter and calculate the pulse
void Arbiter::update(Contact* contacts, int numContacts_)
{
    Contact mergedContacts[2];

    for(int i = 0; i < numContacts_; i++)
    {
        Contact* cNew = &contacts[i];
        int k = -1;
        for (int j = 0; j < numContacts_; ++j)
        {
            Contact* cOld = &contacts[j];
            if (cNew->feature.value == cOld->feature.value)
            {
                k = j;
                break;
            }
        }

        if (k > -1)
        {
            Contact* c = mergedContacts + i;
            Contact* cOld = contacts + k;
            *c = *cNew;
            if (World::warmStarting)
            {
                c->Pn = cOld->Pn;
                c->Pt = cOld->Pt;
                c->Pnb = cOld->Pnb;
            }
            else
            {
                c->Pn = 0.0f;
                c->Pt = 0.0f;
                c->Pnb = 0.0f;
            }
        }
        else
        {
            mergedContacts[i] = contacts[i];
        }
    }

    for (int i = 0; i < numContacts; ++i)
        contacts[i] = mergedContacts[i];

    this->numContacts = numContacts_;
}

void Arbiter::PreStep(float inv_dt)
{
    const float k_allowedPenetration = 0.01f;
    float k_biasFactor = World::positionCorrection ? 0.2f : 0.0f;

    for (int i = 0; i < numContacts; ++i)
    {
        Contact* c = &contacts[i];
        Box* body1 = dynamic_cast<Box*>(this->body1);
        Box* body2 = dynamic_cast<Box*>(this->body2);

        Vec2 r1 = c->position - body1->position;
        Vec2 r2 = c->position - body2->position;

        // effective mass
        // Precompute normal mass, tangent mass, and bias.
        // 計算有效質量，切線質量，少部分物理誤差
        float rn1 = Dot(r1, c->normal);
        float rn2 = Dot(r2, c->normal);
        float kNormal = body1->invMass + body2->invMass;
        kNormal += body1->invI * (Dot(r1, r1) - rn1 * rn1) + body2->invI * (Dot(r2, r2) - rn2 * rn2);
        c->massNormal = 1.0f / kNormal;

        Vec2 tangent = Cross(c->normal, 1.0f);
        float rt1 = Dot(r1, tangent);
        float rt2 = Dot(r2, tangent);
        float kTangent = body1->invMass + body2->invMass;
        kTangent += body1->invI * (Dot(r1, r1) - rt1 * rt1) + body2->invI * (Dot(r2, r2) - rt2 * rt2);
        c->massTangent = 1.0f /  kTangent;

        c->bias = -k_biasFactor * inv_dt * Min(0.0f, c->separation + k_allowedPenetration);

        if (World::accumulateImpulses)
        {
            // Apply normal + friction impulse
            Vec2 P = c->Pn * c->normal + c->Pt * tangent;

            body1->velocity -= body1->invMass * P;
            body1->angularVelocity -= body1->invI * Cross(r1, P);

            body2->velocity += body2->invMass * P;
            body2->angularVelocity += body2->invI * Cross(r2, P);
        }
    }
}

void Arbiter::ApplyImpulse()
{
    Box* b1 = dynamic_cast<Box*>(this->body1);
    Box* b2 = dynamic_cast<Box*>(this->body2);

    for (int i = 0; i < numContacts; ++i)
    {
        Contact* c = contacts + i;
        c->r1 = c->position - b1->position;
        c->r2 = c->position - b2->position;

        // Relative velocity at contact
        Vec2 dv = b2->velocity + Cross(b2->angularVelocity, c->r2) - b1->velocity - Cross(b1->angularVelocity, c->r1);

        // Compute normal impulse
        float vn = Dot(dv, c->normal);

        float dPn = c->massNormal * (-vn + c->bias);

        if (World::accumulateImpulses)
        {
            // Clamp the accumulated impulse
            float Pn0 = c->Pn;
            c->Pn = Max(Pn0 + dPn, 0.0f);
            dPn = c->Pn - Pn0;
        }
        else
        {
            dPn = Max(dPn, 0.0f);
        }

        // Apply contact impulse
        Vec2 Pn = dPn * c->normal;

        b1->velocity -= b1->invMass * Pn;
        b1->angularVelocity -= b1->invI * Cross(c->r1, Pn);

        b2->velocity += b2->invMass * Pn;
        b2->angularVelocity += b2->invI * Cross(c->r2, Pn);

        // Relative velocity at contact
        dv = b2->velocity + Cross(b2->angularVelocity, c->r2) - b1->velocity - Cross(b1->angularVelocity, c->r1);

        Vec2 tangent = Cross(c->normal, 1.0f);
        float vt = Dot(dv, tangent);
        float dPt = c->massTangent * (-vt);

        if (World::accumulateImpulses)
        {
            // Compute friction impulse
            float maxPt = friction * c->Pn;

            // Clamp friction
            float oldTangentImpulse = c->Pt;
            c->Pt = Clamp(oldTangentImpulse + dPt, -maxPt, maxPt);
            dPt = c->Pt - oldTangentImpulse;
        }
        else
        {
            float maxPt = friction * dPn;
            dPt = Clamp(dPt, -maxPt, maxPt);
        }

        // Apply contact impulse
        Vec2 Pt = dPt * tangent;

        b1->velocity -= b1->invMass * Pt;
        b1->angularVelocity -= b1->invI * Cross(c->r1, Pt);

        b2->velocity += b2->invMass * Pt;
        b2->angularVelocity += b2->invI * Cross(c->r2, Pt);
    }
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