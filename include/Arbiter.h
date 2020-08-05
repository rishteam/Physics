#pragma once
#include "Shape.h"
#include "box.h"
#include "VClip.h"
#include "World.h"
#include "vector_math.h"


class Contact
{
public:
    Contact() : Pn(0.0f), Pt(0.0f), Pnb(0.0f) {}
    Vec2 position;
    Vec2 normal;
    Vec2 r1, r2;
    float separation;
    //累加衝量
    float Pn;	// accumulated normal impulse
    //切線方向的衝量
    float Pt;	// accumulated tangent impulse
    //類加衝量加位置的偏差量
    float Pnb;	// accumulated normal impulse for position bias
    float massNormal, massTangent;
    float bias;
    FeaturePair feature;
};

class ArbiterKey
{
public:
    //小的放前面
    ArbiterKey(Shape* b1, Shape* b2)
    {
        if (b1 < b2)
        {
            body1 = b1; body2 = b2;
        }
        else
        {
            body1 = b2; body2 = b1;
        }
    }

    Shape* body1;
    Shape* body2;
};

inline bool operator < (const ArbiterKey& a1, const ArbiterKey& a2)
{
    if (a1.body1 < a2.body1)
        return true;

    if (a1.body1 == a2.body1 && a1.body2 < a2.body2)
        return true;

    return false;
}

class Arbiter : public Vclip
{
public:
    Arbiter(Shape* b1, Shape* b2);
    ~Arbiter() = default;
    void update(Contact* contacts, int numContacts);
    void PreStep(float inv_dt);
    void ApplyImpulse();
    void calContactPoints(Contact *contacts, Shape* b1, Shape* b2);
    void ComputeIncidentEdge(ClipVertex c[2], const Vec2& h, const Vec2& pos,
                             const Mat22& Rot, const Vec2& normal);

    Contact contacts[2];
    int numContacts;

    Shape* body1;
    Shape* body2;

    //combined friction
    float friction;
};

