#pragma once
#include "Shape.h"
#include "Box.h"
#include "World.h"
#include "vector_math.h"

enum class FeaturePair{
    inEdge1,
    outEdge1,
    inEdge2,
    outEdge2
};

struct Contact
{
    Contact(): Pn(0.0f), Pt(0.0f), Pnb(0.0f) {}
    Vec2 position;
    Vec2 normal;
    Vec2 r1;
    Vec2 r2;
    float penetration;
    //累加衝量
    float Pn;	// accumulated normal impulse
    //切線方向的衝量
    float Pt;	// accumulated tangent impulse
    //加衝量加位置的偏差量
    float Pnb;	// accumulated normal impulse for position bias
    float massNormal, massTangent;
    float bias;
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

class Arbiter
{
public:
    Arbiter(Shape* b1_, Shape* b2_);
    ~Arbiter() = default;
    void Update();
    void PreStep(float inv_dt);
    void ApplyImpulse();
    int calContactPoints(Contact *contacts, Shape* b1, Shape* b2);

    Contact contacts[2];
    int contactCounter = 0;
    float e = 0;
    float df = 0;
    float sf = 0;
    float friction = 0.0f;

    Shape* b1;
    Shape* b2;
};

