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
    Contact() = default;
    Vec2 position;
};

class ArbiterKey
{
public:
    //小的放前面
    ArbiterKey(Shape* b1, Shape* b2)
    {
        if (b1 > b2)
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
    void Solve();
    void PositionalCorrection();
    void PreStep(float inv_dt, Vec2 gravity);
    void ApplyImpulse();
    void Update();

    Contact contacts[2];
    int contactCounter = 0;
    Vec2 normal;
    float e = 0;
    float df = 0;
    float sf = 0;
    float friction = 0.0f;
    float penetration = 0;

    Shape* b1;
    Shape* b2;
};

