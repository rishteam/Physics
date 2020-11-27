#pragma once
#include "vector_math.h"

class Shape;


class Joint{
public:
    Joint();
    ~Joint() = default;
    void Set(Shape *b1_, Shape *b2_, const Vec2 &anchor_);
    void PreStep(float inv_dt);
    void ApplyImpulse();

    Mat22 M;
    Vec2 anchor;
    Vec2 localAnchor1, localAnchor2;
    Vec2 r1, r2;
    Vec2 bias;
    Vec2 P;		// accumulated impulse
    Shape* body1;
    Shape* body2;
    float biasFactor;
    float softness;
};
