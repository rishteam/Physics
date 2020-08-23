#pragma once
#include "vector_math.h"
#include "Shape.h"
#include "World.h"
#include "Box.h"


class Joint{

    Joint();
    ~Joint() = default;
    void Set(Shape *b1, Shape *b2, const Vec2& anchor);
    void PreStep(float inv_dt);
    void ApplyImpulse();

    Mat22 M;
    Vec2 localAnchor1, localAnchor2;
    Vec2 r1, r2;
    Vec2 bias;
    Vec2 P;		// accumulated impulse
    Shape* body1;
    Shape* body2;
    float biasFactor;
    float softness;
};