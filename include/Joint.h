#include "vector_math.h"
#include "Shape.h"
#include "Box.h"

#ifndef Joint_H
#define Joint_H

class Joint{
public:
    Joint();
    ~Joint() = default;
    void Set(Shape *b1_, Shape *b2_, const Vec2 &anchor);
    void PreStep(float inv_dt);
    void ApplyImpulse();

    Mat22 M;
    Vec2 localAnchor1, localAnchor2;
    Vec2 r1, r2;
    Vec2 bias;
    Vec2 P;		// accumulated impulse
    Shape* b1;
    Shape* b2;
    float biasFactor;
    float softness;
};

#endif