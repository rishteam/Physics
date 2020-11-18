#pragma once

#include "vector_math.h"

class Shape;

class Manifold
{
public:
    Manifold(Shape *a, Shape *b);

    void Solve();
    void Initilaize();
    void ApplyImpulse();
    void PositionalCorrection();
    void InfiniteMassCorrection();

    Shape *A;
    Shape *B;
    float penetration;
    Vec2 normal;
    Vec2 Contacts[2];
    int contactCounter = 0;
    float e;
    float dfriction;
    float sfriction;
};