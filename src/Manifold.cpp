#include "Manifold.h"
#include "Shape.h"

Manifold::Manifold(Shape *a, Shape *b)
{
    A = a;
    B = b;
}