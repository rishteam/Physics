#include "vector_math.h"

Vector::Vector(float x_, float y_)
{
    x = x_;
    y = y_;
}

float Vector::getLength()
{
    return sqrt(x*x+y*y);
}

float Vector::dot(Vector &vec2)
{
    return x * vec2.x + y * vec2.y;
}

float Vector::projectLengthOnto(Vector &vec2)
{
    float tmp_dot = this->dot(vec2);
    float vec2_len = vec2.getLength();
    return tmp_dot / vec2_len;
}

void Vector::print_Vector()
{
    fmt::print("({}, {})\n", x, y);
}

Vector Vector::normalL()
{
    return Vector(y*-1, x);
}

Vector Vector::normalR()
{
    return Vector(y, x*-1);
}

void Vector::rotate(float angle)
{
    float tmp_x = (x * cos(angle)) - (y * sin(angle));
    float tmp_y = (x * sin(angle)) + (y * cos(angle));
    x = tmp_x;
    y = tmp_y;
}

void Vector::rotate_ref(float angle, Vector &ref)
{
    float tmp_x = (x - ref.x) * cos(angle) - (y - ref.y * sin(angle)) + ref.x;
    float tmp_y = (y - ref.y) * cos(angle) + (x - ref.x * sin(angle)) + ref.y;
    x = tmp_x;
    y = tmp_y;
}