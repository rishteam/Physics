#include "vector_math.h"
#include <cmath>

Vector::Vector(double x, double y)
{
    _x = x;
    _y = y;
}

void Vector::set_x(double x)
{
    _x = x;
}

void Vector::set_y(double y)
{
    _y = y;
}

double Vector::get_x()
{
    return _x;
}

double Vector::get_y()
{
    return _y;
}

double Vector::getLength()
{
    return sqrt(_x*_x+_y*_y);
}

double Vector::dot(Vector &vec2)
{
    return _x * vec2.get_x() + _y * vec2.get_y();
}

double Vector::projectLengthOnto(Vector &vec2)
{
    double tmp_dot = this->dot(vec2);
    double vec2_len = vec2.getLength();
    return tmp_dot / vec2_len;
}

std::ostream &operator<<(std::ostream &os, const Vector &vec)
{
    os << "(" << vec._x << ", " << vec._y << ")";
    return os;
}

Vector Vector::normalL()
{
    return Vector(-_y, _x);
}

Vector Vector::normalR()
{
    return Vector(_y, -_x);
}

void Vector::rotate(double angle)
{
    double tmp_x = (_x * cos(angle)) - (_y * sin(angle));
    double tmp_y = (_x * sin(angle)) - (_y * cos(angle));
    _x = tmp_x;
    _y = tmp_y;
}

void Vector::rotate_ref(double angle, Vector &ref)
{
    double tmp_x = ((_x - ref.get_x()) * cos(angle)) - ((_y - ref.get_y()) * sin(angle)) + ref.get_x();
    double tmp_y = ((_y - ref.get_y()) * cos(angle)) + ((_x - ref.get_x()) * sin(angle)) + ref.get_y();
    _x = tmp_x;
    _y = tmp_y;
}