#include <cmath>

#include "Circle.h"
#include "Polygon.h"
#include "Box.h"

Circle::Circle(float x, float y, float radius_)
{
    this->setPosition(x, y);
    this->setRotation(0.0f);
    radius = radius_;
}

float Circle::get_radius()
{
    return radius;
}

void Circle::set_debug_draw()
{
    circle.setPosition(this->getPosition().x - radius, this->getPosition().y - radius);
    circle.setRadius(radius);
    if (this->selected)
    {
        circle.setFillColor(sf::Color::Red);
    }
    else
    {
        circle.setFillColor(sf::Color::White);
    }
}

//判斷兩園心的的距離，有沒有小於兩半徑之和
bool Circle::isCollide(Circle &c)
{
    auto circleA = c.getPosition();
    auto circleB = this->getPosition();
    auto a = abs(circleA.x - circleB.x);
    auto b = abs(circleA.y - circleB.y);
    return sqrt(a*a + b*b) < this->get_radius() + c.get_radius();
}

bool Circle::isCollide(Polygon &p)
{
    //find polygon SAT
    p.setVertices();
    p.findSAT();
    auto poly_sat = p.getSAT();
    auto C = this->getPosition();
    Vec2 center(C.x, C.y);

    bool isSeparated = false;
    for (int i = 0; i < poly_sat.size(); i++)
    {
        auto minMax_P = getMinMax(poly_sat[i], p.getVertices());
        auto proj_c = center.projectLengthOnto(poly_sat[i]);
        float min_C = proj_c - this->get_radius();
        float max_C = proj_c + this->get_radius();
        isSeparated = (min_C > minMax_P.second || minMax_P.first > max_C);
        // 只要發現有一條分離線，就代表物體沒有發生碰撞
        if (isSeparated)
            return false;
    }
    return true;
}

bool Circle::isCollide(Box &b)
{
    b.setVertices();
    b.findSAT();
    auto box_sat = b.getSAT();
    auto tmp = b.getVertices();
    auto C = this->getPosition();
    Vec2 center(C.x, C.y);

    for (int i = 0; i < box_sat.size(); i++)
    {
        auto minMax = getMinMax(box_sat[i], tmp);
        float proj_c = center.projectLengthOnto(box_sat[i]);
        float min_C = proj_c - this->get_radius();
        float max_C = proj_c + this->get_radius();
        if (min_C > minMax.second || minMax.first > max_C)
            return false;
    }
    return true;
}