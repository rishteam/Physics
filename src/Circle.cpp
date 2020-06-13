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

//判斷兩園心的的距離，有沒有小於兩半徑之和
bool Circle::isCollide(Circle &c)
{
    auto circleA = c.getPosition();
    auto circleB = this->getPosition();

    return sqrt(abs(circleA.x - circleB.x) * abs(circleA.y - circleB.y)) < this->get_radius() + c.get_radius();
}

bool Circle::isCollide(Polygon &p)
{
    //find polygon SAT
    p.setVertices();
    p.findSAT();
    auto poly_sat = p.getSAT();
    auto C = this->getPosition();
    Vector center(C.x, C.y);

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
            return true;
    }
    return false;
}

bool Circle::isCollide(Box &b)
{
    b.setVertices();
    b.findSAT();
    auto box_sat = b.getSAT();
    auto C = this->getPosition();
    Vector center(C.x, C.y);

    bool isSeparated = false;

    for (int i = 0; i < box_sat.size(); i++)
    {
        auto minMax_A = getMinMax(box_sat[i], b.getVertices());
        auto proj_c = center.projectLengthOnto(box_sat[i]);
        float min_C = proj_c - this->get_radius();
        float max_C = proj_c + this->get_radius();

        isSeparated = (min_C > minMax_A.second || minMax_A.first > max_C);
        // 只要發現有一條分離線，就代表物體沒有發生碰撞
        if (isSeparated)
            return true;
    }
    return false;
}

void Circle::set_debug_draw()
{
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