#include "Shape.h"
#include <cmath>

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


virtual bool Circle::iscollide(const Circle &c) const override
{
    auto circleA = c.getPosition();
    auto circleB = this.getPosition();

    return sqrt(abs(circleA.x - circleB.x) * abs(circleA.y - circleB.y)) < circleA.get_radius() + circleB.get_radius();
}

virtual bool Circle::iscollide(const Polygon &p) const override
{
    //find polygon SAT
    auto poly_sat = p->findSAT();
    auto C = this.getPosition();
    Vector center(C.x, C.y);

    bool isSeparated = false;
    for (int i = 0; i < poly_sat.size(); i++)
    {
        auto minMax_P = getMinMax(poly_sat[i], p.Vertices);
        auto proj_c = center.projectLengthOnto(poly_sat[i]);
        float min_C = proj_c - c.get_radius();
        float max_C = proj_c + c.get_radius();
        isSeparated = (min_C > minMax_P.second || minMax_P.first > max_C);
        // 只要發現有一條分離線，就代表物體沒有發生碰撞
        if (isSeparated)
            return true;
    }
    return false;
}

virtual bool Circle::iscollide(const Box &b) const override
{
    b.setVertices();
    auto box_sat = b.find_SAT();
    auto C = this.getPosition();
    Vector center(C.x, C.y);

    bool isSeparated = false;

    for (int i = 0; i < box_sat.size(); i++)
    {
        float minMax_A = getMinMax(this.Vertices, box_sat[i]);
        auto proj_c = center.projectLengthOnto(poly_sat[i]);
        float min_C = proj_c - c.get_radius();
        float max_C = proj_c + c.get_radius();

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
    circle.setFillColor(sf::Color::White);
}