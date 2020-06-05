#include "Shape.h"

Polygon::Polygon(std::deque<Vector> pt){
    Vertices = pt;
}

Polygon::set_debug_draw() override
{
    polygon.setPointCount(Vertices.size());
    for (int i = 0; i < cnt; i++)
    {
        polygon.setPoint(i, sf::Vector2f(Vertices[i].x, Vertices[i].y));
    }
    polygon.setFillColor(sf::Color::White);
}

Polygon::setVertices(std::deque<Vector> &pt)
{
    Vertices = pt;
}

virtual bool Polygon::isCollide(const Polygon &p) const override
{
    auto poly_a_sat = this->findSAT();
    auto poly_b_sat = p.findSAT();

    bool isSeparated = false;
    // poly_a_sat check
    for (int i = 0; i < poly_a_sat.size(); i++)
    {
        float minMax_A = getMinMax(this.Vertices, poly_a_sat[i]);
        float minMax_B = getMinMax(p.Vertices, poly_a_sat[i]);

        isSeparated = (minMax_B.min > minMax_A.max || minMax_A.min > minMax_B.max);
        // 只要發現有一條分離線，就代表物體沒有發生碰撞
        if (isSeparated)
            return true;
    }

    // poly_b_sat check
    for (int i = 0; i < poly_b_sat.size(); i++)
    {
        float minMax_A = getMinMax(this.Vertices, poly_b_sat[i]);
        float minMax_B = getMinMax(p.Vertices, poly_b_sat[i]);

        isSeparated = (minMax_B.min > minMax_A.max || minMax_A.min > minMax_B.max);
        // 只要發現有一條分離線，就代表物體沒有發生碰撞
        if (isSeparated)
            return true;
    }
    return false;
}

virtual bool Polygon::isCollide(const Circle &c) const override
{
    //find polygon SAT
    auto poly_sat = this->findSAT();
    auto C = c.getPosition();
    Vector center(C.x, C.y);

    bool isSeparated = false;
    for (int i = 0; i < poly_sat.size(); i++)
    {
        auto minMax_P = getMinMax(poly_sat[i], this.Vertices);
        auto proj_c = center.projectLengthOnto(poly_sat[i]);
        float min_C = proj_c - c.get_radius();
        float max_C = proj_c + c.get_radius();
        isSeparated = (min_C > minMax_P.second || minMax_P.first > max_C);
        // 只要發現有一條分離線，就代表物體沒有發生碰撞
        if (isSeparated)
            return true;
    }
}

virtual bool Polygon::isCollide(const Box &b) const override
{
    //poly_sat
    auto poly_sat = this->findSAT();

    //Box sat
    b.setVertices();
    auto box_sat = b.findSAT();

    bool isSeparated = false;

    // poly_sat check
    for (int i = 0; i < poly_sat.size(); i++)
    {
        float minMax_A = getMinMax(this.Vertices, poly_sat[i]);
        float minMax_B = getMinMax(b.Vertices, poly_sat[i]);

        isSeparated = (minMax_B.min > minMax_A.max || minMax_A.min > minMax_B.max);
        // 只要發現有一條分離線，就代表物體沒有發生碰撞
        if (isSeparated)
            return true;
    }

    // box_sat check
    for (int i = 0; i < box_sat.size(); i++)
    {
        float minMax_A = getMinMax(this.Vertices, box_sat[i]);
        float minMax_B = getMinMax(b.Vertices, box_sat[i]);

        isSeparated = (minMax_B.min > minMax_A.max || minMax_A.min > minMax_B.max);
        // 只要發現有一條分離線，就代表物體沒有發生碰撞
        if (isSeparated)
            return true;
    }

    return false;
}