#include "Polygon.h"
#include "Box.h"
#include "Circle.h"
#include "windows.h"

Polygon::Polygon(std::deque<Vector> &pt, Vector pos){
    corner = pt;
    this->setPosition(pos.x, pos.y);
    this->setRotation(0.0f);
    this->setVertices();
}

void Polygon::set_debug_draw()
{
    polygon.setPointCount(Vertices.size());
    for (int i = 0; i < Vertices.size(); i++)
    {
        polygon.setPoint(i, sf::Vector2f(Vertices[i].x, Vertices[i].y));
    }
    if(this->selected)
    {
        polygon.setFillColor(sf::Color::Red);
    }
    else
    {
        polygon.setFillColor(sf::Color::White);
    }
}

void Polygon::setVertices()
{
    this->Vertices.clear();
    float angle_rad = degreesToRadians(this->getRotation());

    auto tmp_cent = this->getPosition();
    Vector cent(tmp_cent.x, tmp_cent.y);

    for (auto idx : this->corner)
    {
        Vector vec(cent.x + idx.x, cent.y + idx.y);
        vec.rotate_ref(angle_rad, cent);
        Vertices.push_back(vec);
    }
}

bool Polygon::isCollide(Polygon &p)
{
    this->setVertices();
    this->findSAT();
    auto poly_a_sat = this->getSAT();
    p.setVertices();
    p.findSAT();
    auto poly_b_sat = p.getSAT();

    // poly_a_sat check
    for (int i = 0; i < poly_a_sat.size(); i++)
    {
        auto A = getMinMax(poly_a_sat[i], this->Vertices);
        auto B = getMinMax(poly_a_sat[i], p.getVertices());
        // fmt::print("{} {} {} {}\n", A.first, A.second, B.first, B.second);
        // 只要發現有一條分離線，就代表物體沒有發生碰撞
        if (B.first > A.second || A.first > B.second)
        {
            return false;
        }
    }

    // poly_b_sat check
    for (int i = 0; i < poly_b_sat.size(); i++)
    {
        auto A = getMinMax(poly_b_sat[i], this->Vertices);
        auto B = getMinMax(poly_b_sat[i], p.getVertices());

        // 只要發現有一條分離線，就代表物體沒有發生碰撞
        if (B.first > A.second || A.first > B.second)
        {
            return false;
        }
    }
    return true;
}


//min, max
bool Polygon::isCollide(Box &b)
{
    //poly_sat
    this->setVertices();
    this->findSAT();
    auto poly_sat = this->getSAT();

    //Box sat
    b.setVertices();
    b.findSAT();
    auto box_sat = b.getSAT();

    bool isSeparated = false;
    // poly_sat check
    for (int i = 0; i < poly_sat.size(); i++)
    {
        // poly_sat[i].print_Vector();
        auto minMax_A = getMinMax(poly_sat[i], b.getVertices());
        auto minMax_B = getMinMax(poly_sat[i], this->Vertices);

        isSeparated = (minMax_B.first > minMax_A.second || minMax_A.first > minMax_B.second);
        // 只要發現有一條分離線，就代表物體沒有發生碰撞
        if (isSeparated)
            return true;
    }

    // box_sat check
    for (int i = 0; i < box_sat.size(); i++)
    {
        auto minMax_A = getMinMax(box_sat[i], this->Vertices);
        auto minMax_B = getMinMax(box_sat[i], b.getVertices());

        isSeparated = (minMax_B.first > minMax_A.second || minMax_A.first > minMax_B.second);
        // 只要發現有一條分離線，就代表物體沒有發生碰撞
        if (isSeparated)
            return true;
    }
    return false;
}

bool Polygon::isCollide(Circle &c)
{
    //find polygon SAT
    this->setVertices();
    this->findSAT();
    auto poly_sat = this->getSAT();
    auto C = c.getPosition();
    Vector center(C.x, C.y);

    bool isSeparated = false;
    for (int i = 0; i < poly_sat.size(); i++)
    {
        auto minMax_P = getMinMax(poly_sat[i], this->Vertices);
        auto proj_c = center.projectLengthOnto(poly_sat[i]);
        auto min_C = proj_c - c.get_radius();
        auto max_C = proj_c + c.get_radius();
        isSeparated = (min_C > minMax_P.second || minMax_P.first > max_C);
        // 只要發現有一條分離線，就代表物體沒有發生碰撞
        if (isSeparated)
            return false;
    }

    return true;
}