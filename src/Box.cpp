#include <SFML/Graphics.hpp>

#include "Box.h"
#include "Polygon.h"
#include "Circle.h"

Box::Box(float x, float y, float w, float h)
{
    this->setPosition(x, y);
    _w = w;
    _h = h;
    this->setRotation(0.0f);
    corner.push_back(Vector(w / 2.0, -h / 2.0));
    corner.push_back(Vector(w / 2.0, h / 2.0));
    corner.push_back(Vector(-w / 2.0, h / 2.0));
    corner.push_back(Vector(-w / 2.0, -h / 2.0));
}

void Box::set_debug_draw()
{
    int cnt = Vertices.size();
    polygon.setPointCount(cnt);
    for (int i = 0; i < cnt; i++)
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

void Box::setVertices()
{
    this->Vertices.clear();
    float angle_rad = degreesToRadians(this->getRotation());
    sf::Vector2f center = this->getPosition();
    Vector cent(center.x, center.y);

    for (auto &idx : corner)
    {
        Vector vec(center.x + idx.x, center.y + idx.y);
        vec.rotate_ref(angle_rad, cent);
        Vertices.push_back(vec);
    }
}

bool Box::isCollide(Box &b)
{
    this->setVertices();
    this->findSAT();

    b.setVertices();
    b.findSAT();
    auto b_sat = b.getSAT();

    //分離軸枚舉，只需枚舉兩個向量，因為另外兩個只是反向而已
    auto PA = getMinMax(this->SAT[0], this->Vertices);
    auto PB = getMinMax(this->SAT[0], b.getVertices());
    auto QA = getMinMax(this->SAT[1], this->Vertices);
    auto QB = getMinMax(this->SAT[1], b.getVertices());

    auto WA = getMinMax(b_sat[0], this->Vertices);
    auto WB = getMinMax(b_sat[0], b.getVertices());
    auto XA = getMinMax(b_sat[1], this->Vertices);
    auto XB = getMinMax(b_sat[1], b.getVertices());


    //檢查分離軸上投影區段是否分開
    bool sep_P = (PB.first > PA.second) || (PA.first > PB.second);
    bool sep_Q = (QB.first > QA.second) || (QA.first > QB.second);

    bool sep_W = (WB.first > WA.second) || (WA.first > WB.second);
    bool sep_X = (XB.first > XA.second) || (XA.first > XB.second);

    if (sep_P || sep_Q || sep_W || sep_X)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool Box::isCollide(Polygon &p)
{
    //box_sat
    this->setVertices();
    this->findSAT();
    auto box_sat = this->getSAT();

    //poly_sat
    p.setVertices();
    p.findSAT();
    auto poly_sat = p.getSAT();

    bool isSeparated = false;

    // box_sat check
    for (int i = 0; i < box_sat.size(); i++)
    {
        auto minMax_A = getMinMax(box_sat[i], this->Vertices);
        auto minMax_B = getMinMax(box_sat[i], p.getVertices());

        isSeparated = (minMax_B.first > minMax_A.second || minMax_A.first > minMax_B.second);
        // 只要發現有一條分離線，就代表物體沒有發生碰撞
        if (isSeparated)
            return true;
    }

    // poly_sat check
    for (int i = 0; i < poly_sat.size(); i++)
    {
        auto minMax_A = getMinMax(poly_sat[i], this->Vertices);
        auto minMax_B = getMinMax(poly_sat[i], p.getVertices());

        isSeparated = (minMax_B.first > minMax_A.second || minMax_A.first > minMax_B.second);
        // 只要發現有一條分離線，就代表物體沒有發生碰撞
        if (isSeparated)
            return true;
    }

    return false;
}

bool Box::isCollide(Circle &c)
{
    this->setVertices();
    this->findSAT();
    auto box_sat = this->getSAT();
    
    // printf("cir collide box\n");

    auto C_pos = c.getPosition();
    
    Vector center(C_pos.x, C_pos.y);

    bool isSeparated = false;

    for (int i = 0; i < box_sat.size(); i++)
    {
        auto minMax_A = getMinMax(box_sat[i], this->Vertices);
        auto proj_c = center.projectLengthOnto(box_sat[i]);
        float min_C = proj_c - c.get_radius();
        float max_C = proj_c + c.get_radius();

        isSeparated = (min_C > minMax_A.second || minMax_A.first > max_C);
        // 只要發現有一條分離線，就代表物體沒有發生碰撞
        if (isSeparated)
            return true;
    }
    return false;
}
