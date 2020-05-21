#include "Box.h"
#include <SFML/Graphics.hpp>

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

std::deque<Vector> Box::getSAT()
{
    return SAT;
}

std::deque<Vector> Box::getVertices()
{
    return Vertices;
}

void Box::setVertices()
{
    Vertices.clear();
    float angle_rad = degreesToRadians(this->getRotation());
    sf::Vector2f center = this->getPosition();
    Vector cent(center.x, center.y);

    for (auto &idx : corner)
    {
        Vector vec(center.x + idx.x, center.y + idx.y);
        vec.rotate_ref(angle_rad, cent);
        Vertices.push_back(vec);
    }
    set_debug_draw();
}

void Box::findSAT()
{
    SAT.clear();
    for (int i = 1; i < Vertices.size(); i++)
    {
        float tmp_x = Vertices[i].x - Vertices[i-1].x;
        float tmp_y = Vertices[i].y - Vertices[i-1].y;

        Vector tmp(tmp_x, tmp_y);
        SAT.push_back(tmp.normalL());
    }
    Vector tmp2((Vertices[0].x - Vertices[3].x), (Vertices[0].y - Vertices[3].y));
    SAT.push_back(tmp2.normalL());
}

std::pair<float, float> Box::getMinMax(Vector &axis, std::deque<Vector> Vertices)
{
    float min_DotProduct = Vertices[0].projectLengthOnto(axis);
    float max_DotProduct = Vertices[0].projectLengthOnto(axis);
    int min_index = 0, max_index = 0;

    for (int i = 1; i < Vertices.size(); i++)
    {
        float temp = Vertices[i].projectLengthOnto(axis);

        if (temp < min_DotProduct)
        {
            min_DotProduct = temp;
            min_index = i;
        }

        if (temp > max_DotProduct)
        {
            max_DotProduct = temp;
            max_index = i;
        }
    }

    return std::make_pair(min_DotProduct, max_DotProduct);
}

bool Box::isCollide(Box &other)
{
    this->setVertices();
    other.setVertices();

    this->findSAT();
    other.findSAT();
    auto other_sat = other.getSAT();

    //分離軸枚舉，只需枚舉兩個向量，因為另外兩個只是反向而已
    auto PA = getMinMax(this->SAT[0], this->Vertices);
    auto PB = getMinMax(this->SAT[0], other.getVertices());
    auto QA = getMinMax(this->SAT[1], this->Vertices);
    auto QB = getMinMax(this->SAT[1], other.getVertices());

    auto WA = getMinMax(other_sat[0], this->Vertices);
    auto WB = getMinMax(other_sat[0], other.getVertices());
    auto XA = getMinMax(other_sat[1], this->Vertices);
    auto XB = getMinMax(other_sat[1], other.getVertices());


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

void Box::set_debug_draw()
{
    polygon.setPointCount(4);
    polygon.setPoint(0, sf::Vector2f(Vertices[0].x, Vertices[0].y));
    polygon.setPoint(1, sf::Vector2f(Vertices[1].x, Vertices[1].y));
    polygon.setPoint(2, sf::Vector2f(Vertices[2].x, Vertices[2].y));
    polygon.setPoint(3, sf::Vector2f(Vertices[3].x, Vertices[3].y));
    polygon.setFillColor(sf::Color::White);
}