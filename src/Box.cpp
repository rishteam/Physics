#include "Box.h"


Box::Box(double x, double y, double w, double h)
{
    center.set_x(x);
    center.set_y(y);
    _w = w;
    _h = h;
    _angle = 0;
    corner.push_back(Vector(-w / 2.0, -h / 2.0));
    corner.push_back(Vector(w / 2.0, -h / 2.0));
    corner.push_back(Vector(w / 2.0, h / 2.0));
    corner.push_back(Vector(-w / 2.0, h / 2.0));
}

void Box::set_x(double x)
{
    center.set_x(x);
}

void Box::set_y(double y)
{
    center.set_y(y);
}

double Box::get_x()
{
    return center.get_x();
}

double Box::get_y()
{
    return center.get_y();
}

double Box::get_w()
{
    return _w;
}

double Box::get_h()
{
    return _h;
}

std::deque<Vector> Box::getSAT()
{
    return SAT;
}

std::deque<Vector> Box::getVertices()
{
    return Vertices;
}

void Box::setAngle(double angle)
{
    _angle = angle;
}

// default: 旋轉0度
void Box::setVertices()
{
    double angle_rad = degreesToRadians(_angle);
    Vertices.clear();
    for (auto &idx : corner)
    {
        Vector vec(center.get_x() + idx.get_x(), center.get_y() + idx.get_y());
        vec.rotate_ref(angle_rad, center);
        Vertices.push_back(vec);
    }
}

void Box::findSAT()
{
    for (int i = 1; i < Vertices.size(); i++)
    {
        Vector tmp((Vertices[i].get_x() - Vertices[i - 1].get_x()), (Vertices[i].get_y() - Vertices[i - 1].get_y()));
        SAT.push_back(tmp.normalL());
    }
    Vector tmp((Vertices[0].get_x() - Vertices[Vertices.size() - 1].get_x()), (Vertices[0].get_y() - Vertices[Vertices.size() - 1].get_y()));
    SAT.push_back(tmp.normalL());
}

std::pair<double, double> Box::getMinMax(Vector &axis, std::deque<Vector> Vertices)
{
    double min_dot = MAX_DOUBLE;
    double max_dot = MIN_DOUBLE;

    for(auto &idx : Vertices)
    {
        double tmp = idx.projectLengthOnto(axis);
        min_dot = std::min(min_dot, tmp);
        max_dot = std::max(max_dot, tmp);
    }
    return std::make_pair(min_dot, max_dot);
}

bool Box::SAT_collision(Box &other)
{
    this->setVertices();
    this->findSAT();

    for(auto i : SAT)
        std::cout << i << '\n';
    other.setVertices();
    other.findSAT();


    //分離軸枚舉，只需枚舉兩個向量，因為另外兩個只是反向而已
    auto PA = getMinMax(this->SAT[0], Vertices);
    auto PB = getMinMax(this->SAT[0], other.getVertices());
    auto QA = getMinMax(this->SAT[1], Vertices);
    auto QB = getMinMax(this->SAT[1], other.getVertices());
    auto other_sat = other.getSAT();
    auto RA = getMinMax(other_sat[0], Vertices);
    auto RB = getMinMax(other_sat[0], other.getVertices());
    auto SA = getMinMax(other_sat[1], Vertices);
    auto SB = getMinMax(other_sat[1], other.getVertices());

    //檢查分離軸上投影區段是否分開
    bool sep_P = (PB.first > PA.second) || (PA.first > PB.second);
    bool sep_Q = (QB.first > QA.second) || (QA.first > QB.second);
    bool sep_R = (RB.first > RA.second) || (RA.first > RB.second);
    bool sep_S = (SB.first > SA.second) || (SA.first > SB.second);

    if (sep_P || sep_Q || sep_R || sep_S)
    {
        return false;
    }
    else
    {
        return true;
    }
}

void Box::draw(sf::RenderWindow &window, bool coll)
{
    sf::ConvexShape polygon;
    polygon.setPointCount(4);
    polygon.setPoint(0, sf::Vector2f(Vertices[0].get_x(), Vertices[0].get_y()));
    polygon.setPoint(1, sf::Vector2f(Vertices[1].get_x(), Vertices[1].get_y()));
    polygon.setPoint(2, sf::Vector2f(Vertices[2].get_x(), Vertices[2].get_y()));
    polygon.setPoint(3, sf::Vector2f(Vertices[3].get_x(), Vertices[3].get_y()));
    if (coll)
        polygon.setFillColor(sf::Color::Red);
    else
        polygon.setFillColor(sf::Color::White);
    polygon.setPosition(get_x() - (_w / 2), get_y() - (_h / 2));
    // sf::RectangleShape window_box;
    // window_box.setSize(sf::Vector2f(_w, _h));

    window.draw(polygon);
}
