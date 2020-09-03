#include <SFML/Graphics.hpp>

#include "Box.h"
#include "Polygon.h"
#include "Circle.h"

Box::Box(float x, float y, float w, float h, float m)
{
    this->setPosition(x, y);
    _w = w;
    _h = h;
    this->setRotation(0.0f);
    this->initPhysics(m);
    this->TransformPhysicsCoordinate(this->getPosition().x, this->getPosition().y, this->getwidth(), this->getheight(), this->getRotation());
    corner.push_back(Vec2(w / 2.0, -h / 2.0));
    corner.push_back(Vec2(w / 2.0, h / 2.0));
    corner.push_back(Vec2(-w / 2.0, h / 2.0));
    corner.push_back(Vec2(-w / 2.0, -h / 2.0));
    this->setVertices();
}


void Box::set_debug_draw()
{
    sf::Color color(255, 0, 0, 122);
    int cnt = Vertices.size();
    polygon.setPointCount(cnt);
    this->setVertices();
    for (int i = 0; i < cnt; i++)
    {
        polygon.setPoint(i, sf::Vector2f(Vertices[i].x, Vertices[i].y));
    }
    polygon.setFillColor(color);
}

void Box::setVertices()
{
    this->Vertices.clear();
    sf::Vector2f center = this->getPosition();
    float angle_rad = degreesToRadians(this->getRotation());
    Vec2 cent(center.x, center.y);
    for (auto &idx : corner)
    {
        Vec2 vec(center.x + idx.x, center.y + idx.y);
        vec.rotate_ref(angle_rad, cent);
        Vertices.push_back(vec);
    }
}

void Box::initPhysics(float m)
{
    velocity.Set(0.0f, 0.0f);
    angularVelocity = 0.0f;
    force.Set(0.0f, 0.0f);
    torque = 0.0f;
    friction = 0.2f;
    mass = m;

    if (mass < MAX_float)
    {
        invMass = 1.0f / mass;
        I = mass * (wh.x * wh.x + wh.y * wh.y) / 12.0f;
        invI = 1.0f / I;
    }
    else
    {
        invMass = 0.0f;
        I = MAX_float;
        invI = 0.0f;
    }
}

float Box::getwidth()
{
    return _w;
}

float Box::getheight()
{
    return _h;
}
//TODO::for GJK
Vec2 Box::supportPoint(Vec2 D)
{
    this->setVertices();

    Vec2 MAXP = Vertices[0];
    float MAXN = MAXP.dot(D);

    for(int i = 1 ; i < 4 ; i++ ){
        Vec2 P = Vertices[i];
        float tmpF = P.dot(D);
        if( tmpF > MAXN ){
            MAXN =tmpF;
            MAXP = P;
        }
    }
    return MAXP;
}

//TODO::for GJK
bool Box::isCollide(Shape &s)
{
    if (World::collision_type == COLLISION::SAT)
    {
        return s.isCollide(*this);
    }
    else if(World::collision_type == COLLISION::GJK)
    {
        int PointNum = 0;
        Vec2 Simplex[3];
        Vec2 A_center(this->getPosition().x, this->getPosition().y);
        Vec2 B_center(s.getPosition().x, s.getPosition().y);
        Vec2 D(A_center, B_center);
        if( D.x == 0 && D.y == 0 )
        {
            D = Vec2(1.0f, 0.0f);
        }
        Simplex[PointNum++] = SupportFun(*this, s, D);
        Simplex[PointNum++] = SupportFun(*this, s, -D);

        while( true ) {
            Vec2 V(Simplex[0], Simplex[1]);
            if (V.AtTheSameSide(-Vec2(Simplex[0]))) {
                D = Vec2(-V.y, V.x);
            } else {
                D = Vec2(V.y, -V.x);
            }

            Simplex[PointNum++] = SupportFun(*this, s, D);

            if (Simplex[2].dot(D) < 0) {
                return false;
            }

            Vec2 AB(Simplex[2], Simplex[1]);
            Vec2 AC(Simplex[2], Simplex[0]);

            Vec2 DAB, DAC;
            if (AB.AtTheSameSide(-Vec2(Simplex[2]))) {
                DAB = Vec2(-AB.y, AB.x);
            } else {
                DAB = Vec2(AB.y, -AB.x);
            }

            if (AC.AtTheSameSide(-Vec2(Simplex[2])))
                DAC = Vec2(-AC.y, AC.x);
            else DAC = Vec2(AC.y, -AC.x);

            if (DAB.dot(AC) >= 0.0f) {
                if (DAC.dot(AB) >= 0.0f)
                    return true;
                Simplex[1] = Simplex[2];
                PointNum--;
                continue;
            }
            if (DAC.dot(AB) >= 0.0f) {
                Simplex[0] = Simplex[2];
                PointNum--;
                continue;
            }
            return false;
        }
    }
    return false;
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
            return false;
    }

    // poly_sat check
    for (int i = 0; i < poly_sat.size(); i++)
    {
        auto minMax_A = getMinMax(poly_sat[i], this->Vertices);
        auto minMax_B = getMinMax(poly_sat[i], p.getVertices());

        isSeparated = (minMax_B.first > minMax_A.second || minMax_A.first > minMax_B.second);
        // 只要發現有一條分離線，就代表物體沒有發生碰撞
        if (isSeparated)
            return false;
    }
    return true;
}

bool Box::isCollide(Circle &c)
{
    this->setVertices();
    this->findSAT();
    auto box_sat = this->getSAT();
    Vec2 center(c.getPosition().x, c.getPosition().y);

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
            return false;
    }
    return true;
}
