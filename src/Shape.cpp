#include "Shape.h"

std::deque<Vec2> Shape::getSAT() const
{
    return SAT;
}

std::deque<Vec2> Shape::getVertices() const
{
    return Vertices;
}

void Shape::findSAT()
{
    this->SAT.clear();
    int total_pt = this->Vertices.size();
    for (int i = 1; i < total_pt; i++)
    {
        float tmp_x = Vertices[i].x - Vertices[i - 1].x;
        float tmp_y = Vertices[i].y - Vertices[i - 1].y;

        Vec2 tmp(tmp_x, tmp_y);
        this->SAT.push_back(tmp.normalL());
    }
    Vec2 tmp2((Vertices[0].x - Vertices[total_pt - 1].x), (Vertices[0].y - Vertices[total_pt - 1].y));
    this->SAT.push_back(tmp2.normalL());
}
