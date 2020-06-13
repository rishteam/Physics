#include "Shape.h"

std::deque<Vector> Shape::getSAT() const
{
    return SAT;
}

std::deque<Vector> Shape::getVertices() const
{
    return Vertices;
}

void Shape::findSAT()
{
    SAT.clear();
    int total_pt = Vertices.size();
    for (int i = 1; i < total_pt; i++)
    {
        float tmp_x = Vertices[i].x - Vertices[i - 1].x;
        float tmp_y = Vertices[i].y - Vertices[i - 1].y;

        Vector tmp(tmp_x, tmp_y);
        SAT.push_back(tmp.normalL());
    }
    Vector tmp2((Vertices[0].x - Vertices[total_pt - 1].x), (Vertices[0].y - Vertices[total_pt - 1].y));
    SAT.push_back(tmp2.normalL());
}