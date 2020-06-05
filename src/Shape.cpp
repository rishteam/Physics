#include "Shape.h"

std::deque<Vector> Shape::getSAT()
{
    return SAT;
}

std::deque<Vector> Shape::getVertices()
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
    Vector tmp2((Vertices[0].x - Vertices[total - 1].x), (Vertices[0].y - Vertices[total - 1].y));
    SAT.push_back(tmp2.normalL());
}

std::pair<float, float> Shape::getMinMax(Vector &axis, std::deque<Vector> Vertices)
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
