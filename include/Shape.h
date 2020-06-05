#pragma once

#include "vector_math.h"
#include "Box.h"
#include "Polygon.h"
#include "Circle.h"

class Shape : public sf::Transformable, public sf::Drawable
{
public:
    std::deque<Vector> getSAT();
    std::deque<Vector> getVertices();
    void findSAT();
    virtual bool iscollide(const Box &b) const = 0;
    virtual bool iscollide(const Polygon &p) const = 0;
    virtual bool iscollide(const Circle &c) const = 0;
    /**
     * @brief 取得頂點在分離軸上的最大值與最小值
     * @param axis 分離軸
     * @param corner 頂點
     */
    std::pair<float, float> getMinMax(Vector &axis, std::deque<Vector> corner);
    virtual void set_debug_draw();

protected:
    /**
     * @brief 實際頂點位置
     * @details 因應各個不同角度，計算出實際頂點位置
     */
    std::deque<Vector> Vertices;
    /**
     * @brief 分離軸
     */
    std::deque<Vector> SAT;
};