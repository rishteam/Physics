#pragma once
#include "vector_math.h"
#include <deque>
#include <SFML/Graphics.hpp>
#include <SFML/Audio.hpp>


class Polygon : public Shape {
public:
    Polygon(std::deque<Vector> pt);
    ~Polygon() = default;
    void set_debug_draw() override;
    void setVertices(std::deque<Vector> &point);
    virtual bool isCollide(const Box &b) const override;
    virtual bool isCollide(const Polygon &s) const override;
    virtual bool isCollide(const Circle &c) const override;
private:
    /**
     * @brief 用sf::ConvexShape畫至sf::window上
     */
    sf::ConvexShape polygon;
    /**
     * @brief draw square
     * @details 配合sf::drawable
     */
    virtual void draw(sf::RenderTarget &target, sf::RenderStates states) const
    {
        target.draw(polygon, states);
    };
}