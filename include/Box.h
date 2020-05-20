#pragma once
#include "vector_math.h"
#include <deque>
#include <SFML/Graphics.hpp>
#include <SFML/Audio.hpp>

/**
 * @file Box.h
 * @author  halloworld <william31212@gmail.com>
 * @brief Box為單位做為碰撞
 *
 * @section LICENSE
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option)any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details at
 * http://www.gnu.org/copyleft/gpl.html
 *
 * @section DESCRIPTION
 *
 * Example class
 * Lorem ipsum dolor sit amet, consectetur adipiscing elit. Vivamus egestas erat nec nulla placerat venenatis.
 * Proin a libero ac sapien mattis facilisis. Morbi nunc dui, rutrum nec condimentum at, volutpat vel odio.
 * Vivamus pharetra purus ac diam condimentum convallis. Proin lacinia vulputate leo ut ultricies.
 */


class Box : public sf::Transformable, public sf::Drawable {
public:
    /**
     * @brief 設定Box x, y, w, h
     */
    Box(float x, float y, float w, float h);
    /**
     * @brief Destroy the Example object
     */
    ~Box() = default;
    std::deque<Vector> getSAT();
    /**
     * @brief get corner point
     *
     */
    std::deque<Vector> getVertices();
    /**
     * @brief setting the angle(deg) you want to rotate
     * @param angle
     */
    void setVertices();
    /**
     * @brief find SAT
     */
    void findSAT();
    /**
     * @brief get left min and right max point in order to judge
     */
    std::pair<float, float> getMinMax(Vector &axis, std::deque<Vector> corner);
    /**
     * @brief judge the box collision
     */
    bool isCollide(Box &other);
    void set_draw();
    

private:
    sf::ConvexShape polygon;
    virtual void draw(sf::RenderTarget &target, sf::RenderStates states) const
    {
        states.transform *= getTransform();
        target.draw(polygon, states);
    };
    std::deque<Vector> corner;
    std::deque<Vector> Vertices;
    std::deque<Vector> SAT;
    float _w, _h;
};