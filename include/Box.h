#pragma once
#include "vector_math.h"
#include <deque>
#include <SFML/Graphics.hpp>
#include <SFML/Audio.hpp>

/**
 * @file Box.h
 * @author  halloworld <william31212@gmail.com>
 * @brief 以四邊形(Box)為單位做為碰撞，繼承sfml::transformable與sfml::drawable
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

/**
 * @brief 四邊形的設定，計算
 */
class Box : public sf::Transformable, public sf::Drawable {
public:
    /**
     * @brief 設定四邊形之x, y, w, h
     */
    Box(float x, float y, float w, float h);
    /**
     * @brief Deconstructer
     */
    ~Box() = default;
    /**
     * @brief 取得分離軸
     */
    std::deque<Vector> getSAT();
    /**
     * @brief 取得頂點陣列
     * @retval std::deque<Vector> SAT
     */
    std::deque<Vector> getVertices();
    /**
     * @brief 設定頂點陣列
     * @retval std::deque<Vector> Vertices
     */
    void setVertices();
    /**
     * @brief 尋找分離軸
     */
    void findSAT();
    /**
     * @brief 取得頂點在分離軸上的最大值與最小值
     * @param axis 分離軸
     * @param corner 頂點
     */
    std::pair<float, float> getMinMax(Vector &axis, std::deque<Vector> corner);
    /**
     * @brief 判斷是否有碰撞
     */
    bool isCollide(Box &other);
    /**
     * @brief 設定畫布，設定使用sf::ConvexShape畫
     */
    void set_debug_draw();

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
    /**
     * @brief 計算前頂點
     * @details 只存長、寬
     */
    std::deque<Vector> corner;
    /**
     * @brief 計算後頂點
     * @details 因應各個不同角度，計算出實際頂點位置
     */
    std::deque<Vector> Vertices;
    /**
     * @brief 分離軸
     */
    std::deque<Vector> SAT;
    /**
     * @brief 寬
     */
    float _w;
    /**
     * @brief 高
     */
    float _h;
};