#pragma once
#include <iostream>
#include <limits>
#include <cmath>
#include <fmt/core.h>
#include <SFML/Graphics.hpp>

/**
 * @file vector_math.h
 * @author  halloworld <william31212@gmail.com>
 * @brief 更詳細的向量數學計算function，繼承sf::Vector2<float>
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

/** @brief 徑度與度度量的轉換 */
#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)

/** @brief float_max, float_min常數 */
#define MAX_float std::numeric_limits<float>::max()
#define MIN_float std::numeric_limits<float>::min()

/**
 * @brief 補足sf::Vector2f無法做的計算，自行建立一個特化的Vector
 * @details 包括向量長度、內積、左右法向量等數學基本工具
 */
class Vector : public sf::Vector2<float> {
public:
    /**
     * @brief 向量x, y
     */
    Vector(float x, float y);
    /**
     * @brief Deconstructer
     */
    ~Vector() = default;
    /**
     * @brief 印出向量
     */
    void print_Vector();
    /**
     * @brief 獲得向量長度
     */
    float getLength();
    /**
     * @brief 計算兩向量內積
     * @param vec2 另一個向量
     * @retval float 內積結果
     */
    float dot(Vector &vec2);
    /**
     * @brief 計算vec在vec2上的正射影(投影長)
     * @param vec2 另一個向量
     * @retval float 投影長
     */
    float projectLengthOnto(Vector &vec2);
    /**
     * @brief 計左法向量
     */
    Vector normalL();
    /**
     * @brief 右法向量
     */
    Vector normalR();
    /**
     * @brief 旋轉
     * @param angle 旋轉角度(徑度)
     * @retval float 旋轉過後的向量
     */
    void rotate(float angle);
    //x′1=[(x1−x0)cosθ−(y1−y0)sinθ]+x0
    //y′1=[(y1−y0)cosθ+(x1−x0)sinθ]+y0
    /**
     * @brief 旋轉
     * @details 依照參考點旋轉該向量
     * @param ref 參考點
     * @param angle 旋轉角度(徑度)
     * @retval float 旋轉過後的向量
     */
    void rotate_ref(float angle, Vector &ref);
};